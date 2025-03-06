/*
 * Copyright (C) 2021 Matteo Barbera <matteo.barbera97@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include "lib/vision/image.h"
#include "depth_estimation.h"
#include "depth_model.h"
#include "modules/computer_vision/cv.h"
#include <time.h>

#include <stdio.h>

// Define variables from config file
#ifndef DEPTH_ESTIMATION_FPS
#define DEPTH_ESTIMATION_FPS 0
#endif

#ifndef INPUT_DOWN_SAMPLE_FACTOR
#define INPUT_DOWN_SAMPLE_FACTOR 1
#endif

#define PRINT(string, ...) fprintf(stderr, "[depth_estimation->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct image_t downsampled_img = {.buf=NULL, .buf_size=0};

/*
  Struct for keeping track of all the module settings
*/
struct depth_estimation depth_estimation = {
  .in_ds_factor = INPUT_DOWN_SAMPLE_FACTOR,
  .in_cam_fps = DEPTH_ESTIMATION_FPS,
};

/*
  Convert image values from uint8 to float array for use in depth_model. Find a better way...
  I hardcoded all the sizes, so this won't work when downsampling
*/
void convert_uint8_img_to_float(const uint8_t (*in_buffer)[1][520][240], float (*out_buffer)[1][520][240]) {
  for (int row = 0; row < 520; row++) {
      for (int col = 0; col < 240; col++) {
          // Convert uint8_t (0-255) to float (0.0 - 1.0)
          out_buffer[0][0][row][col] = (float)(in_buffer[0][0][row][col]) / 255.0f;
      }
  }
}

/*
  Video callback. Processes a camera image when available, and returns a depth map
*/
struct image_t *depth_estimation_cb(struct image_t *img, uint8_t camera_id __attribute__((unused))) {
  // Down sample image
  if(downsampled_img.buf_size < img->buf_size/(depth_estimation.in_ds_factor*depth_estimation.in_ds_factor)){
    image_free(&downsampled_img);
    image_create(&downsampled_img,
                 img->w / depth_estimation.in_ds_factor,
                 img->h / depth_estimation.in_ds_factor,
                 IMAGE_YUV422);
  }

  image_yuv422_downsample(img, &downsampled_img, depth_estimation.in_ds_factor);

  // Convert to grayscale. Using the original yuyv values requires quite a few modifications to
  // the neural network input, and some code to read neighboring pixels (see crash course page
  // 22 or the function "find_object_centroid" in "cv_detect_color_object.c")

  // SUGGESTION: Could maybe do this in place by indexing smartly. image_to_grayscale seems to
  // just grab the y values from uyvy. We could do that as well somehow, without copying an
  // image into different memory again (which is what happens when we create gray_img).
  // Could maybe combine downsample and grayscale steps as well, saving memory
  struct image_t gray_img;
  image_create(&gray_img, downsampled_img.w, downsampled_img.h, IMAGE_GRAYSCALE);
  image_to_grayscale(&downsampled_img, &gray_img);

  // Run neural network using gray_img. The buffer is indexed in one number. So if the image
  // has 120 columns, row 10 column 5 has index 10*120+5 = 1205
  const uint8_t (*in_buffer)[1][520][240] = (const uint8_t (*)[1][520][240]) gray_img.buf;

  float float_buffer[1][1][520][240] = {0};
  convert_uint8_img_to_float(in_buffer, float_buffer);

  float out_buffer[1][1][520][240] = {0};  // Initialize output buffer

  static clock_t start_time, end_time;
  static double elapsed_time;

  start_time = clock();
  printf("test");
  entry(float_buffer, out_buffer);
  end_time = clock();

  elapsed_time = ((double)(end_time - start_time)) / CLOCKS_PER_SEC;
  printf("Time taken to create depth map: %f\n", elapsed_time);

  image_free(&gray_img);

  printf("Buffer size original: %d\n", img->buf_size);
  printf("Buffer size downsample: %d\n", downsampled_img.buf_size);
  printf("Buffer size gray: %d\n", gray_img.buf_size);
  printf("Gray image shape: width: %d, height: %d\n", gray_img.w, gray_img.h);
  
  return &downsampled_img; // Return (original / modified) image
}

/*
  Initialize the module
*/
void depth_estimation_init(void) {
  cv_add_to_device(&DEPTH_ESTIMATION_CAMERA, depth_estimation_cb, depth_estimation.in_cam_fps, 0);
}
