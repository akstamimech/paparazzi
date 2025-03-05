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
#include "modules/computer_vision/cv.h"

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
struct image_t *downsampled_img_ptr = &downsampled_img;

/*
  Struct for keeping track of all the module settings
*/
struct depth_estimation depth_estimation = {
  .in_ds_factor = INPUT_DOWN_SAMPLE_FACTOR,
  .in_cam_fps = DEPTH_ESTIMATION_FPS,
};

/*
  Video callback. Processes a camera image when available, and returns a depth map
*/
struct image_t *depth_estimation_cb(struct image_t *img, uint8_t camera_id __attribute__((unused))) {
  // Down sample image
  if(downsampled_img_ptr->buf_size < img->buf_size/(depth_estimation.in_ds_factor*depth_estimation.in_ds_factor)){
    if(downsampled_img_ptr->buf != NULL){
      image_free(downsampled_img_ptr);
    }
    image_create(downsampled_img_ptr,
                 img->w / depth_estimation.in_ds_factor,
                 img->h / depth_estimation.in_ds_factor,
                 IMAGE_YUV422);
  }

  image_yuv422_downsample(img, downsampled_img_ptr, depth_estimation.in_ds_factor);

  printf("Width: %d\n", downsampled_img_ptr->w);

  // Run  neural network using img
  uint8_t *buffer = img->buf;
  printf("IMAGE BUFFER CONTENTS: --------------------------------------------------------");
  

  return downsampled_img_ptr; // Return (original / modified) image
}

/*
  Initialize the module
*/
void depth_estimation_init(void) {
  cv_add_to_device(&DEPTH_ESTIMATION_CAMERA, depth_estimation_cb, depth_estimation.in_cam_fps, 0);
}
