#include "lib/vision/image.h"
#include "depth_model.h"
#include "modules/computer_vision/cv.h"
#include "pthread.h"
#include "modules/core/abi.h"
#include <time.h>
#include <stdbool.h>
#include <time.h>
#include <stdio.h>

#ifndef PAPARAZZI_DEPTH_ESTIMATION_H
#define PAPARAZZI_DEPTH_ESTIMATION_H

#define PRINT(string, ...) fprintf(stderr, "[depth_estimation->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

// Define variables from config file
#ifndef DEPTH_ESTIMATION_FPS
#define DEPTH_ESTIMATION_FPS 0
#endif

#ifndef INPUT_DOWN_SAMPLE_FACTOR
#define INPUT_DOWN_SAMPLE_FACTOR 1
#endif

#ifndef DEPTH_VECTOR_SIZE
#define DEPTH_VECTOR_SIZE 65
#endif


// Define functions

struct depth_msg {
  struct timeval time_stamp;
  float depth_vector[DEPTH_VECTOR_SIZE];
  bool updated;
};

struct depth_estimation {
  uint8_t in_ds_factor;
  uint8_t in_cam_fps;
};
extern struct depth_estimation depth_estimation;

void convert_uint8_img_to_float(const uint8_t (*in_buffer)[1][520][480], float (*out_buffer)[1][520][480]);

void downsample_img(struct image_t *img);

struct image_t *depth_estimation_cb(struct image_t *img, uint8_t camera_id __attribute__((unused)));

extern void depth_estimation_init(void);

extern void depth_estimation_periodic(void);

#endif //PAPARAZZI_DEPTH_ESTIMATION_H
