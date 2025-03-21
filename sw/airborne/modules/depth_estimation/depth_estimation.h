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

// Define variables from config file
#ifndef DEPTH_ESTIMATION_FPS
#define DEPTH_ESTIMATION_FPS 0
#endif

#ifndef INPUT_DOWN_SAMPLE_FACTOR
#define INPUT_DOWN_SAMPLE_FACTOR 1
#endif

#ifndef DEPTH_VECTOR_SIZE
#define DEPTH_VECTOR_SIZE 16
#endif

#ifndef DRAW_ON_IMAGE
#define DRAW_ON_IMAGE false
#endif

#ifndef PROFILE_CNN
#define PROFILE_CNN false
#endif

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

void save_input_array(const char *filename, float input_array[1][3][260][120]);

void draw_depth_vector(struct image_t *img, float depth_vector[1][DEPTH_VECTOR_SIZE]);

void print_array(float arr[], int size);

void print_int_array(uint8_t arr[], int size);

// void uyvy_to_yuv(float input_array[1][3][520][240], uint8_t *buf, uint16_t width, uint16_t height);
void uyvy_to_yuv(float input_array[1][3][260][120], uint8_t *buf, uint16_t width, uint16_t height);

void downsample_img(struct image_t *img);

struct image_t *depth_estimation_cb(struct image_t *img, uint8_t camera_id __attribute__((unused)));

extern void depth_estimation_init(void);

extern void depth_estimation_periodic(void);

#endif //PAPARAZZI_DEPTH_ESTIMATION_H
