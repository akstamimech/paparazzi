#include "depth_estimation.h"

#define PRINT(string, ...) fprintf(stderr, "[depth_estimation->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

struct image_t downsampled_img = {.buf=NULL, .buf_size=0};

uint8_t white[4] = {127, 255, 127, 255};

/*
  Struct for keeping track of all the module settings
*/
struct depth_estimation depth_estimation = {
  .in_ds_factor = INPUT_DOWN_SAMPLE_FACTOR,
  .in_cam_fps = DEPTH_ESTIMATION_FPS,
};

/*
  Globally updated depth message, which is updated in the video callback and copied to local in the periodic function using
  a mutex. If the message was updated, it's sent This ensures that the message is sent on the autopilot thread instead of
  the callback / videa thread
*/
struct depth_msg global_depth_msg;

static pthread_mutex_t mutex;

static clock_t start_time, end_time;
static double elapsed_time;



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void draw_depth_vector(struct image_t *img, float depth_vector[1][DEPTH_VECTOR_SIZE]) {
  // Find max depth value for normalization
  float max_depth = 0.0;
  for (int i = 0; i < DEPTH_VECTOR_SIZE; i++) {
      if (depth_vector[0][i] > max_depth) {
          max_depth = depth_vector[0][i];
      }
  }

  // Draw bars
  for (int i = 0; i < DEPTH_VECTOR_SIZE; i++) {
      float normalized_height = depth_vector[0][i] / max_depth;
      int bar_height = (int)(normalized_height * (float)img->w / 10);

      // Compute Y positions (bars are stacked from top to bottom)
      int y_min = (img->h / DEPTH_VECTOR_SIZE) * i;
      int y_max = y_min + (img->h / DEPTH_VECTOR_SIZE);

      image_draw_rectangle(img, 0, bar_height, y_min, y_max, white);
  }
}

void print_array(float arr[], int size) {
  printf("[");
  for (int i = 0; i < size; i++) {
    printf("%.2f ", arr[i]);
  }
  printf("]\n\n");
}

void print_int_array(uint8_t arr[], int size) {
  printf("[");
  for (int i = 0; i < size; i++) {
    printf("%d ", arr[i]);
  }
  printf("]\n\n");
}

void uyvy_to_yuv(float input_array[1][3][260][120], uint8_t *buf, uint16_t width, uint16_t height) {
  int x, y, idx;
  
  for (y = 0; y < height; y++) {
    for (x = 0; x < width; x += 2) {  // Process two pixels at a time
        idx = (y * width + x) * 2;  // Compute buffer index

        uint8_t u = buf[idx];      // U value for both pixels
        uint8_t y1 = buf[idx + 1]; // Y value for first pixel
        uint8_t v = buf[idx + 2];  // V value for both pixels
        uint8_t y2 = buf[idx + 3]; // Y value for second pixel

        // Store values in the input array (convert to float)
        input_array[0][0][y][x] = (float)y1;  // Y channel
        input_array[0][0][y][x + 1] = (float)y2;  // Y for the next pixel
        input_array[0][1][y][x] = (float)u;
        input_array[0][1][y][x + 1] = (float)u;
        input_array[0][2][y][x] = (float)v;
        input_array[0][2][y][x + 1] = (float)v;
    }
  }
}

void save_input_array(const char *filename, float input_array[1][3][260][120]) {
  FILE *file = fopen(filename, "w");
  if (!file) {
      perror("Failed to open file");
      return;
  }

  for (int c = 0; c < 3; c++) {
      for (int h = 0; h < 260; h++) {
          for (int w = 0; w < 120; w++) {
              fprintf(file, "%.6f ", input_array[0][c][h][w]);  // Space-separated values
          }
      }
  }

  fclose(file);
}

/*
  Video callback. Processes a camera image when available, and returns a depth map
*/
struct image_t *depth_estimation_cb(struct image_t *img, uint8_t camera_id __attribute__((unused))) {
  start_time = clock();
  
  // Down sample image
  if(downsampled_img.buf_size < img->buf_size/(depth_estimation.in_ds_factor*depth_estimation.in_ds_factor)){
    image_free(&downsampled_img);
    image_create(&downsampled_img,
                 img->w / depth_estimation.in_ds_factor,
                 img->h / depth_estimation.in_ds_factor,
                 IMAGE_YUV422);
  }

  image_yuv422_downsample(img, &downsampled_img, depth_estimation.in_ds_factor);

  float input_array[1][3][260][120];
  float depth_vector[1][DEPTH_VECTOR_SIZE];

  // uyvy_to_yuv(input_array, img->buf, img->w, img->h);
  uyvy_to_yuv(input_array, downsampled_img.buf, downsampled_img.w, downsampled_img.h);

  entry(input_array, depth_vector);
  // print_array(depth_vector[0], DEPTH_VECTOR_SIZE);

  pthread_mutex_lock(&mutex);
  global_depth_msg.time_stamp = img->ts;
  memcpy(global_depth_msg.depth_vector, depth_vector[0], DEPTH_VECTOR_SIZE * sizeof(float));
  global_depth_msg.updated = true;
  pthread_mutex_unlock(&mutex);

  if (DRAW_ON_IMAGE) {
    draw_depth_vector(img, depth_vector);
  }

  if (PROFILE_CNN) {
    end_time = clock();
    elapsed_time = ((double)(end_time - start_time)) / CLOCKS_PER_SEC;
    printf("Time taken to create depth vector: %.2f s\n", elapsed_time);
  }
  
  return img;
}

void depth_estimation_init(void) {
  cv_add_to_device(&DEPTH_ESTIMATION_CAMERA, depth_estimation_cb, depth_estimation.in_cam_fps, 0);

  memset(&global_depth_msg, 0, sizeof(struct depth_msg));
  pthread_mutex_init(&mutex, NULL);
}

void depth_estimation_periodic(void) {
  static struct depth_msg local_depth_msg;
  pthread_mutex_lock(&mutex);
  memcpy(&local_depth_msg, &global_depth_msg, sizeof(struct depth_msg));
  pthread_mutex_unlock(&mutex);

  if (local_depth_msg.updated) {
    AbiSendMsgDEPTH_VECTOR(DEPTH_VECTOR_ID, local_depth_msg.time_stamp, local_depth_msg.depth_vector);

    pthread_mutex_lock(&mutex);
    global_depth_msg.updated = false;
    pthread_mutex_unlock(&mutex);
  }
}