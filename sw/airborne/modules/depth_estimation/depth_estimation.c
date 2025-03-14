#include "depth_estimation.h"

#define PRINT(string, ...) fprintf(stderr, "[depth_estimation->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

struct image_t downsampled_img = {.buf=NULL, .buf_size=0};

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



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



/*
  Convert image values from uint8 to float array for use in depth_model. Find a better way...
  I hardcoded all the sizes, so this won't work when downsampling
*/
void convert_uint8_img_to_float(const uint8_t (*in_buffer)[1][520][480], float (*out_buffer)[1][520][480]) {
  for (int row = 0; row < 520; row++) {
      for (int col = 0; col < 480; col++) {
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

  // struct image_t gray_img;
  // image_create(&gray_img, downsampled_img.w, downsampled_img.h, IMAGE_GRAYSCALE);
  // image_to_grayscale(&downsampled_img, &gray_img);

  // Run neural network using gray_img. The buffer is indexed in one number. So if the image
  // has 120 columns, row 10 column 5 has index 10*120+5 = 1205
  // But uyvy has 2 * width, so really it's 10*120 * 2 +5 = 1205
  // const uint8_t (*in_buffer)[1][520][240] = (const uint8_t (*)[1][520][240]) gray_img.buf;
  const uint8_t (*in_buffer)[1][520][480] = (const uint8_t (*)[1][520][480]) downsampled_img.buf;

  float float_buffer[1][1][520][480] = {0};
  convert_uint8_img_to_float(in_buffer, float_buffer);

  float out_buffer[1][DEPTH_VECTOR_SIZE] = {0};  // Initialize output buffer

  static clock_t start_time, end_time;
  static double elapsed_time;

  start_time = clock();
  entry(float_buffer, out_buffer);
  end_time = clock();

  elapsed_time = ((double)(end_time - start_time)) / CLOCKS_PER_SEC;
  printf("Time taken to create depth vector: %f\n", elapsed_time);

  // image_free(&gray_img);

  pthread_mutex_lock(&mutex);
  global_depth_msg.time_stamp = img->ts;
  memcpy(global_depth_msg.depth_vector, out_buffer[0], DEPTH_VECTOR_SIZE * sizeof(float));
  global_depth_msg.updated = true;
  pthread_mutex_unlock(&mutex);
  
  return &downsampled_img; // Return (original / modified) image
}

/*
  Initialize the module
*/
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

    local_depth_msg.updated = false;
    pthread_mutex_lock(&mutex);
    memcpy(&global_depth_msg, &local_depth_msg, sizeof(struct depth_msg));
    pthread_mutex_unlock(&mutex);
  }
}