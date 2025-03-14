#include "depth_guidance.h"

#define PRINT(string,...) fprintf(stderr, "[depth_guidance->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

static abi_event depth_vector_ev;

// FSM states
enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  OUT_OF_BOUNDS
};

void print_array(float arr[], int size) {
  printf("[");
  for (int i = 0; i < size; i++) {
    printf("%.2f ", arr[i]);
  }
  printf("]\n\n");
}

/*
* ABI message callback function
*/
void depth_vector_cb(uint8_t __attribute__((unused)) sender_id, struct timeval time_stamp, float depth_vector[DEPTH_VECTOR_SIZE]) {
  printf("Received message: Timestamp: %.6f seconds\n", time_stamp.tv_sec + time_stamp.tv_usec / 1e6);

  print_array(depth_vector, DEPTH_VECTOR_SIZE);
}

/*
* Initialisation function
*/
void depth_guidance_init(void) {
  AbiBindMsgDEPTH_VECTOR(DEPTH_VECTOR_ID, &depth_vector_ev, depth_vector_cb);
}

/*
* Periodic function that checks it is safe to move forwards, and then sets a forward velocity setpoint or changes the heading
*/
void depth_guidance_periodic(void) {
  enum navigation_state_t navigation_state = SAFE;

  switch (navigation_state){
    case SAFE:
      // Get depth vector
      break;
    case OBSTACLE_FOUND:
      break;
    case OUT_OF_BOUNDS:
      break;
    default:
      break;
  }
  return;
}