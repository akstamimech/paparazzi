#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <time.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>

#ifndef PAPARAZZI_DEPTH_GUIDANCE_H
#define PAPARAZZI_DEPTH_GUIDANCE_H

// Define variables from config file
#ifndef DEPTH_VECTOR_ID
#define DEPTH_VECTOR_ID 38
#endif

#ifndef DEPTH_VECTOR_SIZE
#define DEPTH_VECTOR_SIZE 16
#endif

#ifndef FORW_SPEED
#define FORW_SPEED 0.5
#endif

#ifndef MOV_AVG_FAC
#define MOV_AVG_FAC 0.3
#endif

#ifndef COST_THRESHOLD
#define COST_THRESHOLD 1
#endif

#ifndef LOCAL_IDX_RANGE
#define LOCAL_IDX_RANGE 2
#endif

#ifndef W_DEPTH
#define W_DEPTH 1.0
#define W_PROX 1.0
#define W_DIST 0.2
#define W_NEIGHBOR1 0.5
#define W_NEIGHBOR2 0.2
#endif

#ifndef CAM_FOV
#define CAM_FOV 2.0
#endif

#ifndef TURN_TOLERANCE
#define TURN_TOLERANCE 0.3
#endif

#ifndef M_PI
#define M_PI 3.14159265358
#endif

typedef enum {
  TRAVEL,
  REORIENT
} navigation_state_t;

void depth_vector_cb(uint8_t __attribute__((unused)) sender_id, struct timeval time_stamp __attribute__((unused)), float msg_depth_vector[DEPTH_VECTOR_SIZE]);

extern void depth_guidance_init(void);

extern void depth_guidance_periodic(void);

float calc_heading_cost(int idx, bool is_local_heading);

float calc_angle_diff(float angle1, float angle2);

#endif //PAPARAZZI_DEPTH_GUIDANCE_H