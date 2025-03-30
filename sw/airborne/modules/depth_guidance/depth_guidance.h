#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <time.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>

#define NAV_C // Needed for geofencing, gives "InsideObstacleZone" function
#include "generated/flight_plan.h"

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

#ifndef MIN_SPEED
#define MIN_SPEED 0.25
#endif

#ifndef MAX_SPEED
#define MAX_SPEED 0.8
#endif

#ifndef MIN_SPEED
#define MIN_SPEED 0.5
#endif

#ifndef USE_DYNAMIC_SPEED
#define USE_DYNAMIC_SPEED false
#endif

#ifndef MOV_AVG_FAC
#define MOV_AVG_FAC 0.3
#endif

#ifndef COST_THRESHOLD
#define COST_THRESHOLD 1
#endif

#ifndef REORIENT_THRESHOLD
#define REORIENT_THRESHOLD 5
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
#define W_OUTSIDE_OBJS 0.1
#endif

#ifndef CAM_FOV
#define CAM_FOV 2.0
#endif

#ifndef TURN_TOLERANCE
#define TURN_TOLERANCE 0.3
#endif

#ifndef TURN_RATE
#define TURN_RATE 30
#endif

#ifndef BOUNDARY_CHECK_DIST
#define BOUNDARY_CHECK_DIST 1.0
#endif

#ifndef M_PI
#define M_PI 3.14159265358
#endif

#ifndef min
#define min(a, b) ((a) < (b) ? (a) : (b))
#endif

#ifndef max
#define max(a, b) ((a) > (b) ? (a) : (b))
#endif

typedef enum {
  TRAVEL,
  REORIENT,
  AVOID_BOUNDARY
} navigation_state_t;

void depth_vector_cb(uint8_t __attribute__((unused)) sender_id, struct timeval time_stamp __attribute__((unused)), float msg_depth_vector[DEPTH_VECTOR_SIZE]);

float calc_heading_cost(int idx, bool is_local_heading);

float calc_angle_diff(float angle1, float angle2);

void calc_future_pos_with_vel(float *future_x, float *future_y);

extern void depth_guidance_init(void);

extern void depth_guidance_periodic(void);

#endif //PAPARAZZI_DEPTH_GUIDANCE_H