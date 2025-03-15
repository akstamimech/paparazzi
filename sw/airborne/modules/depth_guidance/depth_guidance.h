#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include <time.h>
#include "modules/core/abi.h"
#include <stdio.h>
#include <time.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#ifndef PAPARAZZI_DEPTH_GUIDANCE_H
#define PAPARAZZI_DEPTH_GUIDANCE_H

// Define variables from config file
#ifndef DEPTH_VECTOR_ID
#define DEPTH_VECTOR_ID 38
#endif

#ifndef DEPTH_VECTOR_SIZE
#define DEPTH_VECTOR_SIZE 65
#endif


// Define functions

void depth_vector_cb(uint8_t __attribute__((unused)) sender_id, struct timeval time_stamp, float depth_vector[DEPTH_VECTOR_SIZE]);

float cal_yaw(void);

bool out_of_bounds(void);

extern void depth_guidance_init(void);

extern void depth_guidance_periodic(void);

// Function declarations
uint8_t increase_nav_heading(float incrementDegrees);

uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters);

uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);

uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);

#endif //PAPARAZZI_DEPTH_GUIDANCE_H
