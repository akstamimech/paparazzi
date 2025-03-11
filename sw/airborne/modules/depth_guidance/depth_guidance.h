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

extern void depth_guidance_init(void);

extern void depth_guidance_periodic(void);

#endif //PAPARAZZI_DEPTH_GUIDANCE_H
