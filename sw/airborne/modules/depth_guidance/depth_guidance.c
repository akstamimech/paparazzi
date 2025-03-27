#include "depth_guidance.h"

/*
Current states:
    TRAVEL: Fly forward and continuously turn towards the best heading within a certain range (so only
    angles right in front of the drone). If there's no good heading locally, go to REORIENT. When the
    drone is about to fly out of the green zone, go to AVOID_BOUNDARY

    REORIENT: Stop the drone and turn towards the best heading of all of the non-local options (so make
    a large turn). Once this heading is reached, go back to TRAVEL

    AVOID_BOUNDARY: Stop the drone and turn counterclockwise to valid heading
*/

static float depth_vector[DEPTH_VECTOR_SIZE];
static navigation_state_t current_state = TRAVEL;
const int local_idx_start = DEPTH_VECTOR_SIZE / 2 - 1 - LOCAL_IDX_RANGE; // Assuming even DEPTH_VECTOR_SIZE
const int local_idx_end = DEPTH_VECTOR_SIZE / 2 + LOCAL_IDX_RANGE;
static abi_event depth_vector_ev;
static uint8_t reorient_count;
static float left_avg_sq_depth;
static float center_avg_sq_depth; // Maybe use to adjust forward speed?
static float right_avg_sq_depth;
const char *state_names[] = {"TRAVEL", "REORIENT", "AVOID_BOUNDARY"};

void depth_vector_cb(uint8_t __attribute__((unused)) sender_id,
                     struct timeval time_stamp __attribute__((unused)),
                     float msg_depth_vector[DEPTH_VECTOR_SIZE]) {
    // Receive depth vector and calculate moving average over last 2 depth vectors

    for (int i = 0; i < DEPTH_VECTOR_SIZE; i++) {
        depth_vector[i] = fmax(0, fmin(1, MOV_AVG_FAC * msg_depth_vector[i] + (1.0f - MOV_AVG_FAC) * depth_vector[i]));
    }

    // Calculate squared depth for both sides, used in cost function
    left_avg_sq_depth = 0.0;
    center_avg_sq_depth = 0.0;
    right_avg_sq_depth = 0.0;

    for (int i = 0; i < local_idx_start; i++) {
        left_avg_sq_depth += depth_vector[i] * depth_vector[i];
    }
    for (int i = local_idx_start; i < local_idx_end+1; i++) {
        center_avg_sq_depth += depth_vector[i] * depth_vector[i];
    }
    for (int i = local_idx_end+1; i < DEPTH_VECTOR_SIZE; i++) {
        right_avg_sq_depth += depth_vector[i] * depth_vector[i];
    }

    left_avg_sq_depth /= (float)(local_idx_start + 1);
    center_avg_sq_depth /= (float)(2 * LOCAL_IDX_RANGE);
    right_avg_sq_depth /= (float)(local_idx_start + 1);
}

void depth_guidance_init(void) {
    AbiBindMsgDEPTH_VECTOR(DEPTH_VECTOR_ID, &depth_vector_ev, depth_vector_cb);
    reorient_count = 0;
}

float calc_heading_cost(int idx, bool is_local_heading) {
    /*
    Score a heading based on 4 variables: the depth in that direction, proximity to other high depth
    values (obstacles), distance (in terms of yaw angle from straight forward heading) and squared average
    depth on that side of the image. Lower cost is better
    */
    float depth = depth_vector[idx];

    float proximity = 0.0;
    if (idx > 0 && idx < DEPTH_VECTOR_SIZE - 1) {
        proximity += W_NEIGHBOR1 * (depth_vector[idx - 1] + depth_vector[idx + 1]);
        proximity += (idx > 1 && idx < DEPTH_VECTOR_SIZE - 2) ? W_NEIGHBOR2 * (depth_vector[idx - 2] + depth_vector[idx + 2]): 0.0;
    }

    float dist;
    if(is_local_heading) {
        int dist1 = abs(idx - (DEPTH_VECTOR_SIZE / 2 - 1));
        int dist2 = abs(idx - DEPTH_VECTOR_SIZE / 2);
        dist = (float)min(dist1, dist2);
    } else {
        dist = 0.0;
    }

    float side_sq_depth = (idx < DEPTH_VECTOR_SIZE / 2) ? left_avg_sq_depth: right_avg_sq_depth;
    // printf("%.2f ", W_OUTSIDE_OBJS * side_sq_depth);

    return W_DEPTH * depth + W_PROX * proximity + W_DIST * dist + W_OUTSIDE_OBJS * side_sq_depth;
}

float calc_angle_diff(float angle1, float angle2) {
    float diff = fmodf(angle2 - angle1, 2 * M_PI);
    if (diff > M_PI) {
        diff -= 2 * M_PI;
    } else if (diff < -M_PI) {
        diff += 2 * M_PI;
    }
    return diff;
}

float fsign(float val) {
    if (val < 0.0) {
        return -1.0;
    }
    return 1.0;
}

void calc_future_pos_with_vel(float *future_x, float *future_y) {
    float x = stateGetPositionEnu_f()->x;
    float y = stateGetPositionEnu_f()->y;
    float vx = stateGetSpeedEnu_f()->x;
    float vy = stateGetSpeedEnu_f()->y;
    
    float speed = sqrtf(vx * vx + vy * vy);
    
    // Avoid division by zero and unreliable values
    if (speed < 0.1) {
        *future_x = x;
        *future_y = y;
        return;
    }
    
    float dir_x = vx / speed;
    float dir_y = vy / speed;
    
    *future_x = x + dir_x * BOUNDARY_CHECK_DIST;
    *future_y = y + dir_y * BOUNDARY_CHECK_DIST;
}

void calc_future_pos_with_angle(float *future_x, float *future_y, float heading) {
    *future_x = stateGetPositionEnu_f()->x + cosf(heading) * 1.2 * BOUNDARY_CHECK_DIST;
    *future_y = stateGetPositionEnu_f()->y + sinf(heading) * 1.2 * BOUNDARY_CHECK_DIST;
}

float find_valid_heading(float start_heading) {
    // Check 8 headings in clockwise increments of pi/4 (45 degrees), starting from start_heading

    for (int i = 0; i < 8; i++) {
        float delta_heading = (float)i * (M_PI / 4);
        float new_heading = start_heading + delta_heading;

        if (new_heading < 0) {
            new_heading += 2 * M_PI;
        }

        float future_x, future_y;
        calc_future_pos_with_angle(&future_x, &future_y, new_heading);

        if (InsideCyberZoo(future_x, future_y)) {
            printf("NEW TARGET x: %f  y: %f \n", future_x, future_y);
            return delta_heading;  // Return first valid heading found
        }
    }

    return M_PI;  // No valid heading found, just turn around
}

void depth_guidance_periodic(void) {
    static float global_target_heading;
    static float boundary_avoid_angle;
    float current_heading = stateGetNedToBodyEulers_f()->psi;
    float best_global_cost = 9999.0;
    float best_local_cost = 9999.0;
    int best_global_index = -1;
    int best_local_index = -1;
    float future_pos_x;
    float future_pos_y;
    
    // printf("Outside cost: ");
    // Find the best local and global headings
    for (int i = 0; i < DEPTH_VECTOR_SIZE; i++) {
        bool is_local_heading = i >=  local_idx_start && i <= local_idx_end;

        float cost = calc_heading_cost(i, is_local_heading);
        printf("%.2f ", cost);
        
        if (is_local_heading) {
            if (cost < best_local_cost) {
                best_local_cost = cost;
                best_local_index = i;
            }
        } else {
            if (cost < best_global_cost) {
                best_global_cost = cost;
                best_global_index = i;
            }
        }
    }
    printf("\n");

    // See if drone will be out of bounds
    calc_future_pos_with_vel(&future_pos_x, &future_pos_y);
    bool boundary_ok = InsideCyberZoo(future_pos_x, future_pos_y); // Can also be InsideObstacleZone, but that is smaller area
    // bool boundary_ok = InsideObstacleZone(future_pos_x, future_pos_y);

    // Prevent running the drone when not in guided mode
    if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
        current_state = TRAVEL;
        reorient_count = 0;
        return;
    }

    // FSM logic
    switch (current_state) {
        case TRAVEL:
            if (!boundary_ok) {
                // float search_heading_start = atan2f(stateGetSpeedEnu_f()->y, stateGetSpeedEnu_f()->x);
                // boundary_avoid_angle = current_heading + find_valid_heading(search_heading_start);

                boundary_avoid_angle = current_heading + 0.5 * M_PI;
                current_state = AVOID_BOUNDARY;

            } else if (best_local_cost < COST_THRESHOLD || reorient_count > REORIENT_THRESHOLD) {
                float delta_heading = (float)(best_local_index + 1 - DEPTH_VECTOR_SIZE / 2) * (CAM_FOV / (2*DEPTH_VECTOR_SIZE));

                float forw_speed = FORW_SPEED;
                if (USE_DYNAMIC_SPEED) {
                    float forw_cost = (calc_heading_cost(DEPTH_VECTOR_SIZE/2-1, true) + calc_heading_cost(DEPTH_VECTOR_SIZE/2, true)) / 2;
                    if (forw_cost < 0.32) {
                        forw_speed = MAX_SPEED;
                    } else if (forw_cost > 0.6) {
                        forw_speed = MIN_SPEED;
                    } else {
                        forw_speed = FORW_SPEED;
                    }
                }

                // float side_speed = 2 * delta_heading * forw_speed;
                float side_speed = 0;

                guidance_h_set_body_vel(forw_speed, side_speed);
                guidance_h_set_heading(current_heading + delta_heading);

                reorient_count = max(0, reorient_count - 1);

            } else {
                global_target_heading = current_heading + (float)(best_global_index + 1 - DEPTH_VECTOR_SIZE / 2) * (CAM_FOV / 2);
                current_state = REORIENT; // No good heading locally, reorient
            }
            break;

        case REORIENT:
            // Turn in place towards global optimum heading
            float angle_diff = calc_angle_diff(global_target_heading, current_heading); // used to have fabs

            guidance_h_set_body_vel(0.0, 0.0);
            guidance_h_set_heading(global_target_heading);

            // float heading_dir = fsign(angle_diff);
            // guidance_h_set_heading_rate(heading_dir * RadOfDeg(TURN_RATE));

            if (fabs(angle_diff) < TURN_TOLERANCE) {
                reorient_count += 3;
                guidance_h_set_heading(global_target_heading); // remove if not work
                current_state = TRAVEL;
            }
            break;

        case AVOID_BOUNDARY:
            float bound_angle_diff = calc_angle_diff(boundary_avoid_angle, current_heading); // used to have fabs

            guidance_h_set_body_vel(0.0, 0.0);
            // guidance_h_set_heading(boundary_avoid_angle);

            // float bound_heading_dir = fsign(bound_angle_diff);
            // guidance_h_set_heading_rate(bound_heading_dir * RadOfDeg(TURN_RATE));
            guidance_h_set_heading_rate(RadOfDeg(TURN_RATE));

            // printf("angle diff: %f", fabs(bound_angle_diff));

            if (fabs(bound_angle_diff) < TURN_TOLERANCE) {
                reorient_count += 6;
                guidance_h_set_heading(boundary_avoid_angle); // remove if not work
                current_state = TRAVEL;
            }
            break;
    }

    printf("Current state: %s \n", state_names[current_state]);
    // printf("Reorient count: %d \n", reorient_count);
}