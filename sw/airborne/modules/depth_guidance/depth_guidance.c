#include "depth_guidance.h"

// Current states:
// TRAVEL: Fly forward and continuously turn towards the best heading within a certain range (so only
// angles right in front of the drone). If there's no good heading locally, go to REORIENT. When the
// drone is about to fly out of the green zone, go to AVOID_BOUNDARY
//
// REORIENT: Stop the drone and turn towards the best heading of all of the non-local options (so make
// a large turn). Once this heading is reached, go back to TRAVEL
//
// AVOID_BOUNDARY: Stop the drone and turn 180 degrees

static float depth_vector[DEPTH_VECTOR_SIZE];
static navigation_state_t current_state = TRAVEL;
const int local_idx_start = DEPTH_VECTOR_SIZE / 2 - 1 - LOCAL_IDX_RANGE; // Assuming even DEPTH_VECTOR_SIZE
const int local_idx_end = DEPTH_VECTOR_SIZE / 2 + LOCAL_IDX_RANGE;
static abi_event depth_vector_ev;
static uint8_t reorient_count;
static float left_avg_sq_depth;
static float center_avg_sq_depth; // Maybe use to adjust forward speed?
static float right_avg_sq_depth;

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
    printf("start: %d   end: %d", local_idx_start, local_idx_end);
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
    
    // printf("reorient count: %d \n", reorient_count);
    
    // Find the best local and global headings
    for (int i = 0; i < DEPTH_VECTOR_SIZE; i++) {
        bool is_local_heading = i >=  local_idx_start && i <= local_idx_end;

        float cost = calc_heading_cost(i, is_local_heading);
        // printf("%.2f ", cost);
        
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

    // See if drone will be out of bounds
    calc_future_pos(&future_pos_x, &future_pos_y);
    bool boundary_ok = InsideCyberZoo(future_pos_x, future_pos_y); // Can also be InsideObstacleZone, but this is smaller area

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
                float curr_vel_heading = atan2f(stateGetSpeedNed_f()->y, stateGetSpeedNed_f()->x);
                boundary_avoid_angle = curr_vel_heading + M_PI; // Couldn't get Lemon's code to work yet, that's way better than this
                current_state = AVOID_BOUNDARY;

            } else if (best_local_cost < COST_THRESHOLD || reorient_count > REORIENT_THRESHOLD) {
                float delta_heading = (float)(best_local_index + 1 - DEPTH_VECTOR_SIZE / 2) * (CAM_FOV / (2*DEPTH_VECTOR_SIZE));
                float side_speed = delta_heading * FORW_SPEED;
                // printf("Side speed: %f", side_speed);

                guidance_h_set_body_vel(FORW_SPEED, side_speed);
                guidance_h_set_heading(current_heading + delta_heading);

                reorient_count = max(0, reorient_count - 1);

            } else {
                global_target_heading = current_heading + (float)(best_global_index + 1 - DEPTH_VECTOR_SIZE / 2) * (CAM_FOV / 2);
                current_state = REORIENT; // No good heading locally, reorient
            }
            break;

        case REORIENT:
            // Turn in place towards global optimum heading
            float angle_diff = calc_angle_diff(global_target_heading, current_heading);

            guidance_h_set_body_vel(0.0, 0.0);
            guidance_h_set_heading(global_target_heading);

            if (angle_diff < TURN_TOLERANCE) {
                reorient_count += 3;
                current_state = TRAVEL;
            }
            break;

        case AVOID_BOUNDARY:
            float bound_angle_diff = calc_angle_diff(boundary_avoid_angle, current_heading);

            guidance_h_set_body_vel(0.0, 0.0);
            guidance_h_set_heading(boundary_avoid_angle);

            if (bound_angle_diff < TURN_TOLERANCE) {
                current_state = TRAVEL;
            }
            break;
    }

    // printf("State: %d", current_state);
}

float calc_heading_cost(int idx, bool is_local_heading) {
    /*
    Score a heading based on 4 variables: the depth in that direction, proximity to other high depth
    values (obstacles), distance (in terms of yaw angle from straight forward heading) and squared average
    depth on that side of the image. Lower cost is better
    */
    // Possible addition for local costs: Distance to global optimum cost
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

    return W_DEPTH * depth + W_PROX * proximity + W_DIST * dist + W_OUTSIDE_OBJS * side_sq_depth;
    // return W_DEPTH * depth;
    // return W_PROX * proximity;
    // return W_DIST * dist;
    // return W_OUTSIDE_OBJS * side_sq_depth;
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


void calc_future_pos(float *future_x, float *future_y) {
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