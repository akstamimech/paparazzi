#include "depth_guidance.h"

static float depth_vector[DEPTH_VECTOR_SIZE];
static navigation_state_t current_state = TRAVEL;
const int local_idx_start = DEPTH_VECTOR_SIZE / 2 - LOCAL_IDX_RANGE;
const int local_idx_end = DEPTH_VECTOR_SIZE / 2 + 1 + LOCAL_IDX_RANGE;
static abi_event depth_vector_ev;

void depth_vector_cb(uint8_t __attribute__((unused)) sender_id,
                     struct timeval time_stamp __attribute__((unused)),
                     float msg_depth_vector[DEPTH_VECTOR_SIZE]) {
    // Receive depth vector and calculate moving average over last 2 depth vectors

    for (int i = 0; i < DEPTH_VECTOR_SIZE; i++) {
        depth_vector[i] = MOV_AVG_FAC * msg_depth_vector[i] + (1.0f - MOV_AVG_FAC) * depth_vector[i];
    }
}

void depth_guidance_init(void) {
    AbiBindMsgDEPTH_VECTOR(DEPTH_VECTOR_ID, &depth_vector_ev, depth_vector_cb); 
}

void depth_guidance_periodic(void) {
    if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
        current_state = TRAVEL;
        return;
    }

    static float global_target_heading = 0.0;
    float current_heading = stateGetNedToBodyEulers_f()->psi;
    float best_global_cost = 9999.0;
    float best_local_cost = 9999.0;
    int best_global_index = -1;
    int best_local_index = -1;
    
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
    printf("\nBest local cost: %f \n\n", best_local_cost);

    switch (current_state) {
        case TRAVEL:
            if (best_local_cost < COST_THRESHOLD) {
                float delta_heading = (float)(best_local_index + 1 - DEPTH_VECTOR_SIZE / 2) * (CAM_FOV / (2*DEPTH_VECTOR_SIZE));
                printf("\nDelta: %f", delta_heading);

                guidance_h_set_body_vel(FORW_SPEED, 0.0);
                guidance_h_set_heading(current_heading + delta_heading);
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
                current_state = TRAVEL;
            }
            break;
    }

    // printf("State: %d", current_state);
}

float calc_heading_cost(int idx, bool is_local_heading) {
    /*
    Score a heading based on 3 variables: the depth in that direction, proximity to high depth values (obstacles)
    and distance (in terms of yaw angle from straight forward heading). Lower cost is better
    */
    // Possible addition for local costs: Distance to global optimum cost
    float depth = depth_vector[idx];

    float proximity = 0.0;
    if (idx > 0 && idx < DEPTH_VECTOR_SIZE - 1) {
        proximity += W_NEIGHBOR1 * (depth_vector[idx - 1] + depth_vector[idx + 1]);
        proximity += (idx > 1 && idx < DEPTH_VECTOR_SIZE - 2) ? W_NEIGHBOR2 * (depth_vector[idx - 2] + depth_vector[idx + 2]): 0.0;
    }

    float dist = (is_local_heading) ? (float)(idx - DEPTH_VECTOR_SIZE / 2): 0.0;

    return W_DEPTH * depth + W_PROX * proximity + W_DIST * dist;
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