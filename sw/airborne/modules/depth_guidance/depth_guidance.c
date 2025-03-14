#include "depth_guidance.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <time.h>
#include <stdio.h>

#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"

#define PRINT(string,...) fprintf(stderr, "[depth_guidance->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

static abi_event depth_vector_ev;

// FSM states
enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  AVOID,
  OUT_OF_BOUNDS,
  REENTER
};

// Define and initialise global variables
u_int8_t WIDTH = 65;
float FOV = 0.7175;

enum navigation_state_t navigation_state = SAFE; // initial state
u_int8_t max_depth_index = 0;         // index of max depth in depth vector  
float prev_yaw = 0;                   // previous yaw value
float new_yaw = 0;                    // new yaw value
float max_speed = 0.5f;               // max flight speed [m/s]
float yaw_threshold = 0.1f;           // yaw threshold for changing heading [rad]
float maxDistance = 0.25;             // max waypoint displacement [m]
float patience = 5.0;           // time to wait before going back to arena [s]
float current_time = 0;               // current time [s]
float center_x = 0;
float center_y = 0;
bool quadrant = false;


/*
* ABI message callback function
*/
void depth_vector_cb(uint8_t __attribute__((unused)) sender_id, struct timeval time_stamp, float depth_vector[DEPTH_VECTOR_SIZE]) {
  // printf("Received message: Timestamp: %.6f seconds\n", time_stamp.tv_sec + time_stamp.tv_usec / 1e6);
  float max_depth = 0;
  int max_index = 0;
  for(int i=0; i<DEPTH_VECTOR_SIZE; i++) {
    float cur_depth = depth_vector[i];
    if (cur_depth > max_depth) {
      max_depth = cur_depth;
      max_index = i;
    }
    // printf("%f ", cur_depth);
  }
  max_depth_index = max_index;
  // printf("The max depth of current frame is %f, according index is %d", max_depth, max_index);
}

/*
* Initialisation function
*/
void depth_guidance_init(void) {
  // System initialisation
  srand(time(NULL));
  AbiBindMsgDEPTH_VECTOR(DEPTH_VECTOR_ID, &depth_vector_ev, depth_vector_cb); 

  // Initialise starting yaw -> starting direction
  prev_yaw = cal_yaw();
  nav.heading = prev_yaw;
  PRINT("Initialised yaw of %f\n", prev_yaw);

  // Set center of the arena
  center_x = stateGetPositionEnu_f()->x;
  center_y = stateGetPositionEnu_f()->y;
  PRINT("Center of the arena: x: %f, y: %f\n", center_x, center_y);
}

/*
Calculate yaw from max depth index
*/
float cal_yaw() {
  return (max_depth_index - WIDTH/2) * FOV / WIDTH;
}


/* 
Check if the drone is out of bounds
*/
bool out_of_bounds() {
  float x = stateGetPositionEnu_f()->x;
  float y = stateGetPositionEnu_f()->y;
  float dis = x*x + y*y;
  /*
    if drone is in the 1, 3 quadrant, then rotate by 45 degrees
    else rotate by -45 degrees
  */
  bool quadrant = (x-center_x) * (y-center_y) > 0;
  if (dis < 8){
    max_speed = 1.0f;
  } else if (dis < 10){
    max_speed = 0.8f;
  } else if (dis > 11) {
    max_speed = 0.5f;
    patience = (dis - 10) / max_speed;
    printf("Patience: %f\n", patience);
    return true;
  }
  return false;
}

/*
* Function that checks it is safe to move forwards, and then sets a forward velocity setpoint or changes the heading
*/
void depth_guidance_periodic(void) {
  // only evaluate our state machine if we are flying
  if(!autopilot_in_flight()){
    return;
  }

  if (navigation_state != REENTER) {
    if (out_of_bounds()) {
      navigation_state = OUT_OF_BOUNDS;
    }
  }

  PRINT("Current state: %d\n", navigation_state);

  switch (navigation_state){
    case SAFE:
      moveWaypointForward(WP_TRAJECTORY, 1.5f * maxDistance);
      // Get depth vector
      new_yaw = cal_yaw();
      PRINT("Current yaw: %f\n", new_yaw);
      if (abs(new_yaw - prev_yaw) > yaw_threshold) {
        navigation_state = OBSTACLE_FOUND;
      } else {
        moveWaypointForward(WP_GOAL, maxDistance);
        moveWaypointForward(WP_RETREAT, -1.0f * maxDistance);
      }
      prev_yaw = new_yaw;
      break;

    case OBSTACLE_FOUND:
      // stop
      waypoint_move_here_2d(WP_GOAL);
      waypoint_move_here_2d(WP_RETREAT);
      waypoint_move_here_2d(WP_TRAJECTORY);

      navigation_state = AVOID;
      break;

    case AVOID:
      increase_nav_heading(new_yaw - prev_yaw);
      if (abs(cal_yaw() - prev_yaw) < yaw_threshold) {
        navigation_state = SAFE;
      }
      break;

    case OUT_OF_BOUNDS:
      float rotation = 270;
      if (quadrant)
      {
        rotation = -75;
      }
      increase_nav_heading(rotation);
      current_time = clock();;
      navigation_state = REENTER;
      break;

    case REENTER:
      float duration = (clock() - current_time) / CLOCKS_PER_SEC;
      PRINT("Duration: %f\n", duration);
      if (duration > patience) {
        if (out_of_bounds()) {
          navigation_state = OUT_OF_BOUNDS;
        } else {
          navigation_state = SAFE;
        }
      } else {
        moveWaypointForward(WP_GOAL, maxDistance);
      }
      break;

    default:
      break;
  }
  return;
}

/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(float incrementDegrees)
{
  float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(incrementDegrees);

  // normalize heading to [-pi, pi]
  FLOAT_ANGLE_NORMALIZE(new_heading);

  // set heading, declared in firmwares/rotorcraft/navigation.h
  nav.heading = new_heading;

  PRINT("Increasing heading to %f\n", DegOfRad(new_heading));
  return -1;
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return -1;
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
  float heading  = stateGetNedToBodyEulers_f()->psi;

  // Now determine where to place the waypoint you want to go to
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
  // PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters,	
  //               POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y),
  //               stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, DegOfRad(heading));
  return -1;
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x),
                POS_FLOAT_OF_BFP(new_coor->y));
  waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
  return -1;
}