#include "behavior.h"

using namespace std;

Behavior::Behavior(vector<vector<double>> const &sensor_fusion, CarData car, Predictions const &predictions) {
  // generate self vehicle behavior
  Target target;
  target.time = 2.0;
  double car_speed_target = car.speed_target; // target speed

  double safety_distance = predictions.get_safety_distance();

  // ------------------------------------computing the target speed---------------------------------
  // reasoning point by point (not end_point by end_point)
  if (car.emergency) {
    car_speed_target = car.speed; // current speed
  }   

  bool too_close = false; 
  int ref_vel_inc = 0; // -1 for max deceleration, 0 for constant speed, +1 for max acceleration

  double ref_vel_ms = mph_to_ms(car_speed_target); // target speed or current speed
  double closest_speed_ms = PARAM_MAX_SPEED; // PARAM_MAX_SPEED=22
  double closest_dist = INF;
  
  // find ref_v to use based on car in front of ego
  for (size_t i = 0; i < sensor_fusion.size(); i++) {
    // other vehicles in our lane
    float d = sensor_fusion[i][6]; // d value
    if (d > get_dleft(car.lane) && d < get_dright(car.lane)) { // self lane
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx*vx+vy*vy);
      double check_car_s = sensor_fusion[i][5]; // s value

      // cout<<"obj_idx: "<<i<<" "<<"ref_vel_ms: "<<ref_vel_ms<<" "<<"check_speed: "<<check_speed<<endl;

      if ((check_car_s > car.s) && ((check_car_s - car.s) < safety_distance)) {
        // lower reference velocity so we dont crash into the car infront of us
        too_close = true;
        double dist_to_check_car_s = check_car_s - car.s;
        if (dist_to_check_car_s < closest_dist) {
          closest_dist = dist_to_check_car_s;
          closest_speed_ms = check_speed;
        }
      }
      //  
    }
    //
  }
  
  if (too_close) { // true with acceleration
    if (ref_vel_ms > closest_speed_ms) { // [ms]
      car_speed_target -= PARAM_MAX_SPEED_INC_MPH; // [mph]
      if (closest_dist <= 10 && car_speed_target > closest_speed_ms) {
        car_speed_target -= 5 * PARAM_MAX_SPEED_INC_MPH;
      }
    }

    car_speed_target = max(car_speed_target, 0.0); // no backwards driving ... just in case
    ref_vel_inc = -1;
  } else if (car_speed_target < PARAM_MAX_SPEED_MPH) { // false with decelerate
    car_speed_target += PARAM_MAX_SPEED_INC_MPH;
    car_speed_target = min(car_speed_target, PARAM_MAX_SPEED_MPH);
    ref_vel_inc = 1;
  }

  // our nominal target to keep lane
  target.lane = car.lane;
  target.velocity = car_speed_target;
  // ------------------------------------computing the target speed---------------------------------
  //
  if (fabs(car.d - get_dcenter(car.lane)) <= 0.01) {
    target.time = 0.0; // as soon as possible and identified as emergency target
    target.velocity = ms_to_mph(closest_speed_ms);
    target.accel = 0.7 * PARAM_MAX_ACCEL;
    double car_speed_ms = mph_to_ms(car.speed);
    if (closest_speed_ms < car_speed_ms && closest_dist <= safety_distance)
      target.accel *= -1.0;
  }
  //
  targets_.push_back(target);

  // -----------------------------------------------------------------------------------
  // -----------------------------------------------------------------------------------
  // -----------------------------------------------------------------------------------

  // XXX temp just for testing purposes
  target.velocity = car_speed_target; // XXX TEMP just for testing
  target.time = 2.0;
  // XXX temp just for testing purposes

  // backup target lanes
  vector<int> backup_lanes;
  switch (car.lane) // current lane
  {
    case 2: // turn left
      backup_lanes.push_back(1);
      break;
    case 1: // turn left or right
      backup_lanes.push_back(2);
      backup_lanes.push_back(0);
      break;
    case 0: // turn right
      backup_lanes.push_back(1);
      break;
    default:
      assert(1 == 0); // something wrong
      break;
  }

  // backup target speeds and only lower speed so far
  vector<double> backup_vel;
  switch (ref_vel_inc) // -1, 0, +1
  {
    case 1:
      backup_vel.push_back(car_speed_target - PARAM_MAX_SPEED_INC_MPH);
      backup_vel.push_back(car_speed_target - 2 * PARAM_MAX_SPEED_INC_MPH);
      break;
    case 0:
      backup_vel.push_back(car_speed_target - PARAM_MAX_SPEED_INC_MPH); // already max speed
      break;
    case -1:
      backup_vel.push_back(car_speed_target - PARAM_MAX_SPEED_INC_MPH); // emergency breaking
      break;
    default:
      assert(1 == 0); // something wrong
      break;
  }

  // [1] backup velocities on target lane
  target.lane = car.lane;
  for (size_t i = 0; i < backup_vel.size(); i++) {
    target.velocity = backup_vel[i];
    targets_.push_back(target);
  }

  // [2] target velocity on backup lanes
  target.velocity = car_speed_target;
  for (size_t i = 0; i < backup_lanes.size(); i++) {
    target.lane = backup_lanes[i];
    targets_.push_back(target);
  }

  // [3] backup velocities on backup lanes
  for (size_t i = 0; i < backup_vel.size(); i++) {
    target.velocity = backup_vel[i];
    for (size_t j = 0; j < backup_lanes.size(); j++) {
      target.lane = backup_lanes[j];
      targets_.push_back(target);
    }
  }

  // last target for emergency trajectory (just in case we have no better choice)
  target.lane = car.lane;
  target.velocity = predictions.get_lane_speed(car.lane);
  target.time = 0.0; // identified as emergency target
  target.accel = -0.85 * PARAM_MAX_ACCEL;
  targets_.push_back(target);

  // cout<<"-----------------------------"<<endl;
  // cout<<targets_.size()<<endl;
}

Behavior::~Behavior() {}

vector<Target> Behavior::get_targets() {
  return targets_;
}
