#include "predictions.h"


using namespace std;


Predictions::Predictions(vector<vector<double>> const &sensor_fusion, CarData const &car, int horizon)
{
  /*
    Input: sensor_fusion, car, and horizon
  */
  // finding closest other vehicles in each lane that including front and back vehicle
  vector<int> closest_objects = find_closest_objects(sensor_fusion, car); 
  // predict other vehicles location in the future 1 second with uniform speed
  for (int i = 0; i < closest_objects.size(); i++) {
    int fusion_index = closest_objects[i];
    if (fusion_index >= 0) {
      double x = sensor_fusion[fusion_index][1];
      double y = sensor_fusion[fusion_index][2];
      double vx = sensor_fusion[fusion_index][3];
      double vy = sensor_fusion[fusion_index][4];
      vector<Coord> prediction;
      for (int j = 0; j < horizon; j++) {
        Coord coord; // (x,y)
        // uniform speed of other vehicles
        coord.x = x + vx * j*PARAM_DT;
        coord.y = y + vy * j*PARAM_DT;
        prediction.push_back(coord);
      }
      predictions_[fusion_index] = prediction;
    }
  }
  //
  set_safety_distances(sensor_fusion, car); // self car in all lanes
  set_lane_info(sensor_fusion, car);
}

Predictions::~Predictions() {}


//double get_sdistance(double s1, double s2)
//{
//  // account for s wraparound at MAX_S
//  double sdistance = min( fabs(s1 - s2), min(fabs((s1+MAX_S) - s2), fabs(s1 - (s2+MAX_S))) );
//  return sdistance;
//}


double get_sensor_fusion_vel(vector<vector<double>> const &sensor_fusion, int idx, double default_vel)
{
  /*
    Getting the velocity in given index other vehicles
    Input: sensor_fusion, other vehicles index, and default velocity
    Output: velocity of the index velocity
  */
  double vx, vy, vel;
  if (idx >= 0 && idx < sensor_fusion.size()) {
    vx = sensor_fusion[idx][3];
    vy = sensor_fusion[idx][4];
    vel = sqrt(vx*vx+vy*vy);
  } else {
    vel = default_vel;
  }
  return vel;
}

double Predictions::get_safety_distance(double vel_back, double vel_front, double time_latency)
{
  /*
    compute the sefety distance two vehicles that one back and one front
    Input: vel_back, vel_front, and time_latency
    Output: safety distance given velocity of two vehicles
  */
  double safety_distance = PARAM_SD_LC;

  if (vel_back > vel_front) {
      double time_to_decelerate = (vel_back - vel_front) / decel_ + time_latency;
      safety_distance = vel_back * time_to_decelerate + 1.5 * PARAM_CAR_SAFETY_L;
  }
  safety_distance = max(safety_distance, PARAM_SD_LC);  // conservative
  return safety_distance;
}

void Predictions::set_safety_distances(vector<vector<double>> const &sensor_fusion, CarData const &car)
{
  // the stop time from current speed to 0
  vel_ego_ = mph_to_ms(car.speed);
  decel_ = 0.8 * PARAM_MAX_ACCEL; // slightly conservative
  time_to_stop_ = vel_ego_ / decel_;

  // the velocity and distance of front vehicle in self lane
  vel_front_ = get_sensor_fusion_vel(sensor_fusion, front_[car.lane], PARAM_MAX_SPEED);
  dist_front_ = front_dmin_[car.lane];

  if (vel_ego_ > vel_front_) {
    time_to_collision_ = dist_front_ / (vel_ego_ - vel_front_);
    time_to_decelerate_ = (vel_ego_ - vel_front_) / decel_;
    safety_distance_ = vel_ego_ * time_to_decelerate_ + 1.75 * PARAM_CAR_SAFETY_L;
  } else {
    time_to_collision_ = INF;
    time_to_decelerate_ = 0;
    safety_distance_ = 1.75 * PARAM_CAR_SAFETY_L; 
  }

  paranoid_safety_distance_ = vel_ego_ * time_to_stop_ + 2 * PARAM_CAR_SAFETY_L;

  // the safety distance in all lanes
  for (int i = 0; i < PARAM_NB_LANES; i++) {
    // front vehicle infos in all lanes
    front_velocity_[i] = get_sensor_fusion_vel(sensor_fusion, front_[i], PARAM_MAX_SPEED);
    front_safety_distance_[i] = get_safety_distance(vel_ego_, front_velocity_[i], 0.0);
    // back vehicle infos in all lanes
    back_velocity_[i] = get_sensor_fusion_vel(sensor_fusion, back_[i], 0);
    back_safety_distance_[i] = get_safety_distance(back_velocity_[i], vel_ego_, 2.0);
  }
}


void Predictions::set_lane_info(vector<vector<double>> const &sensor_fusion, CarData const &car)
{
  /*
    Input: sensor_fusion and car infos
  */
  int car_lane = get_lane(car.d);
  // other vehicles infos in all lanes
  for (size_t i = 0; i < front_.size(); i++) {
    int lane = i;
    // !!! This should be part of the behavior planner behavior.cpp
    if (front_[i] >= 0) {
      if (lane != car_lane && (back_dmin_[i] <= back_safety_distance_[i] || front_dmin_[i] <= front_safety_distance_[i])) {
        lane_speed_[i] = 0;
        lane_free_space_[i] = 0; // too dangerous
      } else {
        double vx = sensor_fusion[front_[i]][3];
        double vy = sensor_fusion[front_[i]][4];
        lane_speed_[i] = sqrt(vx*vx+vy*vy);
        lane_free_space_[i] = front_dmin_[i];
      }
    } else {
      if (lane != car_lane && back_dmin_[i] <= back_safety_distance_[i]) {
        lane_speed_[i] = 0;
        lane_free_space_[i] = 0; // too dangerous
      } else {
        lane_speed_[i] = PARAM_MAX_SPEED_MPH;
        lane_free_space_[i] = PARAM_FOV;
      }
    }
  }
}


vector<int> Predictions::find_closest_objects(vector<vector<double>> const &sensor_fusion, CarData const &car) {
  /*
    Finding closest other vehicles in each lane that including the front and back vehicles
    Input: sensor_fusion and car infos
    Output: the distance of closest other vehicles in each lane

    Only finding other vehicles from [sfov_min, sfov_max] range    
  */
  double sfov_min = car.s - PARAM_FOV;
  double sfov_max = car.s + PARAM_FOV;
  double sfov_shit = 0;

  if (sfov_min < 0) {
    sfov_shit = -sfov_min;
  } else if (sfov_max > MAX_S) {
    sfov_shit = MAX_S - sfov_max;
  }

  sfov_min += sfov_shit;
  sfov_max += sfov_shit;
  assert(sfov_min >= 0 && sfov_min <= MAX_S);
  assert(sfov_max >= 0 && sfov_max <= MAX_S);

  double car_s = car.s;
  car_s += sfov_shit;

  /*
  cout<<"00000000000000000000000"<<endl;
  cout<<"sfov_shit: "<<sfov_shit<<endl;
  cout<<"sfov_min: "<<sfov_min<<endl;
  cout<<"sfov_max: "<<sfov_max<<endl;
  cout<<"00000000000000000000000"<<endl;
  */

  for (size_t i = 0; i < sensor_fusion.size(); i++) {
    double s = sensor_fusion[i][5] + sfov_shit;
    // objects in FOV wraparound
    if (s >= sfov_min && s <= sfov_max) {
      double d = sensor_fusion[i][6];
      // check other vehicle in which lane?
      int lane = get_lane(d);
      // discard some garbage values
      if (lane < 0 || lane > 2)
        continue;
      
      double dist = fabs(s - car_s);
      // front_dmin_[lane] for front vehicles and back_dmin_[lane] for back vehicles
      if (s >= car_s) {
        if (dist < front_dmin_[lane]) {
          front_[lane] = i;
          front_dmin_[lane] = dist;
        }
      } else {
        if (dist < back_dmin_[lane]) {
          back_[lane] = i;
          back_dmin_[lane] = dist;
        }
      }
    }
  }

  return { front_[0], back_[0], front_[1], back_[1], front_[2], back_[2] }; // lane order
}

double Predictions::get_lane_speed(int lane) const {
  if (lane >= 0 && lane <= 3) {
    return lane_speed_[lane];
  } else {
    return 0;
  }
}

double Predictions::get_lane_free_space(int lane) const {
  if (lane >= 0 && lane <= 3) {
    return lane_free_space_[lane];
  } else {
    return 0;
  }
}
