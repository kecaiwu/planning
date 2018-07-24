#include "trajectory.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

TrajectoryJMT JMT_init(double car_s, double car_d)
{
  TrajectoryJMT traj_jmt; // xy and sd
  vector<PointC2> store_path_s(PARAM_NB_POINTS, PointC2(car_s, 0, 0)); // initialize the store_path_s vector
  vector<PointC2> store_path_d(PARAM_NB_POINTS, PointC2(car_d, 0, 0)); // initialize the store_path_d vector

  traj_jmt.path_sd.path_s = store_path_s;
  traj_jmt.path_sd.path_d = store_path_d;

  return traj_jmt;
}


Trajectory::Trajectory(std::vector<Target> targets, Map &map, CarData &car, PreviousPath &previous_path, Predictions &predictions)
{
  for (size_t i = 0; i < targets.size(); i++) {
    // generate trajectories for each target
    TrajectoryXY trajectory;
    if (PARAM_TRAJECTORY_JMT) { // true
      TrajectoryJMT traj_jmt;
      if (targets[i].time == 0) {
        traj_jmt = generate_trajectory_sd(targets[i], map, car, previous_path); // EMERGENCY
      } else {
        traj_jmt = generate_trajectory_jmt(targets[i], map, previous_path); // Normal JMT
      }
      trajectory = traj_jmt.trajectory;
      trajectories_sd_.push_back(traj_jmt.path_sd);
    } else { // false
      // generate spline trajectory in (x,y)
      trajectory = generate_trajectory(targets[i], map, car, previous_path);
    }
  
    // compute cost for each trajectory
    Cost cost = Cost(trajectory, targets[i], predictions, car.lane);

    costs_.push_back(cost);
    trajectories_.push_back(trajectory);
  }
  
  // find the lowest cost trajectory with min_cost_ and min_cost_index_
  min_cost_ = INF;
  min_cost_index_ = 0;
  for (size_t i = 0; i < costs_.size(); i++) {
    if (costs_[i].get_cost() < min_cost_) {
      min_cost_ = costs_[i].get_cost();
      min_cost_index_ = i;
    }
  }

  // enforce emergency trajectory, choose the last one, in case of unavoidable collision we prefer lower speed
  if (min_cost_ >= PARAM_COST_FEASIBILITY) { // PARAM_COST_FEASIBILITY=10000
    min_cost_index_ = costs_.size() - 1;
    min_cost_ = costs_[min_cost_index_].get_cost();
  }
  // check the emergency trajectory
  if (targets[min_cost_index_].time == 0) {
    car.emergency = true;
    cout << "!!!!!!!!!!!!!!!!!!!! EMERGENCY !!!!!!!!!!!!!!!!!!!!"<<endl;
  } else {
    car.emergency = false;
  }
}
  

vector<double> Trajectory::JMT(vector< double> start, vector <double> end, double T)
{
  /* 
      calculate the Jerk Minimizing Trajectory taht connects the initial and final state in T 

      INPUT:
      start: [location, velocity, acceleration]
      end: [location, velocity, acceleration]
      T: the duration, in seconds, over which this maneuver should occur

      OUTPUT:
      an array of length 6, each value corresponding to a coefficent in the quintic polynomial
  */
  MatrixXd A(3,3);
  VectorXd b(3);
  VectorXd x(3);
  A << pow(T,3), pow(T,4), pow(T,5),
       3*pow(T,2), 4*pow(T,3), 5*pow(T,4),
       6*T, 12*pow(T,2), 20*pow(T,3);
  b << end[0] - (start[0] + start[1]*T + 0.5*start[2]*T*T), 
       end[1] - (start[1] + start[2]*T), 
       end[2] - start[2];
  x = A.inverse() * b;
  return {start[0], start[1], start[2]/2, x[0], x[1], x[2]};
}

// normal polynomial
double Trajectory::polyeval(vector<double> c, double t) {
  double res = 0.0;
  for (size_t i = 0; i < c.size(); i++) {
    res += c[i] * pow(t, i);
  }
  return res;
}

// 1st derivative of a polynomial
double Trajectory::polyeval_dot(vector<double> c, double t) {
  double res = 0.0;
  for (size_t i = 1; i < c.size(); ++i) {
    res += i * c[i] * pow(t, i-1);
  }
  return res;
}

// 2nd derivative of a polynomial
double Trajectory::polyeval_ddot(vector<double> c, double t) {
  double res = 0.0;
  for (size_t i = 2; i < c.size(); ++i) {
    res += i * (i-1) * c[i] * pow(t, i-2);
  }
  return res;
}



TrajectoryJMT Trajectory::generate_trajectory_jmt(Target target, Map &map, PreviousPath const &previous_path)
{
  // generate trajectory in normal situation
  TrajectoryJMT traj_jmt;

  // -------------- previous path info --------------
  TrajectoryXY previous_path_xy = previous_path.xy; // struct TrajectoryXY
  int prev_size = previous_path.num_xy_reused;
  TrajectorySD prev_path_sd = previous_path.sd; // struct TrajectorySD

  vector<double> previous_path_x = previous_path_xy.x_vals;
  vector<double> previous_path_y = previous_path_xy.y_vals;
  vector<PointC2> prev_path_s = prev_path_sd.path_s;
  vector<PointC2> prev_path_d = prev_path_sd.path_d;
  // -------------- previous path info --------------

  vector<PointC2> new_path_s(PARAM_NB_POINTS, PointC2(0,0,0)); // 50
  vector<PointC2> new_path_d(PARAM_NB_POINTS, PointC2(0,0,0)); // 50

  int last_point;
  if (PARAM_PREV_PATH_XY_REUSED < PARAM_NB_POINTS) { // 5<50
    last_point = PARAM_NB_POINTS - previous_path_x.size() + prev_size - 1;
  } else {
    last_point = PARAM_NB_POINTS - 1;
  }

  double T = target.time; // 2 seconds si car_d center of line

  // initial configure parameters
  double si, si_dot, si_ddot;
  double di, di_dot, di_ddot;

  si      = prev_path_s[last_point].f;
  si_dot  = prev_path_s[last_point].f_dot;
  si_ddot = prev_path_s[last_point].f_ddot;

  di      = prev_path_d[last_point].f;
  di_dot  = prev_path_d[last_point].f_dot;
  di_ddot = prev_path_d[last_point].f_ddot;

  // final configure parameters
  double sf, sf_dot, sf_ddot;
  double df, df_dot, df_ddot;

  if (target.velocity <= 10) { // mph // ???
    // lateral of final point
    df_ddot =  0;
    df_dot  =  0;
    df      = di;
    // longitudinal of final point
    sf_ddot = 0;
    sf_dot  = mph_to_ms(target.velocity);

    // function is uncertainly
    // sf_dot = min(sf_dot, si_dot + 10 * PARAM_MAX_SPEED_INC); // 10*0.2
    // sf_dot = max(sf_dot, si_dot - 10 * PARAM_MAX_SPEED_INC); // 10*0.2

    sf      = si + 2 * sf_dot * T;
  } else {
    // lateral of final point
    df_ddot = 0;
    df_dot  = 0;
    df      = get_dcenter(target.lane);
    // longitudinal of final point
    sf_ddot = 0;
    sf_dot  = mph_to_ms(target.velocity);
    // use JMT for lane changes only, and no need to reach max speed during lane changes
    sf_dot  = min(sf_dot, 0.9 * PARAM_MAX_SPEED); // 0.9*22=19.8

    // function is uncertainly
    // sf_dot = min(sf_dot, si_dot + 10 * PARAM_MAX_SPEED_INC);
    // sf_dot = max(sf_dot, si_dot - 10 * PARAM_MAX_SPEED_INC);

    sf      = si + sf_dot * T;
  }

  vector<double> start_s = {si, si_dot, si_ddot};
  vector<double> end_s   = {sf, sf_dot, sf_ddot};

  vector<double> start_d = {di, di_dot, di_ddot};
  vector<double> end_d   = {df, df_dot, df_ddot};

  /////////////////////////////////////////////////////////////

  vector<double> poly_s = JMT(start_s, end_s, T);
  vector<double> poly_d = JMT(start_d, end_d, T);

  vector<double> next_x_vals;
  vector<double> next_y_vals;
  
  // reused the previous path
  for (int i = 0; i < prev_size; i++) {
    new_path_s[i] = prev_path_s[PARAM_NB_POINTS - previous_path_x.size() + i];
    new_path_d[i] = prev_path_d[PARAM_NB_POINTS - previous_path_x.size() + i];

    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  double t = PARAM_DT; // 0.02
  for (int i = prev_size; i < PARAM_NB_POINTS; i++) {
    double s = polyeval(poly_s, t);
    double s_dot = polyeval_dot(poly_s, t);
    double s_ddot = polyeval_ddot(poly_s, t);

    double d = polyeval(poly_d, t);
    double d_dot = polyeval_dot(poly_d, t);
    double d_ddot = polyeval_ddot(poly_d, t);

    new_path_s[i] = PointC2(s, s_dot, s_ddot);
    new_path_d[i] = PointC2(d, d_dot, d_ddot);

    vector<double> point_xy = map.getXYspline(s, d);

    next_x_vals.push_back(point_xy[0]);
    next_y_vals.push_back(point_xy[1]);

    t += PARAM_DT;
  }

  traj_jmt.trajectory = TrajectoryXY(next_x_vals, next_y_vals);
  traj_jmt.path_sd = TrajectorySD(new_path_s, new_path_d);

  return traj_jmt;
}

TrajectoryJMT Trajectory::generate_trajectory_sd(Target target, Map &map, CarData const &car, PreviousPath const &previous_path)
{
  /*
    without lane change and with constant accel or decel in next 2 seconds, not JMT here
  */
  // generate trajectory in emergency situation
  TrajectoryJMT traj_jmt;

  // -------------- previous path info --------------
  TrajectoryXY previous_path_xy = previous_path.xy;
  int prev_size = previous_path.num_xy_reused;
  TrajectorySD prev_path_sd = previous_path.sd;

  vector<double> previous_path_x = previous_path_xy.x_vals;
  vector<double> previous_path_y = previous_path_xy.y_vals;
  vector<PointC2> prev_path_s = prev_path_sd.path_s;
  vector<PointC2> prev_path_d = prev_path_sd.path_d;
  // -------------- previous path info --------------

  vector<PointC2> new_path_s(PARAM_NB_POINTS, PointC2(0,0,0));
  vector<PointC2> new_path_d(PARAM_NB_POINTS, PointC2(0,0,0));

  vector<double> next_x_vals;
  vector<double> next_y_vals;

  double target_velocity_ms = mph_to_ms(target.velocity);

  double s, s_dot, s_ddot;
  double d, d_dot, d_ddot;
  if (prev_size > 0) {
    for (int i = 0; i < prev_size; i++) {
      new_path_s[i] = prev_path_s[PARAM_NB_POINTS - previous_path_x.size() + i];
      new_path_d[i] = prev_path_d[PARAM_NB_POINTS - previous_path_x.size() + i];

      next_x_vals.push_back(previous_path_x[i]);
      next_y_vals.push_back(previous_path_y[i]);
    }

    // initial configure for lateral and longitudinal
    s      = new_path_s[prev_size-1].f;
    s_dot  = new_path_s[prev_size-1].f_dot;

    d      = new_path_d[prev_size-1].f;
    d_dot  = 0;
    d_ddot = 0;
  } else { // 0
    // initial configure for lateral and longitudinal
    s      = car.s;
    s_dot  = car.speed;

    d      = car.d;
    d_dot  = 0;
    d_ddot = 0;
  }

  s_ddot = target.accel;  // deceleration

  double t = PARAM_DT; // 0.02
  double prev_s_dot = s_dot;
  for (int i = prev_size; i < PARAM_NB_POINTS; i++) {
    // accelerate or decelerate till target velocity is reached
    s_dot += s_ddot * PARAM_DT; 
    if ((target.accel > 0 && prev_s_dot <= target_velocity_ms && s_dot > target_velocity_ms)
       ||
       (target.accel < 0 && prev_s_dot >= target_velocity_ms && s_dot < target_velocity_ms)) 
    {
      s_dot = target_velocity_ms;
    }
    //
    s_dot = max(min(s_dot, 0.9 * PARAM_MAX_SPEED), 0.0);
    s += s_dot * PARAM_DT;

    prev_s_dot = s_dot;

    new_path_s[i] = PointC2(s, s_dot, s_ddot);
    new_path_d[i] = PointC2(d, d_dot, d_ddot);

    vector<double> point_xy = map.getXYspline(s, d);

    next_x_vals.push_back(point_xy[0]);
    next_y_vals.push_back(point_xy[1]);

    t += PARAM_DT;
  }

  traj_jmt.trajectory = TrajectoryXY(next_x_vals, next_y_vals);
  traj_jmt.path_sd = TrajectorySD(new_path_s, new_path_d);

  return traj_jmt;
}


TrajectoryXY Trajectory::generate_trajectory(Target target, Map &map, CarData const &car, PreviousPath const &previous_path)
{
  // generate trajectory with spline function
  TrajectoryXY previous_path_xy = previous_path.xy;
  int prev_size = previous_path.num_xy_reused;

  vector<double> previous_path_x = previous_path_xy.x_vals;
  vector<double> previous_path_y = previous_path_xy.y_vals;

  vector<double> ptsx;
  vector<double> ptsy;
  
  double ref_x = car.x;
  double ref_y = car.y;
  double ref_yaw = deg2rad(car.yaw);

  if (prev_size < 2) {
    double prev_car_x = car.x - cos(car.yaw);
    double prev_car_y = car.y - sin(car.yaw);
  
    ptsx.push_back(prev_car_x);
    ptsx.push_back(car.x);
  
    ptsy.push_back(prev_car_y);
    ptsy.push_back(car.y);
  } else {
    ref_x = previous_path_x[prev_size-1];
    ref_y = previous_path_y[prev_size-1];
  
    double ref_x_prev = previous_path_x[prev_size-2];
    double ref_y_prev = previous_path_y[prev_size-2];
    ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
  
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);
  
    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  vector<double> next_wp0 = map.getXYspline(car.s+30, get_dcenter(target.lane));
  vector<double> next_wp1 = map.getXYspline(car.s+60, get_dcenter(target.lane));
  vector<double> next_wp2 = map.getXYspline(car.s+90, get_dcenter(target.lane));
  
  
  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);
  
  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);
  
  
  for (int i = 0; i < ptsx.size(); i++) {
    // shift car reference angle to 0 degrees
    // transformation to local car's coordinates (cf MPC)
    // last point of previous path at origin and its angle at zero degree
  
    // shift and rotation
    double shift_x = ptsx[i]-ref_x;
    double shift_y = ptsy[i]-ref_y;
  
    ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0 - ref_yaw));
    ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0 - ref_yaw));
  }
  
  
  tk::spline spl;
  spl.set_points(ptsx, ptsy);
  
  vector<double> next_x_vals;
  vector<double> next_y_vals;
  
  for (int i = 0; i < prev_size; i++) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }
  
  // Calculate how to break up spline points so that we travel at our desired reference velocity
  double target_x = 30.0;
  double target_y = spl(target_x);
  double target_dist = sqrt(target_x*target_x + target_y*target_y);
  
  double x_add_on = 0;
  
  // fill up the rest of our path planner after filing it with previous points
  // here we will always output 50 points
  for (int i = 1; i <= PARAM_NB_POINTS - prev_size; i++) {
    double N = (target_dist / (PARAM_DT * mph_to_ms(target.velocity))); // divide by 2.24: mph -> m/s
    double x_point = x_add_on + target_x/N;
    double y_point = spl(x_point);
  
    x_add_on = x_point;
  
    double x_ref = x_point; // x_ref IS NOT ref_x !!!
    double y_ref = y_point;
  
    // rotate back to normal after rotating it earlier
    x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
    y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
  
    x_point += ref_x;
    y_point += ref_y;
  
    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }

  //return { next_x_vals, next_y_vals };
  return TrajectoryXY(next_x_vals, next_y_vals);
}
