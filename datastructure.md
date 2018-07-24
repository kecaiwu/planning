

* **PointC2**

```c++
struct PointC2 {
    double f;
    double f_dot;
    double f_ddot;
    PointC2 (double y=0, double y_dot=0, double y_ddot=0) :f(y), f_dot(y_dot), f_ddot(y_ddot) {}
};
```

* **TrajectoryXY**

```c++
struct TrajectoryXY {
    vector<double> x_vals;
    vector<double> y_vals;
    TrajectoryXY (vector<double> X={}, vector<double> Y={}) : x_vals(X), y_vals(Y) {}
};
```

* **TrajectorySD**

```c++
struct TrajectorySD {
    vector<PointC2> path_s;
	vector<PointC2> path_d;
    TrajectorySD (vector<PointC2> s={}, vector<PointC2> D={} : path_s(S), path_d(D)) {}
};
```

* **TrajectoryJMT**

```c++
struct TrajectoryJMT {
    TrajectoryXY trajectory;
    TrajectorySD path_sd;
};
```

* **PreviousPath**

```c++
struct PreviousPath {
    TrajectoryXY xy;
    TrajectorySD sd;
    int num_xy_reused;
    PreviousPath (TrajectoryXY XY={}, TrajectorySD SD={}, int N=0) : xy(XY), sd(SD), num_xy_reused(N) {}
};
```


* **Coord**

```c++
struct Coord {
    double x;
    double y;
};
```

* **CarData**

```c++
struct CarData {
    double x;
    double y;
    double s;
    double d;
    double yaw;
    double speed; // current speed
    double speed_target;
    int lane;
    bool emergency;
    CarData (double X=0, double Y=0, double S=0, double D=0, double YAW=0, double V=0, double VF=0, double L=0, double E=false) : x(X), y(Y), s(S), yaw(YAW), speed(V), speed_target(VF), lane(L), emergency(E) {}
};
```



* **Target**

```c++
struct Target {
    double lane;
    double velocity;
    double time; // 2.0
    double accel;
    Target(double l=0, double v=0, double t=0, double a=0) : lane(l), velocity(v), time(t), accel(a) {}
};
```



generate_trajectory_sd # 生成紧急的轨迹

generate_trajectory_jmt # 生成正常的轨迹

generate_trajectory # 插值



仿真器中读取的数据：

```c++
// 自车数据
car.x = j[1]["x"]
car.y = j[1]["y"]
car.s = j[1]["s"]
car.d = j[1]["d"]
car.yaw = j[1]["yaw"]
car.speed = j[1]["speed"]
// 车道线数据
vector<double> previous_path_x = j[1]["previous_path_x"]
vector<double> previous_path_y = j[1]["previous_path_y"]
double end_path_s = j[1]["end_path_s"]
double end_path_d = j[1]["end_path_d"]
// 他车数据
vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"]
double id = sensor_fusion[i][0]
double x = sensor_fusion[i][1]
double y = sensor_fusion[i][2]
double vx = sensor_fusion[i][3]
double vy = sensor_fusion[i][4]
double s = sensor_fusion[i][5]
double d = sensor_fusion[i][6]
```























