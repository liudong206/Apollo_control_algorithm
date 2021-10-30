#ifndef TRAJECTORY_H
#define TRAJECTORY_H
#include "common/base_types.h"
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include "vehicle_info/vehicle_params.h"
#include "common/debug.h"
#define pi 3.1415926
enum class TrajrefType
{
    SinWave,
    BigCircle,
    LineStright
};
struct TrajrefParams{
    double dist_interval = 0.2;
    double traj1_dist = 60;
    double r2 = 10;
    double traj3_dist = 30;
    double r4 = 10;
    double r5 = 10;
    double traj6_dist = 30;
};

class Trajectory{
public:
    Trajectory();
    ~Trajectory();
    double angle_mod(double angle_origin);
    bool generate_trajref(const TrajrefType tra_type,const double &linear_v,const TrajrefParams &trajref_params,
                          std::vector<TrajectoryPoint> &planning_trajectory);
};
#endif // TRAJECTORY_H
