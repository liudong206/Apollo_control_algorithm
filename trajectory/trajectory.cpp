#include "trajectory.h"

Trajectory::Trajectory(){};
Trajectory::~Trajectory(){};

double Trajectory::angle_mod(double angle_origin){
    double angle_mod;
    double angle_temp = fmod(angle_origin,2*pi);
    if(angle_temp < 0){
        angle_mod = angle_temp + 2*pi;
    }else{
        angle_mod = angle_temp;
    }
    return angle_mod;
}
bool Trajectory::generate_trajref(const TrajrefType tra_type,const double &linear_v,const TrajrefParams &trajref_params,
                                  std::vector<TrajectoryPoint> &planning_trajectory){
    switch (tra_type) {
        case TrajrefType::SinWave:{
            TrajectoryPoint tra;
            TrajectoryPoint tra_1;
            double distance = 0;
            double dist_interval = trajref_params.dist_interval;
            std::fstream tra_x("log/tra_x.xls",std::ios::app);
            std::fstream tra_y("log/tra_y.xls",std::ios::app);
            for (double i = 0;distance <= 180; i = i + dist_interval) {
                tra.x = i;
                tra.y = 20*std::sin(0.02*i);
                double y_derivative = 0.4 * std::cos(0.02 * i);//First derivative
                double yy_derivative = -0.008 * std::sin(0.02 * i);//Second derivative
                tra.kappa = yy_derivative / std::pow((1 + std::pow(y_derivative,2)),1.5);
                tra.v = linear_v;//mps

                tra_1.x = i + dist_interval;
                tra_1.y = 20*std::sin(0.02*(i + dist_interval));
                tra.theta = std::atan2((tra_1.y - tra.y),(tra_1.x - tra.x));
                tra.theta = angle_mod(tra.theta);
                double point_distance = std::sqrt(std::pow((tra_1.y - tra.y),2) + std::pow((tra_1.x - tra.x),2));
                distance = distance + point_distance;
                planning_trajectory.push_back(tra);
                if (!tra_x.fail()&&!tra_y.fail()){
                tra_x << tra.x<<std::endl;
                tra_y << tra.y<<std::endl;
                }else {
                  std::cout<<"打开文件失败"<<std::endl;
                }
                tra_x.close();
                tra_y.close();

            }
            break;
        }
        case TrajrefType::BigCircle:{
            double dist_interval = trajref_params.dist_interval;
            double default_r = 10000;
            TrajectoryPoint traj_point1 , traj_point2 , traj_point3,
                    traj_point4 , traj_point5 , traj_point6;
            std::vector<TrajectoryPoint> traj1;
            for (double i = 0;i <= trajref_params.traj1_dist;i = i + dist_interval) {
                traj_point1.x = i;
                traj_point1.y = 0;
                traj_point1.theta = 0;
                traj_point1.kappa = 1 / default_r;
                traj_point1.v = linear_v;
                traj1.push_back(traj_point1);
            }
            std::vector<TrajectoryPoint> traj2;
            double theta_interval_2 = dist_interval / trajref_params.r2;
            for (double theta_2 = -pi/2 + theta_interval_2 ; theta_2 <= pi/2 ; theta_2 = theta_2 + theta_interval_2) {
                traj_point2.x = trajref_params.r2 * std::cos(theta_2) + traj1.back().x;
                traj_point2.y = trajref_params.r2 * std::sin(theta_2) + traj1.back().y + trajref_params.r2;
                traj_point2.theta = 0;
                traj_point2.kappa = 1 / trajref_params.r2;
                traj_point2.v = linear_v;
                traj2.push_back(traj_point2);
            }
            std::vector<TrajectoryPoint> traj3;
            for (double i = -dist_interval ; i >= -trajref_params.traj3_dist ; i = i - dist_interval) {
                traj_point3.x = traj2.back().x + i;
                traj_point3.y = traj2.back().y;
                traj_point3.theta = 0;
                traj_point3.kappa = 1 / default_r;
                traj_point3.v = linear_v;
                traj3.push_back(traj_point3);
            }
            std::vector<TrajectoryPoint> traj4;
            double theta_interval_4 = dist_interval / trajref_params.r4;
            for (double theta_4 = 3*pi / 2 - theta_interval_4 ; theta_4 >= pi/2 ; theta_4 = theta_4 - theta_interval_4) {
                traj_point4.x = trajref_params.r4 * std::cos(theta_4) + traj3.back().x;
                traj_point4.y = trajref_params.r4 * std::sin(theta_4) + traj3.back().y + trajref_params.r4;
                traj_point4.theta = 0;
                traj_point4.kappa = 1 / (-trajref_params.r4);
                traj_point4.v = linear_v;
                traj4.push_back(traj_point4);
            }
            std::vector<TrajectoryPoint> traj5;
            double theta_interval_5 = dist_interval / trajref_params.r5;
            for (double theta_5 = -pi / 2 + theta_interval_5 ; theta_5 <= 0 ; theta_5 = theta_5 + theta_interval_5) {
                traj_point5.x = trajref_params.r5 * std::cos(theta_5) + traj4.back().x;
                traj_point5.y = trajref_params.r5 * std::sin(theta_5) + traj4.back().y + trajref_params.r5;
                traj_point5.theta = 0;
                traj_point5.kappa = 1 / trajref_params.r5;
                traj_point5.v = linear_v;
                traj5.push_back(traj_point5);
            }
            std::vector<TrajectoryPoint> traj6;
            for (double i = dist_interval ; i <= trajref_params.traj6_dist ; i = i + dist_interval) {
                traj_point6.x = traj5.back().x;
                traj_point6.y = traj5.back().y + i;
                traj_point6.theta = 0;
                traj_point6.kappa = 1 / default_r;
                traj_point6.v = linear_v;
                traj6.push_back(traj_point6);
            }
            planning_trajectory.insert(planning_trajectory.end(),traj1.begin(),traj1.end());
            planning_trajectory.insert(planning_trajectory.end(),traj2.begin(),traj2.end());
            planning_trajectory.insert(planning_trajectory.end(),traj3.begin(),traj3.end());
            planning_trajectory.insert(planning_trajectory.end(),traj4.begin(),traj4.end());
            planning_trajectory.insert(planning_trajectory.end(),traj5.begin(),traj5.end());
            planning_trajectory.insert(planning_trajectory.end(),traj6.begin(),traj6.end());
            std::vector<TrajectoryPoint>::iterator iter;
            for (iter = planning_trajectory.begin() + 1 ; iter != planning_trajectory.end() ; iter++) {
                (*iter).theta = std::atan2(((*(iter)).y - (*(iter - 1)).y),((*(iter)).x - (*(iter - 1)).x));
                (*iter).theta = angle_mod((*iter).theta);
            }
            std::cout<<"Successfully generated track"<<std::endl;

            std::fstream tra_print_x("log/tra_print_x.xls",std::ios::app);
            std::fstream tra_print_y("log/tra_print_y.xls",std::ios::app);
            std::fstream tra_print_theta("log/tra_print_theta.xls",std::ios::app);
            std::fstream tra_print_kappa("log/tra_print_kappa.xls",std::ios::app);
            for (iter = planning_trajectory.begin() ; iter != planning_trajectory.end() ; iter++) {
                if (!tra_print_x.fail() && !tra_print_y.fail() &&
                        !tra_print_kappa.fail() && !tra_print_theta.fail()){
                    tra_print_x<<(*iter).x<<std::endl;
                    tra_print_y<<(*iter).y<<std::endl;
                    tra_print_theta<<(*iter).theta<<std::endl;
                    tra_print_kappa<<(*iter).kappa<<std::endl;
                }else {
                    std::cout<<"打开文件失败"<<std::endl;
                }
            }
            tra_print_x.close();
            tra_print_y.close();
            tra_print_theta.close();
            tra_print_kappa.close();

            break;
        }
        case TrajrefType::LineStright:{
            TrajectoryPoint tra;
            double dist_interval = trajref_params.dist_interval;
            std::fstream tra_x("log/tra_x.xls",std::ios::app);
            std::fstream tra_y("log/tra_y.xls",std::ios::app);
            for (double i = 0;i <= 180; i = i + dist_interval) {
                tra.x = i;
                tra.y = 0;
                tra.kappa = 1 / 10000;
                tra.v = linear_v;//mps
                tra.theta = 0;
                planning_trajectory.push_back(tra);
                if (!tra_x.fail() && !tra_y.fail()){
                    tra_x << tra.x<<std::endl;
                    tra_y << tra.y<<std::endl;
                }else {
                    std::cout<<"打开文件失败"<<std::endl;
                }
            }
            tra_x.close();
            tra_y.close();
            break;
        }
        }


    return true;
}
