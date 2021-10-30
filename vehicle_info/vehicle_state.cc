#include "vehicle_info/vehicle_state.h"
#include <iostream>
#include <cmath>
VehicleState::VehicleState(double x , double y ,
                           double heading , double v){
    x_ = x;
    y_ = y;
    heading_ = heading;
    linear_v_ = v;
}

double VehicleState::x() const { return x_; }

double VehicleState::y() const { return y_; }

double VehicleState::z() const { return z_; }

double VehicleState::heading() const { return heading_; }

double VehicleState::pitch() const { return pitch_; };

double VehicleState::roll() const { return roll_; };

double VehicleState::yaw() const { return yaw_; };

double VehicleState::linear_velocity() const { return linear_v_; }

double VehicleState::angular_velocity() const { return angular_v_; }

double VehicleState::linear_acceleration() const { return linear_a_; }

int VehicleState::direction() const { return direction_; }

void VehicleState::set_x(const double x) { x_ = x; }

void VehicleState::set_y(const double y) { y_ = y; }

void VehicleState::set_z(const double z) { z_ = z; }

void VehicleState::set_pitch(const double pitch) { pitch_ = pitch; }

void VehicleState::set_roll(const double roll) { roll_ = roll; };

void VehicleState::set_yaw(const double yaw) {yaw_ = yaw;};

void VehicleState::set_heading(const double heading) { heading_ = heading; }
void VehicleState::set_linear_velocity(const double linear_velocity) {
    linear_v_ = linear_velocity;
}

void VehicleState::set_angular_velocity(const double angular_velocity) {
    angular_v_ = angular_velocity;
}

void VehicleState::update(VehicleState &veh_sta,double acc,double delta,
                          double t_,double Wheelbase){
    double velocity = veh_sta.linear_velocity();
    double delta_dist = velocity *t_;
    double tol = 0.001;//Steering threshold
    double update_state_x = 0;
    double update_state_y = 0;
    double update_state_heading = 0;
    double update_state_angular_v = 0;
    double update_state_line_v = velocity + acc * t_;
    //航迹推算
    //method 1
    if(fabs(delta) > tol){//turn left&right   turnleft = +
        double radius = Wheelbase / tan(delta);
        double center_x = veh_sta.x() - radius * std::sin(veh_sta.heading());
        double center_y = veh_sta.y() + radius * std::cos(veh_sta.heading());

        double theta = veh_sta.heading() + delta_dist / radius;

        update_state_x = center_x + radius * std::sin(theta);
        update_state_y = center_y - radius * std::cos(theta);
        update_state_heading = theta;
        update_state_angular_v = delta_dist / radius / t_;
    }else {//stright
        update_state_x = veh_sta.x() + delta_dist * std::cos(veh_sta.heading());
        update_state_y = veh_sta.y() + delta_dist * std::sin(veh_sta.heading());
        update_state_heading = veh_sta.heading();
    }

    veh_sta.set_x(update_state_x);
    veh_sta.set_y(update_state_y);
    veh_sta.set_heading(update_state_heading);
    veh_sta.set_angular_velocity(update_state_angular_v);
    veh_sta.set_linear_velocity(update_state_line_v);

}

