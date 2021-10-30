#include "vehicle_info/vehicle_params.h"

VehicleParams::VehicleParams() {

   init(VehicleType::Q_EV);
}

VehicleParams::~VehicleParams(){
}

VehicleType VehicleParams::type() const { return vehicle_type_;}

double VehicleParams::length() const { return length_; }

double VehicleParams::width() const { return width_; }

double VehicleParams::Wheelbase() const { return wheelbase_; }

double VehicleParams::front_suspension_length() const { return front_suspension_length_; }

double VehicleParams::rear_suspension_length() const { return rear_suspension_length_; }

double VehicleParams::front_axle_length() const { return front_axle_length_; }

double VehicleParams::rear_axle_length() const { return rear_axle_length_; }

double VehicleParams::minTurningRadius() const { return minTurningRadius_; }

double VehicleParams::maxSteeringAngle() const { return maxSteeringAngle_; }

double VehicleParams::maxSteeringAngleVelocity() const { return maxSteeringAngleVelocity_; }

double VehicleParams::maxFrontWheelAngleVelocity() const { return maxFrontWheelAngleVelocity_;};

double VehicleParams::steeringRatio() const { return steeringRatio_; }

double VehicleParams::steeringRatio_inside() const { return steeringRatio_inside_; }

double VehicleParams::steeringRatio_outside() const { return steeringRatio_outside_; }

double VehicleParams::steerRatio_ahead() const { return steerRatio_ahead_; }

double VehicleParams::steerRatio_inside_ahead() const { return steerRatio_inside_ahead_; }

double VehicleParams::steerRatio_outside_ahead() const { return steerRatio_outside_ahead_; }

double VehicleParams::dist_gear_pulse() const { return dist_gear_pulse_; }

double VehicleParams::dist_gear_pulse_front() const { return dist_gear_pulse_front_; }

double VehicleParams::dist_gear_pulse_back() const { return dist_gear_pulse_back_; }

int VehicleParams::max_num_gear_pulse() const{ return max_num_gear_pulse_; }

void VehicleParams::set_type(const VehicleType vehicle_type) { vehicle_type_ = vehicle_type;}

void VehicleParams::set_length(const double length) { length_ = length; }

void VehicleParams::set_width(const double width) { width_ = width; }

void VehicleParams::set_wheelbase(const double wheelbase) { wheelbase_ = wheelbase; }

void VehicleParams::set_front_suspension_length(const double front_suspension_length) {
       front_suspension_length_ = front_suspension_length;
}

void VehicleParams::set_rear_suspension_length(const double rear_suspension_length) {
    rear_suspension_length_ = rear_suspension_length;
}

void VehicleParams::set_front_axle_length(const double front_axle_length) {
    front_axle_length_ = front_axle_length;
}

void VehicleParams::set_rear_axle_length(const double rear_axle_length) {
    rear_axle_length_ = rear_axle_length;
}

void VehicleParams::set_minTurningRadius(const double minTurningRadius) {
    minTurningRadius_ = minTurningRadius;
}

void VehicleParams::set_maxSteeringAngle(const double maxSteeringAngle) {
    maxSteeringAngle_ = maxSteeringAngle;
}

void VehicleParams::set_maxSteeringAngleVelocity(const double maxSteeringAngleVelocity) {
    maxSteeringAngleVelocity_ = maxSteeringAngleVelocity;
}

void VehicleParams::set_steeringRatio(const double steeringRatio) {
    steeringRatio_ = steeringRatio;
}

void VehicleParams::set_steeringRatio_inside(const double steeringRatioInSide) {
    steeringRatio_inside_ = steeringRatioInSide;
}

void VehicleParams::set_steeringRatio_outside(const double steeringRatioOutSide) {
    steeringRatio_outside_ = steeringRatioOutSide;
}

void VehicleParams::set_steerRatio_ahead(const double steeringRatio) {
    steerRatio_ahead_ = steeringRatio;
}

void VehicleParams::set_steerRatio_inside_ahead(const double steeringRatio) {
    steerRatio_inside_ahead_ = steeringRatio;
}

void VehicleParams::set_steerRatio_outside_ahead(const double steeringRatio) {
    steerRatio_outside_ahead_ = steeringRatio;
}

void VehicleParams::set_dist_gear_pulse(const double dist_gear_pulse) {
    dist_gear_pulse_ = dist_gear_pulse;
}

void VehicleParams::set_max_num_gear_pulse(const int max_num_gear_pulse){
    max_num_gear_pulse_ = max_num_gear_pulse;
}

void VehicleParams::set_maxfrontwheelangle(const double maxFrontWheelAngleVelocity){
    maxFrontWheelAngleVelocity_ = maxFrontWheelAngleVelocity;
}

void VehicleParams::init(VehicleType vehicle_type) {
    vehicle_type_ = vehicle_type;

    if(VehicleType::Q_EV == vehicle_type) {
        length_ = 4.675;
        width_ = 1.770;
        wheelbase_ = 2.67;
        front_suspension_length_ = 1.105;  //
        rear_suspension_length_ = 0.90;  //
        front_axle_length_ = 1.3;
        rear_axle_length_= 1.37;
        minTurningRadius_ = 5.3;           // 2.15/tan(520/18.55) = 4.04; 2.15/tan(570/18.55) = 3.62; 5.3
        steerAngle_cmp_ = 0.0;             // 方向盘校准值
        maxSteeringAngle_ = 520;           // (R-525, L-570) 左正右负
        maxSteeringAngleLeft = 520.0;
        maxSteeringAngleRight = -520.0;
        maxSteeringAngleVelocity_ = 400;  // 500;
        steeringRatio_ = 16.48;//16.48;
        maxFrontWheelAngleVelocity_ = 23.6;//  rad/s
        steeringRatio_inside_ = 16.39;//16.39;     // inside表示往左边转弯
        steeringRatio_outside_ = 16.57;//16.57;    // outside表示往右边转弯
        steerRatio_ahead_ = 16.25;//16.25
        steerRatio_inside_ahead_ = 16.14;//16.14;  // inside表示往左边转弯,柏油路面,胎压2.4Kpa,车外温度30
        steerRatio_outside_ahead_ = 16.37;//16.37; // outside表示往右边转弯,柏油路面,胎压2.4Kpa,车外温度30
        dist_gear_pulse_ =  0.0207;         // 测量的左前轮,后轮比前轮稍微小一点
        dist_gear_pulse_front_ = 0.0207;
        dist_gear_pulse_back_ = 0.0204;
        max_num_gear_pulse_ = 1024;
    }
}
