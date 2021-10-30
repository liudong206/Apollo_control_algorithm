#include "lat_controller.h"
#include <iomanip>
#include <string>
#include <stdio.h>
#include <cstdlib>
#include <utility>
#include <queue>
#include "linear_quadratic_regulator.h"

using namespace math;
namespace control {
namespace lqr {
using Matrix = Eigen::MatrixXd;
namespace
{
    std::string GetLogFileName() {
    time_t raw_time;
    char name_buffer[80];
    std::time(&raw_time);
    strftime(name_buffer, 80, "tmp/steer_log_simple_optimal_%F_%H%M%S.csv",
           localtime(&raw_time));
    return std::string(name_buffer);
}


void WriteHeaders(std::ofstream &file_stream) {
  file_stream << "lateral_error,"
              << "ref_heading,"
              << "heading,"
              << "heading_error,"
              << "heading_error_rate,"
              << "lateral_error_rate,"
              << "curvature,"
              << "steer_angle,"
              << "steer_angle_feedforward,"
              << "steer_angle_feedback,"
              << "v,"
              << "vehicle_x,"
              << "vehicle_y,"
              << "trajref_x,"
              << "trajref_y,"
              << std::endl;
}
}

LatController::LatController() : name_("LQR-based Lateral Controller"){
  if (FLAGS_enable_csv_debug) {
    steer_log_file_.open(GetLogFileName());
    steer_log_file_ << std::fixed;
    steer_log_file_ << std::setprecision(6);
    WriteHeaders(steer_log_file_);
  }
  AINFO << "Using " << name_;

  ControlConf *control_conf = new ControlConf();
  Init(control_conf);
};

LatController::~LatController(){
    CloseLogFile();
};

bool LatController::LoadControlConf(){
    VehicleParams vehicle_params;
    // control time interval
    ts_ = 0.02;
    // corner stiffness; front
    cf_ = 158000.0;
    // corner stiffness; rear
    cr_ = 192000.0;
    preview_window_ = 0.0;
    // distance between front and rear wheel center
    wheelbase_ = vehicle_params.Wheelbase();
    steer_transmission_ratio_ = vehicle_params.steeringRatio();
    steer_single_direction_max_degree_ = vehicle_params.maxSteeringAngle();
    maxFrontWheelAngleVelocity_ = vehicle_params.maxFrontWheelAngleVelocity();
    max_lat_acc_ = 5.0;
    // mass of the vehicle
    mass_ = 2100.0;
    // distance from front wheel center to COM
    lf_ = vehicle_params.front_axle_length();
    // distance from rear wheel center to COM
    lr_ = vehicle_params.rear_axle_length();
    const double mass_front = mass_ * (1.0 - lf_ / wheelbase_);
    const double mass_rear = mass_ * (1.0 - lr_ / wheelbase_);

    // moment of inertia
    iz_ = lf_ * lf_ * mass_front + lr_ * lr_ * mass_rear;

    lqr_eps_ = 0.01;  //LQR精度步长
    lqr_max_iteration_ = 150; //LQR最大迭代次数
    minimum_speed_protection_ = 0.1;
    return true;
};

void LatController::ProcessLogs(const SimpleDebug *debug) {
  // StrCat supports 9 arguments at most.

    const std::string log_str
            = std::to_string(debug->lateral_error) + "," + std::to_string(debug->ref_heading) + "," +
              std::to_string(debug->heading) + "," + std::to_string(debug->heading_error) + "," +
              std::to_string(debug->heading_error_rate) + "," + std::to_string(debug->lateral_error_rate) + "," +
              std::to_string(debug->curvature) + "," + std::to_string(debug->steer_angle) + "," +
              std::to_string(debug->steer_angle_feedforward) + "," + std::to_string(debug->steer_angle_feedback) + "," +
              std::to_string(debug->linear_velocity) + "," + std::to_string(debug->vehicle_x) + "," +
              std::to_string(debug->vehicle_y) + "," + std::to_string(debug->trajref_x) + "," +
              std::to_string(debug->trajref_y);
    if (FLAGS_enable_csv_debug) {
      steer_log_file_ << log_str << std::endl;
    }
    ADEBUG << "Steer_Control_Detail: " << log_str;
}

void LatController::LogInitParameters() {
    AINFO << name_ << " begin.";
    AINFO << "[LatController parameters]"
          << " mass_: " << mass_ << ","
          << " iz_: " << iz_ << ","
          << " lf_: " << lf_ << ","
          << " lr_: " << lr_;
}

void LatController::LoadLatGainScheduler(){
    Interpolation1D::DataType xy1, xy2;
    for (auto scheduler:lat_err_gain_scheduler) {
        xy1.push_back(std::make_pair(scheduler.first,scheduler.second));
    }
    for (auto scheduler:heading_err_gain_scheduler) {
        xy2.push_back(std::make_pair(scheduler.first,scheduler.second));
    }
    lat_err_interpolation_.reset(new Interpolation1D);
    if(!lat_err_interpolation_->Init(xy1)){
        ADEBUG<<"Fail to load lateral error gain scheduler";
    }
    heading_err_interpolation_.reset(new Interpolation1D);
    if(!heading_err_interpolation_->Init(xy2)){
        ADEBUG<<"Fail to load heading error gain scheduler";
    }
}

void LatController::InitializeFilters(const ControlConf *control_conf) {
    // Low pass filter
    std::vector<double> den(3, 0.0);
    std::vector<double> num(3, 0.0);
    common::LpfCoefficients(
        ts_, control_conf->lat_controller_conf.cutoff_freq, &den, &num);
    digital_filter_.set_coefficients(den, num);
    lateral_error_filter_ = common::MeanFilter(
        control_conf->lat_controller_conf.mean_filter_window_size);
    heading_error_filter_ = common::MeanFilter(
        control_conf->lat_controller_conf.mean_filter_window_size);
    kalman_init(&state,0.0,1000.0);
}

double LatController::LimitSteerAngleByVelocity(double steer_cmd,double max_angle_vel,double time_step){
    double max_delta_steer_angle =  max_angle_vel * time_step;
    double min_delta_steer_angle =  -max_angle_vel * time_step;

    return std::max(std::min(max_delta_steer_angle,steer_cmd),min_delta_steer_angle);
}

bool LatController::Init(const ControlConf *control_conf){
    if (!LoadControlConf()) {
        AERROR << "failed to load control conf";
        return false;
      }
    // Matrix init operations.
    const int matrix_size = basic_state_size_ + preview_window_;
    matrix_a_ = Matrix::Zero(basic_state_size_, basic_state_size_);
    matrix_ad_ = Matrix::Zero(basic_state_size_, basic_state_size_);
    matrix_adc_ = Matrix::Zero(matrix_size, matrix_size);
    matrix_a_(0, 1) = 1.0;
    matrix_a_(1, 2) = (cf_ + cr_) / mass_;
    matrix_a_(2, 3) = 1.0;
    matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;

    matrix_a_coeff_ = Matrix::Zero(matrix_size, matrix_size);
    matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
    matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
    matrix_a_coeff_(2, 3) = 1.0;
    matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
    matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;

    matrix_b_ = Matrix::Zero(basic_state_size_, 1);
    matrix_bd_ = Matrix::Zero(basic_state_size_, 1);
    matrix_bdc_ = Matrix::Zero(matrix_size, 1);
    matrix_b_(1, 0) = cf_ / mass_;
    matrix_b_(3, 0) = lf_ * cf_ / iz_;
    matrix_bd_ = matrix_b_ * ts_;

    matrix_state_ = Matrix::Zero(matrix_size, 1);
    matrix_k_ = Matrix::Zero(1, matrix_size);
    matrix_r_ = 20*Matrix::Identity(1, 1);//1
    matrix_q_ = Matrix::Zero(matrix_size, matrix_size);

    int q_param_size = 4 + preview_window_;
    double matrix_q[q_param_size];
    matrix_q[0] = 0.05;//0.1//0.1//0.05apollo//0.05
    matrix_q[1] = 0.0;//0.1//0.1 //0.0       //
    matrix_q[2] = 1.0;//1.0//2.0 //1.0       //10.0 ok
    matrix_q[3] = 0.0;//0.1//0.1//0.0
    for (int i = 0; i < q_param_size; ++i) {
        matrix_q_(i, i) = matrix_q[i];
    }
    if (matrix_size != q_param_size) {
        std::cout << " in parameter file not equal to matrix_size " << std::endl;
    }
    matrix_q_updated_ = matrix_q_;

    InitializeFilters(control_conf);
    LoadLatGainScheduler();
    LogInitParameters();
    return true;
}

void LatController::CloseLogFile() {
    if (FLAGS_enable_csv_debug && steer_log_file_.is_open()) {
    steer_log_file_.close();
  }
}

int LatController::ComputeControlCommand(const std::vector<TrajectoryPoint>* planning_trajectory,
                                         SimpleDebug *debug,
                                         const VehicleState &vehicle_state){
    std::size_t index_min = find_closest_point(planning_trajectory,vehicle_state.x(),vehicle_state.y());
    TrajectoryPoint tartget_point = (*planning_trajectory)[index_min+10];
    debug->ref_heading = tartget_point.theta;
    debug->curvature = tartget_point.kappa;
    UpdateStateAnalyticalMatching(tartget_point,debug,vehicle_state);
    UpdateMatrix();
    UpdateMatrixCompound();
    int enable_gain_scheduler = 1;
    if (enable_gain_scheduler) {
        matrix_q_updated_(0, 0) = matrix_q_(0, 0) *
                lat_err_interpolation_->Interpolate(linear_velocity);
        matrix_q_updated_(2, 2) = matrix_q_(2, 2) *
                heading_err_interpolation_->Interpolate(linear_velocity);
        math::SolveLQRProblem(matrix_adc_, matrix_bdc_, matrix_q_updated_,
                              matrix_r_, lqr_eps_, lqr_max_iteration_,
                              &matrix_k_);
    } else {
        math::SolveLQRProblem(matrix_adc_, matrix_bdc_, matrix_q_,
                              matrix_r_, lqr_eps_, lqr_max_iteration_,
                              &matrix_k_);
        std::cout<<"matrix_k_(0, 0) = "<<matrix_k_(0, 0)<<std::endl;
        std::cout<<"matrix_k_(0, 1) = "<<matrix_k_(0, 1)<<std::endl;
        std::cout<<"matrix_k_(0, 2) = "<<matrix_k_(0, 2)<<std::endl;
        std::cout<<"matrix_k_(0, 3) = "<<matrix_k_(0, 3)<<std::endl;
    }

    // feedback = - K * state
    // Convert vehicle steer angle from rad to degree and then to steer degree
    // then to 420% ratio
    const double steer_angle_feedback = -(matrix_k_ * matrix_state_)(0, 0) * 180 /
                                        M_PI * steer_transmission_ratio_ /
                                        steer_single_direction_max_degree_ * 420;

    const double steer_angle_feedforward = ComputeFeedForward(tartget_point.kappa);

    std::cout<<"steer_angle_feedback = "<<steer_angle_feedback<<std::endl;
    std::cout<<"steer_angle_feedforward = "<<steer_angle_feedforward<<std::endl;
    // Clamp the steer angle to -100.0 to 100.0
  //  double steer_angle = common::math::Clamp(
  //      steer_angle_feedback + steer_angle_feedforward, -100.0, 100.0);
    double steer_angle = steer_angle_feedback + steer_angle_feedforward;
    double frant_wheel_delta = steer_angle / 180 * M_PI
                               / steer_transmission_ratio_;
    bool FLAGS_set_steer_limit = true;
    if (FLAGS_set_steer_limit) {
        const double steer_limit =
            std::atan(max_lat_acc_ * wheelbase_ / linear_velocity * linear_velocity) *
            steer_transmission_ratio_ * 180 / M_PI /
            steer_single_direction_max_degree_ * 100;

        // Clamp the steer angle
        double steer_angle_limited =
            Clamp(steer_angle, -steer_limit, steer_limit);
        steer_angle_limited = digital_filter_.Filter(steer_angle_limited);
        steer_angle = steer_angle_limited;
        //double frant_wheel_delta_limited = LimitSteerAngleByVelocity(
        //           frant_wheel_delta,maxFrontWheelAngleVelocity_,debug->time_step);
      } else {
        steer_angle = digital_filter_.Filter(steer_angle);
      }
    // 求最近5次计算的方向盘角度平均值
    static std::queue<double> averSteerAngle;
    static double sum = 0.0;
    double calAverAngle = 0.0;
    if(averSteerAngle.size() < 5) {
        averSteerAngle.push(steer_angle);
        sum = sum + averSteerAngle.back();
    }
    else {
        averSteerAngle.push(steer_angle);
        sum = sum + averSteerAngle.back() - averSteerAngle.front();
        averSteerAngle.pop();
    }

    calAverAngle = sum / averSteerAngle.size();
    calAverAngle = kalman_filter(&state,calAverAngle);
    steer_angle = calAverAngle;
    debug->steer_angle_feedforward = steer_angle_feedforward;
    debug->steer_angle_feedback = steer_angle_feedback;
    frant_wheel_delta = steer_angle / 180 * M_PI
                               / steer_transmission_ratio_;
    debug->frant_wheel_delta = frant_wheel_delta;
    debug->steer_angle = steer_angle;
    ProcessLogs(debug);
    return 0;
}

void LatController::UpdateStateAnalyticalMatching(const TrajectoryPoint &closet_point,
                                                  SimpleDebug *debug,
                                                  const VehicleState &vehicle_state){
    ComputeLateralErrors(closet_point,debug,vehicle_state);
    matrix_state_ = Matrix::Zero(basic_state_size_, 1);
    matrix_state_(0,0) = debug->lateral_error;
    matrix_state_(1,0) = debug->lateral_error_rate;
    matrix_state_(2,0) = debug->heading_error;
    matrix_state_(3,0) = debug->heading_error_rate;
}

void LatController::ComputeLateralErrors(const TrajectoryPoint &ref_point,
                                         SimpleDebug *debug,
                                         const VehicleState &vehicle_state){
    double VehicleSpd;
    if(0.0 == vehicle_state.linear_velocity()){
        VehicleSpd = 0.01;
    }
    else {
        VehicleSpd = vehicle_state.linear_velocity();
    }
    linear_velocity = VehicleSpd;
    debug->linear_velocity = linear_velocity;

    const double dx = vehicle_state.x() - ref_point.x;
    const double dy = vehicle_state.y() - ref_point.y;
    const double dtheta = AngleNormalization(vehicle_state.heading() - ref_point.theta);
    const double raw_lateral_error = dy * std::cos(ref_point.theta) -
                                     dx * std::sin(ref_point.theta);
    bool FLAGS_use_navigation_mode = true;
    if(FLAGS_use_navigation_mode){
        double filtered_lateral_error =lateral_error_filter_.Update(raw_lateral_error);
        debug->lateral_error = filtered_lateral_error;

    }else {
        debug->lateral_error = raw_lateral_error;
    }
    // d_error_dot = linear_v * sin_delta_theta;
    debug->lateral_error_rate = vehicle_state.linear_velocity() * std::sin(dtheta);

    if(FLAGS_use_navigation_mode){
        debug->heading_error = heading_error_filter_.Update(dtheta);
    }else {
        debug->heading_error = dtheta;
    }
    // theta_error_dot = angular_v - target_point.path_point().kappa() *
    // target_point.v();
    debug->heading_error_rate = vehicle_state.angular_velocity() - ref_point.v * ref_point.kappa;
}

void LatController::UpdateMatrix() {
    const double v = linear_velocity;
    matrix_a_(1, 1) = matrix_a_coeff_(1, 1) / v;
    matrix_a_(1, 3) = matrix_a_coeff_(1, 3) / v;
    matrix_a_(3, 1) = matrix_a_coeff_(3, 1) / v;
    matrix_a_(3, 3) = matrix_a_coeff_(3, 3) / v;
    Matrix matrix_i = Matrix::Identity(matrix_a_.cols(), matrix_a_.cols());
    matrix_ad_ = (matrix_i - ts_ * 0.5 * matrix_a_).inverse()*
                 (matrix_i + ts_ * 0.5 * matrix_a_);
}

void LatController::UpdateMatrixCompound() {
    // Initialize preview matrix
    matrix_adc_.block(0, 0, basic_state_size_, basic_state_size_) = matrix_ad_;
    matrix_bdc_.block(0, 0, basic_state_size_, 1) = matrix_bd_;
    if (preview_window_ > 0) {
      matrix_bdc_(matrix_bdc_.rows() - 1, 0) = 1;
      // Update A matrix;
      for (int i = 0; i < preview_window_ - 1; ++i) {
          matrix_adc_(basic_state_size_ + i, basic_state_size_ + 1 + i) = 1;
      }
    }
}

double LatController::ComputeFeedForward(double ref_curvature) const{
    const double kv =
          lr_ * mass_ / 2 / cf_ / wheelbase_ - lf_ * mass_ / 2 / cr_ / wheelbase_;

    // then change it from rad to %
    const double v = linear_velocity;
    const double steer_angle_feedforwardterm =
         (wheelbase_ * ref_curvature + kv * v * v * ref_curvature -
          matrix_k_(0, 2) *
          (lr_ * ref_curvature - lf_ * mass_ * v * v * ref_curvature / 2 / cr_ / wheelbase_)) *
          180 / M_PI * steer_transmission_ratio_ /
          steer_single_direction_max_degree_ * 420;
    return steer_angle_feedforwardterm;
}

}//namespace
}//namespace
