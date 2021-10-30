#include "mpc_controller.h"
#include <iomanip>
namespace control {
namespace mpc {

using Matrix = Eigen::MatrixXd;
namespace  {
    std::string GetLogFileName() {
    time_t raw_time;
    char name_buffer[80];
    std::time(&raw_time);
    strftime(name_buffer, 80, "log/mpc_log_file_optimal_%F_%H%M%S.csv",
           localtime(&raw_time));
    std::string file = std::string(name_buffer);
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

MPCController::MPCController() :name_("MPC Controller"){
    if (FLAGS_enable_csv_debug) {
      std::string filename = GetLogFileName();
      mpc_log_file_.open(filename);
      mpc_log_file_ << std::fixed;
      mpc_log_file_ << std::setprecision(6);
      WriteHeaders(mpc_log_file_);
    }
    AINFO << "Using " << name_;
    ControlConf *control_conf = new  ControlConf();
    Init(control_conf);

}
MPCController::~MPCController(){
    CloseLogFile();
}

void MPCController::CloseLogFile() {
    if (FLAGS_enable_csv_debug && mpc_log_file_.is_open()) {
    mpc_log_file_.close();
  }
}

void MPCController::ProcessLogs(const SimpleDebug *debug) {
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
      mpc_log_file_ << log_str << std::endl;
    }
    ADEBUG << "Steer_Control_Detail: " << log_str;
}

void MPCController::LogInitParameters() {
    AINFO << name_ << " begin.";
    AINFO << "[MPCController parameters]"
          << " mass_: " << mass_ << ","
          << " iz_: " << iz_ << ","
          << " lf_: " << lf_ << ","
          << " lr_: " << lr_;
}


void MPCController::InitializeFilters(const ControlConf *control_conf){
    // Low pass filter
    std::vector<double> den(3, 0.0);
    std::vector<double> num(3, 0.0);
    common::LpfCoefficients(ts_, control_conf->mpc_controller_conf.cutoff_freq, &den, &num);
    digital_filter_.set_coefficients(den, num);
    lateral_error_filter_ = common::MeanFilter(
         control_conf->mpc_controller_conf.mean_filter_window_size);
    heading_error_filter_ = common::MeanFilter(
         control_conf->mpc_controller_conf.mean_filter_window_size);
}

double MPCController::LimitSteerAngleByVelocity(double steer_cmd,double max_angle_vel,double time_step){
    double max_delta_steer_angle =  max_angle_vel * time_step;
    double min_delta_steer_angle =  -max_angle_vel * time_step;

    return std::max(std::min(max_delta_steer_angle,steer_cmd),min_delta_steer_angle);
}

double MPCController::LimitSteerAngle(double steer_cmd,double max_steer_angle){
    double steer_angle = 0;
    if(steer_cmd > max_steer_angle){
        steer_angle = max_steer_angle;
    }else if (steer_cmd < -max_steer_angle) {
        steer_angle = -max_steer_angle;
    }else {
        steer_angle = steer_cmd;
    }
    return steer_angle;
}

int MPCController::CalculateClosestPoint(const std::vector<TrajectoryPoint>* trajectory,
                                         const double x,
                                         const double y){
    auto func_distance_square = [](const PathPoint& point, const double x,
                                   const double y) {
      double dx = point.x - x;
      double dy = point.y - y;
      return dx * dx + dy * dy;
    };
    double d_min = func_distance_square(trajectory->front(), x, y);
    int index_min = 0;
    for (int i = 1; i < trajectory->size(); ++i) {
      double d_temp = func_distance_square((*trajectory)[i], x, y);
      if (d_temp < d_min) {
        d_min = d_temp;
        index_min = i;
      }
    }
    return index_min;
}

bool MPCController::LoadControlConf(){
    VehicleParams vehicle_params;
    ts_ = 20.0/1000.0;//apoll 0.05 matlab 0.01
    // control time interval
    cf_ = 158000.0;
    // corner stiffness; rear
    cr_ = 192000.0;
    wheelbase_ = vehicle_params.Wheelbase();
    steer_transmission_ratio_ = vehicle_params.steeringRatio();
    steer_single_direction_max_degree_ = vehicle_params.maxSteeringAngle();
    maxFrontWheelAngleVelocity_ = vehicle_params.maxFrontWheelAngleVelocity();
    maxSteeringAngle = vehicle_params.maxSteeringAngle();
    max_lat_acc_ = 5.0;
    max_acceleration_ = 10.0;
    max_deceleration_ = -10.0;
//    const double mass_fl = control_conf->mpc_controller_conf().mass_fl();
//    const double mass_fr = control_conf->mpc_controller_conf().mass_fr();
//    const double mass_rl = control_conf->mpc_controller_conf().mass_rl();
//    const double mass_rr = control_conf->mpc_controller_conf().mass_rr();
//    const double mass_front = mass_fl + mass_fr;
//    const double mass_rear = mass_rl + mass_rr;
//    mass_ = mass_front + mass_rear;
    // mass of the vehicle
    mass_ = 2100.0;

    // distance from front wheel center to COM
    lf_ = vehicle_params.front_axle_length();
    // distance from rear wheel center to COM
    lr_ = vehicle_params.rear_axle_length();
    const double mass_front = mass_ * (1.0 - lf_ / wheelbase_);
    const double mass_rear = mass_ * (1.0 - lr_ / wheelbase_);
    iz_ = lf_ * lf_ * mass_front + lr_ * lr_ * mass_rear;
    mpc_eps_ = 0.01;
    mpc_max_iteration_ = 150;
    throttle_deadzone_ = 18.0;
    brake_deadzone_ = 15.0;
    minimum_speed_protection_ = 0.1;
    standstill_acceleration_ = -3.0;
//    LoadControlCalibrationTable(control_conf->mpc_controller_conf());
    std::cout << "MPC conf loaded";
    return true;
}

bool MPCController::Init(const ControlConf *control_conf){
    if (!LoadControlConf()) {
        std::cout << " failed to load control conf " << std::endl;
    }
    // Matrix init operations.
    const int matrix_size = basic_state_size_;
    matrix_a_ = Matrix::Zero(basic_state_size_, basic_state_size_);
    matrix_ad_ = Matrix::Zero(basic_state_size_, basic_state_size_);
    matrix_a_(0, 1) = 1.0;
    matrix_a_(1, 2) = (cf_ + cr_) / mass_;
    matrix_a_(2, 3) = 1.0;
    matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;
    matrix_a_(4, 5) = 1.0;
    matrix_a_(5, 5) = 0.0;
    // TODO(QiL): expand the model to accomendate more combined states.

    matrix_a_coeff_ = Matrix::Zero(basic_state_size_, basic_state_size_);
    matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
    matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
    matrix_a_coeff_(2, 3) = 1.0;
    matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
    matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;

    matrix_b_ = Matrix::Zero(basic_state_size_, controls_);
    matrix_bd_ = Matrix::Zero(basic_state_size_, controls_);
    matrix_b_(1, 0) = cf_ / mass_;
    matrix_b_(3, 0) = lf_ * cf_ / iz_;
    matrix_b_(4, 1) = 0.0;
    matrix_b_(5, 1) = -1.0;
    matrix_bd_ = matrix_b_ * ts_;

    matrix_c_ = Matrix::Zero(basic_state_size_, 1);
    matrix_c_(5, 0) = 1.0;
    matrix_cd_ = Matrix::Zero(basic_state_size_, 1);

    matrix_state_ = Matrix::Zero(basic_state_size_, 1);
    matrix_k_ = Matrix::Zero(1, basic_state_size_);

    matrix_r_ = Matrix::Identity(controls_, controls_);

    matrix_q_ = Matrix::Zero(basic_state_size_, basic_state_size_);
    int q_param_size = 6 ;
    if (matrix_size != q_param_size) {
      std::cout << " in parameter file not equal to matrix_size " << std::endl;
    }
    double matrix_q[q_param_size];
    matrix_q[0] = 0.05;//0.05//0.1
    matrix_q[1] = 0.0;//0.0 //0.0
    matrix_q[2] = 0.5;//1.0 //6.0
    matrix_q[3] = 0.0;
    matrix_q[4] = 0.0;//0.2 //0.0
    matrix_q[5] = 0.0;//0.3 //0.0
    for (int i = 0; i < q_param_size; ++i) {
    //matrix_q_(i, i) = control_conf->lat_controller_conf().matrix_q(i);
      matrix_q_(i, i) = matrix_q[i];
    }
    // Update matrix_q_updated_ and matrix_r_updated_
    matrix_r_updated_ = matrix_r_;
    matrix_q_updated_ = matrix_q_;

    InitializeFilters(control_conf);
    LogInitParameters();
    return true;
}

void MPCController::ComputeLongitudinalErrors(const TrajectoryPoint &ref_point,
                                              SimpleDebug *debug,
                                              const VehicleState &vehicle_state){
    double dx = vehicle_state.x() - ref_point.x;
    double dy = vehicle_state.y() - ref_point.y;
    double dtheta = vehicle_state.heading() - ref_point.theta;
    double curvature = ref_point.kappa;
    double one_min_k = 1 - curvature;
    if(one_min_k <= 0){
        one_min_k = 0.01;
    }
    debug->station_error = -(dx * std::cos(ref_point.theta) + dy * std::sin(ref_point.theta));
    debug->speed_error = ref_point.v - vehicle_state.linear_velocity() * std::cos(dtheta)/one_min_k;
}

void MPCController::ComputeLateralErrors(const TrajectoryPoint &ref_point,
                                         SimpleDebug *debug,
                                         const VehicleState &vehicle_state){
    double dx = vehicle_state.x() - ref_point.x;
    double dy = vehicle_state.y() - ref_point.y;
    double dtheta = vehicle_state.heading() - ref_point.theta;
    debug->lateral_error = dy * std::cos(ref_point.theta) - dx * std::sin(ref_point.theta);
    debug->lateral_error_rate = vehicle_state.linear_velocity() * std::sin(dtheta);
    debug->heading_error = AngleNormalization(dtheta);
    debug->heading_error_rate = vehicle_state.angular_velocity() - ref_point.v * ref_point.kappa;
}

double MPCController::FeedforwardUpdate(double curvature){

    double kv = lr_ * mass_ / 2 / cf_ / wheelbase_ - lf_ * mass_ / 2 / cr_ / wheelbase_ ;
    // then change it from rad to %
    const double v = linear_velocity;
    double steer_angle_feedforwardterm =
            (wheelbase_ * curvature + kv * v * v * curvature)*
            180 / M_PI * steer_transmission_ratio_ /
            steer_single_direction_max_degree_ * 420;;
    return steer_angle_feedforwardterm;
}

void MPCController::UpdateMatrix(SimpleDebug *debug){
    const double v = std::max(linear_velocity,minimum_speed_protection_);
    matrix_a_(1, 1) = matrix_a_coeff_(1, 1) / v;
    matrix_a_(1, 3) = matrix_a_coeff_(1, 3) / v;
    matrix_a_(3, 1) = matrix_a_coeff_(3, 1) / v;
    matrix_a_(3, 3) = matrix_a_coeff_(3, 3) / v;

    Matrix matrix_i = Matrix::Identity(matrix_a_.cols(), matrix_a_.cols());
    matrix_ad_ = (matrix_i - ts_ * 0.5 * matrix_a_).inverse() *
                 (matrix_i + ts_ * 0.5 * matrix_a_);

    matrix_c_(1, 0) = (lr_ * cr_ - lf_ * cf_) / mass_ / v - v;
    matrix_c_(3, 0) = -(lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_ / v;
    matrix_cd_ = matrix_c_ * debug->heading_error_rate * ts_;
}

int MPCController::ComputeControlCommand(const std::vector<TrajectoryPoint>* planning_trajectory,
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

    size_t index = find_closest_point(planning_trajectory,vehicle_state.x(),vehicle_state.y());
    TrajectoryPoint tartget_point = (*planning_trajectory)[index+10];//10
    debug->ref_heading = tartget_point.theta;
    debug->curvature = tartget_point.kappa;
    ComputeLongitudinalErrors(tartget_point,debug,vehicle_state);
    ComputeLateralErrors(tartget_point,debug,vehicle_state);
    //caculate the state
    matrix_state_ = Matrix::Zero(basic_state_size_, 1);
    matrix_state_(0,0) = debug->lateral_error;
    matrix_state_(1,0) = debug->lateral_error_rate;
    matrix_state_(2,0) = debug->heading_error;
    matrix_state_(3,0) = debug->heading_error_rate;
    matrix_state_(4,0) = debug->station_error;
    matrix_state_(5,0) = debug->speed_error;
    UpdateMatrix(debug);
    double steer_angle_feedforwardterm =
            FeedforwardUpdate(tartget_point.kappa);
    //
    Eigen::MatrixXd control_matrix(controls_, 1);
    control_matrix << 0, 0;
    Eigen::MatrixXd reference_state(basic_state_size_, 1);
    reference_state << 0, 0, 0, 0, 0, 0;

    std::vector<Eigen::MatrixXd> reference(horizon_, reference_state);

    Eigen::MatrixXd lower_bound(controls_, 1);
    lower_bound << -steer_single_direction_max_degree_/
                   steer_transmission_ratio_/180*M_PI, max_deceleration_;

    Eigen::MatrixXd upper_bound(controls_, 1);
    upper_bound << steer_single_direction_max_degree_/
                   steer_transmission_ratio_/180*M_PI, max_acceleration_;

    std::vector<Eigen::MatrixXd> control(horizon_, control_matrix);
    if (apollo::common::math::SolveLinearMPC(
            matrix_ad_, matrix_bd_, matrix_cd_, matrix_q_updated_,
            matrix_r_updated_, lower_bound, upper_bound, matrix_state_, reference,
            mpc_eps_, mpc_max_iteration_, &control) != true) {
      std::cout << "MPC solver failed";
    } else {
      std::cout << "MPC problem solved! ";
    }
    double steer_angle_feedback = control[0](0, 0) * 180 / M_PI *
                                    steer_transmission_ratio_ /
                                    steer_single_direction_max_degree_ * 420;
    double steer_angle_raw =
          steer_angle_feedback + steer_angle_feedforwardterm;
    double steer_angle_limited = LimitSteerAngle(steer_angle_raw,maxSteeringAngle);
    double steer_angle_filtered = digital_filter_.Filter(steer_angle_limited);
    double frant_wheel_delta = debug->steer_angle / 180 * M_PI
                                        / steer_transmission_ratio_;
    double frant_wheel_delta_limited = LimitSteerAngleByVelocity(
                frant_wheel_delta,maxFrontWheelAngleVelocity_,debug->time_step);
    std::cout<<"steer_angle_feedforwardterm:"<<steer_angle_feedforwardterm<<std::endl;
    std::cout<<"steer_angle_feedback:"<<steer_angle_feedback<<std::endl;
    std::cout<<"steer_angle##################:"<<steer_angle_filtered<<std::endl;
    debug->steer_angle_feedback = steer_angle_feedback;
    debug->steer_angle_feedforward = steer_angle_feedforwardterm;
    debug->steer_angle = steer_angle_filtered;
    debug->frant_wheel_delta = frant_wheel_delta_limited;//debug->steer_angle / 180 * M_PI
                                                        //     / steer_transmission_ratio_;
    debug->acceleration = control[0](1, 0);;
    ProcessLogs(debug);
    return 0;
    }

}
}
