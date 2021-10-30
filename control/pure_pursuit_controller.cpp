#include <queue>
#include <ctime>
#include <cstdlib>
#include <utility>
#include <iomanip>

#include "control/pure_pursuit_controller.h"
#include "common/log.h"

namespace control {
namespace
{
std::string GetLogFileName() {
    time_t raw_time;
    char name_buffer[80];
    std::time(&raw_time);
    strftime(name_buffer, 80, "debug/pure_pursuit_log_%F_%H%M%S.csv",
           localtime(&raw_time));
    return std::string(name_buffer);
}

void WriteHeaders(std::ofstream &file_stream){
      file_stream << "ref_heading,"
                  << "heading,"
                  << "curvature,"
                  << "frant_wheel_delta,"
                  << "steer_angle,"
                  << "v,"
                  << "vehicle_x,"
                  << "vehicle_y,"
                  << std::endl;
    }
}

PurePursuitController::PurePursuitController() : name_("pure pursuit Controller"){
    if (FLAGS_enable_csv_debug) {
      steer_log_file_.open(GetLogFileName());
      steer_log_file_ << std::fixed;
      steer_log_file_ << std::setprecision(6);
      WriteHeaders(steer_log_file_);
    }
    AINFO << "Using " << name_;
    if(!Init()){
      ADEBUG<<"Init failed";
    }
}

void PurePursuitController::ProcessLogs(const SimpleDebug *debug) {
  // StrCat supports 9 arguments at most.
    const std::string log_str
            = std::to_string(debug->ref_heading) + "," +
              std::to_string(debug->heading) + "," +
              std::to_string(debug->curvature) + "," +
              std::to_string(debug->frant_wheel_delta) + "," +
              std::to_string(debug->steer_angle) + "," +
              std::to_string(debug->linear_velocity) + "," +
              std::to_string(debug->vehicle_x) + "," +
              std::to_string(debug->vehicle_y);
    if (FLAGS_enable_csv_debug) {
      steer_log_file_ << log_str << std::endl;
    }
    ADEBUG << "Steer_Control_Detail: " << log_str;
}

void PurePursuitController::LogInitParameters() {
    AINFO << name_ << " begin.";
    AINFO << "[pure pursuit controller parameters]"
          << " L_: " << L_ << ","
          << " SteerRatio_: " << SteerRatio_ << ","
          << " minTurningRadius_: " << minTurningRadius_;
}

void PurePursuitController::CloseLogFile() {
    if (FLAGS_enable_csv_debug && steer_log_file_.is_open()) {
    steer_log_file_.close();
  }
}

PurePursuitController::~PurePursuitController() {
    CloseLogFile();
}

bool PurePursuitController::Init(){
    VehicleParams vehicle_params;
    L_ = vehicle_params.Wheelbase(); // [m] wheel base of vehicle
    minTurningRadius_ = vehicle_params.minTurningRadius(); // L / atan(maxSteerAngle * M_PI / 180);
    SteerRatio_ = vehicle_params.steeringRatio(); // wheel steer transmission ratio
    VehicleType_ = vehicle_params.type();
    SteerRatioAhead_ = vehicle_params.steerRatio_ahead();
    maxSteeringAngleLeft_ = vehicle_params.maxSteeringAngleLeft;
    maxSteeringAngleRight_ = vehicle_params.maxSteeringAngleRight;
    LogInitParameters();
    return true;
}

int PurePursuitController::ComputeControlCommand(const std::vector<TrajectoryPoint>* planning_trajectory,
                                                 SimpleDebug *debug,
                                                 const VehicleState &vehicle_state){
    auto func_distance_square = [](const PathPoint& point, const double x,
                                   const double y) {
      double dx = point.x - x;
      double dy = point.y - y;
      return dx * dx + dy * dy;
    };

    double Lfc = 1.0; // look-ahead distance  
    double k = 0.3; // look forward gain
    double Lf = Lfc + k * vehicle_state.linear_velocity();
    debug->linear_velocity = vehicle_state.linear_velocity();
    double alpha, delta;
    std::size_t target_point_index, index_min;

    index_min = find_closest_point(planning_trajectory, vehicle_state.x(), vehicle_state.y());
    closet_point = (*planning_trajectory)[index_min];

    for (std::size_t i = index_min; i < planning_trajectory->size(); ++i) {
      double d_temp = func_distance_square((*planning_trajectory)[i], vehicle_state.x(),
                                           vehicle_state.y());
      if (d_temp > Lf * Lf) {
        target_point_index = i;
        break;
      }
      target_point_index = i;
    }

    target_point = (*planning_trajectory)[target_point_index];
    std::cout<<"target_point.x:"<<target_point.x<<std::endl;
    std::cout<<"target_point.y:"<<target_point.y<<std::endl;
    debug->ref_heading = target_point.theta;
    debug->curvature = target_point.kappa;
    alpha = (std::atan2((target_point.y - vehicle_state.y()),(target_point.x - vehicle_state.x()))
            - vehicle_state.heading())* 180/ M_PI ;

    if(alpha > 180) {
        alpha = alpha - 360;
    }
    else if (alpha <= -180) {
        alpha = alpha + 360;
    }

    if (std::fabs(alpha) > std::asin(Lf / 2 / minTurningRadius_) * 180 / M_PI) {
        std::cout << "alpha is out of limited range." << alpha << std::endl;
    }

    delta = std::atan2(2.0 * L_ * std::sin(alpha * M_PI / 180) , Lf) * 180  / M_PI;

    if((VehicleType::Q_EV == VehicleType_) && (1 == target_point.directions)) {
        SteerRatio_ = SteerRatioAhead_;
    }
    double tarSteerWheelAngle = target_point.directions * delta * SteerRatio_;

    if(tarSteerWheelAngle > maxSteeringAngleLeft_) {
        tarSteerWheelAngle = maxSteeringAngleLeft_;
    }
    else if(tarSteerWheelAngle < maxSteeringAngleRight_) {
        tarSteerWheelAngle = maxSteeringAngleRight_;
    }

    static std::queue<double> averSteerAngle;
    static double sum = 0.0;
    double calAverAngle = 0.0;
    if(averSteerAngle.size() < 5) {
        averSteerAngle.push(tarSteerWheelAngle);
        sum = sum + averSteerAngle.back();
    }
    else {
        averSteerAngle.push(tarSteerWheelAngle);
        sum = sum + averSteerAngle.back() - averSteerAngle.front();
        averSteerAngle.pop();
    }

    calAverAngle = sum / averSteerAngle.size();
    debug->steer_angle = calAverAngle;
    debug->frant_wheel_delta = debug->steer_angle / 180 * M_PI / SteerRatio_;
    ProcessLogs(debug);
    return 0;

}



}

