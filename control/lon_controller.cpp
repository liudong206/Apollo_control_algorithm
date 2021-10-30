#include "lon_controller.h"
#include <ctime>
#include <iomanip>
#include "common/log.h"
namespace control {
double GRA_ACC = 9.8;
namespace
{
    std::string GetLogFileName() {
    time_t raw_time;
    char name_buffer[80];
    std::time(&raw_time);
    strftime(name_buffer, 80, "tmp/speed_log_%F_%H%M%S.csv",
           localtime(&raw_time));
    return std::string(name_buffer);
}

void WriteHeaders(std::ofstream &file_stream) {
    file_stream << "lon_station_error,"
                << "station_error_limited,"
                << "preview_station_error,"
                << "speed_reference,"
                << "lon_speed_error,"
                << "preview_speed_reference,"
                << "preview_speed_error,"
                << "preview_acceleration_reference,"
                << "acceleration_cmd_closeloop,"
                << "slope_offset_compensation,"
                << "acceleration_cmd,"
                << std::endl;
}
}//namespace

LonController::LonController() : name_("PID-base Longitudinal Controller"){
    if (FLAGS_enable_csv_debug) {
      speed_log_file_.open(GetLogFileName());
      speed_log_file_ << std::fixed;
      speed_log_file_ << std::setprecision(6);
      WriteHeaders(speed_log_file_);
    }
    AINFO << "Using " << name_;
    ControlConf *control_conf = new ControlConf();
    Init(control_conf);
}

LonController::~LonController(){
    CloseLogFile();
};

void LonController::ProcessLogs(const SimpleDebug *debug) {
  // StrCat supports 9 arguments at most.

    const std::string log_str
            = std::to_string(debug->lon_station_error) + "," + std::to_string(debug->station_error_limited) + "," +
              std::to_string(debug->preview_station_error) + "," + std::to_string(debug->speed_reference) + "," +
              std::to_string(debug->lon_speed_error) + "," + std::to_string(debug->preview_speed_reference) + "," +
              std::to_string(debug->preview_speed_error) + "," + std::to_string(debug->preview_acceleration_reference) + "," +
              std::to_string(debug->slope_offset_compensation) + "," + std::to_string(debug->acceleration_cmd_closeloop) + "," +
              std::to_string(debug->acceleration_cmd);
    if (FLAGS_enable_csv_debug) {
      speed_log_file_ << log_str << std::endl;
    }
    ADEBUG << "Steer_Control_Detail: " << log_str;
}

void LonController::CloseLogFile() {
    if (FLAGS_enable_csv_debug && speed_log_file_.is_open()) {
    speed_log_file_.close();
  }
}

void LonController::Stop() { CloseLogFile(); }

bool LonController::Init(const ControlConf *control_conf){
    control_conf_ = control_conf;
    if(nullptr == control_conf_){
        controller_initialized_ = false;
        AERROR << "get_longitudinal_param() nullptr";
        return false;
    }
    LonControllerConf lon_controller_conf;

    station_pid_controller_.Init(station_pid_conf_);
    speed_pid_controller_.Init(low_speed_pid_conf_);

    SetDigitalFilterPitchAngle(lon_controller_conf);
    LoadControlCalibrationTable(lon_controller_conf);
    controller_initialized_ = true;
    return true;
}

void LonController::SetDigitalFilterPitchAngle(
    const LonControllerConf &lon_controller_conf) {
    double cutoff_freq = lon_controller_conf.cutoff_freq;
    double ts = lon_controller_conf.ts;
    SetDigitalFilter(ts, cutoff_freq, &digital_filter_pitch_angle_);
}

void LonController::SetDigitalFilter(double ts, double cutoff_freq,
                                     common::DigitalFilter *digital_filter) {
    std::vector<double> denominators;
    std::vector<double> numerators;
    common::LpfCoefficients(ts, cutoff_freq, &denominators, &numerators);
    digital_filter->set_coefficients(denominators, numerators);
}

void LonController::LoadControlCalibrationTable(const LonControllerConf &lon_controller_conf){

}

int LonController::ComputeControlCommand(const std::vector<TrajectoryPoint>* planning_trajectory,
                                         SimpleDebug *debug,const VehicleState &vehicle_state){
    const LonControllerConf &lon_controller_conf =
            control_conf_->lon_controller_conf;
    double ts = lon_controller_conf.ts;
    double preview_window = lon_controller_conf.preview_window;
    if(preview_window < 0.0){
        const auto error_msg =
                "Preview window set as: " + std::to_string(preview_window) + " less than 0";
        return -1;
    }
    size_t index = find_closest_point(planning_trajectory,vehicle_state.x(),vehicle_state.y());
    TrajectoryPoint ref_point = (*planning_trajectory)[index];
    TrajectoryPoint preview_point = (*planning_trajectory)[index + preview_window];
    if(FLAGS_enable_speed_station_preview){
        ComputerLongitudinalErrors(preview_point,debug,vehicle_state);
        debug->preview_station_error = debug->lon_station_error;
        debug->preview_speed_error = debug->lon_speed_error;
    }else {
        ComputerLongitudinalErrors(ref_point,debug,vehicle_state);
    }
    double station_error_limit = lon_controller_conf.station_error_limit;
    double station_error_limited = 0.0;
    if (FLAGS_enable_speed_station_preview) {
       station_error_limited =
           Clamp(debug->preview_station_error,
                               -station_error_limit, station_error_limit);
     } else {
       station_error_limited = Clamp(
           debug->lon_station_error, -station_error_limit, station_error_limit);
     }
    double speed_offset =
          station_pid_controller_.Control(station_error_limited, ts);
    double speed_controller_input = 0.0;
    double speed_controller_input_limit =
          lon_controller_conf.speed_controller_input_limit;
    double speed_controller_input_limited = 0.0;
      if (FLAGS_enable_speed_station_preview) {
        speed_controller_input = speed_offset + debug->preview_speed_error;
      } else {
        speed_controller_input = speed_offset + debug->lon_speed_error;
      }
    speed_controller_input_limited =
          Clamp(speed_controller_input, -speed_controller_input_limit,
                                        speed_controller_input_limit);
    double acceleration_cmd_closeloop = 0.0;
    if(vehicle_state.linear_velocity() < lon_controller_conf.switch_speed){
        speed_pid_controller_.SetPID(low_speed_pid_conf_);
           acceleration_cmd_closeloop =
               speed_pid_controller_.Control(speed_controller_input_limited, ts);
    }else {
        speed_pid_controller_.SetPID(high_speed_pid_conf_);
            acceleration_cmd_closeloop =
                speed_pid_controller_.Control(speed_controller_input_limited, ts);
    }
    double slope_offset_compenstaion = digital_filter_pitch_angle_.Filter(
          GRA_ACC * std::sin(vehicle_state.pitch()));
    debug->slope_offset_compensation = slope_offset_compenstaion;
    double acceleration_cmd =
          acceleration_cmd_closeloop + debug->preview_acceleration_reference +
          FLAGS_enable_slope_offset * debug->slope_offset_compensation;
    debug->acceleration_cmd = acceleration_cmd;
    ProcessLogs(debug);
    return 0;
}

void LonController::ComputerLongitudinalErrors(const TrajectoryPoint &ref_point,
                                               SimpleDebug *debug,
                                               const VehicleState &vehicle_state){
    double delta_theta = vehicle_state.heading() - ref_point.theta;
    double speed_error = ref_point.v - vehicle_state.linear_velocity() * cos(delta_theta);
    double dx = vehicle_state.x() - ref_point.x;
    double dy = vehicle_state.y() - ref_point.y;
    double station_error = -(dx * cos(ref_point.theta) + dy * sin(ref_point.theta));
    debug->lon_station_error = station_error;
    debug->lon_speed_error = speed_error;
    debug->speed_reference = vehicle_state.linear_velocity();
    debug->preview_acceleration_reference = ref_point.a;


}



}
