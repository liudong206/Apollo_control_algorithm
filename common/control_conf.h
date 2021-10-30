#ifndef CONTROL_CONF_H
#define CONTROL_CONF_H
#include <cstdint>

struct LatControllerConf {
    const std::uint_fast8_t cutoff_freq = 10;
    const std::uint_fast8_t mean_filter_window_size = 10;

};

struct LonControllerConf{
    const double ts = 0.01;
    const double brake_deadzone = 15.5;
    const double throttle_deadzone = 18.0;
    const double speed_controller_input_limit = 2.0;
    const double station_error_limit = 2.0;
    const double preview_window = 20.0;//预瞄视野
    const double standstill_acceleration = -3.0;
    const double switch_speed =  2.0;
    const std::uint_fast8_t cutoff_freq = 5;
};

struct MpcControllerConf{
    const std::uint_fast8_t cutoff_freq = 10;
    const std::uint_fast8_t mean_filter_window_size = 2;
};

class PidConf{
public:
    bool integrator_enable;
    double integrator_saturation_level;
    double kp;
    double ki;
    double kd;
};

class StationPidConf : public PidConf{
public:
    StationPidConf(){
        integrator_enable = false;
        integrator_saturation_level = 0.3;
        kp = 0.2;
        ki = 0.0;
        kd = 0.0;
    }
};

class LowSpeedPidConf : public PidConf{
public:
    LowSpeedPidConf(){
        integrator_enable = true;
        integrator_saturation_level = 0.3;
        kp = 0.5;
        ki = 0.3;
        kd = 0.0;
    }
};

class HighSpeedPidConf : public PidConf{
public:
    HighSpeedPidConf(){
        integrator_enable = true;
        integrator_saturation_level = 0.3;
        kp = 1.0;
        ki = 0.3;
        kd = 0.0;
    }
};



struct ControlConf {
    struct LatControllerConf lat_controller_conf;
    struct LonControllerConf lon_controller_conf;
    struct MpcControllerConf mpc_controller_conf;
};
#endif // CONTROL_CONF_H
