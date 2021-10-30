#ifndef PURE_PURSUIT_CONTROLLER_H
#define PURE_PURSUIT_CONTROLLER_H
#include <memory>
#include <fstream>
#include "control/controller.h"
#include "common/debug.h"

namespace control {

class PurePursuitController : public Controller{
public:
    /**
     * @brief constructor
     */
    PurePursuitController() ;
    /**
     * @brief destructor
     */
    virtual ~PurePursuitController();
    /**
     * @brief initialize PP Controller
     * @param control_conf control configurations
     * @return Status initialization status
     */
    //common::Status Init(const ControlConf *control_conf) override;
    bool Init();

    int ComputeControlCommand(const std::vector<TrajectoryPoint>* planning_trajectory,
                              SimpleDebug *debug,
                              const VehicleState &vehicle_state);
protected:
    void ProcessLogs(const SimpleDebug *debug);

    void LogInitParameters();

    void CloseLogFile();
    const std::string name_;
    //debug log flag
    bool FLAGS_enable_csv_debug = true;
    // for logging purpose
    std::ofstream steer_log_file_;
    // [m] wheel base of vehicle
    double L_;
    //wheel steer transmission ratio
    double SteerRatio_;
    //L / atan(maxSteerAngle * M_PI / 180);
    double minTurningRadius_;
    //vehicle type
    VehicleType VehicleType_;
    //max steering angle turn left
    double maxSteeringAngleLeft_;
    //max steering angle turn right;
    double maxSteeringAngleRight_;

    double SteerRatioAhead_;
private:
    TrajectoryPoint closet_point;

    TrajectoryPoint target_point;

};

}
#endif // PURE_PURSUIT_CONTROLLER_H
