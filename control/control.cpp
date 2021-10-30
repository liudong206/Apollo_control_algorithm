#include "control.h"
#include "controller_factory.h"

namespace control {

controller::controller(){
    ptr_lat_controller = ControllerFactory::CreateInstance(ControllerType::PPController);
    ptr_lon_controller = ControllerFactory::CreateInstance(ControllerType::PIDController);;
}
controller::~controller(){

}

int controller::LatAndLonController(const std::vector<TrajectoryPoint>* planning_trajectory,
                                    SimpleDebug *debug,
                                    const VehicleState &vehicle_state){
    ptr_lat_controller->ComputeControlCommand(planning_trajectory,debug,vehicle_state);
    ptr_lon_controller->ComputeControlCommand(planning_trajectory,debug,vehicle_state);

    return 0;

}
}
