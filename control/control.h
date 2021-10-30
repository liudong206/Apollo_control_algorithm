#ifndef CONTROL_H
#define CONTROL_H
#include <memory>
#include "control/controller.h"

namespace control {

class controller{
public:
    controller();

    ~controller();

    int LatAndLonController(const std::vector<TrajectoryPoint>* planning_trajectory,
                            SimpleDebug *debug,
                            const VehicleState &vehicle_state);

private:
    std::unique_ptr<Controller> ptr_lat_controller;
    std::unique_ptr<Controller> ptr_lon_controller;
};

}

#endif // CONTROL_H
