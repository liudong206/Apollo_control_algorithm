#include "controller_factory.h"
#include "control/lat_controller.h"
#include "mpc_controller.h"
#include "control/pure_pursuit_controller.h"
#include "control/lon_controller.h"
std::unique_ptr<control::Controller> ControllerFactory::CreateInstance(
        const ControllerType &controller_type){
    switch (controller_type) {
        case ControllerType::MPCController:{
            return std::unique_ptr<control::Controller>(new control::mpc::MPCController());
        }
        case ControllerType::LQRController:{
            return std::unique_ptr<control::Controller>(new control::lqr::LatController());
        }
        case ControllerType::PPController:{
            return std::unique_ptr<control::Controller>(new control::PurePursuitController());
        }
        case ControllerType::PIDController:{
            return std::unique_ptr<control::Controller>(new control::LonController);
        }
        default:
            return nullptr;

    }
}
