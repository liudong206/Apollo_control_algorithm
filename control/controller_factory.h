#ifndef CONTROLLER_FACTORY_H
#define CONTROLLER_FACTORY_H
#include <memory>
#include "control/controller.h"

/**
 * @class LatControllerType
 * @brief LatControllerType is a enum class used to specify a Controller instance.
 */
enum class ControllerType{ MPCController,
                           LQRController,
                           PPController,
                           PIDController,
                           OTHER };

class ControllerFactory{
public:
    ControllerFactory() = delete;

    /**
     * @brief Generate a controller instance.
     * @param LatControllerType The specific type of controller
     * @return A unique pointer pointing to the controller instance.
     */
    static std::unique_ptr<control::Controller> CreateInstance(
        const ControllerType& controller_type);

};
#endif // CONTROLLER_FACTORY_H
