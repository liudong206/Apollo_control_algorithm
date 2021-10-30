#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H
#include "common/control_conf.h"
namespace control {
  /**
   * @class PIDController
   * @brief A proportional–integral–derivative controller for speed and steering
   using defualt integral hold
   */
class PIDController{
public:
  /**
   * @brief constructor
   */
  PIDController();

  /**
   * @brief destructor
   */
  virtual ~PIDController();
  /**
   * @brief initialize pid controller
   * @param pid_conf configuration for pid controller
   */
  void Init(const PidConf &pid_conf);

  /**
   * @brief set pid controller coefficients for the proportional,
   * integral, and derivative
   * @param pid_conf configuration for pid controller
   */
  void SetPID(const PidConf &pid_conf);

  /**
   * @brief reset variables for pid controller
   */
  void Reset();

  /**
   * @brief compute control value based on the error
   * @param error error value, the difference between
   * a desired value and a measured value
   * @param dt sampling time interval
   * @return control value based on PID terms
   */
  virtual double Control(const double error, const double dt);

  /**
   * @brief get saturation status
   * @return saturation status
   */
  int IntegratorSaturationStatus() const;

  /**
   * @brief get saturation status
   * @return saturation status
   */
  bool IntegratorHold() const;

  /**
   * @brief set whether to hold integrator component at its current value.
   * @param hold
   */
  void SetIntegratorHold(bool hold);

protected:
  double kp_ = 0.0;
  double ki_ = 0.0;
  double kd_ = 0.0;
  double kaw_ = 0.0;
  double previous_error_ = 0.0;
  double previous_output_ = 0.0;
  double integral_ = 0.0;
  double integrator_saturation_high_ = 0.0;
  double integrator_saturation_low_ = 0.0;
  bool first_hit_ = false;
  bool integrator_enabled_ = false;
  bool integrator_hold_ = false;
  int integrator_saturation_status_ = 0;
};
}
#endif // PID_CONTROLLER_H
