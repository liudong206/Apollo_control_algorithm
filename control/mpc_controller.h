/**
 * @file
 * @brief Defines the MPCController class.
 */
#ifndef MPC_CONTROLLER_H
#define MPC_CONTROLLER_H
#include <fstream>
#include <memory>
#include <string>
#include <time.h>
#include <ctime>
#include "common/log.h"
//#include "eigen3/Eigen/Core"
//#include "eigen3/Eigen/LU"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/LU"
#include "control/controller.h"
#include "mpc_solver.h"
#include "filters/digital_filter.h"
#include "filters/digital_filter_coefficients.h"
#include "filters/mean_filter.h"
namespace control {
namespace mpc {

/**
 * @class MPCController
 *
 * @brief MPCController, combined lateral and logitudinal controllers
 */
class MPCController : public Controller{
 public:
  /**
   * @brief constructor
   */
  MPCController();

  /**
   * @brief destructor
   */
  virtual ~MPCController();

  /**
   * @brief initialize MPC Controller
   * @param control_conf control configurations
   * @return Status initialization status
   */
  //common::Status Init(const ControlConf *control_conf) override;
  bool Init(const ControlConf *control_conf);

  int ComputeControlCommand(const std::vector<TrajectoryPoint>* planning_trajectory,
                            SimpleDebug *debug,
                            const VehicleState &vehicle_state);

  int CalculateClosestPoint(const std::vector<TrajectoryPoint>* trajectory,
                            const double x,
                            const double y);
  double LimitSteerAngleByVelocity(double steer_cmd,double max_angle_vel,double time_step);

  // number of states, includes
  // lateral error, lateral error rate, heading error, heading error rate,
  // station error, velocity error,



protected:

  void InitializeFilters(const ControlConf *control_conf);

  double FeedforwardUpdate(double curvature);

  void UpdateMatrix(SimpleDebug *debug);

  void ComputeLongitudinalErrors(const TrajectoryPoint &ref_point,
                                 SimpleDebug *debug,
                                 const VehicleState &vehicle_state);

  void ComputeLateralErrors(const TrajectoryPoint &ref_point,
                            SimpleDebug *debug,
                            const VehicleState &vehicle_state);

  bool LoadControlConf();

  double LimitSteerAngle(double steer_cmd,double max_steer_angle);

  void CloseLogFile();

  void ProcessLogs(const SimpleDebug *debug);

  void LogInitParameters();

  // the following parameters are vehicle physics related.
  // control time interval
  double ts_;
  // corner stiffness; front
  double cf_;
  // corner stiffness; rear
  double cr_;
  // distance between front and rear wheel center
  double wheelbase_;
  // mass of the vehicle
  double mass_;
  // distance from front wheel center to COM
  double lf_;
  // distance from rear wheel center to COM
  double lr_;
  // rotational inertia
  double iz_;
  // the ratio between the turn of the steering wheel and the turn of the wheels
  double steer_transmission_ratio_;
  // the maximum turn of steer
  double steer_single_direction_max_degree_;

  // limit steering to maximum theoretical lateral acceleration
  double max_lat_acc_;
  //
  double max_acceleration_;
  //
  double max_deceleration_;
  //
  double throttle_deadzone_;
  //
  double brake_deadzone_;
  //
  double minimum_speed_protection_;
  //
  double maxFrontWheelAngleVelocity_;
  //
  double maxSteeringAngle;
  //
  double standstill_acceleration_;
  //
  const int basic_state_size_ = 6;

  const int controls_ = 2;

  const int horizon_ = 8;//apollo&matlab 10
  // vehicle state matrix
  Eigen::MatrixXd matrix_a_;
  // vehicle state matrix (discrete-time)
  Eigen::MatrixXd matrix_ad_;

  // control matrix
  Eigen::MatrixXd matrix_b_;
  // control matrix (discrete-time)
  Eigen::MatrixXd matrix_bd_;

  // offset matrix
  Eigen::MatrixXd matrix_c_;
  // offset matrix (discrete-time)
  Eigen::MatrixXd matrix_cd_;

  // gain matrix
  Eigen::MatrixXd matrix_k_;
  // control authority weighting matrix
  Eigen::MatrixXd matrix_r_;
  // updated control authority weighting matrix
  Eigen::MatrixXd matrix_r_updated_;
  // state weighting matrix
  Eigen::MatrixXd matrix_q_;
  // updated state weighting matrix
  Eigen::MatrixXd matrix_q_updated_;
  // vehicle state matrix coefficients
  Eigen::MatrixXd matrix_a_coeff_;
  // 4 by 1 matrix; state matrix
  Eigen::MatrixXd matrix_state_;

  // heading error of last control cycle
  double previous_heading_error_ = 0.0;
  // lateral distance to reference trajectory of last control cycle
  double previous_lateral_error_ = 0.0;
  // parameters for mpc solver; number of iterations
  int mpc_max_iteration_ = 0;
  // parameters for mpc solver; threshold for computation
  double mpc_eps_ = 0.0;

  double linear_velocity = 0.0;

  common::DigitalFilter digital_filter_;

  // MeanFilter heading_error_filter_;
  common::MeanFilter lateral_error_filter_;
  // MeanFilter lateral_error_filter;
  common::MeanFilter heading_error_filter_;
  // for logging purpose
  std::ofstream mpc_log_file_;

  const std::string name_;

  bool FLAGS_enable_csv_debug = true;//debug log flag
};

}
}

#endif // MPC_CONTROLLER_H
