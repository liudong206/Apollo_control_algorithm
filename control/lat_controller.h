#ifndef LQR_CONTROLLER_H
#define LQR_CONTROLLER_H
#include <memory>
#include <fstream>
#include <string>
#include <iostream>
#include <time.h>
#include <ctime>
#include <cmath>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/LU"
#include "control/controller.h"
#include "filters/digital_filter.h"
#include "filters/mean_filter.h"
#include "filters/kalman_filter.h"
#include "filters/digital_filter_coefficients.h"
#include "common/log.h"
#include "vehicle_info/vehicle_params.h"
#include "vehicle_info/vehicle_state.h"
#include "interpolation_1d/interpolation_1d.h"

namespace control {
namespace lqr {

class LatController : public Controller{
public:
    /**
     * @brief constructor
     */
  LatController();
    /**
     * @brief destructor
     */
  virtual ~LatController();
    /**
     * @brief initialize LQR Controller
     * @param control_conf control configurations
     * @return Status initialization status
     */
    //common::Status Init(const ControlConf *control_conf) override;
  bool Init(const ControlConf *control_conf);

  int ComputeControlCommand(const std::vector<TrajectoryPoint>* planning_trajectory,
                            SimpleDebug *debug,
                            const VehicleState &vehicle_state);

protected:

  void UpdateStateAnalyticalMatching(const TrajectoryPoint &closet_point,
                                     SimpleDebug *debug,
                                     const VehicleState &vehicle_state);

  void UpdateMatrix();

  void UpdateMatrixCompound();

  double ComputeFeedForward(double ref_curvature) const;

  void ComputeLateralErrors(const TrajectoryPoint &ref_point,
                            SimpleDebug *debug,
                            const VehicleState &vehicle_state);
  bool LoadControlConf();

  void LoadLatGainScheduler();

  void InitializeFilters(const ControlConf *control_conf);

  double LimitSteerAngleByVelocity(double steer_cmd,double max_angle_vel,double time_step);

  void ProcessLogs(const SimpleDebug *debug);

  void LogInitParameters();
  //  void ProcessLogs(const SimpleLateralDebug *debug,
  //                   const canbus::Chassis *chassis);

  void CloseLogFile();

  // the following parameters are vehicle physics related.
  // control time interval
  double ts_ = 0.0;
  // corner stiffness; front
  double cf_ = 0.0;
  // corner stiffness; rear
  double cr_ = 0.0;
  // distance between front and rear wheel center
  double wheelbase_ = 0.0;
  // mass of the vehicle
  double mass_ = 0.0;
  // distance from front wheel center to COM
  double lf_ = 0.0;
  // distance from rear wheel center to COM
  double lr_ = 0.0;
  // rotational inertia
  double iz_ = 0.0;
  //car_width
  double carwidth_= 0.0;
  //rotational inertia weight
  double kz_ = 0.0;
  // the ratio between the turn of the steering wheel and the turn of the wheels
  double steer_transmission_ratio_ = 0.0;
  // the maximum turn of steer
  double steer_single_direction_max_degree_ = 0.0;
  // limit steering to maximum theoretical lateral acceleration
  double max_lat_acc_ = 0.0;
  // number of control cycles look ahead (preview controller)
  int preview_window_ = 0;
  // number of states without previews, includes
  // lateral error, lateral error rate, heading error, heading error rate
  const int basic_state_size_ = 4;
  // vehicle state matrix
  Eigen::MatrixXd matrix_a_;
  // vehicle state matrix (discrete-time)
  Eigen::MatrixXd matrix_ad_;
  // vehicle state matrix compound; related to preview
  Eigen::MatrixXd matrix_adc_;
  // control matrix
  Eigen::MatrixXd matrix_b_;
  // control matrix (discrete-time)
  Eigen::MatrixXd matrix_bd_;
  // control matrix compound
  Eigen::MatrixXd matrix_bdc_;
  // gain matrix
  Eigen::MatrixXd matrix_k_;
  // control authority weighting matrix
  Eigen::MatrixXd matrix_r_;
  // state weighting matrix
  Eigen::MatrixXd matrix_q_;
  // updated state weighting matrix
  Eigen::MatrixXd matrix_q_updated_;
  // vehicle state matrix coefficients
  Eigen::MatrixXd matrix_a_coeff_;
  // 4 by 1 matrix; state matrix
  Eigen::MatrixXd matrix_state_;

  // parameters for lqr solver; number of iterations
  int lqr_max_iteration_ = 0;
  // parameters for lqr solver; threshold for computation
  double lqr_eps_ = 0.0;

  common::DigitalFilter digital_filter_;
  // MeanFilter heading_error_filter_;
  common::MeanFilter lateral_error_filter_;
  // MeanFilter lateral_error_filter;
  common::MeanFilter heading_error_filter_;
  //kalman filter
  kalman_state state;
  // for logging purpose
  std::ofstream steer_log_file_;

  std::unique_ptr<Interpolation1D> lat_err_interpolation_;

  std::unique_ptr<Interpolation1D> heading_err_interpolation_;

  bool FLAGS_enable_csv_debug = true;//debug log flag

  const std::string name_;

  double pre_steer_angle_ = 0.0;

  double minimum_speed_protection_ = 0.1;
  //
  double maxFrontWheelAngleVelocity_;

  double current_trajectory_timestamp_ = -1.0;

  double init_vehicle_x_ = 0.0;

  double init_vehicle_y_ = 0.0;

  double init_vehicle_heading_ = 0.0;

  double linear_velocity = 0;

};//Controller

}
}//namespace

#endif // LQR_CONTROLLER_H
