
/**
 * @file vehicle_state.h
 *
 * @brief Declaration of the class VehicleState.
 */
#ifndef __VEHICLE_STATE_H_
#define __VEHICLE_STATE_H_

#include "vehicle_info/vehicle_params.h"
/**
 * @class VehicleState
 * @brief The class of vehicle state.
 *        It includes basic information and computation
 *        about the state of the vehicle.
 */
class VehicleState {
 public:
  /**
   * @brief Empty constructor.
   */
  VehicleState() = default;
  VehicleState(double x , double y , double heading , double v);

  /**
   * @brief Constructor only by information of localization.
   * @param localization Localization information of the vehicle.
   */


  /**
   * @brief Get the x-coordinate of vehicle position.
   * @return The x-coordinate of vehicle position.
   */
  double x() const;

  /**
   * @brief Get the y-coordinate of vehicle position.
   * @return The y-coordinate of vehicle position.
   */
  double y() const;

  /**
   * @brief Get the z coordinate of vehicle position.
   * @return The z coordinate of vehicle position.
   */
  double z() const;

  /**
   * @brief Get the heading of vehicle position, which is the angle
   *        between the vehicle's heading direction and the x-axis.
   * @return The angle between the vehicle's heading direction
   *         and the x-axis.
   */
  double heading() const;

  /**
    *@brief Get the vehicle's pitch angle.
    *@return pitch angle
    */
  double pitch() const;

  /**
    *@brief Get the vehicle's roll angle.
    *@return roll angle
    */
  double roll() const;

  /**
    *@brief Get the vehicle's yaw angle.
    *@return yaw angle
    */
  double yaw() const;
  /**
   * @brief Get the vehicle's linear velocity.
   * @return The vehicle's linear velocity.
   */
  double linear_velocity() const;

  /**
   * @brief Get the vehicle's angular velocity.
   * @return The vehicle's angular velocity.
   */
  double angular_velocity() const;

  /**
   * @brief Get the vehicle's linear acceleration.
   * @return The vehicle's linear acceleration.
   */
  double linear_acceleration() const;

  /**
   * @brief Get the vehicle's direction
   * @return The vehicle's direction 1:move forward  -1:move back 0:stop move
   */
  int direction() const;

  /**
   * @brief Set the x-coordinate of vehicle position.
   * @param x The x-coordinate of vehicle position.
   */
  void set_x(const double x);

  /**
   * @brief Set the y-coordinate of vehicle position.
   * @param y The y-coordinate of vehicle position.
   */
  void set_y(const double y);

  /**
   * @brief Set the z coordinate of vehicle position.
   * @param z The z coordinate of vehicle position.
   */
  void set_z(const double z);

  /**
   * @brief Set the heading of vehicle position, which is the angle
   *        between the vehicle's heading direction and the x-axis.
   * @param heading The angle between the vehicle's heading direction
   *         and the x-axis.
   */
  void set_heading(const double heading);

  /**
   * @brief Set the vehicle's linear velocity.
   * @param linear_velocity The value to set the vehicle's linear velocity.
   */
  void set_linear_velocity(const double linear_velocity);

  /**
   * @brief Set the vehicle's angular velocity.
   * @param angular_velocity The vehicle's angular velocity.
   */
  void set_angular_velocity(const double angular_velocity);

  /**
    *@brief Set the vehicle's pitch angle.
    *@param pitch angle
    */
  void set_pitch(const double pitch);

  /**
    *@brief Set the vehicle's roll angle.
    *@param roll angle
    */
  void set_roll(const double roll);

  /**
    *@brief Set the vehicle's yaw angle.
    *@param yaw angle
    */
  void set_yaw(const double yaw);

  /**
   *@brief update vehicle state.
   *@param current vehicle state , acclerate , vehicle front wheel angle ,
   *        time t , vehicle params.
   */
  void update(VehicleState &state,double acc,double delta,
              double t_,double Wheelbase);

private:
  double x_ = 0.0;

  double y_ = 0.0;

  double z_ = 0.0;

  double heading_ = 0.0;

  double linear_v_ = 0.0;

  double angular_v_ = 0.0;

  double linear_a_ = 0.0;

  double current_steer_ = 0.0;

  int direction_ = 0; //0:stand still 1: forward  -1:backward

  double pitch_ = 0.0;//俯仰角

  double roll_ = 0.0; // 侧倾角

  double yaw_ = 0.0; // 横摆角




  const VehicleParams vehicle_params;

};

#endif /* __VEHICLE_STATE_H_ */
