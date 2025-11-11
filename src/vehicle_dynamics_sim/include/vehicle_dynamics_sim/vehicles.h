#ifndef VEHICLE_DYNAMICS_SIM_VEHICLES_H
#define VEHICLE_DYNAMICS_SIM_VEHICLES_H

#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <string_view>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <vehicle_dynamics_sim/localization.h>
#include <vehicle_dynamics_sim/Pose2D.h>

namespace vehicle_dynamics_sim
{
enum class VehicleName : uint8_t
{
  BICYCLE,
  DIFFERENTIAL,
  OMNI,
};

std::string toString(const VehicleName & name);
VehicleName toVehicleName(const std::string_view & name);

class ModelBase
{
protected:
  rclcpp::Time time_{static_cast<int64_t>(0), RCL_ROS_TIME};
};

class DeadTimeDelay : public ModelBase
{
public:
  DeadTimeDelay(rclcpp::Node & node, const std::string & ns);
  void enter(const rclcpp::Time & time, const double value);
  double get(const rclcpp::Time & time);

private:
  // Parameters
  const double dead_time_;
  // State
  // oldest value first, newest last (timestamps increasing)
  std::deque<std::pair<rclcpp::Time, double>> queue_;
};

class DriveActuator : public ModelBase
{
public:
  DriveActuator(rclcpp::Node & node, const std::string & ns);
  inline double get_max_velocity() const { return max_velocity_; }
  double get_new_velocity(const rclcpp::Time & time, const double & reference_velocity);

private:
  // Parameters
  const double max_velocity_;
  const double time_constant_;
  const double max_acceleration_;
  // Submodels
  DeadTimeDelay deadTimeDelay_;
  // State
  double prev_velocity_ = 0;
};

class SteeringActuator : public ModelBase
{
public:
  SteeringActuator(rclcpp::Node & node, const std::string & ns);
  inline double get_max_position() const { return max_position_; }
  double get_new_position(const rclcpp::Time & time, const double & reference_position);

private:
  // Parameters
  const double max_position_;
  const double time_constant_;
  const double max_velocity_;
  // Submodels
  DeadTimeDelay deadTimeDelay_;
  // State
  double prev_position_ = 0;
};

class Vehicle : public ModelBase
{
public:
  Vehicle(rclcpp::Node & node, const std::string & ns);
  std::string name() const;
  Pose2D get_pose() const;
  inline double get_base_link_offset() const { return base_link_offset_; }
  virtual void update(
    const rclcpp::Time & time, const geometry_msgs::msg::TwistStamped & reference_twist) = 0;
  inline geometry_msgs::msg::TwistStamped get_actual_twist() { return actual_twist_; }

protected:
  void store_actual_twist(
    const rclcpp::Time & time, const double vx, const double vy, const double oz);

  // Parameters
  const double base_link_offset_;
  // State
  Eigen::Vector2d position_ = Eigen::Vector2d::Zero();
  double heading_ = 0;
  // For publishing thereafter
  geometry_msgs::msg::TwistStamped actual_twist_;
};

class BicycleVehicle : public Vehicle
{
public:
  BicycleVehicle(rclcpp::Node & node, const std::string & ns);

  void update(
    const rclcpp::Time & time, const geometry_msgs::msg::TwistStamped & reference_twist) override;

private:
  // Params
  const double wheel_base_;
  const bool drive_on_steered_wheel_;
  const bool reverse_;
  // Actuators
  DriveActuator drive_actuator_;
  SteeringActuator steering_actuator_;
};

class DifferentialVehicle : public Vehicle
{
public:
  DifferentialVehicle(rclcpp::Node & node, const std::string & ns);

  void update(
    const rclcpp::Time & time, const geometry_msgs::msg::TwistStamped & reference_twist) override;

private:
  // Params
  const double track_;
  // Actuators
  DriveActuator drive_actuator_left_;
  DriveActuator drive_actuator_right_;
};

std::unique_ptr<Vehicle> toModelBase(
  const VehicleName model, rclcpp::Node & node, const std::string & ns);
}  // namespace vehicle_dynamics_sim
#endif  // VEHICLE_DYNAMICS_SIM_VEHICLES_H
