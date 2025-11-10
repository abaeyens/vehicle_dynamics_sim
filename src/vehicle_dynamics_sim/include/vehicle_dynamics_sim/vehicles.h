#ifndef VEHICLE_DYNAMICS_SIM_VEHICLES_H
#define VEHICLE_DYNAMICS_SIM_VEHICLES_H

#include <cstdint>
#include <memory>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

namespace sim
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
public:
  ModelBase(rclcpp::Node & node) : node_(node) {}
  virtual ~ModelBase() = default;

protected:
  rclcpp::Node & node_;
  rclcpp::Time time_{static_cast<int64_t>(0), RCL_ROS_TIME};
};

class DriveActuator : public ModelBase
{
public:
  DriveActuator(rclcpp::Node & node, const std::string & ns);

  double get_new_velocity(const rclcpp::Time & time, const double & reference_velocity);

private:
  const double dead_time_;
  // double time_constant_;
  // double max_acceleration_;
  // double max_velocity_;
};

class SteeringActuator : public ModelBase
{
public:
  SteeringActuator(rclcpp::Node & node, const std::string & ns);

  double get_new_position(const rclcpp::Time & time, const double & reference_position);

private:
  // double dead_time_;
  // double time_constant_;
  // double max_velocity_;
  // double max_position_;
};

class Vehicle : public ModelBase
{
public:
  Vehicle(rclcpp::Node & node, const std::string & ns);
  std::pair<Eigen::Vector2d, double> get_pose();
  inline double get_base_link_offset() { return base_link_offset_; }
  virtual void update(
    const rclcpp::Time & time, const geometry_msgs::msg::TwistStamped & reference_twist) = 0;

protected:
  void store_actual_twist(
    const rclcpp::Time & time, const double vx, const double vy, const double oz);

  // Parameters
  const double base_link_offset_;
  // State
  Eigen::Vector2d position_;
  double heading_;
  // For publishing thereafter
  geometry_msgs::msg::TwistStamped actual_twist_;
};

class BicycleModel : public Vehicle
{
public:
  BicycleModel(rclcpp::Node & node, const std::string & ns);

  void update(
    const rclcpp::Time & time, const geometry_msgs::msg::TwistStamped & reference_twist) override;

private:
  // Params
  const double wheel_base_;
  const bool reverse_;
  // Actuators
  DriveActuator drive_actuator_;
  SteeringActuator steering_actuator_;
};

std::unique_ptr<Vehicle> toModelBase(
  const VehicleName model, rclcpp::Node & node, const std::string & ns);
}  // namespace sim
#endif  // VEHICLE_DYNAMICS_SIM_VEHICLES_H
