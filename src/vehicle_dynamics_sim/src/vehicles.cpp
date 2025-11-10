#include <vehicle_dynamics_sim/vehicles.h>

#include <cmath>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fmt/format.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <std_msgs/msg/string.hpp>

#include <vehicle_dynamics_sim/declare_and_get_parameter.h>
#include <vehicle_dynamics_sim/utils.h>

namespace sim
{
std::string toString(const VehicleName & name)
{
  switch (name) {
    case VehicleName::BICYCLE:
      return "bicycle";
    case VehicleName::DIFFERENTIAL:
      return "differential";
    case VehicleName::OMNI:
      return "omni";
  }
  throw std::invalid_argument(
    fmt::format("Unknown vehicle name enum value: {}", static_cast<int>(name)));
}

VehicleName toVehicleName(const std::string_view & name)
{
  if (name == "bicycle")
    return VehicleName::BICYCLE;
  else if (name == "differential")
    return VehicleName::DIFFERENTIAL;
  else if (name == "omni")
    return VehicleName::OMNI;
  throw std::invalid_argument(fmt::format("Unknown vehicle name: {}", name));
}

DriveActuator::DriveActuator(rclcpp::Node & node, const std::string & ns)
: ModelBase(node), dead_time_(declare_and_get_parameter(node, ns + ".dead_time", 0.0))
{
}

double DriveActuator::get_new_velocity(const rclcpp::Time & time, const double & reference_velocity)
{
  // TODO implement limits, LP filter etc.
  time_ = time;
  return reference_velocity;
}

SteeringActuator::SteeringActuator(rclcpp::Node & node, const std::string & ns) : ModelBase(node) {}

double SteeringActuator::get_new_position(
  const rclcpp::Time & time, const double & reference_position)
{
  // TODO implement limits, LP filter etc.
  time_ = time;
  return reference_position;
}

Vehicle::Vehicle(rclcpp::Node & node, const std::string & ns)
: ModelBase(node), base_link_offset_(declare_and_get_parameter(node, ns + ".base_link_offset", 0.0))
{
}

std::pair<Eigen::Vector2d, double> Vehicle::get_pose()
{
  const Eigen::Vector2d position =
    position_ + Eigen::Vector2d{std::cos(heading_), std::sin(heading_)} * base_link_offset_;
  return std::make_pair(position, heading_);
}

void Vehicle::store_actual_twist(
  const rclcpp::Time & time, const double vx, const double vy, const double oz)
{
  actual_twist_.header.stamp = time;
  actual_twist_.header.frame_id = "base_link";
  actual_twist_.twist.linear.x = vx;
  actual_twist_.twist.linear.y = vy + oz * base_link_offset_;
  actual_twist_.twist.linear.z = 0.0;
  actual_twist_.twist.angular.x = 0.0;
  actual_twist_.twist.angular.y = 0.0;
  actual_twist_.twist.angular.z = oz;
}

BicycleModel::BicycleModel(rclcpp::Node & node, const std::string & ns)
: Vehicle(node, ns),
  wheel_base_(declare_and_get_parameter(node, ns + ".wheel_base", 2.0)),
  reverse_(declare_and_get_parameter_bool(node, ns + ".reverse", false)),
  drive_actuator_(DriveActuator(node, ns + ".drive_actuator")),
  steering_actuator_(SteeringActuator(node, ns + ".steering_actuator"))
{
}

void BicycleModel::update(
  const rclcpp::Time & time, const geometry_msgs::msg::TwistStamped & reference_twist)
{
  const double timestep = (time - time_).seconds();
  const double forward_velocity =
    drive_actuator_.get_new_velocity(time, reference_twist.twist.linear.x);
  double wheel_angle = 0;
  if (std::abs(reference_twist.twist.linear.x) > 1e-12) {
    const double reference_steering_position =
      std::atan(wheel_base_ * reference_twist.twist.angular.z / reference_twist.twist.linear.x);
    wheel_angle = steering_actuator_.get_new_position(time, reference_steering_position);
  }
  const double angular_velocity = forward_velocity * std::tan(wheel_angle) / wheel_base_;
  const double new_heading = mod_2pi(heading_ + timestep * angular_velocity);
  const double mean_heading = mod_2pi(heading_ + mod_pi(new_heading - heading_));
  position_ +=
    Eigen::Vector2d{std::cos(mean_heading), std::sin(mean_heading)} * forward_velocity * timestep;
  heading_ = new_heading;
  store_actual_twist(time, forward_velocity, 0.0, angular_velocity);
  time_ = time;
}

std::unique_ptr<Vehicle> toModelBase(
  const VehicleName model, rclcpp::Node & node, const std::string & ns)
{
  switch (model) {
    case VehicleName::BICYCLE:
      return std::make_unique<BicycleModel>(node, ns);
    default:
      // TODO
      throw std::invalid_argument(
        fmt::format("Unsupported model name enum value: {}", static_cast<int>(model)));
      return nullptr;
  }
  throw std::invalid_argument(
    fmt::format("Unknown model name enum value: {}", static_cast<int>(model)));
}
}  // namespace sim
