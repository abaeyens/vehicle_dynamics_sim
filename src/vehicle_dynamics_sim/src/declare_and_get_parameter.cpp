#include <vehicle_dynamics_sim/declare_and_get_parameter.h>

#include <cstdint>

namespace sim
{
double declare_and_get_parameter(
  rclcpp::Node & node, const std::string & key, const double & default_value)
{
  node.declare_parameter<double>(key, default_value);
  return node.get_parameter(key).as_double();
}

int64_t declare_and_get_parameter(
  rclcpp::Node & node, const std::string & key, const int64_t & default_value)
{
  node.declare_parameter<int64_t>(key, default_value);
  return node.get_parameter(key).as_int();
}

bool declare_and_get_parameter_bool(
  rclcpp::Node & node, const std::string & key, const bool & default_value)
{
  node.declare_parameter<bool>(key, default_value);
  return node.get_parameter(key).as_bool();
}

std::string declare_and_get_parameter(
  rclcpp::Node & node, const std::string & key, const std::string & default_value)
{
  node.declare_parameter<std::string>(key, default_value);
  return node.get_parameter(key).as_string();
}
}  // namespace sim