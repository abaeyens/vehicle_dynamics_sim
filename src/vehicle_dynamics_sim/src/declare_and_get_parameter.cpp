#include <vehicle_dynamics_sim/declare_and_get_parameter.h>

#include <cstdint>

namespace vehicle_dynamics_sim
{
double declare_and_get_parameter(rclcpp::Node & node, const std::string & key, double default_value)
{
  if (!node.has_parameter(key)) node.declare_parameter<double>(key, default_value);
  return node.get_parameter(key).as_double();
}

int64_t declare_and_get_parameter(
  rclcpp::Node & node, const std::string & key, int64_t default_value)
{
  if (!node.has_parameter(key)) node.declare_parameter<int64_t>(key, default_value);
  return node.get_parameter(key).as_int();
}

bool declare_and_get_parameter(rclcpp::Node & node, const std::string & key, bool default_value)
{
  if (!node.has_parameter(key)) node.declare_parameter<bool>(key, default_value);
  return node.get_parameter(key).as_bool();
}

std::string declare_and_get_parameter(
  rclcpp::Node & node, const std::string & key, const std::string & default_value)
{
  if (!node.has_parameter(key)) node.declare_parameter<std::string>(key, default_value);
  return node.get_parameter(key).as_string();
}
}  // namespace vehicle_dynamics_sim
