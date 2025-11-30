#ifndef VEHICLE_DYNAMICS_SIM__DECLARE_AND_GET_PARAMETER_H_
#define VEHICLE_DYNAMICS_SIM__DECLARE_AND_GET_PARAMETER_H_

#include <cstdint>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace vehicle_dynamics_sim
{

double declare_and_get_parameter(
  rclcpp::Node & node, const std::string & key, double default_value);

int64_t declare_and_get_parameter(
  rclcpp::Node & node, const std::string & key, int64_t default_value);

bool declare_and_get_parameter(rclcpp::Node & node, const std::string & key, bool default_value);

std::string declare_and_get_parameter(
  rclcpp::Node & node, const std::string & key, const std::string & default_value);

// Delete overloads for common implicit conversions to prevent casting mistakes
template <typename T>
T declare_and_get_parameter(rclcpp::Node & node, const std::string & key, T default_value) = delete;

// Explicitly allow const char* for string overload
inline std::string declare_and_get_parameter(
  rclcpp::Node & node, const std::string & key, const char * default_value)
{
  return declare_and_get_parameter(node, key, std::string(default_value));
}
}  // namespace vehicle_dynamics_sim
#endif  // VEHICLE_DYNAMICS_SIM__DECLARE_AND_GET_PARAMETER_H_
