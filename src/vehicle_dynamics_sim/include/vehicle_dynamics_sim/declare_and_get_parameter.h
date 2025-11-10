#ifndef VEHICLE_DYNAMICS_SIM_DECLARE_AND_GET_PARAMETER_H
#define VEHICLE_DYNAMICS_SIM_DECLARE_AND_GET_PARAMETER_H

#include <cstdint>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace sim
{

double declare_and_get_parameter(
  rclcpp::Node & node, const std::string & key, const double & default_value);

int64_t declare_and_get_parameter(
  rclcpp::Node & node, const std::string & key, const int64_t & default_value);

bool declare_and_get_parameter_bool(
  rclcpp::Node & node, const std::string & key, const bool & default_value);

std::string declare_and_get_parameter(
  rclcpp::Node & node, const std::string & key, const std::string & default_value);
}  // namespace sim
#endif  // VEHICLE_DYNAMICS_SIM_DECLARE_AND_GET_PARAMETER_H
