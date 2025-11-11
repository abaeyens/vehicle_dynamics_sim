#ifndef VEHICLE_DYNAMICS_SIM_CONVERSIONS_H
#define VEHICLE_DYNAMICS_SIM_CONVERSIONS_H
#include <string_view>

#include <rclcpp/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <vehicle_dynamics_sim/Pose2D.h>

namespace vehicle_dynamics_sim
{
/// Pose2D => ROS TransformStamped
geometry_msgs::msg::TransformStamped to_transformstamped(
  const Pose2D & pose, const rclcpp::Time & time, const std::string_view & frame_id,
  const std::string_view & child_frame_id);

/// Pose2D + ROS Twist => ROS Odometry
nav_msgs::msg::Odometry to_odometry(
  const Pose2D & pose, const geometry_msgs::msg::Twist & twist, const rclcpp::Time & time,
  const std::string_view & frame_id, const std::string_view & child_frame_id);
}  // namespace vehicle_dynamics_sim
#endif  // VEHICLE_DYNAMICS_SIM_CONVERSIONS_H
