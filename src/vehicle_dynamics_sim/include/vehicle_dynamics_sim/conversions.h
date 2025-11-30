#ifndef VEHICLE_DYNAMICS_SIM__CONVERSIONS_H_
#define VEHICLE_DYNAMICS_SIM__CONVERSIONS_H_
#include <string_view>

#include <rclcpp/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <vehicle_dynamics_sim/Pose2D.h>

namespace vehicle_dynamics_sim
{
/// Pose2D => ROS TransformStamped
[[nodiscard]] geometry_msgs::msg::TransformStamped to_transformstamped(
  const Pose2D & pose, const rclcpp::Time & time, std::string_view frame_id,
  std::string_view child_frame_id);

/// Pose2D + ROS Twist => ROS Odometry
[[nodiscard]] nav_msgs::msg::Odometry to_odometry(
  const Pose2D & pose, const geometry_msgs::msg::Twist & twist, const rclcpp::Time & time,
  std::string_view frame_id, std::string_view child_frame_id);
}  // namespace vehicle_dynamics_sim
#endif  // VEHICLE_DYNAMICS_SIM__CONVERSIONS_H_
