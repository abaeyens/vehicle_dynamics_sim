#include <vehicle_dynamics_sim/conversions.h>

#include <string_view>

#include <Eigen/Dense>

#include <rclcpp/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <vehicle_dynamics_sim/Pose2D.h>

namespace vehicle_dynamics_sim
{
geometry_msgs::msg::TransformStamped to_transformstamped(
  const Pose2D & pose, const rclcpp::Time & stamp, const std::string_view & frame_id,
  const std::string_view & child_frame_id)
{
  const Eigen::Quaterniond orientation =
    Eigen::Quaterniond(Eigen::AngleAxisd(pose.theta(), Eigen::Vector3d::UnitZ()));

  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.child_frame_id = child_frame_id;
  msg.transform.translation.x = pose.x();
  msg.transform.translation.y = pose.y();
  msg.transform.translation.z = 0;
  msg.transform.rotation.x = orientation.x();
  msg.transform.rotation.y = orientation.y();
  msg.transform.rotation.z = orientation.z();
  msg.transform.rotation.w = orientation.w();
  return msg;
}

nav_msgs::msg::Odometry to_odometry(
  const Pose2D & pose, const geometry_msgs::msg::Twist & twist, const rclcpp::Time & time,
  const std::string_view & frame_id, const std::string_view & child_frame_id)
{
  const Eigen::Quaterniond orientation =
    Eigen::Quaterniond(Eigen::AngleAxisd(pose.theta(), Eigen::Vector3d::UnitZ()));

  nav_msgs::msg::Odometry msg;
  msg.header.stamp = time;
  msg.header.frame_id = frame_id;
  msg.child_frame_id = child_frame_id;
  msg.pose.pose.position.x = pose.x();
  msg.pose.pose.position.y = pose.y();
  msg.pose.pose.position.z = 0;
  msg.pose.pose.orientation.x = orientation.x();
  msg.pose.pose.orientation.y = orientation.y();
  msg.pose.pose.orientation.z = orientation.z();
  msg.pose.pose.orientation.w = orientation.w();
  msg.twist.twist = twist;
  return msg;
}
}  // namespace vehicle_dynamics_sim
