#ifndef VEHICLE_DYNAMICS_SIM_SIM_H
#define VEHICLE_DYNAMICS_SIM_SIM_H

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <vehicle_dynamics_sim/vehicles.h>

namespace vehicle_dynamics_sim
{
class SimNode : public rclcpp::Node
{
public:
  SimNode();

private:
  void store_twist_reference(geometry_msgs::msg::Twist::SharedPtr msg);
  void store_twist_reference(geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void tick_simulation();

  // Params
  const double step_rate_;
  const double pub_rate_;
  const double twist_reference_max_oldness_;
  const bool be_reference_clock_;

  // State
  std::unique_ptr<Vehicle> vehicle_;
  rclcpp::Time time_{static_cast<int64_t>(0), RCL_ROS_TIME};
  rclcpp::Time previous_pub_time_{static_cast<int64_t>(0), RCL_ROS_TIME};
  geometry_msgs::msg::TwistStamped twist_reference_;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_twist_stamped_;

  // Publishers
  rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr pub_clock_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_tf_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_tf_static_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;

  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace vehicle_dynamics_sim
#endif  // VEHICLE_DYNAMICS_SIM_SIM_H
