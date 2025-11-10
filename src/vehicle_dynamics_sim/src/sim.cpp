#include <vehicle_dynamics_sim/declare_and_get_parameter.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fmt/format.h>

#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <vehicle_dynamics_sim/sim.h>
#include <vehicle_dynamics_sim/vehicles.h>

namespace vehicle_dynamics_sim
{

SimNode::SimNode()
: rclcpp::Node("sim_node"),
  step_rate_(declare_and_get_parameter(*this, "step_rate", 1000.0)),
  pub_rate_(declare_and_get_parameter(*this, "pub_rate", 50.0)),
  twist_reference_max_oldness_(
    declare_and_get_parameter(*this, "twist_reference_max_oldness", 1.0)),
  be_reference_clock_(declare_and_get_parameter(*this, "be_reference_clock", false))
{
  // Validate clock configuration
  {
    const bool use_sim_time = this->get_parameter("use_sim_time").as_bool();
    RCLCPP_INFO(
      this->get_logger(),
      fmt::format("use_sim_time = {}, be_reference_clock = {}", use_sim_time, be_reference_clock_)
        .c_str());
    if (use_sim_time && be_reference_clock_) {
      throw std::runtime_error(
        "Cannot both listen to external clock and be reference clock. "
        "Set either or both 'use_sim_time' or 'be_reference_clock' to "
        "false.");
    }
  }
  // Load vehicle and configure it
  const VehicleName vehicle_name =
    toVehicleName(declare_and_get_parameter(*this, "model", toString(VehicleName::BICYCLE)));
  const std::string ns = "model_params." + toString(vehicle_name);
  vehicle_ = toModelBase(vehicle_name, *this, ns);
  RCLCPP_INFO(
    this->get_logger(), fmt::format("Instantiated vehicle of name {}", vehicle_->name()).c_str());

  // Subscribers
  sub_twist_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "twist_reference", 10,
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) { this->store_twist_reference(msg); });
  sub_twist_stamped_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "twist_stamped_reference", 10, [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
      this->store_twist_reference(msg);
    });

  // Publishers
  if (be_reference_clock_) {
    pub_clock_ = this->create_publisher<builtin_interfaces::msg::Time>("clock", 50);
  }
  pub_tf_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);
  pub_tf_static_ = this->create_publisher<tf2_msgs::msg::TFMessage>(
    "/tf_static", rclcpp::QoS(10).transient_local());
  pub_twist_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("twist_actual", 10);
  pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

  // Timer, which will tick the simulator at step_rate_
  timer_ = this->create_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / step_rate_)),
    std::bind(&SimNode::tick_simulation, this));

  // Publish rear axle frame as static transform
  {
    tf2_msgs::msg::TFMessage msg;
    {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.frame_id = "base_link";
      tf.child_frame_id = "rear_axle";
      tf.transform.translation.x = -vehicle_->get_base_link_offset();
      msg.transforms.push_back(tf);
    }
    pub_tf_static_->publish(msg);
  }
}

void SimNode::store_twist_reference(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  twist_reference_.header.stamp = time_;
  twist_reference_.twist = *msg;
  // Transform to base frame
  const double & offset = vehicle_->get_base_link_offset();
  if (offset != 0) {
    twist_reference_.twist.linear.y -= offset * twist_reference_.twist.angular.z;
  }
}

void SimNode::store_twist_reference(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  twist_reference_ = *msg;
  // Transform to base frame
  const double& offset = vehicle_->get_base_link_offset();
  if (offset != 0)
  {
    twist_reference_.twist.linear.y -= offset * twist_reference_.twist.angular.z;
  }
}

void SimNode::tick_simulation()
{
  // Update time
  // TODO using now() creates quite a bit of jitter
  // can we get the time from the timer?
  time_ = be_reference_clock_ ? time_ + rclcpp::Duration(0, static_cast<int32_t>(1e9 / step_rate_))
                              : this->get_clock()->now();

  // Check that clock type of time is ROS_TIME
  if (time_.get_clock_type() != RCL_ROS_TIME) {
    throw std::invalid_argument("BicycleVehicle::update: time must use ROS_TIME clock type");
  }

  // If reference twist too old, substitute with zero
  if ((time_ - twist_reference_.header.stamp).seconds() > twist_reference_max_oldness_)
    twist_reference_ = geometry_msgs::msg::TwistStamped{};

  // Simulate the vehicle
  vehicle_->update(time_, twist_reference_);

  // Clock
  if (be_reference_clock_ && pub_clock_)
    pub_clock_->publish(static_cast<builtin_interfaces::msg::Time>(time_));
  // And finally, as applicable, publish the new simulation state
  if ((time_ - previous_pub_time_).seconds() > 1.0 / pub_rate_) {
    previous_pub_time_ = time_;
    // Actual twist
    pub_twist_->publish(vehicle_->get_actual_twist());
    // Pose (over tf)
    const auto [position, heading] = vehicle_->get_pose();
    const Eigen::Quaterniond orientation =
      Eigen::Quaterniond(Eigen::AngleAxisd(heading, Eigen::Vector3d::UnitZ()));
    {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = time_;
      tf.header.frame_id = "map";
      tf.child_frame_id = "base_link";
      tf.transform.translation.x = position.x();
      tf.transform.translation.y = position.y();
      tf.transform.translation.z = 0;
      tf.transform.rotation.x = orientation.x();
      tf.transform.rotation.y = orientation.y();
      tf.transform.rotation.z = orientation.z();
      tf.transform.rotation.w = orientation.w();
      tf2_msgs::msg::TFMessage msg;
      msg.transforms.push_back(tf);
      pub_tf_->publish(msg);
    }
    // Odometry (= twist + pose, Nav2 wants/needs this)
    {
      nav_msgs::msg::Odometry msg;
      msg.header.stamp = time_;
      msg.header.frame_id = "map";
      msg.child_frame_id = "base_link";
      msg.pose.pose.position.x = position.x();
      msg.pose.pose.position.y = position.y();
      msg.pose.pose.position.z = 0;
      msg.pose.pose.orientation.x = orientation.x();
      msg.pose.pose.orientation.y = orientation.y();
      msg.pose.pose.orientation.z = orientation.z();
      msg.pose.pose.orientation.w = orientation.w();
      msg.twist.twist = vehicle_->get_actual_twist().twist;
      pub_odom_->publish(msg);
    }
  }
}
}  // namespace vehicle_dynamics_sim
