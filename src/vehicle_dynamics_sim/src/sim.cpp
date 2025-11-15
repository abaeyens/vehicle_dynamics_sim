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
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <vehicle_dynamics_sim/conversions.h>
#include <vehicle_dynamics_sim/localization.h>
#include <vehicle_dynamics_sim/Pose2D.h>
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
  be_reference_clock_(declare_and_get_parameter(*this, "be_reference_clock", false)),
  simulate_localization_(declare_and_get_parameter(*this, "simulate_localization", true))
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

  // Load localization and configure it
  if (simulate_localization_) {
    localization_ = std::make_unique<Localization>(*this, "localization");
    RCLCPP_INFO(
      this->get_logger(),
      "Set up localization simulation, will publish odom frame and noisy measurements.");
  } else {
    RCLCPP_INFO(
      this->get_logger(),
      "Localization will be ideal (no noise + 'odom' frame fixed to 'map' frame).");
  }

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
  pub_joint_states_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  pub_robot_description_ = this->create_publisher<std_msgs::msg::String>(
    "/robot_description", rclcpp::QoS(1).transient_local());
  pub_twist_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("twist_actual", 10);
  pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

  // Timer, which will tick the simulator at step_rate_
  timer_ = this->create_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / step_rate_)),
    std::bind(&SimNode::tick_simulation, this));

  if (!simulate_localization_) {
    tf2_msgs::msg::TFMessage msg;
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = "map";
    tf.child_frame_id = "odom";
    msg.transforms.push_back(tf);
    pub_tf_static_->publish(msg);
  }

  // Publish robot description
  // used by the `robot_state_publisher` node to publish static and dynamic tfs,
  // and by RViz to visualize the vehicle
  {
    std_msgs::msg::String msg;
    msg.data = vehicle_->get_robot_description();
    if (!msg.data.empty()) pub_robot_description_->publish(msg);
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
  const double & offset = vehicle_->get_base_link_offset();
  if (offset != 0) {
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
    if (simulate_localization_) {
      // Pose (over tf)
      if (!localization_) throw std::runtime_error("Localization not initialized");
      const auto [T_M_O, T_O_B] = localization_->update(time_, vehicle_->get_pose());
      const geometry_msgs::msg::TransformStamped tf_M_O =
        to_transformstamped(T_M_O, time_, "map", "odom");
      const geometry_msgs::msg::TransformStamped tf_O_B =
        to_transformstamped(T_O_B, time_, "odom", "base_link");
      tf2_msgs::msg::TFMessage msg;
      msg.transforms.reserve(2);
      msg.transforms.push_back(tf_M_O);
      msg.transforms.push_back(tf_O_B);
      pub_tf_->publish(msg);
      // Odometry
      // TODO simulate noise on velocity measurement here?
      pub_odom_->publish(
        to_odometry(T_O_B, vehicle_->get_actual_twist().twist, time_, "odom", "base_link"));
    } else {
      // Pose (over tf)
      const Pose2D T_M_B = vehicle_->get_pose();
      const geometry_msgs::msg::TransformStamped tf_M_B =
        to_transformstamped(T_M_B, time_, "map", "base_link");
      tf2_msgs::msg::TFMessage msg;
      msg.transforms.push_back(tf_M_B);
      pub_tf_->publish(msg);
      // Odometry (= twist + pose, Nav2 wants/needs this)
      pub_odom_->publish(
        to_odometry(T_M_B, vehicle_->get_actual_twist().twist, time_, "map", "base_link"));
    }
    // Joint states
    {
      const sensor_msgs::msg::JointState msg = vehicle_->get_joint_states();
      if (!msg.name.empty()) pub_joint_states_->publish(msg);
    }
  }
}
}  // namespace vehicle_dynamics_sim
