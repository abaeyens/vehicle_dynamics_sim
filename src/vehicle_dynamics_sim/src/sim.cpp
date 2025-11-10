#include <vehicle_dynamics_sim/declare_and_get_parameter.h>

#include <cmath>
#include <memory>
#include <stdexcept>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fmt/format.h>

#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <vehicle_dynamics_sim/sim.h>
#include <vehicle_dynamics_sim/vehicles.h>

namespace sim
{

SimNode::SimNode()
: rclcpp::Node("sim_node"),
  step_rate_(declare_and_get_parameter(*this, "step_rate", 1000.0)),
  pub_rate_(declare_and_get_parameter(*this, "pub_rate", 50.0)),
  reference_twist_max_oldness_(
    declare_and_get_parameter(*this, "reference_twist_max_oldness", 1.0)),
  be_reference_clock_(declare_and_get_parameter_bool(*this, "be_reference_clock", false))
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

  // Subscribers
  sub_twist_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "twist_reference", 10, [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
      this->reference_twist_ = *msg;
    });

  // Publishers
  if (be_reference_clock_) {
    pub_clock_ = this->create_publisher<builtin_interfaces::msg::Time>("clock", 50);
  }
  pub_tf_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);
  pub_tf_static_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf_static", 10);
  pub_twist_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("twist_actual", 10);

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

void SimNode::tick_simulation()
{
  // Update time
  // TODO using now() creates quite a bit of jitter
  // can we get the time from the timer?
  time_ = be_reference_clock_ ? time_ + rclcpp::Duration(0, static_cast<int32_t>(1e9 / step_rate_))
                              : this->get_clock()->now();

  // If reference twist too old, substitute with zero
  if ((time_ - reference_twist_.header.stamp).seconds() > reference_twist_max_oldness_)
    reference_twist_ = geometry_msgs::msg::TwistStamped{};

  // Simulate the vehicle
  vehicle_->update(time_, reference_twist_);

  // Clock
  if (be_reference_clock_ && pub_clock_)
    pub_clock_->publish(static_cast<builtin_interfaces::msg::Time>(time_));
  // And finally, as applicable, publish the new simulation state
  if ((time_ - previous_pub_time_).seconds() > 1.0 / pub_rate_) {
    previous_pub_time_ = time_;
    // Pose (over tf)
    {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = time_;
      tf.header.frame_id = "map";
      tf.child_frame_id = "base_link";
      const auto [position, heading] = vehicle_->get_pose();
      tf.transform.translation.x = position.x();
      tf.transform.translation.y = position.y();
      tf.transform.translation.z = 0;
      const Eigen::Quaterniond orientation =
        Eigen::Quaterniond(Eigen::AngleAxisd(heading, Eigen::Vector3d::UnitZ()));
      tf.transform.rotation.x = orientation.x();
      tf.transform.rotation.y = orientation.y();
      tf.transform.rotation.z = orientation.z();
      tf.transform.rotation.w = orientation.w();
      tf2_msgs::msg::TFMessage msg;
      msg.transforms.push_back(tf);
      pub_tf_->publish(msg);
    }
  }
}
}  // namespace sim
