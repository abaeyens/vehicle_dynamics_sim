#ifndef VEHICLE_DYNAMICS_SIM__SIM_H_
#define VEHICLE_DYNAMICS_SIM__SIM_H_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <vehicle_dynamics_sim/vehicles.hpp>

namespace vehicle_dynamics_sim
{
/**
 * @brief Main ROS 2 node for vehicle dynamics simulation.
 * 
 * Simulates vehicle motion based on commanded velocities, publishes:
 * - TF transforms (map→odom→base_link or map→base_link depending on config)
 * - Odometry (pose + velocity)
 * - Joint states (for visualization)
 * - Actual twist (simulated velocity after actuator dynamics)
 * - Robot description (URDF)
 * - Clock (optional, when be_reference_clock=true)
 * 
 * Supports two localization modes:
 * 1. Ideal: Perfect localization, odom frame fixed to map
 * 2. Simulated: Noisy localization with drifting odom frame
 * 
 * Can operate as reference clock (be_reference_clock),
 * follow external clock (use_sim_time)
 * or run with wall time (both above params disabled).
 */
class SimNode : public rclcpp::Node
{
public:
  /**
   * @brief Construct and configure the simulation node.
   * 
   * Loads parameters, instantiates vehicle model, sets up publishers/subscribers,
   * and starts simulation timer.
   * 
   * Parameters:
   * - step_rate: Simulation frequency [Hz] (default: 1000)
   * - pub_rate: Publishing frequency [Hz] (default: 50)
   * - twist_reference_max_oldness: Max age for twist commands [s] (default: 1.0)
   * - be_reference_clock: Publish clock messages (default: false)
   * - simulate_localization: Enable localization simulation with noise (default: true)
   * - model: Vehicle type ("bicycle", "differential", "omni")
   * - model_params.*: Vehicle-specific parameters
   * - localization.*: Localization simulation parameters (if enabled)
   * 
   * @throws std::runtime_error if both use_sim_time and be_reference_clock are true
   */
  SimNode();

private:
  /**
   * @brief Store incoming twist command (unstamped).
   * 
   * Stamps message with current simulation time and transforms to vehicle reference frame
   * by compensating for base_link_offset.
   * 
   * @param msg Commanded velocity
   */
  void store_twist_reference(const geometry_msgs::msg::Twist::SharedPtr & msg);

  /**
   * @brief Store incoming twist command (stamped).
   * 
   * Transforms to vehicle reference frame by compensating for base_link_offset.
   * 
   * @param msg Commanded velocity with timestamp
   */
  void store_twist_reference(const geometry_msgs::msg::TwistStamped::SharedPtr & msg);

  /**
   * @brief Main simulation loop callback (runs at step_rate).
   * 
   * Updates time, checks twist command freshness, simulates vehicle dynamics,
   * and publishes outputs at pub_rate.
   */
  void tick_simulation();

  // Parameters
  const double step_rate_;                    ///< Simulation update frequency [Hz]
  const double pub_rate_;                     ///< Output publishing frequency [Hz]
  const double twist_reference_max_oldness_;  ///< Max age for twist commands before zeroing [s]
  const bool be_reference_clock_;             ///< Publish clock messages (simulation time source)
  const bool simulate_localization_;          ///< Enable localization simulation with noise

  // State
  std::unique_ptr<Vehicle> vehicle_{nullptr};                 ///< Vehicle dynamics model
  std::unique_ptr<Localization> localization_{nullptr};       ///< Localization simulator (optional)
  rclcpp::Time time_{static_cast<int64_t>(0), RCL_ROS_TIME};  ///< Current simulation time
  rclcpp::Time previous_pub_time_{static_cast<int64_t>(0), RCL_ROS_TIME};  ///< Last publish time
  geometry_msgs::msg::TwistStamped twist_reference_{};  ///< Most recent twist command

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;  ///< "twist_reference"
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
    sub_twist_stamped_;  ///< "twist_stamped_reference"

  // Publishers
  rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr pub_clock_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_tf_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_tf_static_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_robot_description_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;

  rclcpp::TimerBase::SharedPtr timer_;  ///< Simulation loop timer (runs at step_rate)
};
}  // namespace vehicle_dynamics_sim
#endif  // VEHICLE_DYNAMICS_SIM__SIM_H_
