#ifndef VEHICLE_DYNAMICS_SIM_VEHICLES_H
#define VEHICLE_DYNAMICS_SIM_VEHICLES_H

#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <string_view>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <vehicle_dynamics_sim/localization.h>
#include <vehicle_dynamics_sim/Pose2D.h>

namespace vehicle_dynamics_sim
{
enum class VehicleName : uint8_t
{
  BICYCLE,
  DIFFERENTIAL,
  OMNI,
};

/**
 * @brief Convert a VehicleName enum value to its string representation.
 * @param name The VehicleName enum value
 * @return String representation ("bicycle", "differential", or "omni")
 * @throws std::invalid_argument if the enum value is not recognized
 */
std::string toString(const VehicleName & name);

/**
 * @brief Convert a string to the corresponding VehicleName enum value.
 * @param name String representation of the vehicle name
 * @return Corresponding VehicleName enum value
 * @throws std::invalid_argument if the string doesn't match any known vehicle type
 */
VehicleName toVehicleName(const std::string_view & name);

class ModelBase
{
protected:
  rclcpp::Time time_{static_cast<int64_t>(0), RCL_ROS_TIME};
};

class DeadTimeDelay : public ModelBase
{
public:
  /**
   * @brief Models a pure time delay (transport delay) for actuator dynamics.
   * 
   * Implements dead time delay where output at time t equals input from dead_time_ seconds earlier.
   * Models delays in hydraulic systems, network latency, etc.
   * 
   * @param node ROS node used to fetch parameters
   * @param ns Parameter namespace (e.g., "vehicle.drive_actuator")
   */
  DeadTimeDelay(rclcpp::Node & node, const std::string & ns);

  /**
   * @brief Add a time-stamped value to the delay queue.
   * 
   * @param time Timestamp when the value was commanded (must be monotonically increasing)
   * @param value The input value to be delayed
   */
  void enter(const rclcpp::Time & time, const double value);

  /**
   * @brief Retrieve the delayed value for the specified time.
   * 
   * @param time Current simulation time
   * @return The value from dead_time_ seconds ago (or 0.0 if queue doesn't extend far enough)
   */
  double get(const rclcpp::Time & time);

private:
  // Parameters
  const double dead_time_;
  // State
  // oldest value first, newest last (timestamps increasing)
  std::deque<std::pair<rclcpp::Time, double>> queue_;
};

class DriveActuator : public ModelBase
{
public:
  /**
   * @brief Models drive actuator dynamics with delay, velocity/acceleration limits, and filtering.
   * 
   * Applies: dead time delay → velocity saturation → first order low pass filter → acceleration limits.
   * 
   * @param node ROS node used to fetch parameters
   * @param ns Parameter namespace (e.g., "vehicle.drive_actuator")
   */
  DriveActuator(rclcpp::Node & node, const std::string & ns);

  inline double get_max_velocity() const { return max_velocity_; }

  /**
   * @brief Compute actuator output velocity after applying all dynamic constraints.
   * 
   * @param time Current simulation time
   * @param reference_velocity Commanded velocity [m/s]
   * @return Actual actuator velocity [m/s]
   */
  double get_new_velocity(const rclcpp::Time & time, const double & reference_velocity);

private:
  // Parameters
  const double max_velocity_;
  const double time_constant_;
  const double max_acceleration_;
  // Submodels
  DeadTimeDelay deadTimeDelay_;
  // State
  double prev_velocity_ = 0;
};

class SteeringActuator : public ModelBase
{
public:
  /**
   * @brief Models steering actuator dynamics with delay, position/velocity limits, and filtering.
   * 
   * Applies: dead time delay → position saturation → first order low pass filter → angular velocity limits.
   * All angles wrapped to [-π, π].
   * 
   * @param node ROS node used to fetch parameters
   * @param ns Parameter namespace (e.g., "vehicle.steering_actuator")
   */
  SteeringActuator(rclcpp::Node & node, const std::string & ns);

  inline double get_max_position() const { return max_position_; }

  /**
   * @brief Compute actuator output steering angle after applying all dynamic constraints.
   * 
   * @param time Current simulation time
   * @param reference_position Commanded steering angle [rad]
   * @return Actual steering angle [rad], wrapped to [-π, π]
   */
  double get_new_position(const rclcpp::Time & time, const double & reference_position);

private:
  // Parameters
  const double max_position_;
  const double time_constant_;
  const double max_velocity_;
  // Submodels
  DeadTimeDelay deadTimeDelay_;
  // State
  double prev_position_ = 0;
};

class Vehicle : public ModelBase
{
public:
  /**
   * @brief Base class for vehicle kinematic models.
   * 
   * Tracks pose (position, heading) and provides twist/joint state outputs.
   * base_link_offset is the distance from vehicle reference point (typically rear axle) to base_link frame.
   * 
   * @param node ROS node used to fetch parameters
   * @param ns Parameter namespace (e.g., "vehicle")
   */
  Vehicle(rclcpp::Node & node, const std::string & ns);

  /**
   * @brief Get human-readable class name using RTTI demangling.
   * @return Demangled class name (e.g., "BicycleVehicle", "DifferentialVehicle")
   */
  std::string name() const;

  /**
   * @brief Get current pose of the vehicle's base_link frame.
   * @return Pose2D with global x, y position [m] and heading [rad]
   */
  Pose2D get_pose() const;

  /**
   * @brief Update vehicle state by simulating one timestep.
   * 
   * Processes commanded twist, simulates actuators and kinematics, integrates motion.
   * Must be called with monotonically increasing time.
   * 
   * @param time Current simulation time
   * @param reference_twist Commanded velocities (frame and interpretation vary by vehicle type)
   */
  virtual void update(
    const rclcpp::Time & time, const geometry_msgs::msg::TwistStamped & reference_twist) = 0;

  /**
   * @brief Generate URDF for visualization and kinematics.
   * @return Complete URDF XML string with links, joints, and visual geometry
   */
  virtual std::string get_robot_description() const = 0;
  // Getters
  inline double get_base_link_offset() const { return base_link_offset_; }
  inline geometry_msgs::msg::TwistStamped get_actual_twist() const { return actual_twist_; }
  inline sensor_msgs::msg::JointState get_joint_states() const { return joint_states_; }

protected:
  /**
   * @brief Store actual vehicle twist for publishing to ROS topics.
   * 
   * Converts fixed_axle frame velocities to base_link frame by compensating for base_link_offset.
   * 
   * @param time Current simulation time
   * @param vx Forward velocity at vehicle reference point (fixed_axle frame) [m/s]
   * @param vy Lateral velocity at vehicle reference point (fixed_axle frame) [m/s]
   * @param oz Yaw rate [rad/s]
   */
  void store_actual_twist(
    const rclcpp::Time & time, const double vx, const double vy, const double oz);

  // Parameters
  const double base_link_offset_;
  // State
  Eigen::Vector2d position_ = Eigen::Vector2d::Zero();
  double heading_ = 0;
  // For publishing thereafter
  geometry_msgs::msg::TwistStamped actual_twist_;
  sensor_msgs::msg::JointState joint_states_;
};

class BicycleVehicle : public Vehicle
{
public:
  /**
   * @brief Bicycle (Ackermann) kinematic vehicle model with front-wheel steering.
   * 
   * Implements bicycle kinematics approximating Ackermann steering geometry.
   * Supports front-wheel drive (drive_on_steered_wheel=true) or rear-wheel drive (false).
   * Enforces no-slip constraint with turning radius R = wheelbase / tan(steering_angle).
   * 
   * @param node ROS node used to fetch parameters
   * @param ns Parameter namespace (e.g., "vehicle")
   */
  BicycleVehicle(rclcpp::Node & node, const std::string & ns);

  /**
   * @brief Update bicycle vehicle state using Ackermann kinematics.
   * 
   * Derives steering angle from commanded twist, simulates actuators, integrates kinematics.
   * Uses midpoint integration for accuracy.
   * 
   * @param time Current simulation time
   * @param reference_twist Commanded twist.linear.x [m/s] and twist.angular.z [rad/s]
   */
  void update(
    const rclcpp::Time & time, const geometry_msgs::msg::TwistStamped & reference_twist) override;

  /**
   * @brief Generate URDF with Ackermann steering geometry.
   * @return URDF XML string with base_link, axles, and wheels (single or dual per axle)
   */
  std::string get_robot_description() const override;

private:
  // Params
  const double wheel_base_;
  const bool drive_on_steered_wheel_;
  const bool reverse_;
  const double vis_track_fixed_;
  const double vis_track_steered_;
  const double vis_tire_diameter_;
  // Actuators
  DriveActuator drive_actuator_;
  SteeringActuator steering_actuator_;
};

class DifferentialVehicle : public Vehicle
{
public:
  /**
   * @brief Differential drive vehicle model (two independently driven wheels).
   * 
   * Implements differential drive kinematics:
   *   v_forward = (v_left + v_right) / 2
   *   v_angular = (v_right - v_left) / track
   * Prioritizes rotation by scaling linear velocity if needed to respect actuator limits.
   * 
   * @param node ROS node used to fetch parameters
   * @param ns Parameter namespace (e.g., "vehicle")
   */
  DifferentialVehicle(rclcpp::Node & node, const std::string & ns);

  /**
   * @brief Update differential vehicle state using differential drive kinematics.
   * 
   * Converts twist to wheel velocities, simulates independent actuators, integrates motion.
   * Uses midpoint integration for accuracy.
   * 
   * @param time Current simulation time
   * @param reference_twist Commanded twist.linear.x [m/s] and twist.angular.z [rad/s]
   */
  void update(
    const rclcpp::Time & time, const geometry_msgs::msg::TwistStamped & reference_twist) override;

  /**
   * @brief Generate URDF with left and right drive wheels.
   * @return URDF XML string with base_link, axle, and two fixed wheels
   */
  std::string get_robot_description() const override;

private:
  // Params
  const double track_;
  const double vis_tire_diameter_;
  // Actuators
  DriveActuator drive_actuator_left_;
  DriveActuator drive_actuator_right_;
};

class OmniVehicle : public Vehicle
{
public:
  /**
   * @brief Omni drive vehicle model (four independently driven wheels with mecanum wheels).
   * 
   * @param node ROS node used to fetch parameters
   * @param ns Parameter namespace (e.g., "vehicle")
   */
  OmniVehicle(rclcpp::Node & node, const std::string & ns);

  /**
   * @brief Update omni vehicle state using omni drive kinematics.
   * 
   * Converts twist to wheel velocities, simulates independent actuators, integrates motion.
   * Uses midpoint integration for accuracy.
   * 
   * @param time Current simulation time
   * @param reference_twist Commanded twist.linear.x [m/s] and twist.angular.z [rad/s]
   */
  void update(
    const rclcpp::Time & time, const geometry_msgs::msg::TwistStamped & reference_twist) override;

  /**
   * @brief Generate URDF with left and right drive wheels.
   * @return URDF XML string with base_link, axle, and two fixed wheels
   */
  std::string get_robot_description() const override;

private:
  // Params
  const double wheel_base_;
  const double track_;
  const double vis_wheel_diameter_;
  // Actuators
  // fl = front left, rl = rear left etc.
  DriveActuator drive_actuator_fl_;
  DriveActuator drive_actuator_rl_;
  DriveActuator drive_actuator_rr_;
  DriveActuator drive_actuator_fr_;
};

/**
 * @brief Factory function to instantiate a vehicle model from VehicleName enum.
 * 
 * Creates appropriate derived Vehicle class (BicycleVehicle, DifferentialVehicle, etc.).
 * Note: Historical name (originally returned ModelBase*); now returns Vehicle*.
 * 
 * @param model Vehicle type to instantiate
 * @param node ROS node for parameter loading
 * @param ns Parameter namespace (e.g., "vehicle")
 * @return Unique pointer to the instantiated vehicle model
 * @throws std::invalid_argument if model type is unsupported
 */
std::unique_ptr<Vehicle> to_vehicle(
  const VehicleName model, rclcpp::Node & node, const std::string & ns);
}  // namespace vehicle_dynamics_sim
#endif  // VEHICLE_DYNAMICS_SIM_VEHICLES_H
