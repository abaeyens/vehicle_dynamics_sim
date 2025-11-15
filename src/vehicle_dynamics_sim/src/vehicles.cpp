#include <vehicle_dynamics_sim/vehicles.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>

#include <boost/core/demangle.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fmt/format.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <vehicle_dynamics_sim/declare_and_get_parameter.h>
#include <vehicle_dynamics_sim/Pose2D.h>
#include <vehicle_dynamics_sim/urdf.h>
#include <vehicle_dynamics_sim/utils.h>

namespace vehicle_dynamics_sim
{
std::string toString(const VehicleName & name)
{
  switch (name) {
    case VehicleName::BICYCLE:
      return "bicycle";
    case VehicleName::DIFFERENTIAL:
      return "differential";
    case VehicleName::OMNI:
      return "omni";
  }
  throw std::invalid_argument(
    fmt::format("Unknown vehicle name enum value: {}", static_cast<int>(name)));
}

VehicleName toVehicleName(const std::string_view & name)
{
  if (name == "bicycle")
    return VehicleName::BICYCLE;
  else if (name == "differential")
    return VehicleName::DIFFERENTIAL;
  else if (name == "omni")
    return VehicleName::OMNI;
  throw std::invalid_argument(fmt::format("Unknown vehicle name: {}", name));
}

DeadTimeDelay::DeadTimeDelay(rclcpp::Node & node, const std::string & ns)
: ModelBase(), dead_time_(declare_and_get_parameter(node, ns + ".dead_time", 0.2))
{
  CHECK_GE(dead_time_, 0.0, "'" + ns + ".dead_time' must be strictly positive.");
}

void DeadTimeDelay::enter(const rclcpp::Time & time, const double value)
{
  if (!queue_.empty() && !(time >= queue_.back().first))
    throw std::invalid_argument("DeadTimeDelay::enter: time must be increasing");
  queue_.push_back({time, value});
}

double DeadTimeDelay::get(const rclcpp::Time & time)
{
  const rclcpp::Time fetch_time =
    time - rclcpp::Duration(0, static_cast<int32_t>(1e9 * dead_time_));
  while (queue_.size() > 1 && queue_[1].first < fetch_time) queue_.pop_front();
  const auto & [timestamp, value] = queue_.front();
  return timestamp <= fetch_time ? value : 0.0;
}

DriveActuator::DriveActuator(rclcpp::Node & node, const std::string & ns)
: ModelBase(),
  max_velocity_(declare_and_get_parameter(node, ns + ".max_velocity", 2.0)),
  time_constant_(declare_and_get_parameter(node, ns + ".time_constant", 0.4)),
  max_acceleration_(declare_and_get_parameter(node, ns + ".max_acceleration", 2.0)),
  deadTimeDelay_(node, ns)
{
  CHECK_GE(max_velocity_, 0.0, "'" + ns + ".max_velocity' must be positive.");
  CHECK_GE(time_constant_, 0.0, "'" + ns + ".time_constant' must be positive.");
  CHECK_GT(max_acceleration_, 0.0, "'" + ns + ".max_acceleration' must be strictly positive.");
}

double DriveActuator::get_new_velocity(const rclcpp::Time & time, const double & reference_velocity)
{
  // Dead time
  deadTimeDelay_.enter(time, reference_velocity);
  const double velocity_delayed = deadTimeDelay_.get(time);
  // Velocity limits
  const double velocity_limited = std::clamp(velocity_delayed, -max_velocity_, max_velocity_);
  // Low pass effect
  const double dt = (time - time_).seconds();
  const double alpha = time_constant_ / (time_constant_ + dt);
  const double velocity_delta = (1.0 - alpha) * (velocity_limited - prev_velocity_);
  // Acceleration limits
  const double velocity_delta_limited =
    std::clamp(velocity_delta, -max_acceleration_ * dt, max_acceleration_ * dt);
  // Apply final delta
  const double velocity = prev_velocity_ + velocity_delta_limited;
  // Keep for next iteration
  time_ = time;
  prev_velocity_ = velocity;
  return velocity;
}

SteeringActuator::SteeringActuator(rclcpp::Node & node, const std::string & ns)
: ModelBase(),
  max_position_(declare_and_get_parameter(node, ns + ".max_position", M_PI / 6)),
  time_constant_(declare_and_get_parameter(node, ns + ".time_constant", 0.05)),
  max_velocity_(declare_and_get_parameter(node, ns + ".max_velocity", 2.0)),
  deadTimeDelay_(node, ns)
{
  CHECK_GE(max_position_, 0.0, "'" + ns + ".max_position' must be positive.");
  CHECK_GE(time_constant_, 0.0, "'" + ns + ".time_constant' must be positive.");
  CHECK_GT(max_velocity_, 0.0, "'" + ns + ".max_velocity' must be strictly positive.");
}

double SteeringActuator::get_new_position(
  const rclcpp::Time & time, const double & reference_position)
{
  // Dead time
  deadTimeDelay_.enter(time, reference_position);
  const double position_delayed = deadTimeDelay_.get(time);
  // Position limits
  const double position_limited = (max_position_ != 0)
                                    ? std::clamp(position_delayed, -max_position_, max_position_)
                                    : position_delayed;
  // Low pass effect
  const double dt = (time - time_).seconds();
  const double alpha = time_constant_ / (time_constant_ + dt);
  const double position_delta = (1.0 - alpha) * mod_pi(position_limited - prev_position_);
  // Acceleration limits
  const double position_delta_limited =
    std::clamp(position_delta, -max_velocity_ * dt, max_velocity_ * dt);
  // Apply final delta
  const double position = mod_pi(prev_position_ + position_delta_limited);
  // Keep for next iteration
  time_ = time;
  prev_position_ = position;
  return position;
}

Vehicle::Vehicle(rclcpp::Node & node, const std::string & ns)
: ModelBase(), base_link_offset_(declare_and_get_parameter(node, ns + ".base_link_offset", 0.0))
{
}

std::string Vehicle::name() const { return boost::core::demangle(typeid(*this).name()); }

Pose2D Vehicle::get_pose() const
{
  return Pose2D{
    position_.x() + std::cos(heading_) * base_link_offset_,
    position_.y() + std::sin(heading_) * base_link_offset_, heading_};
}

void Vehicle::store_actual_twist(
  const rclcpp::Time & time, const double vx, const double vy, const double oz)
{
  actual_twist_.header.stamp = time;
  actual_twist_.header.frame_id = "base_link";
  actual_twist_.twist.linear.x = vx;
  actual_twist_.twist.linear.y = vy + oz * base_link_offset_;
  actual_twist_.twist.linear.z = 0.0;
  actual_twist_.twist.angular.x = 0.0;
  actual_twist_.twist.angular.y = 0.0;
  actual_twist_.twist.angular.z = oz;
}

BicycleVehicle::BicycleVehicle(rclcpp::Node & node, const std::string & ns)
: Vehicle(node, ns),
  wheel_base_(declare_and_get_parameter(node, ns + ".wheel_base", 2.0)),
  drive_on_steered_wheel_(declare_and_get_parameter(node, ns + ".drive_on_steered_wheel", false)),
  reverse_(declare_and_get_parameter(node, ns + ".reverse", false)),
  vis_track_fixed_(declare_and_get_parameter(node, ns + ".vis_track_fixed", wheel_base_ * 0.7)),
  vis_track_steered_(declare_and_get_parameter(node, ns + ".vis_track_steered", wheel_base_ * 0.7)),
  vis_tire_diameter_(
    declare_and_get_parameter(node, ns + ".vis_tire_diameter", wheel_base_ * 0.25)),
  drive_actuator_(DriveActuator(node, ns + ".drive_actuator")),
  steering_actuator_(SteeringActuator(node, ns + ".steering_actuator"))
{
  CHECK_GT(wheel_base_, 0.0, "'" + ns + ".wheel_base' must be strictly positive.");
  if (
    !drive_on_steered_wheel_ && (steering_actuator_.get_max_position() == 0 ||
                                 steering_actuator_.get_max_position() > M_PI / 2 - 1e-9)) {
    throw std::invalid_argument(
      "BicycleVehicle::BicycleVehicle: 'steering_actuator.max_position' must be "
      "nonzero and less than 90 degrees if 'drive_on_steered_wheel' is false.");
  }
}

void BicycleVehicle::update(
  const rclcpp::Time & time, const geometry_msgs::msg::TwistStamped & reference_twist)
{
  // Figure out steering wheel angle
  const bool can_derive_valid_steering_position =
    std::abs(reference_twist.twist.linear.x) > 1e-12 ||
    (drive_on_steered_wheel_ && std::abs(reference_twist.twist.angular.z) > 1e-12);
  const double rm = reverse_ ? -1.0 : 1.0;  // "rm" = Reverse Multiplier
  const double steering_position_ref =
    can_derive_valid_steering_position
      ? std::atan2(
          rm * wheel_base_ * reference_twist.twist.angular.z, reference_twist.twist.linear.x)
      : 0.0;
  const double steering_position = steering_actuator_.get_new_position(time, steering_position_ref);
  // Figure out resulting velocities
  double v_forward = 0;
  double v_angular = 0;
  if (drive_on_steered_wheel_) {
    const double drive_velocity_reference = std::norm(
      std::complex<double>{
        wheel_base_ * reference_twist.twist.angular.z, reference_twist.twist.linear.x});
    const double steered_wheel_velocity =
      drive_actuator_.get_new_velocity(time, drive_velocity_reference);
    v_forward = steered_wheel_velocity * std::cos(steering_position);
    v_angular = (steered_wheel_velocity / (rm * wheel_base_)) * std::sin(steering_position);
  } else {
    v_forward = drive_actuator_.get_new_velocity(time, reference_twist.twist.linear.x);
    v_angular = v_forward * std::tan(steering_position) / (rm * wheel_base_);
  }
  // Integrate
  const double dt = (time - time_).seconds();
  const double new_heading = mod_2pi(heading_ + dt * v_angular);
  const double mean_heading = mod_2pi(heading_ + 0.5 * dt * v_angular);
  position_ += Eigen::Vector2d{std::cos(mean_heading), std::sin(mean_heading)} * dt * v_forward;
  heading_ = new_heading;
  store_actual_twist(time, v_forward, 0.0, v_angular);
  // Joint states
  joint_states_.header.stamp = time;
  if (vis_track_steered_ > 0) {
    // Two steering wheels spaced `vis_track_steered_` from each other
    const double turning_radius = -rm * wheel_base_ * std::tan(steering_position + M_PI / 2);
    const double steering_left_wheel =
      mod_pi(std::atan(-(turning_radius - vis_track_steered_ / 2) / (rm * wheel_base_)) - M_PI / 2);
    const double steering_right_wheel =
      mod_pi(std::atan(-(turning_radius + vis_track_steered_ / 2) / (rm * wheel_base_)) - M_PI / 2);
    joint_states_.name = {"steering_left", "steering_right"};
    joint_states_.position = {steering_left_wheel, steering_right_wheel};
  } else {
    // Single steering wheel
    joint_states_.name = {"steering"};
    joint_states_.position = {mod_pi(steering_position)};
  }
  // Prepare for next callback
  time_ = time;
}

std::string BicycleVehicle::get_robot_description() const
{
  std::string urdf;
  urdf += create_header(this->name());
  urdf += create_link("base_link");
  urdf += create_link("fixed_axle");
  urdf += create_fixed_joint(
    "base_link", "fixed_axle", Eigen::Vector3d{-base_link_offset_, 0.0, vis_tire_diameter_ / 2.0});
  const double tire_width = vis_tire_diameter_ * 0.25;
  // Wheel(s) on fixed axle
  if (vis_track_fixed_ > 0) {
    // Two wheels on fixed axle spaced `vis_track_fixed_` apart
    urdf += create_wheel("fixed_wheel_right", vis_tire_diameter_, tire_width);
    urdf += create_fixed_joint(
      "fixed_axle", "fixed_wheel_right", Eigen::Vector3d{0.0, -vis_track_fixed_ / 2.0, 0.0});
    urdf += create_wheel("fixed_wheel_left", vis_tire_diameter_, tire_width);
    urdf += create_fixed_joint(
      "fixed_axle", "fixed_wheel_left", Eigen::Vector3d{0.0, vis_track_fixed_ / 2.0, 0.0});
  } else {
    // Single wheel on fixed axle
    urdf += create_wheel("fixed_wheel", vis_tire_diameter_, tire_width);
    urdf += create_fixed_joint("fixed_axle", "fixed_wheel");
  }
  // Wheel(s) on steered axle
  const double rm = reverse_ ? -1.0 : 1.0;
  if (vis_track_steered_ > 0) {
    // Two steered wheels spaced `vis_track_fixed_` apart
    urdf += create_wheel("steered_wheel_right", vis_tire_diameter_, tire_width);
    urdf += create_steering_joint(
      "fixed_axle", "steered_wheel_right", "steering_right",
      Eigen::Vector3d{rm * wheel_base_, -vis_track_steered_ / 2.0, 0.0});
    urdf += create_wheel("steered_wheel_left", vis_tire_diameter_, tire_width);
    urdf += create_steering_joint(
      "fixed_axle", "steered_wheel_left", "steering_left",
      Eigen::Vector3d{rm * wheel_base_, vis_track_steered_ / 2.0, 0.0});
  } else {
    // Single steered wheel
    urdf += create_wheel("steered_wheel", vis_tire_diameter_, tire_width);
    urdf += create_steering_joint(
      "fixed_axle", "steered_wheel", "steering", Eigen::Vector3d{rm * wheel_base_, 0.0, 0.0});
  }
  // Create body
  // TODO
  urdf += create_tail();
  return urdf;
}

DifferentialVehicle::DifferentialVehicle(rclcpp::Node & node, const std::string & ns)
: Vehicle(node, ns),
  track_(declare_and_get_parameter(node, ns + ".track", 1.4)),
  vis_tire_diameter_(declare_and_get_parameter(node, ns + ".vis_tire_diameter", track_ * 0.4)),
  drive_actuator_left_(DriveActuator(node, ns + ".drive_actuators")),
  drive_actuator_right_(DriveActuator(node, ns + ".drive_actuators"))
{
  CHECK_GT(track_, 0.0, "'" + ns + ".track' must be strictly positive.");
}

std::string DifferentialVehicle::get_robot_description() const
{
  std::string urdf;
  urdf += create_header(this->name());
  urdf += create_link("base_link");
  urdf += create_link("fixed_axle");
  urdf += create_fixed_joint(
    "base_link", "fixed_axle", Eigen::Vector3d{-base_link_offset_, 0.0, vis_tire_diameter_ / 2.0});
  const double tire_width = vis_tire_diameter_ * 0.25;
  // Two wheels on fixed axle spaced `track_` apart
  urdf += create_wheel("fixed_wheel_right", vis_tire_diameter_, tire_width);
  urdf +=
    create_fixed_joint("fixed_axle", "fixed_wheel_right", Eigen::Vector3d{0.0, -track_ / 2.0, 0.0});
  urdf += create_wheel("fixed_wheel_left", vis_tire_diameter_, tire_width);
  urdf +=
    create_fixed_joint("fixed_axle", "fixed_wheel_left", Eigen::Vector3d{0.0, track_ / 2.0, 0.0});
  // Create body
  // TODO
  urdf += create_tail();
  return urdf;
}

void DifferentialVehicle::update(
  const rclcpp::Time & time, const geometry_msgs::msg::TwistStamped & reference_twist)
{
  // Preprocess: give priority to rotation (to stay closer to the intended kinematics)
  // (usual for differential drive robots)
  double v_ref_left =
    reference_twist.twist.linear.x - 0.5 * track_ * reference_twist.twist.angular.z;
  double v_ref_right =
    reference_twist.twist.linear.x + 0.5 * track_ * reference_twist.twist.angular.z;
  const double scale = std::max(
    std::abs(v_ref_left) / drive_actuator_left_.get_max_velocity(),
    std::abs(v_ref_right) / drive_actuator_right_.get_max_velocity());
  if (scale > 1.0) {
    v_ref_left /= scale;
    v_ref_right /= scale;
  }
  // Simulate actuators
  const double v_left = drive_actuator_left_.get_new_velocity(time, v_ref_left);
  const double v_right = drive_actuator_right_.get_new_velocity(time, v_ref_right);
  const double v_forward = 0.5 * (v_left + v_right);
  const double v_angular = (v_right - v_left) / track_;
  // Integrate
  const double dt = (time - time_).seconds();
  const double new_heading = mod_2pi(heading_ + dt * v_angular);
  const double mean_heading = mod_2pi(heading_ + 0.5 * dt * v_angular);
  position_ += Eigen::Vector2d{std::cos(mean_heading), std::sin(mean_heading)} * dt * v_forward;
  heading_ = new_heading;
  store_actual_twist(time, v_forward, 0.0, v_angular);
  time_ = time;
}

std::unique_ptr<Vehicle> to_vehicle(
  const VehicleName model, rclcpp::Node & node, const std::string & ns)
{
  switch (model) {
    case VehicleName::BICYCLE:
      return std::make_unique<BicycleVehicle>(node, ns);
    case VehicleName::DIFFERENTIAL:
      return std::make_unique<DifferentialVehicle>(node, ns);
    default:
      // TODO
      throw std::invalid_argument(
        fmt::format("Unsupported model name enum value: {}", static_cast<int>(model)));
      return nullptr;
  }
  throw std::invalid_argument(
    fmt::format("Unknown model name enum value: {}", static_cast<int>(model)));
}
}  // namespace vehicle_dynamics_sim
