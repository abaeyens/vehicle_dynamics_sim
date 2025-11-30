#include <vehicle_dynamics_sim/localization.hpp>

#include <algorithm>
#include <cmath>
#include <string>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>

#include <vehicle_dynamics_sim/declare_and_get_parameter.hpp>
#include <vehicle_dynamics_sim/utils.hpp>

namespace vehicle_dynamics_sim
{

Localization::Localization(rclcpp::Node & node, const std::string & ns)
: odom_walk_velocity_translation_(
    declare_and_get_parameter(node, ns + ".odom_walk_velocity_translation", std::pow(0.05, 2))),
  odom_walk_velocity_rotation_(declare_and_get_parameter(
    node, ns + ".odom_walk_velocity_rotation", std::pow(5.0 * M_PI / 180.0, 2))),
  map_sample_noise_translation_(
    declare_and_get_parameter(node, ns + ".map_sample_noise_translation", std::pow(0.005, 2))),
  map_sample_noise_rotation_(declare_and_get_parameter(
    node, ns + ".map_sample_noise_rotation", std::pow(0.1 * M_PI / 180.0, 2))),
  rng_(RNG_SEED),
  normal_(0.0, 1.0)
{
  CHECK_GE(
    odom_walk_velocity_translation_, 0.0,
    "'" + ns + ".odom_walk_velocity_translation' must be positive.");
  CHECK_GE(
    odom_walk_velocity_rotation_, 0.0,
    "'" + ns + ".odom_walk_velocity_rotation' must be positive.");
  CHECK_GE(
    map_sample_noise_translation_, 0.0,
    "'" + ns + ".map_sample_noise_translation' must be positive.");
  CHECK_GE(
    map_sample_noise_rotation_, 0.0, "'" + ns + ".map_sample_noise_rotation' must be positive.");
}

std::pair<Pose2D, Pose2D> Localization::update(const rclcpp::Time & time, const Pose2D & T_M_B)
{
  // Update odom frame
  // we add some integration noise based on drive distance
  const Pose2D T_Bp_B = T_M_Bp_.inverse() * T_M_B;
  const double driven_distance = T_Bp_B.norm();
  const Pose2D odom_noise{
    std::sqrt(driven_distance * odom_walk_velocity_translation_) * normal_(rng_),
    std::sqrt(driven_distance * odom_walk_velocity_translation_) * normal_(rng_),
    std::clamp(
      std::sqrt(driven_distance * odom_walk_velocity_rotation_) * normal_(rng_), -M_PI, M_PI)};
  T_O_B_ = T_O_B_ * (T_Bp_B * odom_noise);
  // Simulate
  // divide by dt because the fewer the samples, the lower the noise per sample
  // (maintain same information density irrespective of sample rate)
  const double dt = (time - time_).seconds();
  const Pose2D map_noise{
    std::sqrt(map_sample_noise_translation_ / dt) * normal_(rng_),
    std::sqrt(map_sample_noise_translation_ / dt) * normal_(rng_),
    std::clamp(std::sqrt(map_sample_noise_rotation_ / dt) * normal_(rng_), -M_PI, M_PI)};
  const Pose2D T_O_B_noisy = T_O_B_ * map_noise;
  // Get odom pose in map frame (incl. map localization noise)
  const Pose2D T_M_O = T_M_B * T_O_B_noisy.inverse();
  // Keep for next call
  time_ = time;
  T_M_Bp_ = T_M_B;
  return std::make_pair(T_M_O, T_O_B_);
}
}  // namespace vehicle_dynamics_sim
