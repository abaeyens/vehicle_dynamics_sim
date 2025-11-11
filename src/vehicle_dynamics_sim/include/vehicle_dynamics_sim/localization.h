#ifndef VEHICLE_DYNAMICS_SIM_LOCALIZATION_H
#define VEHICLE_DYNAMICS_SIM_LOCALIZATION_H

#include <random>
#include <string>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>

#include <vehicle_dynamics_sim/Pose2D.h>

namespace vehicle_dynamics_sim
{

class Localization
{
public:
  Localization(rclcpp::Node & node, const std::string & ns);
  std::pair<Pose2D, Pose2D> update(const rclcpp::Time & time, const Pose2D & T_M_B);

private:
  // Parameters
  // Random walk velocities transform odom -> base,
  // respectively expressed in m²/m and rad²/m
  const double odom_walk_velocity_translation_;
  const double odom_walk_velocity_rotation_;
  // Sampling noise transform map -> base, res. in m²/s and rad²/s
  const double map_sample_noise_translation_;
  const double map_sample_noise_rotation_;

  // Noise generators
  static constexpr int RNG_SEED = 12345;
  std::mt19937 rng_;
  std::normal_distribution<double> normal_;

  // State
  rclcpp::Time time_{static_cast<int64_t>(0), RCL_ROS_TIME};
  // Base pose in odom fra
  //Eigen::Vector3d T_O_B_ = Eigen::Vector3d::Zero();
  Pose2D T_O_B_ = Pose2D::Zero();
  // Previous base pose
  Pose2D T_M_Bp_ = Pose2D::Zero();
};
}  // namespace vehicle_dynamics_sim
#endif  // VEHICLE_DYNAMICS_SIM_LOCALIZATION_H
