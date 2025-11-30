#ifndef VEHICLE_DYNAMICS_SIM__LOCALIZATION_H_
#define VEHICLE_DYNAMICS_SIM__LOCALIZATION_H_
#include <random>
#include <string>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>

#include <vehicle_dynamics_sim/Pose2D.hpp>

namespace vehicle_dynamics_sim
{

/**
 * @brief Simulates realistic localization with odometry drift and measurement noise.
 * 
 * Models two common localization error sources:
 * 1. Odometry drift: Random walk error accumulating with distance traveled
 *    (simulates wheel slip, encoder errors, IMU drift)
 * 2. Map localization noise: Sampling noise on absolute position measurements
 *    (simulates GPS, visual localization, or other map-referenced sensors)
 * 
 * Maintains a drifting "odom" frame that represents integrated dead-reckoning,
 * and provides noisy measurements of the transform from map to odom frame.
 * This creates realistic behavior where odometry is smooth but drifts over time,
 * while map-based corrections are noisy but unbiased.
 */
class Localization
{
public:
  /**
   * @brief Construct localization simulator with noise parameters.
   * 
   * Parameters loaded (all are variance values):
   * - {ns}.odom_walk_velocity_translation: Translation drift per meter traveled [m²/m]
   * - {ns}.odom_walk_velocity_rotation: Rotation drift per meter traveled [rad²/m]
   * - {ns}.map_sample_noise_translation: Map measurement noise per second [m²/s]
   * - {ns}.map_sample_noise_rotation: Map measurement noise per second [rad²/s]
   * 
   * Note: Parameters are variances (σ²), not standard deviations.
   * 
   * @param node ROS node used to fetch parameters
   * @param ns Parameter namespace (e.g., "localization")
   */
  Localization(rclcpp::Node & node, const std::string & ns);

  /**
   * @brief Update localization simulation for one timestep.
   * 
   * Takes the true vehicle pose in map frame, accumulates odometry drift based on
   * distance traveled, adds measurement noise, and returns both the noisy map→odom
   * transform and the (clean) odom→base_link transform.
   * 
   * The odom frame drifts gradually (random walk), while map measurements are noisy
   * but don't accumulate error. This mimics real systems where wheel odometry
   * integrates smoothly but drifts, and GPS/SLAM provides noisy but bounded corrections.
   * 
   * @param time Current simulation time
   * @param T_M_B True vehicle pose in map frame (ground truth)
   * @return Pair of {T_M_O (map→odom with noise), T_O_B (odom→base_link, no noise)}
   */
  [[nodiscard]] std::pair<Pose2D, Pose2D> update(const rclcpp::Time & time, const Pose2D & T_M_B);

private:
  // Parameters (all are variance values: σ²)
  const double odom_walk_velocity_translation_;  ///< Odometry translation drift variance [m²/m]
  const double odom_walk_velocity_rotation_;     ///< Odometry rotation drift variance [rad²/m]
  const double map_sample_noise_translation_;    ///< Map localization noise variance [m²/s]
  const double map_sample_noise_rotation_;       ///< Map localization noise variance [rad²/s]

  // Noise generators
  static constexpr int RNG_SEED = 12345;  // fix seed to make noise deterministic
  std::mt19937 rng_;
  std::normal_distribution<double> normal_;

  // State
  rclcpp::Time time_{static_cast<int64_t>(0), RCL_ROS_TIME};  ///< Last update time
  Pose2D T_O_B_ = Pose2D::Zero();   ///< Current base_link pose in odom frame
  Pose2D T_M_Bp_ = Pose2D::Zero();  ///< Previous base_link pose in map frame
};
}  // namespace vehicle_dynamics_sim
#endif  // VEHICLE_DYNAMICS_SIM__LOCALIZATION_H_
