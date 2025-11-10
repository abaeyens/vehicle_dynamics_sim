#ifndef VEHICLE_DYNAMICS_SIM_UTILS_H
#define VEHICLE_DYNAMICS_SIM_UTILS_H

#include <cmath>

namespace vehicle_dynamics_sim
{
/**
 * @brief bring angle in range [-pi, pi)
 */
inline double mod_pi(double angle)
{
  // TODO improve implementation using std::fmod?
  while (angle >= M_PI) angle -= 2 * M_PI;
  while (angle < -M_PI) angle += 2 * M_PI;
  return angle;
}

/**
 * @brief bring angle in range [0, 2*pi)
 */
inline double mod_2pi(double angle)
{
  // TODO improve implementation using std::fmod?
  while (angle >= 2 * M_PI) angle -= 2 * M_PI;
  while (angle < -2 * M_PI) angle += 2 * M_PI;
  return angle;
}
}  // namespace vehicle_dynamics_sim
#endif  // VEHICLE_DYNAMICS_SIM_UTILS_H
