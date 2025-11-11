#ifndef VEHICLE_DYNAMICS_SIM_UTILS_H
#define VEHICLE_DYNAMICS_SIM_UTILS_H

#include <cmath>
#include <stdexcept>

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

template<typename T>
void CHECK_GE(const T& a, const T& b, const std::string& msg)
{
  if (!(a >= b)) throw std::invalid_argument(msg);
}

template <typename T>
void CHECK_GT(const T & a, const T & b, const std::string & msg)
{
  if (!(a > b)) throw std::invalid_argument(msg);
}
}  // namespace vehicle_dynamics_sim
#endif  // VEHICLE_DYNAMICS_SIM_UTILS_H
