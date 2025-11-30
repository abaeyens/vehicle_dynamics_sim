#ifndef VEHICLE_DYNAMICS_SIM__UTILS_H_
#define VEHICLE_DYNAMICS_SIM__UTILS_H_

#include <cmath>
#include <stdexcept>

namespace vehicle_dynamics_sim
{
/**
 * @brief Bring angle in range [-pi, pi)
 */
inline double mod_pi(double angle)
{
  // Use std::remainder for efficient modulo operation
  // std::remainder returns in range [-pi, pi], which is exactly what we want
  return std::remainder(angle, 2.0 * M_PI);
}

/**
 * @brief Bring angle in range [0, 2*pi)
 */
inline double mod_2pi(double angle)
{
  // Use std::fmod for efficient modulo operation
  double result = std::fmod(angle, 2.0 * M_PI);
  // std::fmod can return negative values, so normalize to [0, 2*pi)
  if (result < 0.0) result += 2.0 * M_PI;
  return result;
}

template <typename T>
void CHECK_GE(const T & a, const T & b, const std::string & msg)
{
  if (!(a >= b)) throw std::invalid_argument(msg);
}

template <typename T>
void CHECK_GT(const T & a, const T & b, const std::string & msg)
{
  if (!(a > b)) throw std::invalid_argument(msg);
}

template <typename T>
void CHECK_LE(const T & a, const T & b, const std::string & msg)
{
  if (!(a <= b)) throw std::invalid_argument(msg);
}

template <typename T>
void CHECK_LT(const T & a, const T & b, const std::string & msg)
{
  if (!(a < b)) throw std::invalid_argument(msg);
}
}  // namespace vehicle_dynamics_sim
#endif  // VEHICLE_DYNAMICS_SIM__UTILS_H_
