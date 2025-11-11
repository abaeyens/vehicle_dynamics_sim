#ifndef VEHICLE_DYNAMICS_SIM_Pose2D_H
#define VEHICLE_DYNAMICS_SIM_Pose2D_H

#include <cmath>
#include <stdexcept>

#include <Eigen/Dense>

#include <vehicle_dynamics_sim/utils.h>

namespace vehicle_dynamics_sim
{
class Pose2D : public Eigen::Vector3d
{
public:
  using Eigen::Vector3d::Vector3d;
  Pose2D(const Eigen::Isometry3d & H);
  inline double & theta() { return z(); }
  inline double theta() const { return z(); }
  inline Pose2D operator*(const Pose2D & other) const;
  inline Pose2D inverse() const;
  inline double norm() const;
  Eigen::Isometry3d toIsometry3d() const;
};

// Inline method implementations
inline Pose2D Pose2D::operator*(const Pose2D & other) const
{
  const double c = std::cos(this->theta());
  const double s = std::sin(this->theta());
  const double x_new = this->x() + c * other.x() - s * other.y();
  const double y_new = this->y() + s * other.x() + c * other.y();
  const double theta_new = mod_2pi(this->theta() + other.theta());
  return Pose2D{x_new, y_new, theta_new};
}

inline Pose2D Pose2D::inverse() const
{
  const double c = std::cos(theta());
  const double s = std::sin(theta());
  const double x_new = -c * x() - s * y();
  const double y_new = s * x() - c * y();
  const double theta_new = mod_2pi(-theta());
  return Pose2D{x_new, y_new, theta_new};
}

inline double Pose2D::norm() const { return std::sqrt(x() * x() + y() * y()); }
}  // namespace vehicle_dynamics_sim
#endif  // VEHICLE_DYNAMICS_SIM_Pose2D_H
