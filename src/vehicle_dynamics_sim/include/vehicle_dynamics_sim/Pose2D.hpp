#ifndef VEHICLE_DYNAMICS_SIM__POSE2D_H_
#define VEHICLE_DYNAMICS_SIM__POSE2D_H_

#include <cmath>
#include <stdexcept>

#include <Eigen/Dense>

#include <vehicle_dynamics_sim/utils.hpp>

namespace vehicle_dynamics_sim
{
class Pose2D : public Eigen::Vector3d
{
public:
  using Eigen::Vector3d::Vector3d;
  explicit Pose2D(const Eigen::Isometry3d & H);
  Eigen::Block<Eigen::Vector3d, 2, 1> position() { return this->head<2>(); }
  [[nodiscard]] Eigen::Block<const Eigen::Vector3d, 2, 1> position() const
  {
    return this->head<2>();
  }
  double & heading() { return z(); }
  [[nodiscard]] double heading() const { return z(); }
  [[nodiscard]] inline Pose2D operator*(const Pose2D & other) const;
  [[nodiscard]] inline Pose2D inverse() const;
  [[nodiscard]] inline double norm() const;
  [[nodiscard]] Eigen::Isometry3d toIsometry3d() const;
};

// Inline method implementations
inline Pose2D Pose2D::operator*(const Pose2D & other) const
{
  const double c = std::cos(this->heading());
  const double s = std::sin(this->heading());
  const double x_new = this->x() + c * other.x() - s * other.y();
  const double y_new = this->y() + s * other.x() + c * other.y();
  const double heading_new = mod_2pi(this->heading() + other.heading());
  return Pose2D{x_new, y_new, heading_new};
}

inline Pose2D Pose2D::inverse() const
{
  const double c = std::cos(heading());
  const double s = std::sin(heading());
  const double x_new = -c * x() - s * y();
  const double y_new = s * x() - c * y();
  const double heading_new = mod_2pi(-heading());
  return Pose2D{x_new, y_new, heading_new};
}

inline double Pose2D::norm() const { return std::sqrt(x() * x() + y() * y()); }
}  // namespace vehicle_dynamics_sim
#endif  // VEHICLE_DYNAMICS_SIM__POSE2D_H_
