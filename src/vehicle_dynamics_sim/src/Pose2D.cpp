#include <vehicle_dynamics_sim/Pose2D.h>

#include <iostream>

#include <cmath>
#include <stdexcept>

#include <Eigen/Dense>

namespace vehicle_dynamics_sim
{
Pose2D::Pose2D(const Eigen::Isometry3d & H)
{
  x() = H.translation().x();
  y() = H.translation().y();
  static constexpr double EPS = 1e-9;
  if (std::abs(H.translation().z()) > EPS)
    throw std::invalid_argument("z-translation must be close to zero");
  const Eigen::Vector3d dirX = H.linear() * Eigen::Vector3d::UnitX();
  theta() = std::atan2(dirX.y(), dirX.x());
}

Eigen::Isometry3d Pose2D::toIsometry3d() const
{
  const Eigen::Quaterniond orientation =
    Eigen::Quaterniond(Eigen::AngleAxisd(theta(), Eigen::Vector3d::UnitZ()));
  Eigen::Isometry3d H = Eigen::Isometry3d::Identity();
  H.translation() = Eigen::Vector3d(x(), y(), 0.0);
  H.linear() = orientation.toRotationMatrix();
  return H;
}
}  // namespace vehicle_dynamics_sim
