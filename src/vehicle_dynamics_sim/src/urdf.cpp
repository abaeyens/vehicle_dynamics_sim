#include <vehicle_dynamics_sim/urdf.h>

#include <cmath>
#include <string>

#include <Eigen/Dense>
#include <fmt/format.h>

namespace vehicle_dynamics_sim
{
std::string create_header(const std::string & robot_name)
{
  return fmt::format(
    "<?xml version=\"1.0\"?>"
    "<robot name=\"{0}\">"
    "  <material name=\"wheel\">"
    "    <color rgba=\"0.1 0.1 0.1 1\" />"
    "  </material>"
    "  <material name=\"body\">"
    "    <color rgba=\"0.9 0.9 0.2 1\" />"
    "  </material>",
    robot_name);
}

std::string create_tail() { return "</robot>"; }

std::string create_joint(
  const std::string & parent_link, const std::string & child_link, const std::string & name,
  const std::string & type, const Eigen::Vector3d & position, const Eigen::Vector3d & axis)
{
  return fmt::format(
    "<joint name=\"{3}\" type=\"{2}\">"
    "  <parent link=\"{0}\" />"
    "  <child link=\"{1}\" />"
    "  <origin xyz=\"{4} {5} {6}\" />"
    "  <axis xyz=\"{7} {8} {9}\"/>"
    "  <limit effort=\"0\" velocity=\"0\" lower=\"0\" upper=\"0\"/>"
    "</joint>",
    parent_link, child_link, type, name, position.x(), position.y(), position.z(), axis.x(),
    axis.y(), axis.z());
}

std::string create_fixed_joint(
  const std::string & parent_link, const std::string & child_link, const Eigen::Vector3d & position)
{
  return create_joint(
    parent_link, child_link, fmt::format("{}__{}__joint", parent_link, child_link), "fixed",
    position);
}

std::string create_steering_joint(
  const std::string & parent_link, const std::string & child_link, const std::string & name,
  const Eigen::Vector3d & position)
{
  return create_joint(
    parent_link, child_link, name, "revolute", position, Eigen::Vector3d::UnitZ());
}

std::string create_link(const std::string & link)
{
  return fmt::format("<link name=\"{0}\" />", link);
}

std::string create_wheel(const std::string & link, double diameter, double width)
{
  return fmt::format(
    "<link name=\"{0}\">"
    "  <visual>"
    "    <origin xyz=\"0 0 0\" rpy=\"{3} 0 0\"/>"
    "    <geometry>"
    "      <cylinder length=\"{1}\" radius=\"{2}\" />"
    "    </geometry>"
    "    <material name=\"wheel\"/>"
    "  </visual>"
    "</link>",
    link, width, diameter / 2.0, M_PI / 2);
}

std::string create_body(const std::string & link, double length, double width, double height)
{
  return fmt::format(
    "<link name=\"{0}\">"
    "  <visual>"
    "    <origin xyz=\"0 0 0\" />"
    "    <geometry>"
    "      <box size=\"{1} {2} {3}\"/>"
    "    </geometry>"
    "    <material name=\"body\"/>"
    "  </visual>"
    "</link>",
    link, length, width, height);
}
}  // namespace vehicle_dynamics_sim
