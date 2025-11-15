#ifndef VEHICLE_DYNAMICS_SIM_URDF_H
#define VEHICLE_DYNAMICS_SIM_URDF_H

#include <string>

#include <Eigen/Dense>

namespace vehicle_dynamics_sim
{
/**
 * @brief Create URDF opening tag with material definitions.
 * 
 * Generates XML declaration, <robot> tag, and predefined materials:
 * - "wheel": dark gray (0.1, 0.1, 0.1)
 * - "body": yellow (0.9, 0.9, 0.2)
 * 
 * @param robot_name Name for the robot model
 * @return XML string with header and material definitions
 */
std::string create_header(const std::string & robot_name);

/**
 * @brief Create URDF closing </robot> tag.
 * @return XML string with closing tag
 */
std::string create_tail();

/**
 * @brief Create a URDF joint element.
 * 
 * General-purpose joint generator with configurable type, position, and axis.
 * Position is specified in the parent link's coordinate frame [m].
 * Generates placeholder limits (all zeros) suitable for visualization.
 * 
 * @param parent_link Name of the parent link
 * @param child_link Name of the child link
 * @param name Name of the joint
 * @param type Joint type ("fixed", "revolute", "continuous", etc.)
 * @param position Joint origin in parent link frame [m] (default: zero)
 * @param axis Axis of rotation/translation (default: X axis)
 * @return XML string for the joint element
 */
std::string create_joint(
  const std::string & parent_link, const std::string & child_link, const std::string & name,
  const std::string & type, const Eigen::Vector3d & position = Eigen::Vector3d::Zero(),
  const Eigen::Vector3d & axis = Eigen::Vector3d::UnitX());

/**
 * @brief Create a fixed joint between two links.
 * 
 * Auto-generates joint name as "{parent}__{child}__joint".
 * 
 * @param parent_link Name of the parent link
 * @param child_link Name of the child link
 * @param position Joint origin in parent link frame [m] (default: zero)
 * @return XML string for the fixed joint
 */
std::string create_fixed_joint(
  const std::string & parent_link, const std::string & child_link,
  const Eigen::Vector3d & position = Eigen::Vector3d::Zero());

/**
 * @brief Create a revolute steering joint rotating about the Z axis (yaw).
 * 
 * Uses Z axis following ROS conventions (Z-up coordinate system).
 * Requires explicit joint name for use with robot_state_publisher.
 * 
 * @param parent_link Name of the parent link
 * @param child_link Name of the child link (steered wheel)
 * @param name Name of the joint (must match joint_states messages)
 * @param position Joint origin in parent link frame [m] (default: zero)
 * @return XML string for the revolute steering joint
 */
std::string create_steering_joint(
  const std::string & parent_link, const std::string & child_link, const std::string & name,
  const Eigen::Vector3d & position = Eigen::Vector3d::Zero());

/**
 * @brief Create an empty URDF link element (no geometry, inertia, or collision).
 * 
 * Useful as a joint connection point or coordinate frame placeholder.
 * 
 * @param link Name of the link
 * @return XML string for an empty link element
 */
std::string create_link(const std::string & link);

/**
 * @brief Create a wheel link with cylindrical visual geometry.
 * 
 * Cylinder oriented along Y axis (rolled 90° around X axis).
 * Uses "wheel" material (dark gray). Visual only, no collision or inertia.
 * 
 * @param link Name of the wheel link
 * @param diameter Wheel diameter [m] (cylinder radius = diameter/2)
 * @param width Wheel width [m] (cylinder length along Y axis)
 * @return XML string for the wheel link with visual geometry
 */
std::string create_wheel(const std::string & link, double diameter, double width);

/**
 * @brief Create a body link with box visual geometry.
 * 
 * Box dimensions in X (length/forward), Y (width/lateral), Z (height/vertical).
 * Uses "body" material (yellow). Visual only, no collision or inertia.
 * 
 * @param link Name of the body link
 * @param length Box length along X axis [m]
 * @param width Box width along Y axis [m]
 * @param height Box height along Z axis [m]
 * @return XML string for the body link with visual geometry
 */
std::string create_body(const std::string & link, double length, double width, double height);
}  // namespace vehicle_dynamics_sim
#endif  // VEHICLE_DYNAMICS_SIM_URDF_H
