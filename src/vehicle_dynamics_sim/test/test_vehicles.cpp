#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <tuple>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <robot_state_publisher/robot_state_publisher.hpp>

#include <vehicle_dynamics_sim/vehicles.h>

namespace
{
/// Throws if ROS robot state publisher node fails to parse the URDF
void check_urdf(const std::string & urdf_xml)
{
  robot_state_publisher::RobotStatePublisher foo(
    rclcpp::NodeOptions().parameter_overrides({{"robot_description", urdf_xml}}));
}
}  // anonymous namespace

namespace vehicle_dynamics_sim
{
using namespace std::chrono_literals;

// DeadTimeDelay
TEST(DeadTimeDelay, zero_delay)
{
  /// Arrange
  static constexpr double DEAD_TIME = 0.0;
  DeadTimeDelay dead_time_delay(DEAD_TIME);
  const rclcpp::Time time{static_cast<int64_t>(0), RCL_ROS_TIME};
  /// Act
  dead_time_delay.enter(time, 1.0);
  const double a = dead_time_delay.get(time);
  /// Assert
  EXPECT_EQ(a, 1.0);
}

TEST(DeadTimeDelay, zero_if_get_too_recent)
{
  /// Arrange
  static constexpr double DEAD_TIME = 0.100;
  DeadTimeDelay dead_time_delay(DEAD_TIME);
  const rclcpp::Time time{static_cast<int64_t>(0), RCL_ROS_TIME};
  /// Act
  dead_time_delay.enter(time, 1.0);
  const double a = dead_time_delay.get(time + rclcpp::Duration(99ms));
  /// Assert
  EXPECT_EQ(a, 0.0);
}

TEST(DeadTimeDelay, nominal)
{
  /// Arrange
  static constexpr double DEAD_TIME = 0.100;
  DeadTimeDelay dead_time_delay(DEAD_TIME);
  const rclcpp::Time time{static_cast<int64_t>(0), RCL_ROS_TIME};
  /// Act
  dead_time_delay.enter(time, 1.0);
  const double a = dead_time_delay.get(time + rclcpp::Duration(101ms));
  /// Assert
  EXPECT_EQ(a, 1.0);
}

// DriveActuator
TEST(DriveActuator, max_velocity_and_acceleration)
{
  /// Arrange
  static constexpr double MAX_VELOCITY = 2.0;
  static constexpr double MAX_ACCELERATION = 1.0;
  DriveActuator drive_actuator(MAX_VELOCITY, 0.0, MAX_ACCELERATION, 0.0);
  const rclcpp::Time time0{static_cast<int64_t>(0), RCL_ROS_TIME};
  std::ignore = drive_actuator.get_new_velocity(time0, 0.0);
  /// Act
  const double v0 = drive_actuator.get_new_velocity(time0, 5.0);
  const rclcpp::Time time1 = time0 + rclcpp::Duration(200ms);
  const double v1 = drive_actuator.get_new_velocity(time1, 5.0);
  const rclcpp::Time time2 = time1 + rclcpp::Duration(2000ms);
  const double v2 = drive_actuator.get_new_velocity(time2, 5.0);
  /// Assert
  EXPECT_EQ(v0, 0.0);
  EXPECT_DOUBLE_EQ(v1, MAX_ACCELERATION * (time1 - time0).seconds());
  EXPECT_EQ(v2, MAX_VELOCITY);
}

TEST(DriveActuator, low_pass)
{
  /// Arrange
  static constexpr double TIME_CONSTANT = 0.5;
  DriveActuator drive_actuator(0.0, TIME_CONSTANT, 0.0, 0.0);
  const rclcpp::Time time0{static_cast<int64_t>(0), RCL_ROS_TIME};
  static constexpr double V_REF = 3.0;
  std::ignore = drive_actuator.get_new_velocity(time0, 0.0);
  /// Act
  const double v0 = drive_actuator.get_new_velocity(time0, V_REF);
  rclcpp::Time time1 = time0;
  for (; time1 - time0 < rclcpp::Duration(500ms); time1 += rclcpp::Duration(10ms))
    std::ignore = drive_actuator.get_new_velocity(time1, V_REF);
  const double v1 = drive_actuator.get_new_velocity(time1, V_REF);
  const rclcpp::Time time2 = time1 + rclcpp::Duration(500ms);
  const double v2 = drive_actuator.get_new_velocity(time2, V_REF);
  /// Assert
  EXPECT_EQ(v0, 0.0);
  EXPECT_NEAR(v1, V_REF * (1.0 - std::exp(-(time1 - time0).seconds() / TIME_CONSTANT)), 1e-9);
  EXPECT_NEAR(v2, V_REF * (1.0 - std::exp(-(time2 - time0).seconds() / TIME_CONSTANT)), 1e-9);
}

// SteeringActuator
TEST(SteeringActuator, max_position_and_velocity)
{
  /// Arrange
  static constexpr double MAX_POSITION = 1.0;
  static constexpr double MAX_VELOCITY = 2.0;
  SteeringActuator steering_actuator(MAX_POSITION, 0.0, MAX_VELOCITY, 0.0);
  const rclcpp::Time time0{static_cast<int64_t>(0), RCL_ROS_TIME};
  std::ignore = steering_actuator.get_new_position(time0, 0.0);
  /// Act
  const double p0 = steering_actuator.get_new_position(time0, 1.5);
  const rclcpp::Time time1 = time0 + rclcpp::Duration(100ms);
  const double p1 = steering_actuator.get_new_position(time1, 1.5);
  const rclcpp::Time time2 = time1 + rclcpp::Duration(500ms);
  const double p2 = steering_actuator.get_new_position(time2, 1.5);
  /// Assert
  EXPECT_EQ(p0, 0.0);
  EXPECT_DOUBLE_EQ(p1, MAX_VELOCITY * (time1 - time0).seconds());
  EXPECT_EQ(p2, MAX_POSITION);
}

TEST(SteeringActuator, low_pass)
{
  /// Arrange
  static constexpr double TIME_CONSTANT = 0.5;
  SteeringActuator steering_actuator(0.0, TIME_CONSTANT, 0.0, 0.0);
  const rclcpp::Time time0{static_cast<int64_t>(0), RCL_ROS_TIME};
  static constexpr double P_REF = 2.0;
  std::ignore = steering_actuator.get_new_position(time0, 0.0);
  /// Act
  const double p0 = steering_actuator.get_new_position(time0, P_REF);
  const rclcpp::Time time1 = time0 + rclcpp::Duration(500ms);
  const double p1 = steering_actuator.get_new_position(time1, P_REF);
  const rclcpp::Time time2 = time1 + rclcpp::Duration(500ms);
  const double p2 = steering_actuator.get_new_position(time2, P_REF);
  /// Assert
  EXPECT_EQ(p0, 0.0);
  EXPECT_DOUBLE_EQ(p1, P_REF * (1.0 - std::exp(-(time1 - time0).seconds() / TIME_CONSTANT)));
  EXPECT_DOUBLE_EQ(p2, P_REF * (1.0 - std::exp(-(time2 - time0).seconds() / TIME_CONSTANT)));
}

// Robot description (URDF)
class VehicleDescriptionTest : public ::testing::TestWithParam<VehicleName>
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  rclcpp::Node node_{"test_node"};
};

TEST_P(VehicleDescriptionTest, get_robot_description)
{
  /// Arrange
  const auto vehicle = to_vehicle(GetParam(), node_, "");
  std::cout << "Instantiated vehicle of type " + vehicle->name() << std::endl;
  /// Act
  const std::string urdf_xml = vehicle->get_robot_description();
  /// Assert
  ASSERT_NO_THROW(check_urdf(urdf_xml));
}

INSTANTIATE_TEST_SUITE_P(
  AllVehicles, VehicleDescriptionTest, ::testing::ValuesIn(ALL_VEHICLE_NAMES));
}  // namespace vehicle_dynamics_sim
