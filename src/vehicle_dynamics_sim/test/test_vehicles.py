"""
Test steady-state velocity tracking for all vehicle models.

Tests that each vehicle type correctly tracks commanded velocities
in steady state (after actuator dynamics settle).

Vehicle configurations tested:
- Bicycle (4 variants: reverse × drive_on_steered_wheel)
- Differential
- Omni

Motion primitives tested per vehicle:
- Forward motion (linear.x)
- Curve motion (linear.x + angular.z)
- In-place rotation (angular.z only) - where supported
- Lateral motion (linear.y) - omni only
"""
import os
import time
import unittest

import rclpy
import geometry_msgs.msg

import launch
import launch_ros
import launch_testing
from ament_index_python.packages import get_package_share_directory


# Vehicle configurations by name
VEHICLE_CONFIGS = {
    'bicycle_front_steer_rear_drive':    ('bicycle', False, False, 'bicycle_front_steer_rear_drive'),
    'bicycle_front_steer_front_drive':   ('bicycle', False, True,  'bicycle_front_steer_front_drive'),
    'bicycle_rear_steer_rear_drive':     ('bicycle', True,  False, 'bicycle_rear_steer_rear_drive'),
    'bicycle_rear_steer_front_drive':    ('bicycle', True,  True,  'bicycle_rear_steer_front_drive'),
    'differential':                      ('differential', None, None, 'differential'),
    'omni':                              ('omni', None, None, 'omni'),
}


def generate_test_description_for_config(config_name):
    """Launch simulator with a specific vehicle configuration by name."""
    if config_name not in VEHICLE_CONFIGS:
        raise ValueError(f"Unknown vehicle config: {config_name}")
    model, reverse, drive_on_steered, description = VEHICLE_CONFIGS[config_name]

    # Build parameter list
    params = {
        'use_sim_time': False,
        'be_reference_clock': False,
        'simulate_localization': False,  # No noise for deterministic tests
        'model': model,
        'step_rate': 1000.0,
        'pub_rate': 50.0,
    }

    # Add bicycle-specific parameters
    if model == 'bicycle':
        params['model_params.bicycle.reverse'] = reverse
        params['model_params.bicycle.drive_on_steered_wheel'] = drive_on_steered
        # Fast actuators for quick settling
        params['model_params.bicycle.drive_actuator.max_acceleration'] = 2.0
        params['model_params.bicycle.drive_actuator.max_velocity'] = 2.0
        params['model_params.bicycle.steering_actuator.max_velocity'] = 1.0
        # If drive on steering wheel, it can rotate all the way round
        params['model_params.bicycle.steering_actuator.max_position'] = 0.0 if drive_on_steered else 0.8

    elif model == 'differential':
        params['model_params.differential.track'] = 1.4
        # Fast actuators
        params['model_params.differential.drive_actuators.max_acceleration'] = 2.0
        params['model_params.differential.drive_actuators.max_velocity'] = 2.0

    elif model == 'omni':
        params['model_params.omni.wheel_base'] = 1.4
        params['model_params.omni.track'] = 1.2
        # Fast actuators
        params['model_params.omni.drive_actuators.max_acceleration'] = 2.0
        params['model_params.omni.drive_actuators.max_velocity'] = 2.0

    return (
        launch.LaunchDescription([
            # Global sim time setting
            launch_ros.actions.SetParameter(
                name='use_sim_time',
                value=params['use_sim_time']
            ),
            # Launch simulator node directly with parameters
            launch_ros.actions.Node(
                package='vehicle_dynamics_sim',
                executable='sim_node',
                name='sim_node',
                output='screen',
                parameters=[params],
                # Remap to match expected topic names
                remappings=[
                    ('twist_reference', 'cmd_vel_nav'),
                ],
            ),
            # Robot state publisher for TF
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory(
                            'vehicle_dynamics_sim'),
                        'launch',
                        'robot_state_publisher.launch.py'
                    )
                ),
            ),
            # Launch tests 0.5 s later
            launch.actions.TimerAction(
                period=0.5, actions=[launch_testing.actions.ReadyToTest()]),
        ]),
        {'vehicle_config': VEHICLE_CONFIGS[config_name]}
    )


class TestVehicleVelocityTracking(unittest.TestCase):
    """Test steady-state velocity tracking for all vehicle types."""
    # Test parameters
    # Duration to wait for actuator dynamics to settle [s]
    SETTLING_DURATION = 4.0
    MEASUREMENT_DURATION = 1.0  # Duration to collect measurements [s]
    TOLERANCE = 0.05  # tolerance for velocity matching

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_vehicle_velocity_tracking')
        # Allow node to init
        time.sleep(0.5)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _test_velocity_tracking(self, cmd_linear_x, cmd_linear_y, cmd_angular_z,
                                tolerance_linear=None, tolerance_angular=None):
        """
        Generic test for velocity tracking.

        Publishes a constant velocity command, waits for settling,
        then verifies twist_actual matches the command within tolerance.
        """
        if tolerance_linear is None:
            tolerance_linear = self.TOLERANCE
        if tolerance_angular is None:
            tolerance_angular = self.TOLERANCE

        # Store received twist messages
        twist_messages = []

        def twist_callback(msg):
            twist_messages.append(msg.twist)

        # Subscribe to twist_actual
        sub = self.node.create_subscription(
            geometry_msgs.msg.TwistStamped, 'twist_actual', twist_callback, 10)
        # Create publisher for velocity commands
        pub = self.node.create_publisher(
            geometry_msgs.msg.Twist, 'cmd_vel_nav', 10)

        # Wait briefly for connections to establish
        time.sleep(0.2)

        # Create command message
        cmd_msg = geometry_msgs.msg.Twist()
        cmd_msg.linear.x = cmd_linear_x
        cmd_msg.linear.y = cmd_linear_y
        cmd_msg.angular.z = cmd_angular_z

        # Publish command continuously
        def publish_command():
            pub.publish(cmd_msg)

        timer = self.node.create_timer(0.100, publish_command)

        try:
            # Phase 1: Settling - wait for actuator dynamics
            start_time = time.time()
            while time.time() - start_time < self.SETTLING_DURATION:
                rclpy.spin_once(self.node, timeout_sec=0.100)

            # Clear any messages received during settling
            twist_messages.clear()

            # Phase 2: Measurement - collect steady-state data
            start_time = time.time()
            while time.time() - start_time < self.MEASUREMENT_DURATION:
                rclpy.spin_once(self.node, timeout_sec=0.100)

            # Verify we received messages
            self.assertGreater(len(twist_messages), 10,
                               f"Expected at least 10 messages during measurement, got {len(twist_messages)}")

            # Compute mean velocities
            avg_linear_x = sum(
                t.linear.x for t in twist_messages) / len(twist_messages)
            avg_linear_y = sum(
                t.linear.y for t in twist_messages) / len(twist_messages)
            avg_angular_z = sum(
                t.angular.z for t in twist_messages) / len(twist_messages)

            # Verify linear.x
            if abs(cmd_linear_x) > 0.01:  # Only check if significantly non-zero
                error_x = abs(avg_linear_x - cmd_linear_x)
                max_error_x = abs(cmd_linear_x) * tolerance_linear
                self.assertLess(error_x, max_error_x,
                                f"Linear.x error too large: commanded={cmd_linear_x:.3f}, "
                                f"actual={avg_linear_x:.3f}, error={error_x:.3f}, "
                                f"max_allowed={max_error_x:.3f}")
            else:
                # If commanded near zero, check absolute error
                self.assertLess(abs(avg_linear_x), 0.05,
                                f"Linear.x should be near zero: actual={avg_linear_x:.3f}")

            # Verify linear.y
            if abs(cmd_linear_y) > 0.01:
                error_y = abs(avg_linear_y - cmd_linear_y)
                max_error_y = abs(cmd_linear_y) * tolerance_linear
                self.assertLess(error_y, max_error_y,
                                f"Linear.y error too large: commanded={cmd_linear_y:.3f}, "
                                f"actual={avg_linear_y:.3f}, error={error_y:.3f}, "
                                f"max_allowed={max_error_y:.3f}")
            else:
                self.assertLess(abs(avg_linear_y), 0.05,
                                f"Linear.y should be near zero: actual={avg_linear_y:.3f}")

            # Verify angular.z
            if abs(cmd_angular_z) > 0.01:
                error_z = abs(avg_angular_z - cmd_angular_z)
                max_error_z = abs(cmd_angular_z) * tolerance_angular
                self.assertLess(error_z, max_error_z,
                                f"Angular.z error too large: commanded={cmd_angular_z:.3f}, "
                                f"actual={avg_angular_z:.3f}, error={error_z:.3f}, "
                                f"max_allowed={max_error_z:.3f}")
            else:
                self.assertLess(abs(avg_angular_z), 0.05,
                                f"Angular.z should be near zero: actual={avg_angular_z:.3f}")

        finally:
            # Cleanup
            self.node.destroy_timer(timer)
            self.node.destroy_subscription(sub)
            self.node.destroy_publisher(pub)

    def test_forward_motion(self, vehicle_config):
        """Test forward motion: linear.x > 0, others zero."""
        model, reverse, drive_on_steered, description = vehicle_config
        # All vehicles should support forward motion
        self._test_velocity_tracking(
            cmd_linear_x=1.0,
            cmd_linear_y=0.0,
            cmd_angular_z=0.0
        )

    def test_curve_motion(self, vehicle_config):
        """Test curve motion: linear.x > 0, angular.z != 0."""
        model, reverse, drive_on_steered, description = vehicle_config
        # All vehicles should support curved motion
        self._test_velocity_tracking(
            cmd_linear_x=1.0,
            cmd_linear_y=0.0,
            cmd_angular_z=0.25
        )

    def test_in_place_rotation(self, vehicle_config):
        """Test in-place rotation: angular.z != 0, linear velocities zero."""
        model, reverse, drive_on_steered, description = vehicle_config
        # Only differential, omni, and bicycle with drive_on_steered_wheel can do this
        if model == 'bicycle' and not drive_on_steered:
            self.skipTest(
                f"Bicycle with fixed axle drive cannot execute in place rotation")

        # Note: For bicycle with drive_on_steered_wheel, this requires unlimited steering
        # which we set in the config (max_position: 0.0)
        self._test_velocity_tracking(
            cmd_linear_x=0.0,
            cmd_linear_y=0.0,
            cmd_angular_z=0.5
        )

    def test_lateral_motion(self, vehicle_config):
        """Test lateral motion: linear.y != 0 (omni only)."""
        model, reverse, drive_on_steered, description = vehicle_config
        # Only omni supports lateral motion
        if model != 'omni':
            self.skipTest(f"{model} does not support lateral motion")

        self._test_velocity_tracking(
            cmd_linear_x=0.0,
            cmd_linear_y=1.0,
            cmd_angular_z=0.0
        )

    def test_combined_motion_omni(self, vehicle_config):
        """Test combined motion: all three velocities (omni only)."""
        model, reverse, drive_on_steered, description = vehicle_config
        # Only omni supports full 3-DOF motion
        if model != 'omni':
            self.skipTest(f"{model} does not support 3-DOF motion")

        self._test_velocity_tracking(
            cmd_linear_x=0.5,
            cmd_linear_y=0.5,
            cmd_angular_z=0.2
        )


@launch_testing.post_shutdown_test()
class TestVehicleVelocityTrackingShutdown(unittest.TestCase):
    """Verify nodes shut down cleanly."""

    def test_exit_codes(self, proc_info):
        """Check if the processes exited normally."""
        launch_testing.asserts.assertExitCodes(proc_info)
