"""
Smoke tests for the vehicle dynamics simulator.

Tests basic functionality with default parameters:
- Topic publishing (output topics)
- Topic subscription (input topics)
- TF tree structure (map → odom → base_link)
"""
import os
import time
import unittest

import rclpy
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import geometry_msgs.msg
import nav_msgs.msg
import sensor_msgs.msg
import std_msgs.msg
import tf2_msgs.msg
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from ament_index_python.packages import get_package_share_directory
import launch
import launch_testing
import launch_testing_ros
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_test_description():
    """Launch the vehicle dynamics simulator with default parameters."""
    return (
        launch.LaunchDescription(
            [
                # Launch the simulator
                IncludeLaunchDescription(
                    XMLLaunchDescriptionSource([os.path.join(
                        get_package_share_directory('vehicle_dynamics_sim'),
                        'launch/sim.launch.xml')]),
                ),
                # Launch tests 0.5 s later
                launch.actions.TimerAction(
                    period=0.5, actions=[launch_testing.actions.ReadyToTest()]),
            ]
        ),
        {}
    )


# Active tests
class TestVehicleDynamicsSimSmoke(unittest.TestCase):
    """Smoke tests for vehicle dynamics simulator with default parameters."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_vehicle_sim_smoke')

    def tearDown(self):
        self.node.destroy_node()

    def test_regular_topics_published(self, proc_output):
        """Verify regular (non-transient-local) output topics are being published."""
        expected_topics = [
            ('/tf', tf2_msgs.msg.TFMessage),
            ('/joint_states', sensor_msgs.msg.JointState),
            ('twist_actual', geometry_msgs.msg.TwistStamped),
            ('odom', nav_msgs.msg.Odometry),
        ]
        wait_for_topics = launch_testing_ros.WaitForTopics(
            expected_topics, timeout=10.0)
        assert wait_for_topics.wait(), \
            f"The following topics did not get published within the timeout: " \
            f"{wait_for_topics.topics_not_received()}"
        wait_for_topics.shutdown()

    def test_transient_local_topics_published(self, proc_output):
        """Verify transient local topics (/tf_static, /robot_description) are available."""
        # QoS profile for transient local topics
        transient_local_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        received_msgs = {'tf_static': None, 'robot_description': None}

        def tf_static_callback(msg):
            received_msgs['tf_static'] = msg

        def robot_description_callback(msg):
            received_msgs['robot_description'] = msg

        # Create subscribers with transient local QoS
        tf_static_sub = self.node.create_subscription(
            tf2_msgs.msg.TFMessage, '/tf_static',
            tf_static_callback, transient_local_qos)
        robot_desc_sub = self.node.create_subscription(
            std_msgs.msg.String, '/robot_description',
            robot_description_callback, transient_local_qos)
        # Spin until we receive messages or timeout
        timeout = 5.0
        start_time = time.time()
        while time.time() - start_time < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if all(msg is not None for msg in received_msgs.values()):
                break
        # Cleanup
        self.node.destroy_subscription(tf_static_sub)
        self.node.destroy_subscription(robot_desc_sub)
        # Assert
        self.assertIsNotNone(received_msgs['tf_static'],
                             "/tf_static not received - check if topic is published with transient local QoS")
        self.assertIsNotNone(received_msgs['robot_description'],
                             "/robot_description not received - check if topic is published with transient local QoS")

    def test_input_topics_advertised(self, proc_output):
        """Verify simulator subscribes to velocity command topics."""
        # The simulator should be listening for commands
        # Default remapping makes it listen to cmd_vel_nav
        # (which is remapped from twist_reference)
        # Wait for the node to be fully initialized
        time.sleep(1.0)
        # We publish on cmd_vel_nav which is what the launch file remaps to
        pub = self.node.create_publisher(
            geometry_msgs.msg.Twist, 'cmd_vel_nav', 10)
        # Give it a moment to connect
        time.sleep(0.5)
        # Verify we have at least one connection (which means the simulator is listening)
        num_connections = pub.get_subscription_count()
        # Clean up and assert
        self.node.destroy_publisher(pub)
        self.assertGreater(num_connections, 0,
                           "Simulator did not subscribe to topic cmd_vel_nav")

    def test_tf_tree_structure(self, proc_output):
        """Verify the TF tree has the correct structure: map → odom → base_link."""
        # Create TF buffer and listener
        tf_buffer = Buffer()
        TransformListener(tf_buffer, self.node)
        # Give TF some time to accumulate transforms
        timeout_sec = 5.0
        end_time = time.time() + timeout_sec
        # Keep spinning to receive TF messages
        frames_found = {'map': False, 'odom': False, 'base_link': False}
        while time.time() < end_time and not all(frames_found.values()):
            rclpy.spin_once(self.node, timeout_sec=0.1)

            # Check for each frame
            for frame in frames_found:
                if not frames_found[frame]:
                    if tf_buffer.can_transform('map', frame, rclpy.time.Time(),
                                               rclpy.duration.Duration(seconds=0.0)):
                        frames_found[frame] = True

        # Verify all frames exist
        self.assertTrue(frames_found['map'], "map frame not found in TF tree")
        self.assertTrue(frames_found['odom'],
                        "odom frame not found in TF tree")
        self.assertTrue(frames_found['base_link'],
                        "base_link frame not found in TF tree")

        # Verify the chain: map → odom → base_link
        # This is implicit if we can transform from map to base_link through odom
        try:
            # Should be able to get transform from map to base_link
            transform = tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                rclpy.duration.Duration(seconds=1.0)
            )
            # If we got here, the chain exists
            self.assertIsNotNone(transform)
        except Exception as e:
            self.fail(f"Failed to find transform from map to base_link: {e}")


# Post-shutdown tests
@launch_testing.post_shutdown_test()
class TestVehicleDynamicsSimShutdown(unittest.TestCase):
    """Verify nodes shut down cleanly."""

    def test_exit_codes(self, proc_info):
        """Check the processes exited normally."""
        launch_testing.asserts.assertExitCodes(proc_info)
