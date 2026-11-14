"""Stream a known pose through a real local VRPN connection and check ROS TF."""

import math
import time
import unittest

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import rclpy
from tf2_msgs.msg import TFMessage


def generate_test_description():
    server = ExecuteProcess(
        cmd=[LaunchConfiguration('server')], sigterm_timeout='5', output='screen')
    mocap = Node(
        package='ros_motioncapture',
        executable='motioncapture_node',
        parameters=[{
            'type': 'vrpn',
            'hostname': '127.0.0.1:43883',
            'frame_id': 'vrpn_world',
        }],
        sigterm_timeout='5',
        output='screen',
    )
    return LaunchDescription([
        DeclareLaunchArgument('server'), server, mocap,
        launch_testing.actions.ReadyToTest(),
    ]), {'mocap': mocap, 'server': server}


class TestVrpnPose(unittest.TestCase):
    def test_pose(self):
        rclpy.init()
        node = rclpy.create_node('vrpn_test_listener')
        received = []

        def on_tf(message):
            received.extend(t for t in message.transforms if t.child_frame_id == 'vrpn_test_body')

        node.create_subscription(TFMessage, '/tf', on_tf, 10)
        try:
            deadline = time.monotonic() + 10
            while not received and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            self.assertTrue(received, 'No TF received from the VRPN tracker')
            transform = received[-1]
            self.assertEqual(transform.header.frame_id, 'vrpn_world')
            self.assertGreater(transform.header.stamp.sec, 0)
            position = transform.transform.translation
            self.assertEqual((position.x, position.y, position.z), (1.0, 2.0, 3.0))
            rotation = transform.transform.rotation
            self.assertAlmostEqual(rotation.x, 0.0)
            self.assertAlmostEqual(rotation.y, 0.0)
            self.assertAlmostEqual(rotation.z, math.sqrt(0.5), places=6)
            self.assertAlmostEqual(rotation.w, math.sqrt(0.5), places=6)
        finally:
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestVrpnShutdown(unittest.TestCase):
    def test_clean_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0])
