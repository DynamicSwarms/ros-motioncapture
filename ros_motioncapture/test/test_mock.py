"""Check mock output, then let launch_testing send SIGINT and check clean exit."""

import time
import unittest

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import rclpy
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage


def generate_test_description():
    mocap = Node(
        package='ros_motioncapture',
        executable='motioncapture_node',
        parameters=[{
            'type': 'mock',
            'topic_name': '/mock_test/points',
            'frame_id': 'mock_world',
            'rigid_bodies.test_body.initial_position': [1.0, 2.0, 3.0],
        }],
        sigterm_timeout='5',
        output='screen',
    )
    return LaunchDescription([mocap, launch_testing.actions.ReadyToTest()]), {'mocap': mocap}


class TestMockOutput(unittest.TestCase):
    def test_messages(self):
        rclpy.init()
        node = rclpy.create_node('mock_test_listener')
        received = {}

        def on_tf(message):
            for transform in message.transforms:
                if transform.child_frame_id == 'test_body':
                    received['tf'] = transform

        def on_diagnostics(message):
            for status in message.status:
                if status.hardware_id == 'mock:localhost' and status.message == 'Running':
                    received['diagnostics'] = status

        node.create_subscription(
            PointCloud2, '/mock_test/points', lambda msg: received.update(points=msg), 10)
        node.create_subscription(TFMessage, '/tf', on_tf, 10)
        node.create_subscription(DiagnosticArray, '/diagnostics', on_diagnostics, 10)

        try:
            deadline = time.monotonic() + 10
            while len(received) < 3 and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            self.assertEqual(set(received), {'points', 'tf', 'diagnostics'})

            points = received['points']
            self.assertEqual(points.header.frame_id, 'mock_world')
            self.assertGreater(points.header.stamp.sec, 0)
            self.assertEqual([field.name for field in points.fields], ['x', 'y', 'z'])
            self.assertEqual(points.point_step, 12)
            # The ROS mock configuration supplies bodies, but no point-cloud markers.
            self.assertEqual(points.width, 0)
            self.assertEqual(len(points.data), 0)

            transform = received['tf']
            self.assertEqual(transform.header.frame_id, 'mock_world')
            self.assertGreater(transform.header.stamp.sec, 0)
            position = transform.transform.translation
            self.assertEqual((position.x, position.y, position.z), (1.0, 2.0, 3.0))
            rotation = transform.transform.rotation
            self.assertEqual((rotation.x, rotation.y, rotation.z, rotation.w), (0., 0., 0., 1.))

            status = received['diagnostics']
            self.assertEqual(status.level, DiagnosticStatus.OK)
            values = {item.key: item.value for item in status.values}
            self.assertGreater(int(values['Frames total']), 0)
            self.assertEqual(int(values['Rigid bodies in last frame']), 1)
        finally:
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestMockShutdown(unittest.TestCase):
    def test_clean_exit(self, proc_info, mocap):
        # A hung node is escalated to SIGTERM after five seconds and fails this check.
        launch_testing.asserts.assertExitCodes(proc_info, process=mocap, allowable_exit_codes=[0])
