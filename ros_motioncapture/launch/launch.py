from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Node for motion_capture_tracking
        Node(
            package='ros_motioncapture',
            executable='motioncapture_node',  # the name of the executable file within the package
            name='node',
            output='screen',
            parameters=[{
                'type': 'vicon',
                'hostname': '172.20.37.251',
                'add_labeled_markers_to_pointcloud': True
            }]
        ),

        # python Node for motion_capture_tracking
        #Node(
        #    package='ros_motioncapture',
        #    executable='motioncapture_node.py',  # the name of the executable file within the package
        #    name='node2',
        #    output='screen',
        #    parameters=[{
        #        'type': 'vicon',
        #        'hostname': '172.20.37.251',
        #        'add_labeled_markers_to_pointcloud': True
        #    }]
        #),

        # Node for rviz
        Node(
            package='rviz2',
            executable='rviz2',  # 'rviz2' is the executable in ROS 2 for rviz
            name='rviz',
            output='screen'
        ),
    ])
