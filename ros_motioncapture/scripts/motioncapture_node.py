#!/usr/bin/env python3

import rclpy
import motioncapture
import numpy as np
import tf2_ros

from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2, PointField


class PythonNode(Node):
    def __init__(self):
        super().__init__('motioncapture_node_python')
    
        self.declare_parameter('type', 'vicon')
        self.declare_parameter('hostname', 'localhost')
        self.declare_parameter('add_labeled_markers_to_pointcloud', True)
        self.declare_parameter('topic_name', 'pointCloud')

        type = self.get_parameter('type').value
        hostname = self.get_parameter('hostname').value
        full_pointcloud = self.get_parameter('add_labeled_markers_to_pointcloud').value
        topic_name = self.get_parameter('add_labeled_markers_to_pointcloud').value

        self.pub_point_cloud = self.create_publisher(PointCloud2, topic_name, 1)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.mc = motioncapture.connect(type, {"hostname": hostname, 
                                               "add_labeled_markers_to_pointcloud": "true" if full_pointcloud else "false"})
    
    def _publish_point_cloud(self) -> None:
        pointcloud = self.mc.pointCloud
        msg_point_cloud = PointCloud2()
        msg_point_cloud.header.stamp = self.get_clock().now().to_msg()
        msg_point_cloud.header.frame_id = 'world'
        msg_point_cloud.width = pointcloud.shape[0]
        msg_point_cloud.height = 1
        msg_point_cloud.fields.append(PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1))
        msg_point_cloud.fields.append(PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1))
        msg_point_cloud.fields.append(PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1))
        msg_point_cloud.is_bigendian = False
        msg_point_cloud.point_step = 12
        msg_point_cloud.row_step = msg_point_cloud.point_step * pointcloud.shape[0]
        msg_point_cloud.is_dense = True
        msg_point_cloud.data = pointcloud.astype(np.float32).tobytes()
        self.pub_point_cloud.publish(msg_point_cloud)

    def _broadcast_transforms(self) -> None:
        transforms = []

        for name, rigid_body in self.mc.rigidBodies.items():
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = 'world'
            transform.child_frame_id = name
            transform.transform.translation.x = float(rigid_body.position[0])
            transform.transform.translation.y = float(rigid_body.position[1])
            transform.transform.translation.z = float(rigid_body.position[2])
            transform.transform.rotation.x = rigid_body.rotation.x
            transform.transform.rotation.y = rigid_body.rotation.y
            transform.transform.rotation.z = rigid_body.rotation.z
            transform.transform.rotation.w = rigid_body.rotation.w

            # Handling NaN in rotation (RViz workaround)
            if np.isnan(transform.transform.rotation.x):
                transform.transform.rotation.x = 0
                transform.transform.rotation.y = 0
                transform.transform.rotation.z = 0
                transform.transform.rotation.w = 1

            transforms.append(transform)

        if transforms:
            self.tf_broadcaster.sendTransform(transforms)
    
    def run(self) -> None:
        self.mc.waitForNextFrame()
        self._publish_point_cloud()
        self._broadcast_transforms()


def main(args=None):
    rclpy.init(args=args)
    node = PythonNode()

    while rclpy.ok():
        node.run()
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
