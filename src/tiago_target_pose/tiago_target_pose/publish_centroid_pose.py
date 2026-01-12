#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
import os
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import time

class CentroidPublisher(Node):
    def __init__(self):
        super().__init__('centroid_publisher')

        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.publisher = self.create_publisher(PoseStamped, '/target_pose', qos)

        self.file_path = os.path.expanduser(
            "~/exchange/ros2_ws/src/object_segmentation/object_segmentation/centroid.txt"
        )

        self.create_timer(1.5, self.publish_from_timer)

    def publish_from_timer(self):
        """Wywoływane raz po starcie – GAZEBO clock już działa."""

        if not os.path.exists(self.file_path):
            self.get_logger().error(f"Brak centroid.txt: {self.file_path}")
            return

        centroid = np.loadtxt(self.file_path)

        if centroid.shape[0] != 3:
            self.get_logger().error("Błędny format centroid.txt")
            return

        self.publish_centroid(centroid)

        self.get_logger().info("Centroid wysłany – zamykam node.")
        rclpy.shutdown()
        
    def publish_centroid(self, centroid):
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.header.stamp = self.get_clock().now().to_msg()

        table_height = 0.75
        cube_height = 0.055
        gripper_length = 0.22
        
        fingers_target_z = table_height + (cube_height / 2.0) + 0.10 - 0.375 - 0.1

        wrist_target_z = fingers_target_z + gripper_length

        dod = 0.65 
        pose.pose.position.x = float(centroid[0] + dod)
        pose.pose.position.y = float(centroid[1] )
        pose.pose.position.z = float(wrist_target_z)

        pose.pose.orientation.y = -0.707
        pose.pose.orientation.w = 0.707
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.z = 0.0

        self.publisher.publish(pose)
        
        self.get_logger().info(f"--- KALKULACJA ---")
        self.get_logger().info(f"Centroid: x:{centroid[0]}, y: {centroid[1]}")
        self.get_logger().info(f"Stół: {table_height}m, Chwytak: {gripper_length}m")
        self.get_logger().info(f"Cel PALCÓW (nad kostką): {fingers_target_z:.3f} m")
        self.get_logger().info(f"Cel NADGARSTKA (to wysyłam): {wrist_target_z:.3f} m")

def main(args=None):
    rclpy.init(args=args)
    node = CentroidPublisher()
    time.sleep(3)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

