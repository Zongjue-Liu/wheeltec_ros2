#!/usr/bin/env python3
import csv
import math
import os

import rclpy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('mapping_trajectory_recorder')
        self.declare_parameter(
            'output_file', '/home/wheeltec/mapping_sessions/current_odom_path.csv')
        self.declare_parameter('min_distance', 0.02)
        self.output_file = str(self.get_parameter('output_file').value)
        self.min_distance = float(self.get_parameter('min_distance').value)
        os.makedirs(os.path.dirname(self.output_file), exist_ok=True)
        self.file_handle = open(self.output_file, 'w', newline='', buffering=1)
        self.writer = csv.writer(self.file_handle)
        self.writer.writerow(['stamp_sec', 'x', 'y', 'yaw'])
        self.last_point = None
        self.path = Path()
        self.path.header.frame_id = 'odom_combined'
        self.path_publisher = self.create_publisher(
            Path, '/mapping/driven_path', 10)
        self.create_subscription(Odometry, '/odom_combined', self.on_odom, 20)
        self.get_logger().info(f'recording mapping trajectory: {self.output_file}')

    def on_odom(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.last_point is not None:
            if math.hypot(x - self.last_point[0], y - self.last_point[1]) < self.min_distance:
                return
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self.writer.writerow([f'{stamp_sec:.9f}', f'{x:.6f}', f'{y:.6f}', f'{yaw:.9f}'])
        self.last_point = (x, y)

        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = 'odom_combined'
        pose.pose = msg.pose.pose
        self.path.header.stamp = msg.header.stamp
        self.path.poses.append(pose)
        self.path_publisher.publish(self.path)

    def close(self):
        if not self.file_handle.closed:
            self.file_handle.flush()
            self.file_handle.close()


def main():
    rclpy.init()
    node = TrajectoryRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
