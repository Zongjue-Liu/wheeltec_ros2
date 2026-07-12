import math
import os
import threading

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


class RouteRunner(Node):
    def __init__(self):
        super().__init__('clean_route_runner')
        default_file = os.path.join(
            get_package_share_directory('wheeltec_clean_nav'),
            'config', 'waypoints.yaml')
        self.declare_parameter('waypoints_file', default_file)
        self._status = self.create_publisher(
            String, '/clean_navigation/status', 10)
        self._client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        with open(self.get_parameter('waypoints_file').value, encoding='utf-8') as stream:
            config = yaml.safe_load(stream)
        self._frame = config['route'].get('frame_id', 'map')
        self._loop = bool(config['route'].get('loop', False))
        self._retries = int(config['route'].get('max_retries', 1))
        self._delay = float(config['route'].get('waypoint_delay_sec', 1.0))
        self._waypoints = config['waypoints']
        threading.Thread(target=self._run, daemon=True).start()

    def _publish(self, value):
        self._status.publish(String(data=value))
        self.get_logger().info(value)

    def _goal(self, point):
        pose = PoseStamped()
        pose.header.frame_id = self._frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(point['x'])
        pose.pose.position.y = float(point['y'])
        yaw = float(point.get('yaw', 0.0))
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return NavigateToPose.Goal(pose=pose)

    def _run(self):
        self._publish('WAITING_FOR_NAV2')
        while rclpy.ok() and not self._client.wait_for_server(timeout_sec=1.0):
            pass
        if not rclpy.ok():
            return
        while rclpy.ok():
            for index, point in enumerate(self._waypoints, start=1):
                succeeded = False
                for attempt in range(self._retries + 1):
                    self._publish(f'SENDING:{index}:ATTEMPT:{attempt + 1}')
                    goal_future = self._client.send_goal_async(self._goal(point))
                    rclpy.spin_until_future_complete(self, goal_future)
                    handle = goal_future.result()
                    if handle is None or not handle.accepted:
                        continue
                    result_future = handle.get_result_async()
                    rclpy.spin_until_future_complete(self, result_future)
                    if result_future.result().status == 4:
                        succeeded = True
                        break
                if not succeeded:
                    self._publish(f'FAILED:{index}')
                    return
                self._publish(f'REACHED:{index}')
                self.get_clock().sleep_for(rclpy.duration.Duration(seconds=self._delay))
            if not self._loop:
                self._publish('COMPLETE')
                return


def main(args=None):
    rclpy.init(args=args)
    node = RouteRunner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
