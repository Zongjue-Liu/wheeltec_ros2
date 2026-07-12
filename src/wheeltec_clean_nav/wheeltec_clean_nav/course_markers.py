#!/usr/bin/env python3
"""Publish visual-only road markings for the dimensioned course in RViz."""

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class CourseMarkers(Node):
    def __init__(self) -> None:
        super().__init__('course_markers')
        self.publisher = self.create_publisher(MarkerArray, '/course_markers', 1)
        self.timer = self.create_timer(1.0, self.publish_markers)

    @staticmethod
    def point(x: float, y: float, z: float = 0.025) -> Point:
        point = Point()
        point.x, point.y, point.z = x, y, z
        return point

    def line_marker(self, marker_id: int, points: list[Point], width: float,
                    color: tuple[float, float, float, float]) -> Marker:
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.ns = 'course_markings'
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = width
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.points = points
        return marker

    def arrow_marker(self, marker_id: int, start: tuple[float, float],
                     end: tuple[float, float], color: tuple[float, float, float, float]) -> Marker:
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.ns = 'course_arrows'
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x, marker.scale.y, marker.scale.z = 0.05, 0.10, 0.12
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.points = [self.point(*start, 0.04), self.point(*end, 0.04)]
        return marker

    def crosswalk(self, marker_id: int, center: tuple[float, float],
                  vertical: bool) -> Marker:
        cx, cy = center
        points = []
        for offset in (-0.12, -0.06, 0.0, 0.06, 0.12):
            if vertical:
                points.extend([self.point(cx + offset, cy - 0.16), self.point(cx + offset, cy + 0.16)])
            else:
                points.extend([self.point(cx - 0.16, cy + offset), self.point(cx + 0.16, cy + offset)])
        return self.line_marker(marker_id, points, 0.025, (0.75, 0.75, 0.75, 0.9))

    def publish_markers(self) -> None:
        markers = MarkerArray()
        marker_id = 0

        # Crosswalks at the four center-road approaches and outer-road junctions.
        for center, vertical in (
                ((1.69, 3.28), True), ((1.69, 2.87), False),
                ((1.69, 1.50), False), ((1.69, 0.27), True),
                ((0.27, 1.50), True), ((2.96, 1.50), True),
                ((0.27, 2.40), True), ((2.96, 2.40), True)):
            markers.markers.append(self.crosswalk(marker_id, center, vertical))
            marker_id += 1

        for start, end in (
                ((0.24, 0.45), (0.24, 0.82)),
                ((0.24, 3.10), (0.24, 2.73)),
                ((2.99, 3.10), (2.99, 2.73)),
                ((2.99, 0.45), (2.99, 0.82))):
            markers.markers.append(
                self.arrow_marker(marker_id, start, end, (0.85, 0.85, 0.85, 0.95)))
            marker_id += 1

        # Suggested initial pose: lower-left road, facing toward map +Y.
        markers.markers.append(
            self.arrow_marker(marker_id, (0.25, 0.27), (0.25, 0.70), (0.1, 0.9, 0.2, 1.0)))
        markers.markers[-1].ns = 'suggested_start'
        markers.markers[-1].id = 0
        self.publisher.publish(markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CourseMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
