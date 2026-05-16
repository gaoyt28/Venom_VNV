from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Dict
from typing import List

import rclpy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from venom_overtake_msgs.msg import TrackedObstacle
from venom_overtake_msgs.msg import TrackedObstacleArray

from venom_overtake_manager.lane_geometry import classify_same_lane


@dataclass
class TrackState:
    obstacle_id: int
    x: float
    y: float
    length: float = 1.0
    width: float = 0.8
    relative_vx: float = 0.0
    relative_vy: float = 0.0
    absolute_vx: float = 0.0
    absolute_vy: float = 0.0
    stamp_sec: float = 0.0


@dataclass(frozen=True)
class Detection:
    x: float
    y: float
    length: float
    width: float
    point_count: int


class TrackerNode(Node):
    def __init__(self) -> None:
        super().__init__('tracker_node')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odometry/global')
        self.declare_parameter('cluster_distance_threshold_m', 0.45)
        self.declare_parameter('min_cluster_points', 3)
        self.declare_parameter('min_detection_range_m', 0.5)
        self.declare_parameter('max_detection_range_m', 25.0)
        self.declare_parameter('front_fov_half_angle_rad', 1.35)
        self.declare_parameter('track_association_distance_m', 1.5)
        self.declare_parameter('same_lane_lateral_threshold_m', 0.9)
        self.declare_parameter('static_speed_threshold_mps', 0.25)
        self.declare_parameter('track_timeout_sec', 1.0)

        self._scan_topic = str(self.get_parameter('scan_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        self._cluster_distance_threshold = float(
            self.get_parameter('cluster_distance_threshold_m').value
        )
        self._min_cluster_points = int(self.get_parameter('min_cluster_points').value)
        self._min_detection_range = float(self.get_parameter('min_detection_range_m').value)
        self._max_detection_range = float(self.get_parameter('max_detection_range_m').value)
        self._front_fov_half_angle = float(self.get_parameter('front_fov_half_angle_rad').value)
        self._track_association_distance = float(
            self.get_parameter('track_association_distance_m').value
        )
        self._same_lane_threshold = float(self.get_parameter('same_lane_lateral_threshold_m').value)
        self._static_speed_threshold = float(self.get_parameter('static_speed_threshold_mps').value)
        self._track_timeout_sec = float(self.get_parameter('track_timeout_sec').value)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self._tracks: Dict[int, TrackState] = {}
        self._next_track_id = 1
        self._ego_linear_x = 0.0
        self._ego_linear_y = 0.0

        self.create_subscription(LaserScan, self._scan_topic, self._on_scan, 20)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 20)
        self._pub = self.create_publisher(TrackedObstacleArray, '/perception/tracked_obstacles', 10)
        self.create_timer(1.0 / max(publish_rate_hz, 1.0), self._publish_tracks)

    def _on_odom(self, msg: Odometry) -> None:
        self._ego_linear_x = float(msg.twist.twist.linear.x)
        self._ego_linear_y = float(msg.twist.twist.linear.y)

    def _is_valid_range(self, range_value: float) -> bool:
        return (
            math.isfinite(range_value)
            and self._min_detection_range <= range_value <= self._max_detection_range
        )

    def _scan_to_points(self, msg: LaserScan) -> List[Point]:
        points: List[Point] = []
        angle = msg.angle_min
        for range_value in msg.ranges:
            if self._is_valid_range(range_value) and abs(angle) <= self._front_fov_half_angle:
                point = Point()
                point.x = math.cos(angle) * range_value
                point.y = math.sin(angle) * range_value
                point.z = 0.0
                points.append(point)
            angle += msg.angle_increment
        return points

    def _cluster_points(self, points: List[Point]) -> List[List[Point]]:
        if not points:
            return []

        clusters: List[List[Point]] = []
        current_cluster: List[Point] = [points[0]]

        for point in points[1:]:
            previous = current_cluster[-1]
            distance = math.hypot(point.x - previous.x, point.y - previous.y)
            if distance <= self._cluster_distance_threshold:
                current_cluster.append(point)
            else:
                if len(current_cluster) >= self._min_cluster_points:
                    clusters.append(current_cluster)
                current_cluster = [point]

        if len(current_cluster) >= self._min_cluster_points:
            clusters.append(current_cluster)
        return clusters

    def _cluster_to_detection(self, cluster: List[Point]) -> Detection:
        xs = [point.x for point in cluster]
        ys = [point.y for point in cluster]
        center_x = sum(xs) / len(xs)
        center_y = sum(ys) / len(ys)
        length = max(max(xs) - min(xs), 0.2)
        width = max(max(ys) - min(ys), 0.2)
        return Detection(
            x=center_x,
            y=center_y,
            length=length,
            width=width,
            point_count=len(cluster),
        )

    def _associate_tracks(self, detections: List[Detection], now_sec: float) -> Dict[int, TrackState]:
        remaining_tracks = dict(self._tracks)
        next_tracks: Dict[int, TrackState] = {}

        for detection in detections:
            best_track_id = None
            best_distance = float('inf')
            for track_id, track in remaining_tracks.items():
                distance = math.hypot(detection.x - track.x, detection.y - track.y)
                if distance < best_distance and distance <= self._track_association_distance:
                    best_track_id = track_id
                    best_distance = distance

            if best_track_id is None:
                track_id = self._next_track_id
                self._next_track_id += 1
                next_tracks[track_id] = TrackState(
                    obstacle_id=track_id,
                    x=detection.x,
                    y=detection.y,
                    length=detection.length,
                    width=detection.width,
                    absolute_vx=self._ego_linear_x,
                    absolute_vy=self._ego_linear_y,
                    stamp_sec=now_sec,
                )
                continue

            previous = remaining_tracks.pop(best_track_id)
            relative_vx = 0.0
            relative_vy = 0.0
            dt = now_sec - previous.stamp_sec
            if dt > 1e-3:
                relative_vx = (detection.x - previous.x) / dt
                relative_vy = (detection.y - previous.y) / dt
            next_tracks[best_track_id] = TrackState(
                obstacle_id=best_track_id,
                x=detection.x,
                y=detection.y,
                length=detection.length,
                width=detection.width,
                relative_vx=relative_vx,
                relative_vy=relative_vy,
                absolute_vx=relative_vx + self._ego_linear_x,
                absolute_vy=relative_vy + self._ego_linear_y,
                stamp_sec=now_sec,
            )

        for track_id, track in remaining_tracks.items():
            if now_sec - track.stamp_sec <= self._track_timeout_sec:
                next_tracks[track_id] = track

        return next_tracks

    def _on_scan(self, msg: LaserScan) -> None:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        points = self._scan_to_points(msg)
        clusters = self._cluster_points(points)
        detections = [self._cluster_to_detection(cluster) for cluster in clusters]
        self._tracks = self._associate_tracks(detections, now_sec)

    def _publish_tracks(self) -> None:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        msg = TrackedObstacleArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        for track in self._tracks.values():
            if now_sec - track.stamp_sec > self._track_timeout_sec:
                continue
            obstacle = TrackedObstacle()
            obstacle.header = msg.header
            obstacle.id = track.obstacle_id
            obstacle.pose.position.x = track.x
            obstacle.pose.position.y = track.y
            obstacle.pose.position.z = 0.0
            obstacle.pose.orientation.w = 1.0
            obstacle.twist = Twist()
            obstacle.twist.linear.x = track.absolute_vx
            obstacle.twist.linear.y = track.absolute_vy
            obstacle.speed_mps = float(math.hypot(track.absolute_vx, track.absolute_vy))
            obstacle.length = track.length
            obstacle.width = track.width
            obstacle.confidence = 0.6
            relative_speed = math.hypot(track.relative_vx, track.relative_vy)
            obstacle.is_static = relative_speed < self._static_speed_threshold
            obstacle.longitudinal_s = track.x
            obstacle.lateral_d = track.y
            obstacle.is_same_lane = classify_same_lane(track.y, self._same_lane_threshold)
            msg.obstacles.append(obstacle)

        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = TrackerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
