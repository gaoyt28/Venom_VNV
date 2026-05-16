from __future__ import annotations

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from venom_overtake_msgs.msg import LeadVehicle
from venom_overtake_msgs.msg import TrackedObstacle
from venom_overtake_msgs.msg import TrackedObstacleArray


class LeadSelectorNode(Node):
    def __init__(self) -> None:
        super().__init__('lead_selector_node')
        self.declare_parameter('same_lane_only', True)
        self.declare_parameter('max_forward_distance_m', 30.0)
        self.declare_parameter('max_lateral_offset_m', 1.0)
        self.declare_parameter('odom_topic', '/odometry/global')

        self._same_lane_only = bool(self.get_parameter('same_lane_only').value)
        self._max_forward_distance = float(self.get_parameter('max_forward_distance_m').value)
        self._max_lateral_offset = float(self.get_parameter('max_lateral_offset_m').value)
        odom_topic = str(self.get_parameter('odom_topic').value)

        self._ego_speed_mps = 0.0
        self.create_subscription(TrackedObstacleArray, '/perception/tracked_obstacles', self._on_tracks, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 20)
        self._pub = self.create_publisher(LeadVehicle, '/planning/lead_vehicle', 10)

    def _on_odom(self, msg: Odometry) -> None:
        self._ego_speed_mps = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)

    def _on_tracks(self, msg: TrackedObstacleArray) -> None:
        best: TrackedObstacle | None = None
        best_gap = float('inf')
        for obstacle in msg.obstacles:
            if self._same_lane_only and not obstacle.is_same_lane:
                continue
            if obstacle.longitudinal_s <= 0.0 or obstacle.longitudinal_s > self._max_forward_distance:
                continue
            if abs(obstacle.lateral_d) > self._max_lateral_offset:
                continue
            if obstacle.longitudinal_s < best_gap:
                best_gap = obstacle.longitudinal_s
                best = obstacle

        lead = LeadVehicle()
        lead.header = msg.header
        if best is not None:
            lead.valid = True
            lead.target = best
            lead.gap_m = best_gap
            lead.relative_speed_mps = self._ego_speed_mps - best.speed_mps
        self._pub.publish(lead)


def main() -> None:
    rclpy.init()
    node = LeadSelectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
