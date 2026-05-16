from __future__ import annotations

from enum import Enum
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from venom_overtake_msgs.msg import LeadVehicle
from venom_overtake_msgs.msg import OvertakeDecision
from venom_overtake_msgs.msg import TrackedObstacle
from venom_overtake_msgs.msg import TrackedObstacleArray

from venom_overtake_manager.decision_logic import DecisionConfig
from venom_overtake_manager.lane_route_manager import LaneRouteManager
from venom_overtake_manager.lane_route_manager import RouteNames
from venom_overtake_manager.safety_checks import can_prepare_overtake
from venom_overtake_manager.safety_checks import can_return_to_lane
from venom_overtake_manager.safety_checks import should_follow
from venom_overtake_manager.safety_checks import target_vehicle_passed


class OvertakeState(str, Enum):
    CRUISE = 'CRUISE'
    FOLLOW = 'FOLLOW'
    PREPARE_OVERTAKE = 'PREPARE_OVERTAKE'
    OVERTAKE_LEFT = 'OVERTAKE_LEFT'
    RETURN_RIGHT = 'RETURN_RIGHT'
    ABORT = 'ABORT'


class OvertakeManagerNode(Node):
    def __init__(self) -> None:
        super().__init__('overtake_manager_node')
        self._declare_parameters()
        odom_topic = str(self.get_parameter('odom_topic').value)
        self._config = DecisionConfig(
            slow_vehicle_speed_threshold_mps=float(self.get_parameter('slow_vehicle_speed_threshold_mps').value),
            follow_vehicle_speed_threshold_mps=float(self.get_parameter('follow_vehicle_speed_threshold_mps').value),
            desired_follow_gap_m=float(self.get_parameter('desired_follow_gap_m').value),
            desired_follow_time_s=float(self.get_parameter('desired_follow_time_s').value),
            overtake_completion_buffer_m=float(self.get_parameter('overtake_completion_buffer_m').value),
            min_return_gap_m=float(self.get_parameter('min_return_gap_m').value),
            max_overtake_duration_s=float(self.get_parameter('max_overtake_duration_s').value),
            cruise_speed_limit_mps=float(self.get_parameter('cruise_speed_limit_mps').value),
            follow_speed_limit_mps=float(self.get_parameter('follow_speed_limit_mps').value),
            overtake_speed_limit_mps=float(self.get_parameter('overtake_speed_limit_mps').value),
            lane_change_left_route=str(self.get_parameter('lane_change_left_route').value),
            lane_return_route=str(self.get_parameter('lane_return_route').value),
            lane_cruise_route=str(self.get_parameter('lane_cruise_route').value),
        )
        self._route_manager = LaneRouteManager(
            RouteNames(
                cruise_route=self._config.lane_cruise_route,
                overtake_left_route=self._config.lane_change_left_route,
                return_route=self._config.lane_return_route,
            )
        )
        self._state = OvertakeState.CRUISE
        self._last_lead = LeadVehicle()
        self._ego_speed_mps = 0.0
        self._tracks: dict[int, TrackedObstacle] = {}
        self._active_target_id = 0
        self._state_enter_time = time.monotonic()

        self.create_subscription(LeadVehicle, '/planning/lead_vehicle', self._on_lead_vehicle, 10)
        self.create_subscription(TrackedObstacleArray, '/perception/tracked_obstacles', self._on_tracks, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 20)
        self._decision_pub = self.create_publisher(OvertakeDecision, '/planning/overtake_decision', 10)
        self._route_pub = self.create_publisher(String, '/planning/requested_route', 10)
        self.create_timer(0.1, self._tick)

    def _declare_parameters(self) -> None:
        self.declare_parameter('slow_vehicle_speed_threshold_mps', 1.39)
        self.declare_parameter('follow_vehicle_speed_threshold_mps', 5.56)
        self.declare_parameter('desired_follow_gap_m', 4.0)
        self.declare_parameter('desired_follow_time_s', 1.5)
        self.declare_parameter('overtake_completion_buffer_m', 2.5)
        self.declare_parameter('min_return_gap_m', 1.0)
        self.declare_parameter('max_overtake_duration_s', 8.0)
        self.declare_parameter('cruise_speed_limit_mps', 2.5)
        self.declare_parameter('follow_speed_limit_mps', 1.8)
        self.declare_parameter('overtake_speed_limit_mps', 2.2)
        self.declare_parameter('lane_change_left_route', 'main_lane_overtake_left')
        self.declare_parameter('lane_return_route', 'main_lane_return_right')
        self.declare_parameter('lane_cruise_route', 'main_lane_patrol')
        self.declare_parameter('odom_topic', '/odometry/global')
        self.declare_parameter('left_lane_clear', True)
        self.declare_parameter('return_lane_clear', True)
        self.declare_parameter('overtake_allowed', True)
        self.declare_parameter('target_pass_buffer_m', 1.0)
        self.declare_parameter('target_lost_timeout_s', 1.0)

    def _on_lead_vehicle(self, msg: LeadVehicle) -> None:
        self._last_lead = msg

    def _on_tracks(self, msg: TrackedObstacleArray) -> None:
        self._tracks = {int(obstacle.id): obstacle for obstacle in msg.obstacles}

    def _on_odom(self, msg: Odometry) -> None:
        self._ego_speed_mps = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)

    def _transition(self, next_state: OvertakeState) -> None:
        if next_state == self._state:
            return
        self.get_logger().info(f'Overtake state transition: {self._state.value} -> {next_state.value}')
        self._state = next_state
        self._state_enter_time = time.monotonic()

    def _publish_route(self, route_name: str) -> None:
        msg = String()
        msg.data = route_name
        self._route_pub.publish(msg)

    def _target_track(self) -> TrackedObstacle | None:
        if self._active_target_id == 0:
            return None
        return self._tracks.get(self._active_target_id)

    def _tick(self) -> None:
        lead = self._last_lead
        overtake_allowed = bool(self.get_parameter('overtake_allowed').value)
        left_lane_clear = bool(self.get_parameter('left_lane_clear').value)
        return_lane_clear = bool(self.get_parameter('return_lane_clear').value)
        reason = 'no lead vehicle'
        requested_route = self._route_manager.cruise_route()
        commanded_lane = 'main'
        target_speed = 0.0
        speed_limit = self._config.cruise_speed_limit_mps
        target_id = self._active_target_id

        if self._state == OvertakeState.CRUISE:
            if lead.valid:
                target_speed = lead.target.speed_mps
                target_id = lead.target.id
                if can_prepare_overtake(
                    lead_speed_mps=lead.target.speed_mps,
                    left_lane_clear=left_lane_clear,
                    overtake_allowed=overtake_allowed,
                    slow_threshold_mps=self._config.slow_vehicle_speed_threshold_mps,
                ):
                    self._active_target_id = int(lead.target.id)
                    self._transition(OvertakeState.PREPARE_OVERTAKE)
                    reason = 'slow lead vehicle detected, preparing overtake'
                elif should_follow(
                    lead_speed_mps=lead.target.speed_mps,
                    follow_threshold_mps=self._config.follow_vehicle_speed_threshold_mps,
                ):
                    self._transition(OvertakeState.FOLLOW)
                    reason = 'lead vehicle too fast to overtake, following'
                else:
                    self._transition(OvertakeState.FOLLOW)
                    reason = 'lead vehicle present, conservative follow'

        elif self._state == OvertakeState.FOLLOW:
            speed_limit = self._config.follow_speed_limit_mps
            reason = 'following lead vehicle'
            if not lead.valid:
                self._transition(OvertakeState.CRUISE)
                reason = 'lead vehicle lost, returning to cruise'
            elif can_prepare_overtake(
                lead_speed_mps=lead.target.speed_mps,
                left_lane_clear=left_lane_clear,
                overtake_allowed=overtake_allowed,
                slow_threshold_mps=self._config.slow_vehicle_speed_threshold_mps,
            ):
                self._active_target_id = int(lead.target.id)
                self._transition(OvertakeState.PREPARE_OVERTAKE)
                reason = 'lead vehicle slowed down enough for overtake'
            requested_route = self._route_manager.cruise_route()

        elif self._state == OvertakeState.PREPARE_OVERTAKE:
            speed_limit = self._config.overtake_speed_limit_mps
            requested_route = self._route_manager.overtake_left_route()
            commanded_lane = 'left'
            self._publish_route(requested_route)
            self._transition(OvertakeState.OVERTAKE_LEFT)
            reason = 'requested left overtake route'

        elif self._state == OvertakeState.OVERTAKE_LEFT:
            speed_limit = self._config.overtake_speed_limit_mps
            requested_route = self._route_manager.overtake_left_route()
            commanded_lane = 'left'
            reason = 'executing left overtake'
            target_track = self._target_track()
            if target_track is not None:
                target_speed = float(target_track.speed_mps)
                target_id = int(target_track.id)
                pass_buffer_m = float(self.get_parameter('target_pass_buffer_m').value)
                if target_vehicle_passed(
                    target_longitudinal_s=float(target_track.longitudinal_s),
                    pass_buffer_m=pass_buffer_m,
                ) and can_return_to_lane(
                    lead_gap_ahead_m=abs(float(target_track.longitudinal_s)),
                    return_lane_clear=return_lane_clear,
                    min_return_gap_m=pass_buffer_m,
                ):
                    self._transition(OvertakeState.RETURN_RIGHT)
                    reason = 'target passed, returning to cruise route'
            elif time.monotonic() - self._state_enter_time > float(
                self.get_parameter('target_lost_timeout_s').value
            ):
                self._transition(OvertakeState.RETURN_RIGHT)
                reason = 'target lost after overtake, returning to cruise route'
            if time.monotonic() - self._state_enter_time > self._config.max_overtake_duration_s:
                self._transition(OvertakeState.ABORT)
                reason = 'overtake timeout, aborting'

        elif self._state == OvertakeState.RETURN_RIGHT:
            requested_route = self._route_manager.return_route()
            commanded_lane = 'main'
            self._publish_route(requested_route)
            self._transition(OvertakeState.CRUISE)
            self._active_target_id = 0
            reason = 'requested return route'

        elif self._state == OvertakeState.ABORT:
            requested_route = self._route_manager.cruise_route()
            commanded_lane = 'main'
            self._publish_route(requested_route)
            speed_limit = self._config.follow_speed_limit_mps
            self._active_target_id = 0
            self._transition(OvertakeState.FOLLOW if lead.valid else OvertakeState.CRUISE)
            reason = 'abort complete'

        decision = OvertakeDecision()
        decision.header.stamp = self.get_clock().now().to_msg()
        decision.header.frame_id = 'map'
        decision.state = self._state.value
        decision.follow_mode = self._state == OvertakeState.FOLLOW
        decision.overtake_mode = self._state in {OvertakeState.PREPARE_OVERTAKE, OvertakeState.OVERTAKE_LEFT}
        decision.return_mode = self._state == OvertakeState.RETURN_RIGHT
        decision.target_id = target_id
        decision.target_lane_id = 'main'
        decision.commanded_lane_id = commanded_lane
        decision.requested_route = requested_route
        decision.reason = reason
        decision.target_speed_mps = target_speed
        decision.ego_speed_limit_mps = speed_limit
        self._decision_pub.publish(decision)


def main() -> None:
    rclpy.init()
    node = OvertakeManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
