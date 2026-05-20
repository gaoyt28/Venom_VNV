"""Action-aware waypoint execution helpers."""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
from typing import Sequence

from venom_bringup.craic_waypoint_utils import CraicWaypoint


TURN_RIGHT_ACTION = 2
TURN_LEFT_ACTION = 3
LANE_CHANGE_LEFT_ACTION = 4
LANE_CHANGE_RIGHT_ACTION = 5
OVERTAKE_ACTION = 6
U_TURN_ACTION = 7
PARK_ACTION = 8
SPECIAL_ACTIONS = {
    TURN_RIGHT_ACTION,
    TURN_LEFT_ACTION,
    LANE_CHANGE_LEFT_ACTION,
    LANE_CHANGE_RIGHT_ACTION,
    OVERTAKE_ACTION,
    U_TURN_ACTION,
    PARK_ACTION,
}


def normalize_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(x_value: float, y_value: float, z_value: float, w_value: float) -> float:
    """Return planar yaw from a quaternion."""
    siny_cosp = 2.0 * (w_value * z_value + x_value * y_value)
    cosy_cosp = 1.0 - 2.0 * (y_value * y_value + z_value * z_value)
    return math.atan2(siny_cosp, cosy_cosp)


def compute_staging_pose(
    goal_x: float,
    goal_y: float,
    goal_yaw: float,
    offset_m: float,
) -> tuple[float, float]:
    """Return a point offset backwards from the goal along its heading."""
    return (
        goal_x - math.cos(goal_yaw) * offset_m,
        goal_y - math.sin(goal_yaw) * offset_m,
    )


def compute_intermediate_turn_yaw(
    current_yaw: float,
    target_yaw: float,
    max_turn_step_rad: float,
) -> float:
    """Return an intermediate yaw for large heading changes such as U-turns."""
    yaw_error = normalize_angle(target_yaw - current_yaw)
    if abs(yaw_error) <= max_turn_step_rad:
        return target_yaw

    turn_step = math.copysign(max_turn_step_rad, yaw_error)
    return normalize_angle(current_yaw + turn_step)


def _distance(a_xy: tuple[float, float], b_xy: tuple[float, float]) -> float:
    return math.hypot(b_xy[0] - a_xy[0], b_xy[1] - a_xy[1])


def _heading(a_xy: tuple[float, float], b_xy: tuple[float, float]) -> float:
    return math.atan2(b_xy[1] - a_xy[1], b_xy[0] - a_xy[0])


def _sample_quadratic_bezier(
    p0: tuple[float, float],
    p1: tuple[float, float],
    p2: tuple[float, float],
    num_samples: int,
) -> list[tuple[float, float]]:
    samples = []
    for sample_index in range(num_samples):
        t_value = sample_index / max(num_samples - 1, 1)
        one_minus_t = 1.0 - t_value
        samples.append(
            (
                one_minus_t * one_minus_t * p0[0]
                + 2.0 * one_minus_t * t_value * p1[0]
                + t_value * t_value * p2[0],
                one_minus_t * one_minus_t * p0[1]
                + 2.0 * one_minus_t * t_value * p1[1]
                + t_value * t_value * p2[1],
            )
        )
    return samples


def _sample_cubic_bezier(
    p0: tuple[float, float],
    p1: tuple[float, float],
    p2: tuple[float, float],
    p3: tuple[float, float],
    num_samples: int,
) -> list[tuple[float, float]]:
    samples = []
    for sample_index in range(num_samples):
        t_value = sample_index / max(num_samples - 1, 1)
        one_minus_t = 1.0 - t_value
        samples.append(
            (
                one_minus_t**3 * p0[0]
                + 3.0 * one_minus_t * one_minus_t * t_value * p1[0]
                + 3.0 * one_minus_t * t_value * t_value * p2[0]
                + t_value**3 * p3[0],
                one_minus_t**3 * p0[1]
                + 3.0 * one_minus_t * one_minus_t * t_value * p1[1]
                + 3.0 * one_minus_t * t_value * t_value * p2[1]
                + t_value**3 * p3[1],
            )
        )
    return samples


def _sample_linear_points(
    start_xy: tuple[float, float],
    end_xy: tuple[float, float],
    num_samples: int,
) -> list[tuple[float, float]]:
    samples = []
    for sample_index in range(num_samples):
        t_value = sample_index / max(num_samples - 1, 1)
        samples.append(
            (
                start_xy[0] + (end_xy[0] - start_xy[0]) * t_value,
                start_xy[1] + (end_xy[1] - start_xy[1]) * t_value,
            )
        )
    return samples


def _xy_with_heading(x_value: float, y_value: float, heading_rad: float, distance_m: float) -> tuple[float, float]:
    return (
        x_value + math.cos(heading_rad) * distance_m,
        y_value + math.sin(heading_rad) * distance_m,
    )


def _make_generated_waypoints(
    base_waypoint: CraicWaypoint,
    points_xy: Sequence[tuple[float, float]],
    profile_name: str,
) -> tuple[CraicWaypoint, ...]:
    if not points_xy:
        return ()

    generated_waypoints = []
    for idx, (x_value, y_value) in enumerate(points_xy):
        if idx < len(points_xy) - 1:
            yaw_value = _heading((x_value, y_value), points_xy[idx + 1])
        elif generated_waypoints:
            yaw_value = generated_waypoints[-1].yaw
        else:
            yaw_value = base_waypoint.yaw

        generated_waypoints.append(
            CraicWaypoint(
                index=base_waypoint.index,
                x=x_value,
                y=y_value,
                yaw=normalize_angle(yaw_value),
                action=base_waypoint.action,
                source_a=base_waypoint.source_a,
                source_b=base_waypoint.source_b,
                action_label=f'{base_waypoint.action_label}_{profile_name}_{idx}',
            )
        )
    return tuple(generated_waypoints)


def _build_turn_waypoints(
    waypoints: Sequence[CraicWaypoint],
    start_index: int,
    profile_name: str,
) -> tuple[CraicWaypoint, ...]:
    if start_index <= 0 or start_index >= len(waypoints) - 1:
        return ()

    previous_waypoint = waypoints[start_index - 1]
    action_waypoint = waypoints[start_index]
    next_waypoint = waypoints[start_index + 1]

    prev_xy = (previous_waypoint.x, previous_waypoint.y)
    current_xy = (action_waypoint.x, action_waypoint.y)
    next_xy = (next_waypoint.x, next_waypoint.y)

    inbound_heading = _heading(prev_xy, current_xy)
    outbound_heading = _heading(current_xy, next_xy)
    min_leg_length = min(_distance(prev_xy, current_xy), _distance(current_xy, next_xy))
    if min_leg_length < 0.2:
        return ()

    trim_distance = min(1.2, max(0.35, min_leg_length * 0.35))
    entry_xy = _xy_with_heading(action_waypoint.x, action_waypoint.y, inbound_heading + math.pi, trim_distance)
    control_distance = trim_distance * 0.8
    control_1 = _xy_with_heading(entry_xy[0], entry_xy[1], inbound_heading, control_distance)
    control_2 = _xy_with_heading(action_waypoint.x, action_waypoint.y, outbound_heading + math.pi, control_distance)
    sampled_xy = _sample_cubic_bezier(entry_xy, control_1, control_2, current_xy, num_samples=5)
    return _make_generated_waypoints(action_waypoint, sampled_xy, profile_name)


def _build_lane_change_waypoints(
    waypoints: Sequence[CraicWaypoint],
    start_index: int,
    profile_name: str,
) -> tuple[CraicWaypoint, ...]:
    action_waypoint = waypoints[start_index]
    current_xy = (action_waypoint.x, action_waypoint.y)

    if start_index > 0:
        previous_waypoint = waypoints[start_index - 1]
        start_heading = _heading((previous_waypoint.x, previous_waypoint.y), current_xy)
        start_distance = _distance((previous_waypoint.x, previous_waypoint.y), current_xy)
    elif start_index + 1 < len(waypoints):
        next_waypoint = waypoints[start_index + 1]
        start_heading = _heading(current_xy, (next_waypoint.x, next_waypoint.y))
        start_distance = _distance(current_xy, (next_waypoint.x, next_waypoint.y))
    else:
        return ()

    if start_distance < 0.2:
        return ()

    start_anchor = _xy_with_heading(
        action_waypoint.x,
        action_waypoint.y,
        start_heading + math.pi,
        min(1.8, max(0.8, start_distance * 0.65)),
    )
    heading_unit = (math.cos(start_heading), math.sin(start_heading))
    lateral_delta = (
        (current_xy[0] - start_anchor[0]) * -heading_unit[1]
        + (current_xy[1] - start_anchor[1]) * heading_unit[0]
    )
    longitudinal = min(1.2, max(0.6, start_distance * 0.45))
    control_1 = (
        start_anchor[0] + heading_unit[0] * longitudinal,
        start_anchor[1] + heading_unit[1] * longitudinal,
    )
    control_2 = (
        current_xy[0] - heading_unit[0] * longitudinal + (-heading_unit[1]) * lateral_delta * 0.35,
        current_xy[1] - heading_unit[1] * longitudinal + heading_unit[0] * lateral_delta * 0.35,
    )
    sampled_xy = _sample_cubic_bezier(start_anchor, control_1, control_2, current_xy, num_samples=5)
    return _make_generated_waypoints(action_waypoint, sampled_xy, profile_name)


def _build_u_turn_waypoints(
    waypoints: Sequence[CraicWaypoint],
    start_index: int,
    profile_name: str,
) -> tuple[CraicWaypoint, ...]:
    if start_index <= 0:
        return ()

    previous_waypoint = waypoints[start_index - 1]
    action_waypoint = waypoints[start_index]
    prev_xy = (previous_waypoint.x, previous_waypoint.y)
    current_xy = (action_waypoint.x, action_waypoint.y)
    inbound_heading = _heading(prev_xy, current_xy)
    outbound_heading = action_waypoint.yaw
    leg_length = _distance(prev_xy, current_xy)
    if leg_length < 0.2:
        return ()

    trim_distance = min(1.2, max(0.45, leg_length * 0.35))
    entry_xy = _xy_with_heading(action_waypoint.x, action_waypoint.y, inbound_heading + math.pi, trim_distance)
    turn_direction = math.copysign(1.0, normalize_angle(outbound_heading - inbound_heading))
    turn_radius = min(1.6, max(0.8, leg_length * 0.65))
    control_1 = (
        entry_xy[0] + (-math.sin(inbound_heading)) * turn_direction * turn_radius,
        entry_xy[1] + math.cos(inbound_heading) * turn_direction * turn_radius,
    )
    control_2 = (
        action_waypoint.x
        + (-math.sin(outbound_heading)) * turn_direction * turn_radius
        - math.cos(outbound_heading) * trim_distance * 0.5,
        action_waypoint.y
        + math.cos(outbound_heading) * turn_direction * turn_radius
        - math.sin(outbound_heading) * trim_distance * 0.5,
    )
    sampled_xy = _sample_cubic_bezier(entry_xy, control_1, control_2, current_xy, num_samples=6)
    return _make_generated_waypoints(action_waypoint, sampled_xy, profile_name)


def _build_overtake_waypoints(
    waypoints: Sequence[CraicWaypoint],
    start_index: int,
    profile_name: str,
) -> tuple[CraicWaypoint, ...]:
    action_waypoint = waypoints[start_index]
    current_xy = (action_waypoint.x, action_waypoint.y)

    if start_index > 0:
        previous_xy = (waypoints[start_index - 1].x, waypoints[start_index - 1].y)
        heading_rad = _heading(previous_xy, current_xy)
        leg_length = _distance(previous_xy, current_xy)
    elif start_index + 1 < len(waypoints):
        next_xy = (waypoints[start_index + 1].x, waypoints[start_index + 1].y)
        heading_rad = _heading(current_xy, next_xy)
        leg_length = _distance(current_xy, next_xy)
    else:
        heading_rad = action_waypoint.yaw
        leg_length = 3.0

    if leg_length < 0.2:
        return ()

    lane_shift_m = min(1.6, max(0.9, leg_length * 0.28))
    approach_distance_m = min(3.0, max(1.2, leg_length * 0.60))
    pass_distance_m = min(2.8, max(1.2, leg_length * 0.45))

    heading_unit = (math.cos(heading_rad), math.sin(heading_rad))
    left_normal = (-heading_unit[1], heading_unit[0])

    entry_xy = (
        current_xy[0] - heading_unit[0] * approach_distance_m,
        current_xy[1] - heading_unit[1] * approach_distance_m,
    )
    left_entry_control = (
        entry_xy[0] + heading_unit[0] * (approach_distance_m * 0.35) + left_normal[0] * lane_shift_m * 0.85,
        entry_xy[1] + heading_unit[1] * (approach_distance_m * 0.35) + left_normal[1] * lane_shift_m * 0.85,
    )
    left_lane_start = (
        current_xy[0] - heading_unit[0] * pass_distance_m + left_normal[0] * lane_shift_m,
        current_xy[1] - heading_unit[1] * pass_distance_m + left_normal[1] * lane_shift_m,
    )
    left_lane_end = (
        current_xy[0] - heading_unit[0] * (pass_distance_m * 0.35) + left_normal[0] * lane_shift_m,
        current_xy[1] - heading_unit[1] * (pass_distance_m * 0.35) + left_normal[1] * lane_shift_m,
    )
    return_control = (
        current_xy[0] - heading_unit[0] * (pass_distance_m * 0.15) + left_normal[0] * lane_shift_m * 0.45,
        current_xy[1] - heading_unit[1] * (pass_distance_m * 0.15) + left_normal[1] * lane_shift_m * 0.45,
    )

    entry_curve = _sample_quadratic_bezier(
        entry_xy,
        left_entry_control,
        left_lane_start,
        num_samples=4,
    )
    cruise_segment = _sample_linear_points(
        left_lane_start,
        left_lane_end,
        num_samples=3,
    )
    return_curve = _sample_quadratic_bezier(
        left_lane_end,
        return_control,
        current_xy,
        num_samples=4,
    )

    sampled_xy = entry_curve[:-1] + cruise_segment[:-1] + return_curve
    return _make_generated_waypoints(action_waypoint, sampled_xy, profile_name)


def build_generated_action_waypoints(
    waypoints: Sequence[CraicWaypoint],
    start_index: int,
    profile_name: str,
) -> tuple[CraicWaypoint, ...]:
    action = waypoints[start_index].action
    if action in {TURN_LEFT_ACTION, TURN_RIGHT_ACTION}:
        return _build_turn_waypoints(waypoints, start_index, profile_name)
    if action in {LANE_CHANGE_LEFT_ACTION, LANE_CHANGE_RIGHT_ACTION}:
        return _build_lane_change_waypoints(waypoints, start_index, profile_name)
    if action == OVERTAKE_ACTION:
        return _build_overtake_waypoints(waypoints, start_index, profile_name)
    if action == U_TURN_ACTION:
        return _build_u_turn_waypoints(waypoints, start_index, profile_name)
    return ()


@dataclass(frozen=True)
class WaypointBehaviorConfig:
    default_final_stop_distance_m: float
    cruise_max_linear_speed_mps: float = 1.0
    cruise_max_speed_xy_mps: float = 1.0
    cruise_max_angular_speed_radps: float = 1.0
    cruise_xy_goal_tolerance_m: float = 0.25
    cruise_yaw_goal_tolerance_rad: float = 0.25
    left_turn_max_linear_speed_mps: float = 0.8
    left_turn_max_speed_xy_mps: float = 0.8
    left_turn_max_angular_speed_radps: float = 0.9
    left_turn_position_tolerance_m: float = 0.45
    left_turn_yaw_tolerance_rad: float = 0.22
    left_turn_settle_time_sec: float = 0.35
    right_turn_max_linear_speed_mps: float = 0.65
    right_turn_max_speed_xy_mps: float = 0.65
    right_turn_max_angular_speed_radps: float = 0.8
    right_turn_position_tolerance_m: float = 0.35
    right_turn_yaw_tolerance_rad: float = 0.30
    right_turn_settle_time_sec: float = 0.20
    lane_change_left_max_linear_speed_mps: float = 0.95
    lane_change_left_max_speed_xy_mps: float = 0.95
    lane_change_left_max_angular_speed_radps: float = 0.75
    lane_change_left_position_tolerance_m: float = 0.28
    lane_change_left_yaw_tolerance_rad: float = 0.20
    lane_change_left_settle_time_sec: float = 0.25
    lane_change_right_max_linear_speed_mps: float = 0.90
    lane_change_right_max_speed_xy_mps: float = 0.90
    lane_change_right_max_angular_speed_radps: float = 0.70
    lane_change_right_position_tolerance_m: float = 0.28
    lane_change_right_yaw_tolerance_rad: float = 0.20
    lane_change_right_settle_time_sec: float = 0.25
    overtake_max_linear_speed_mps: float = 1.15
    overtake_max_speed_xy_mps: float = 1.15
    overtake_max_angular_speed_radps: float = 0.75
    overtake_position_tolerance_m: float = 0.40
    overtake_yaw_tolerance_rad: float = 0.28
    overtake_settle_time_sec: float = 0.15
    u_turn_max_linear_speed_mps: float = 0.45
    u_turn_max_speed_xy_mps: float = 0.45
    u_turn_max_angular_speed_radps: float = 0.70
    u_turn_position_tolerance_m: float = 0.25
    u_turn_yaw_tolerance_rad: float = 0.16
    u_turn_settle_time_sec: float = 0.50
    park_max_linear_speed_mps: float = 0.35
    park_max_speed_xy_mps: float = 0.35
    park_max_angular_speed_radps: float = 0.45
    park_position_tolerance_m: float = 0.18
    park_yaw_tolerance_rad: float = 0.12
    park_settle_time_sec: float = 1.0
    special_action_retry_limit: int = 2


@dataclass(frozen=True)
class WaypointExecutionPlan:
    profile_name: str
    start_index: int
    end_index: int
    goal_index: int
    max_linear_speed_mps: float
    max_speed_xy_mps: float
    max_angular_speed_radps: float
    xy_goal_tolerance_m: float
    yaw_goal_tolerance_rad: float
    stop_distance_m: float | None = None
    position_tolerance_m: float | None = None
    yaw_tolerance_rad: float | None = None
    settle_time_sec: float = 0.0
    goal_retry_limit: int = 0
    generated_waypoints: tuple[CraicWaypoint, ...] = ()

    @property
    def is_special_action(self) -> bool:
        return self.profile_name != 'default'


def build_execution_plan(
    waypoints: Sequence[CraicWaypoint],
    start_index: int,
    config: WaypointBehaviorConfig,
) -> WaypointExecutionPlan:
    """Choose how the next mission slice should be executed."""
    waypoint = waypoints[start_index]
    if waypoint.action == TURN_LEFT_ACTION:
        generated_waypoints = build_generated_action_waypoints(
            waypoints,
            start_index,
            'turn_left',
        )
        return WaypointExecutionPlan(
            profile_name='turn_left',
            start_index=start_index,
            end_index=start_index,
            goal_index=start_index,
            max_linear_speed_mps=config.left_turn_max_linear_speed_mps,
            max_speed_xy_mps=config.left_turn_max_speed_xy_mps,
            max_angular_speed_radps=config.left_turn_max_angular_speed_radps,
            xy_goal_tolerance_m=config.left_turn_position_tolerance_m,
            yaw_goal_tolerance_rad=config.left_turn_yaw_tolerance_rad,
            position_tolerance_m=config.left_turn_position_tolerance_m,
            yaw_tolerance_rad=config.left_turn_yaw_tolerance_rad,
            settle_time_sec=config.left_turn_settle_time_sec,
            goal_retry_limit=config.special_action_retry_limit,
            generated_waypoints=generated_waypoints,
        )
    if waypoint.action == TURN_RIGHT_ACTION:
        generated_waypoints = build_generated_action_waypoints(
            waypoints,
            start_index,
            'turn_right',
        )
        return WaypointExecutionPlan(
            profile_name='turn_right',
            start_index=start_index,
            end_index=start_index,
            goal_index=start_index,
            max_linear_speed_mps=config.right_turn_max_linear_speed_mps,
            max_speed_xy_mps=config.right_turn_max_speed_xy_mps,
            max_angular_speed_radps=config.right_turn_max_angular_speed_radps,
            xy_goal_tolerance_m=config.right_turn_position_tolerance_m,
            yaw_goal_tolerance_rad=config.right_turn_yaw_tolerance_rad,
            position_tolerance_m=config.right_turn_position_tolerance_m,
            yaw_tolerance_rad=config.right_turn_yaw_tolerance_rad,
            settle_time_sec=config.right_turn_settle_time_sec,
            goal_retry_limit=config.special_action_retry_limit,
            generated_waypoints=generated_waypoints,
        )
    if waypoint.action == LANE_CHANGE_LEFT_ACTION:
        generated_waypoints = build_generated_action_waypoints(
            waypoints,
            start_index,
            'lane_change_left',
        )
        return WaypointExecutionPlan(
            profile_name='lane_change_left',
            start_index=start_index,
            end_index=start_index,
            goal_index=start_index,
            max_linear_speed_mps=config.lane_change_left_max_linear_speed_mps,
            max_speed_xy_mps=config.lane_change_left_max_speed_xy_mps,
            max_angular_speed_radps=config.lane_change_left_max_angular_speed_radps,
            xy_goal_tolerance_m=config.lane_change_left_position_tolerance_m,
            yaw_goal_tolerance_rad=config.lane_change_left_yaw_tolerance_rad,
            position_tolerance_m=config.lane_change_left_position_tolerance_m,
            yaw_tolerance_rad=config.lane_change_left_yaw_tolerance_rad,
            settle_time_sec=config.lane_change_left_settle_time_sec,
            goal_retry_limit=config.special_action_retry_limit,
            generated_waypoints=generated_waypoints,
        )
    if waypoint.action == LANE_CHANGE_RIGHT_ACTION:
        generated_waypoints = build_generated_action_waypoints(
            waypoints,
            start_index,
            'lane_change_right',
        )
        return WaypointExecutionPlan(
            profile_name='lane_change_right',
            start_index=start_index,
            end_index=start_index,
            goal_index=start_index,
            max_linear_speed_mps=config.lane_change_right_max_linear_speed_mps,
            max_speed_xy_mps=config.lane_change_right_max_speed_xy_mps,
            max_angular_speed_radps=config.lane_change_right_max_angular_speed_radps,
            xy_goal_tolerance_m=config.lane_change_right_position_tolerance_m,
            yaw_goal_tolerance_rad=config.lane_change_right_yaw_tolerance_rad,
            position_tolerance_m=config.lane_change_right_position_tolerance_m,
            yaw_tolerance_rad=config.lane_change_right_yaw_tolerance_rad,
            settle_time_sec=config.lane_change_right_settle_time_sec,
            goal_retry_limit=config.special_action_retry_limit,
            generated_waypoints=generated_waypoints,
        )
    if waypoint.action == OVERTAKE_ACTION:
        generated_waypoints = build_generated_action_waypoints(
            waypoints,
            start_index,
            'overtake',
        )
        return WaypointExecutionPlan(
            profile_name='overtake',
            start_index=start_index,
            end_index=start_index,
            goal_index=start_index,
            max_linear_speed_mps=config.overtake_max_linear_speed_mps,
            max_speed_xy_mps=config.overtake_max_speed_xy_mps,
            max_angular_speed_radps=config.overtake_max_angular_speed_radps,
            xy_goal_tolerance_m=config.overtake_position_tolerance_m,
            yaw_goal_tolerance_rad=config.overtake_yaw_tolerance_rad,
            position_tolerance_m=config.overtake_position_tolerance_m,
            yaw_tolerance_rad=config.overtake_yaw_tolerance_rad,
            settle_time_sec=config.overtake_settle_time_sec,
            goal_retry_limit=config.special_action_retry_limit,
            generated_waypoints=generated_waypoints,
        )
    if waypoint.action == U_TURN_ACTION:
        generated_waypoints = build_generated_action_waypoints(
            waypoints,
            start_index,
            'u_turn',
        )
        return WaypointExecutionPlan(
            profile_name='u_turn',
            start_index=start_index,
            end_index=start_index,
            goal_index=start_index,
            max_linear_speed_mps=config.u_turn_max_linear_speed_mps,
            max_speed_xy_mps=config.u_turn_max_speed_xy_mps,
            max_angular_speed_radps=config.u_turn_max_angular_speed_radps,
            xy_goal_tolerance_m=config.u_turn_position_tolerance_m,
            yaw_goal_tolerance_rad=config.u_turn_yaw_tolerance_rad,
            position_tolerance_m=config.u_turn_position_tolerance_m,
            yaw_tolerance_rad=config.u_turn_yaw_tolerance_rad,
            settle_time_sec=config.u_turn_settle_time_sec,
            goal_retry_limit=config.special_action_retry_limit,
            generated_waypoints=generated_waypoints,
        )
    if waypoint.action == PARK_ACTION:
        return WaypointExecutionPlan(
            profile_name='park',
            start_index=start_index,
            end_index=start_index,
            goal_index=start_index,
            max_linear_speed_mps=config.park_max_linear_speed_mps,
            max_speed_xy_mps=config.park_max_speed_xy_mps,
            max_angular_speed_radps=config.park_max_angular_speed_radps,
            xy_goal_tolerance_m=config.park_position_tolerance_m,
            yaw_goal_tolerance_rad=config.park_yaw_tolerance_rad,
            position_tolerance_m=config.park_position_tolerance_m,
            yaw_tolerance_rad=config.park_yaw_tolerance_rad,
            settle_time_sec=config.park_settle_time_sec,
            goal_retry_limit=config.special_action_retry_limit,
        )

    end_index = start_index
    while end_index + 1 < len(waypoints):
        next_action = waypoints[end_index + 1].action
        if next_action in SPECIAL_ACTIONS:
            break
        end_index += 1

    stop_distance_m = None
    if end_index == len(waypoints) - 1:
        stop_distance_m = config.default_final_stop_distance_m

    return WaypointExecutionPlan(
        profile_name='default',
        start_index=start_index,
        end_index=end_index,
        goal_index=end_index,
        max_linear_speed_mps=config.cruise_max_linear_speed_mps,
        max_speed_xy_mps=config.cruise_max_speed_xy_mps,
        max_angular_speed_radps=config.cruise_max_angular_speed_radps,
        xy_goal_tolerance_m=config.cruise_xy_goal_tolerance_m,
        yaw_goal_tolerance_rad=config.cruise_yaw_goal_tolerance_rad,
        stop_distance_m=stop_distance_m,
    )


def build_resume_plan(
    active_plan: WaypointExecutionPlan,
    current_waypoint_index: int,
) -> WaypointExecutionPlan:
    """Resume a partially executed plan after recovery."""
    if active_plan.is_special_action:
        return active_plan

    resume_start = min(max(current_waypoint_index, active_plan.start_index), active_plan.end_index)
    return replace(active_plan, start_index=resume_start)
