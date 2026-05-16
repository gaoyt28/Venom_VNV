from dataclasses import dataclass


@dataclass(frozen=True)
class DecisionConfig:
    slow_vehicle_speed_threshold_mps: float
    follow_vehicle_speed_threshold_mps: float
    desired_follow_gap_m: float
    desired_follow_time_s: float
    overtake_completion_buffer_m: float
    min_return_gap_m: float
    max_overtake_duration_s: float
    cruise_speed_limit_mps: float
    follow_speed_limit_mps: float
    overtake_speed_limit_mps: float
    lane_change_left_route: str
    lane_return_route: str
    lane_cruise_route: str
