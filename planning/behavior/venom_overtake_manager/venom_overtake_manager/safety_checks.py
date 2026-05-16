def can_prepare_overtake(*, lead_speed_mps: float, left_lane_clear: bool, overtake_allowed: bool, slow_threshold_mps: float) -> bool:
    return overtake_allowed and left_lane_clear and lead_speed_mps < slow_threshold_mps


def should_follow(*, lead_speed_mps: float, follow_threshold_mps: float) -> bool:
    return lead_speed_mps >= follow_threshold_mps


def can_return_to_lane(*, lead_gap_ahead_m: float, return_lane_clear: bool, min_return_gap_m: float) -> bool:
    return return_lane_clear and lead_gap_ahead_m >= min_return_gap_m
