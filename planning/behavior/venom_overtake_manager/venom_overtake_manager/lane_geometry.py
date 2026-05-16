from dataclasses import dataclass


@dataclass(frozen=True)
class LanePose:
    longitudinal_s: float
    lateral_d: float


def classify_same_lane(lateral_offset_m: float, threshold_m: float) -> bool:
    return abs(lateral_offset_m) <= threshold_m
