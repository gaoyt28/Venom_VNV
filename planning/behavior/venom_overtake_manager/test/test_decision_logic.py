from venom_overtake_manager.safety_checks import can_prepare_overtake
from venom_overtake_manager.safety_checks import can_return_to_lane
from venom_overtake_manager.safety_checks import should_follow
from venom_overtake_manager.safety_checks import target_vehicle_passed


def test_can_prepare_overtake_when_left_clear_and_lead_is_slow():
    assert can_prepare_overtake(
        lead_speed_mps=1.0,
        left_lane_clear=True,
        overtake_allowed=True,
        slow_threshold_mps=1.39,
    )


def test_should_follow_when_lead_is_fast():
    assert should_follow(
        lead_speed_mps=5.56,
        follow_threshold_mps=5.56,
    )


def test_can_return_to_lane_requires_gap_and_clearance():
    assert can_return_to_lane(
        lead_gap_ahead_m=3.0,
        return_lane_clear=True,
        min_return_gap_m=1.0,
    )
def test_target_vehicle_passed_requires_target_behind_ego():
    assert target_vehicle_passed(
        target_longitudinal_s=-1.5,
        pass_buffer_m=1.0,
    )
