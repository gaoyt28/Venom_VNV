# LiDAR Overtake Flow

This note documents the CRAIC low-speed overtake flow in `craic_mission_main`.
It is designed for structured outdoor-road tests where a front obstacle or slow
vehicle blocks the nominal waypoint route.

## What it does

When the active waypoint action is `overtake`, the mission commander:

1. Reads the latest `LaserScan`.
2. Checks whether the front corridor contains an obstacle in the trigger range.
3. Checks whether the preferred side corridor is clear; if not, it tries the other side.
4. Sends generated overtake waypoints to Nav2 when a side corridor is safe.
5. Publishes human-readable state on `/venom/overtake_state`.
6. Falls back to the nominal waypoint when no front obstacle is present.

The state string moves through the following names during a normal pass:

```text
CRUISE -> LANE_SHIFT -> PASS -> RETURN -> CRUISE
```

When a front obstacle is present but no side corridor is clear, the state is
`FOLLOW`. When a close obstacle enters the braking zone, the state is `BRAKE`.

## Main topics

- Input scan: `/scan` by default.
- Mission output: existing Nav2 waypoint goals.
- Debug state: `/venom/overtake_state`.

Example:

```bash
ros2 topic echo /venom/overtake_state
```

## Key parameters

These are safe low-speed starting points for Scout Mini tests:

```yaml
overtake_use_obstacle_detection: true
overtake_scan_topic: /scan
overtake_trigger_distance_m: 4.0
overtake_min_trigger_distance_m: 0.5
overtake_brake_distance_m: 0.80
overtake_left_clearance_distance_m: 1.20
overtake_right_clearance_distance_m: 1.20
overtake_preferred_side: left
overtake_max_linear_speed_mps: 0.45
```

## Test recipe

1. Confirm the chassis obeys `/cmd_vel`.
2. Confirm `/scan` shows obstacles in RViz.
3. Run a waypoint route containing action `6` for overtake.
4. Place an obstacle about `2-3 m` in front of the robot.
5. Keep the preferred side corridor open.
6. Watch `/venom/overtake_state` and RViz while the robot passes.

If the robot should not use scan-gated overtake during a dry run, disable it:

```bash
ros2 param set /craic_mission_main overtake_use_obstacle_detection false
```

If scan data is not stable and you want the robot to skip overtake instead of
falling back to fixed geometry:

```bash
ros2 param set /craic_mission_main overtake_require_scan_for_lane_change true
```

## Rollback

This feature is isolated on the `feature/lidar-overtake-manager` branch. To
discard it locally:

```bash
git switch master
git branch -D feature/lidar-overtake-manager
```
