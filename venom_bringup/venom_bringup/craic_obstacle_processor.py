"""Filter live point clouds into Nav2 obstacle points for CRAIC-style avoidance."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener

try:
    from yolo_interfaces.msg import YoloDetections
except ImportError:  # pragma: no cover - depends on workspace submodule availability
    YoloDetections = None


@dataclass(frozen=True)
class PedestrianObservation:
    center_x_px: float
    bbox_height_px: float
    score: float
    expires_at: float


def quaternion_to_rotation_matrix(x_value: float, y_value: float, z_value: float, w_value: float) -> np.ndarray:
    """Convert a quaternion into a 3x3 rotation matrix."""
    xx = x_value * x_value
    yy = y_value * y_value
    zz = z_value * z_value
    xy = x_value * y_value
    xz = x_value * z_value
    yz = y_value * z_value
    wx = w_value * x_value
    wy = w_value * y_value
    wz = w_value * z_value
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float32,
    )


class CraicObstacleProcessor(Node):
    """Publish a compact live obstacle cloud suitable for Nav2 obstacle layers."""

    def __init__(self) -> None:
        super().__init__('craic_obstacle_processor')

        self.declare_parameter('input_topic', 'input')
        self.declare_parameter('output_topic', 'output')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('min_x_m', -1.0)
        self.declare_parameter('max_x_m', 10.0)
        self.declare_parameter('max_abs_y_m', 4.0)
        self.declare_parameter('min_z_m', 0.05)
        self.declare_parameter('max_z_m', 1.80)
        self.declare_parameter('robot_clearance_radius_m', 0.75)
        self.declare_parameter('max_range_m', 12.0)
        self.declare_parameter('voxel_size_m', 0.15)
        self.declare_parameter('max_points', 1500)
        self.declare_parameter('transform_timeout_sec', 0.2)
        self.declare_parameter('min_points_to_publish', 1)
        self.declare_parameter('detections_topic', '/perception/detections')
        self.declare_parameter('enable_pedestrian_detections', True)
        self.declare_parameter('pedestrian_class_names', 'person,pedestrian')
        self.declare_parameter('pedestrian_score_threshold', 0.45)
        self.declare_parameter('pedestrian_hold_sec', 0.6)
        self.declare_parameter('pedestrian_image_width_px', 1280.0)
        self.declare_parameter('pedestrian_lateral_span_m', 5.0)
        self.declare_parameter('pedestrian_min_forward_m', 1.5)
        self.declare_parameter('pedestrian_max_forward_m', 8.0)
        self.declare_parameter('pedestrian_bbox_height_scale', 1600.0)
        self.declare_parameter('pedestrian_obstacle_radius_m', 0.6)
        self.declare_parameter('pedestrian_obstacle_height_m', 1.0)
        self.declare_parameter('pedestrian_points_per_circle', 12)
        self.declare_parameter('pedestrian_max_observations', 3)

        self._input_topic = str(self.get_parameter('input_topic').value)
        self._output_topic = str(self.get_parameter('output_topic').value)
        self._target_frame = str(self.get_parameter('target_frame').value)
        self._min_x_m = float(self.get_parameter('min_x_m').value)
        self._max_x_m = float(self.get_parameter('max_x_m').value)
        self._max_abs_y_m = float(self.get_parameter('max_abs_y_m').value)
        self._min_z_m = float(self.get_parameter('min_z_m').value)
        self._max_z_m = float(self.get_parameter('max_z_m').value)
        self._robot_clearance_radius_m = float(self.get_parameter('robot_clearance_radius_m').value)
        self._max_range_m = float(self.get_parameter('max_range_m').value)
        self._voxel_size_m = max(0.01, float(self.get_parameter('voxel_size_m').value))
        self._max_points = max(1, int(self.get_parameter('max_points').value))
        self._transform_timeout_sec = float(self.get_parameter('transform_timeout_sec').value)
        self._min_points_to_publish = max(0, int(self.get_parameter('min_points_to_publish').value))
        self._detections_topic = str(self.get_parameter('detections_topic').value)
        self._enable_pedestrian_detections = bool(
            self.get_parameter('enable_pedestrian_detections').value
        )
        self._pedestrian_class_names = {
            value.strip().lower()
            for value in str(self.get_parameter('pedestrian_class_names').value).split(',')
            if value.strip()
        }
        self._pedestrian_score_threshold = float(
            self.get_parameter('pedestrian_score_threshold').value
        )
        self._pedestrian_hold_sec = float(self.get_parameter('pedestrian_hold_sec').value)
        self._pedestrian_image_width_px = max(
            1.0, float(self.get_parameter('pedestrian_image_width_px').value)
        )
        self._pedestrian_lateral_span_m = float(
            self.get_parameter('pedestrian_lateral_span_m').value
        )
        self._pedestrian_min_forward_m = float(
            self.get_parameter('pedestrian_min_forward_m').value
        )
        self._pedestrian_max_forward_m = float(
            self.get_parameter('pedestrian_max_forward_m').value
        )
        self._pedestrian_bbox_height_scale = max(
            1.0, float(self.get_parameter('pedestrian_bbox_height_scale').value)
        )
        self._pedestrian_obstacle_radius_m = max(
            0.1, float(self.get_parameter('pedestrian_obstacle_radius_m').value)
        )
        self._pedestrian_obstacle_height_m = float(
            self.get_parameter('pedestrian_obstacle_height_m').value
        )
        self._pedestrian_points_per_circle = max(
            6, int(self.get_parameter('pedestrian_points_per_circle').value)
        )
        self._pedestrian_max_observations = max(
            1, int(self.get_parameter('pedestrian_max_observations').value)
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)
        self._pedestrian_observations: List[PedestrianObservation] = []

        self._obstacle_pub = self.create_publisher(PointCloud2, self._output_topic, 10)
        self._cloud_sub = self.create_subscription(
            PointCloud2,
            self._input_topic,
            self._on_cloud,
            10,
        )
        self._detections_sub = None
        if self._enable_pedestrian_detections:
            if YoloDetections is None:
                self.get_logger().warn(
                    'yolo_interfaces is unavailable; pedestrian detection obstacles are disabled.',
                )
            else:
                self._detections_sub = self.create_subscription(
                    YoloDetections,
                    self._detections_topic,
                    self._on_detections,
                    10,
                )

        self.get_logger().info(
            f'CRAIC obstacle processor ready: {self._input_topic} -> {self._output_topic} '
            f'in frame {self._target_frame}'
        )
        if self._detections_sub is not None:
            self.get_logger().info(
                'Pedestrian detection obstacles enabled: '
                f'{self._detections_topic} -> {self._output_topic}'
            )

    def _transform_points_to_target(self, msg: PointCloud2, points_xyz: np.ndarray) -> Optional[np.ndarray]:
        source_frame = msg.header.frame_id or self._target_frame
        if source_frame == self._target_frame:
            return points_xyz

        try:
            transform = self._tf_buffer.lookup_transform(
                self._target_frame,
                source_frame,
                rclpy.time.Time.from_msg(msg.header.stamp),
                timeout=rclpy.duration.Duration(seconds=self._transform_timeout_sec),
            )
        except TransformException:
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._target_frame,
                    source_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=self._transform_timeout_sec),
                )
            except TransformException as exc:
                self.get_logger().warn(
                    f'Failed to transform {source_frame} -> {self._target_frame}: {exc}',
                    throttle_duration_sec=2.0,
                )
                return None

        rotation = quaternion_to_rotation_matrix(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        )
        translation = np.array(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ],
            dtype=np.float32,
        )
        return points_xyz @ rotation.T + translation

    def _filter_points(self, points_xyz: np.ndarray) -> np.ndarray:
        if points_xyz.size == 0:
            return points_xyz

        planar_distance = np.hypot(points_xyz[:, 0], points_xyz[:, 1])
        mask = (
            (points_xyz[:, 0] >= self._min_x_m)
            & (points_xyz[:, 0] <= self._max_x_m)
            & (np.abs(points_xyz[:, 1]) <= self._max_abs_y_m)
            & (points_xyz[:, 2] >= self._min_z_m)
            & (points_xyz[:, 2] <= self._max_z_m)
            & (planar_distance <= self._max_range_m)
            & (planar_distance >= self._robot_clearance_radius_m)
        )
        filtered = points_xyz[mask]
        if filtered.size == 0:
            return filtered

        quantized = np.floor(filtered / self._voxel_size_m).astype(np.int32)
        _, unique_indices = np.unique(quantized, axis=0, return_index=True)
        unique_indices.sort()
        filtered = filtered[unique_indices]
        if filtered.shape[0] > self._max_points:
            filtered = filtered[: self._max_points]
        return filtered

    def _prune_pedestrian_observations(self, now_monotonic: float) -> None:
        self._pedestrian_observations = [
            observation
            for observation in self._pedestrian_observations
            if observation.expires_at >= now_monotonic
        ]

    def _estimate_pedestrian_position(self, observation: PedestrianObservation) -> tuple[float, float]:
        normalized_x = (observation.center_x_px / self._pedestrian_image_width_px) - 0.5
        lateral_y = normalized_x * self._pedestrian_lateral_span_m
        forward_x = self._pedestrian_bbox_height_scale / max(observation.bbox_height_px, 1.0)
        forward_x = max(self._pedestrian_min_forward_m, min(self._pedestrian_max_forward_m, forward_x))
        return forward_x, lateral_y

    def _build_disc_points(self, center_x: float, center_y: float, radius_m: float) -> np.ndarray:
        points = [[center_x, center_y, self._pedestrian_obstacle_height_m * 0.5]]
        for angle_index in range(self._pedestrian_points_per_circle):
            angle = (2.0 * math.pi * angle_index) / self._pedestrian_points_per_circle
            points.append(
                [
                    center_x + radius_m * math.cos(angle),
                    center_y + radius_m * math.sin(angle),
                    self._pedestrian_obstacle_height_m,
                ]
            )
        return np.asarray(points, dtype=np.float32)

    def _build_pedestrian_points(self) -> np.ndarray:
        now_monotonic = time.monotonic()
        self._prune_pedestrian_observations(now_monotonic)
        if not self._pedestrian_observations:
            return np.empty((0, 3), dtype=np.float32)

        all_points = []
        for observation in self._pedestrian_observations[: self._pedestrian_max_observations]:
            center_x, center_y = self._estimate_pedestrian_position(observation)
            if center_x < self._min_x_m or center_x > self._max_x_m:
                continue
            if abs(center_y) > self._max_abs_y_m:
                continue
            all_points.append(
                self._build_disc_points(center_x, center_y, self._pedestrian_obstacle_radius_m)
            )

        if not all_points:
            return np.empty((0, 3), dtype=np.float32)
        return np.concatenate(all_points, axis=0)

    def _on_detections(self, msg) -> None:
        now_monotonic = time.monotonic()
        observations: List[PedestrianObservation] = []
        for detection in getattr(msg, 'detections', []):
            hypothesis = getattr(detection, 'hypothesis', None)
            bbox = getattr(detection, 'bbox', None)
            if hypothesis is None or bbox is None:
                continue

            class_name = str(getattr(hypothesis, 'class_name', '')).strip().lower()
            score = float(getattr(hypothesis, 'score', 0.0))
            bbox_height_px = float(getattr(bbox, 'size_y', 0.0))
            center_x_px = float(getattr(bbox, 'center_x', 0.0))

            if class_name not in self._pedestrian_class_names:
                continue
            if score < self._pedestrian_score_threshold:
                continue
            if bbox_height_px <= 1.0:
                continue

            observations.append(
                PedestrianObservation(
                    center_x_px=center_x_px,
                    bbox_height_px=bbox_height_px,
                    score=score,
                    expires_at=now_monotonic + self._pedestrian_hold_sec,
                )
            )

        self._pedestrian_observations = observations[: self._pedestrian_max_observations]

    def _publish_cloud(self, stamp, points_xyz: np.ndarray) -> None:
        header = {'stamp': stamp, 'frame_id': self._target_frame}
        if points_xyz.shape[0] < self._min_points_to_publish:
            cloud_msg = point_cloud2.create_cloud_xyz32(header, [])
        else:
            cloud_msg = point_cloud2.create_cloud_xyz32(header, points_xyz.tolist())
        self._obstacle_pub.publish(cloud_msg)

    def _on_cloud(self, msg: PointCloud2) -> None:
        raw_points = list(
            point_cloud2.read_points(
                msg,
                field_names=('x', 'y', 'z'),
                skip_nans=True,
            )
        )
        filtered = np.empty((0, 3), dtype=np.float32)
        if raw_points:
            points_xyz = np.asarray(raw_points, dtype=np.float32)
            transformed = self._transform_points_to_target(msg, points_xyz)
            if transformed is None:
                return
            filtered = self._filter_points(transformed)

        pedestrian_points = self._build_pedestrian_points()
        if filtered.size == 0:
            combined = pedestrian_points
        elif pedestrian_points.size == 0:
            combined = filtered
        else:
            combined = np.concatenate([filtered, pedestrian_points], axis=0)
        self._publish_cloud(msg.header.stamp, combined)


def main() -> None:
    rclpy.init()
    node = CraicObstacleProcessor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
