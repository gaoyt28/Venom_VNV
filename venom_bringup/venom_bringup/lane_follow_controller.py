"""Conservative lane-following controller for first real-vehicle tests."""

from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float32


class LaneFollowController(Node):
    """Convert lane center error into low-speed Twist commands."""

    def __init__(self) -> None:
        super().__init__('lane_follow_controller')

        self.declare_parameter('center_error_topic', '/lane/center_error')
        self.declare_parameter('valid_topic', '/lane/valid')
        self.declare_parameter('cmd_vel_topic', '/venom_cmd_vel')
        self.declare_parameter('obstacle_topic', '/obstacle_points')
        self.declare_parameter('enable_obstacle_stop', True)
        self.declare_parameter('linear_speed_mps', 0.12)
        self.declare_parameter('kp_angular', -0.0025)
        self.declare_parameter('max_angular_z', 0.45)
        self.declare_parameter('lane_timeout_sec', 0.5)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('obstacle_stop_x_m', 1.8)
        self.declare_parameter('obstacle_stop_abs_y_m', 0.65)
        self.declare_parameter('obstacle_hold_sec', 0.4)

        self._linear_speed_mps = float(self.get_parameter('linear_speed_mps').value)
        self._kp_angular = float(self.get_parameter('kp_angular').value)
        self._max_angular_z = float(self.get_parameter('max_angular_z').value)
        self._lane_timeout_sec = float(self.get_parameter('lane_timeout_sec').value)
        self._enable_obstacle_stop = bool(self.get_parameter('enable_obstacle_stop').value)
        self._obstacle_stop_x_m = float(self.get_parameter('obstacle_stop_x_m').value)
        self._obstacle_stop_abs_y_m = float(self.get_parameter('obstacle_stop_abs_y_m').value)
        self._obstacle_hold_sec = float(self.get_parameter('obstacle_hold_sec').value)
        control_rate_hz = float(self.get_parameter('control_rate_hz').value)

        self._last_center_error = 0.0
        self._lane_valid = False
        self._last_lane_time = 0.0
        self._last_obstacle_time = 0.0

        self._cmd_pub = self.create_publisher(
            Twist,
            str(self.get_parameter('cmd_vel_topic').value),
            10,
        )
        self._center_sub = self.create_subscription(
            Float32,
            str(self.get_parameter('center_error_topic').value),
            self._on_center_error,
            10,
        )
        self._valid_sub = self.create_subscription(
            Bool,
            str(self.get_parameter('valid_topic').value),
            self._on_valid,
            10,
        )
        self._obstacle_sub = self.create_subscription(
            PointCloud2,
            str(self.get_parameter('obstacle_topic').value),
            self._on_obstacles,
            10,
        )
        self._timer = self.create_timer(1.0 / max(control_rate_hz, 1.0), self._publish_cmd)
        self.get_logger().info('Lane follow controller ready')

    def _on_center_error(self, msg: Float32) -> None:
        self._last_center_error = float(msg.data)
        self._last_lane_time = time.monotonic()

    def _on_valid(self, msg: Bool) -> None:
        self._lane_valid = bool(msg.data)
        if self._lane_valid:
            self._last_lane_time = time.monotonic()

    def _on_obstacles(self, msg: PointCloud2) -> None:
        if not self._enable_obstacle_stop:
            return
        for point in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            x_value = float(point[0])
            y_value = float(point[1])
            if 0.0 <= x_value <= self._obstacle_stop_x_m and abs(y_value) <= self._obstacle_stop_abs_y_m:
                self._last_obstacle_time = time.monotonic()
                return

    def _publish_stop(self) -> None:
        self._cmd_pub.publish(Twist())

    def _publish_cmd(self) -> None:
        now = time.monotonic()
        lane_fresh = (now - self._last_lane_time) <= self._lane_timeout_sec
        obstacle_active = self._enable_obstacle_stop and (
            (now - self._last_obstacle_time) <= self._obstacle_hold_sec
        )

        if not self._lane_valid or not lane_fresh or obstacle_active:
            self._publish_stop()
            return

        cmd = Twist()
        cmd.linear.x = self._linear_speed_mps
        angular_z = self._kp_angular * self._last_center_error
        cmd.angular.z = max(-self._max_angular_z, min(self._max_angular_z, angular_z))
        self._cmd_pub.publish(cmd)


def main() -> None:
    rclpy.init()
    node = LaneFollowController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
