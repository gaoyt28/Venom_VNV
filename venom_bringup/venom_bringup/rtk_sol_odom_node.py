"""Bridge RTKLIB SOL output to ROS 2 odometry messages.

The node connects to an RTKLIB ``rtkrcv`` TCP solution stream such as
``127.0.0.1:9000`` with ``outstr1-format=llh`` and publishes:

* ``nav_msgs/Odometry`` in a local map frame
* ``sensor_msgs/NavSatFix`` for raw geodetic inspection
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import select
import socket
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from tf2_ros import TransformBroadcaster

from venom_bringup.craic_waypoint_utils import geodetic_to_local_xy


GPS_UNIX_EPOCH = 315964800.0


@dataclass(frozen=True)
class RtkSol:
    """One parsed RTKLIB LLH solution row."""

    gps_week: int
    tow_sec: float
    latitude_deg: float
    longitude_deg: float
    height_m: float
    quality: int
    satellites: int
    std_n_m: float
    std_e_m: float
    std_u_m: float
    cov_ne_m2: float
    cov_eu_m2: float
    cov_un_m2: float
    age_sec: float
    ratio: float


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    """Build a z-axis quaternion from yaw."""
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


def parse_rtk_sol_line(line: str) -> Optional[RtkSol]:
    """Parse one RTKLIB LLH solution line.

    Expected decimal-degree format:
    week tow lat lon height Q ns sdn sde sdu sdne sdeu sdun age ratio
    """
    stripped = line.strip()
    if not stripped or stripped.startswith('%') or stripped.startswith('#'):
        return None

    fields = stripped.split()
    if len(fields) < 15:
        return None

    try:
        return RtkSol(
            gps_week=int(fields[0]),
            tow_sec=float(fields[1]),
            latitude_deg=float(fields[2]),
            longitude_deg=float(fields[3]),
            height_m=float(fields[4]),
            quality=int(fields[5]),
            satellites=int(fields[6]),
            std_n_m=float(fields[7]),
            std_e_m=float(fields[8]),
            std_u_m=float(fields[9]),
            cov_ne_m2=float(fields[10]),
            cov_eu_m2=float(fields[11]),
            cov_un_m2=float(fields[12]),
            age_sec=float(fields[13]),
            ratio=float(fields[14]),
        )
    except ValueError:
        return None


class RtkSolOdomNode(Node):
    """Convert RTKLIB TCP SOL rows into Odometry/NavSatFix."""

    def __init__(self) -> None:
        super().__init__('rtk_sol_odom_node')

        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 9000)
        self.declare_parameter('odom_topic', '/odometry/global')
        self.declare_parameter('navsat_topic', '/rtk/fix')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('max_solution_quality', 2)
        self.declare_parameter('origin_mode', 'auto')
        self.declare_parameter('origin_latitude_deg', 0.0)
        self.declare_parameter('origin_longitude_deg', 0.0)
        self.declare_parameter('origin_height_m', 0.0)
        self.declare_parameter('map_origin_x_m', 0.0)
        self.declare_parameter('map_origin_y_m', 0.0)
        self.declare_parameter('map_origin_yaw_rad', 0.0)
        self.declare_parameter('yaw_rad', 0.0)
        self.declare_parameter('use_solution_time', False)
        self.declare_parameter('gps_utc_leap_seconds', 18.0)
        self.declare_parameter('connect_timeout_sec', 2.0)
        self.declare_parameter('reconnect_period_sec', 1.0)
        self.declare_parameter('socket_poll_period_sec', 0.02)

        self.host = str(self.get_parameter('host').value)
        self.port = int(self.get_parameter('port').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.navsat_topic = str(self.get_parameter('navsat_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.child_frame_id = str(self.get_parameter('child_frame_id').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.max_solution_quality = int(self.get_parameter('max_solution_quality').value)
        self.origin_mode = str(self.get_parameter('origin_mode').value)
        self.origin_latitude_deg = float(self.get_parameter('origin_latitude_deg').value)
        self.origin_longitude_deg = float(self.get_parameter('origin_longitude_deg').value)
        self.origin_height_m = float(self.get_parameter('origin_height_m').value)
        self.map_origin_x_m = float(self.get_parameter('map_origin_x_m').value)
        self.map_origin_y_m = float(self.get_parameter('map_origin_y_m').value)
        self.map_origin_yaw_rad = float(self.get_parameter('map_origin_yaw_rad').value)
        self.yaw_rad = float(self.get_parameter('yaw_rad').value)
        self.use_solution_time = bool(self.get_parameter('use_solution_time').value)
        self.gps_utc_leap_seconds = float(self.get_parameter('gps_utc_leap_seconds').value)
        self.connect_timeout_sec = float(self.get_parameter('connect_timeout_sec').value)
        self.reconnect_period_sec = float(self.get_parameter('reconnect_period_sec').value)
        socket_poll_period_sec = float(self.get_parameter('socket_poll_period_sec').value)

        if self.origin_mode not in {'auto', 'manual'}:
            raise ValueError('origin_mode must be "auto" or "manual"')

        self._origin_ready = self.origin_mode == 'manual'
        self._sock: Optional[socket.socket] = None
        self._rx_buffer = ''
        self._last_connect_attempt = 0.0
        self._last_solution_log = 0.0

        self._odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self._navsat_pub = self.create_publisher(NavSatFix, self.navsat_topic, 10)
        self._tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.create_timer(max(socket_poll_period_sec, 0.005), self._poll_socket)

        self.get_logger().info(
            f'rtk_sol_odom_node connecting to {self.host}:{self.port}, '
            f'publishing {self.odom_topic} and {self.navsat_topic}'
        )

    def _connect(self) -> None:
        now = time.monotonic()
        if now - self._last_connect_attempt < self.reconnect_period_sec:
            return
        self._last_connect_attempt = now

        self._close_socket()
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(self.connect_timeout_sec)
        try:
            sock.connect((self.host, self.port))
        except OSError as exc:
            sock.close()
            self.get_logger().warn(
                f'Unable to connect to RTK SOL stream {self.host}:{self.port}: {exc}'
            )
            return

        sock.setblocking(False)
        self._sock = sock
        self._rx_buffer = ''
        self.get_logger().info(f'Connected to RTK SOL stream {self.host}:{self.port}')

    def _close_socket(self) -> None:
        if self._sock is None:
            return
        try:
            self._sock.close()
        except OSError:
            pass
        self._sock = None

    def _poll_socket(self) -> None:
        if self._sock is None:
            self._connect()
            return

        try:
            readable, _, _ = select.select([self._sock], [], [], 0.0)
            if not readable:
                return
            chunk = self._sock.recv(4096)
        except OSError as exc:
            self.get_logger().warn(f'RTK SOL stream read failed: {exc}')
            self._close_socket()
            return

        if not chunk:
            self.get_logger().warn('RTK SOL stream closed by peer')
            self._close_socket()
            return

        self._rx_buffer += chunk.decode('ascii', errors='ignore')
        while '\n' in self._rx_buffer:
            line, self._rx_buffer = self._rx_buffer.split('\n', 1)
            solution = parse_rtk_sol_line(line)
            if solution is not None:
                self._handle_solution(solution)

    def _stamp_for_solution(self, solution: RtkSol):
        if not self.use_solution_time:
            return self.get_clock().now().to_msg()

        unix_time = (
            GPS_UNIX_EPOCH
            + solution.gps_week * 604800.0
            + solution.tow_sec
            - self.gps_utc_leap_seconds
        )
        sec = int(unix_time)
        nanosec = int((unix_time - sec) * 1e9)
        stamp = self.get_clock().now().to_msg()
        stamp.sec = sec
        stamp.nanosec = nanosec
        return stamp

    def _handle_solution(self, solution: RtkSol) -> None:
        if solution.quality <= 0 or solution.quality > self.max_solution_quality:
            return

        if not self._origin_ready:
            self.origin_latitude_deg = solution.latitude_deg
            self.origin_longitude_deg = solution.longitude_deg
            self.origin_height_m = solution.height_m
            self._origin_ready = True
            self.get_logger().info(
                'RTK local origin initialized from first accepted solution: '
                f'lat={self.origin_latitude_deg:.9f}, '
                f'lon={self.origin_longitude_deg:.9f}, '
                f'h={self.origin_height_m:.3f}'
            )

        map_x, map_y = geodetic_to_local_xy(
            longitude_deg=solution.longitude_deg,
            latitude_deg=solution.latitude_deg,
            origin_longitude_deg=self.origin_longitude_deg,
            origin_latitude_deg=self.origin_latitude_deg,
            map_origin_yaw_rad=self.map_origin_yaw_rad,
            map_origin_x_m=self.map_origin_x_m,
            map_origin_y_m=self.map_origin_y_m,
        )
        map_z = solution.height_m - self.origin_height_m
        stamp = self._stamp_for_solution(solution)

        self._publish_navsat(solution, stamp)
        self._publish_odometry(solution, stamp, map_x, map_y, map_z)

        now = time.monotonic()
        if now - self._last_solution_log > 2.0:
            self._last_solution_log = now
            self.get_logger().info(
                f'RTK Q={solution.quality} ns={solution.satellites} '
                f'lat={solution.latitude_deg:.9f} lon={solution.longitude_deg:.9f} '
                f'xy=({map_x:.3f}, {map_y:.3f}) '
                f'std_enu=({solution.std_e_m:.3f}, {solution.std_n_m:.3f}, {solution.std_u_m:.3f})'
            )

    def _publish_navsat(self, solution: RtkSol, stamp) -> None:
        msg = NavSatFix()
        msg.header.stamp = stamp
        msg.header.frame_id = self.child_frame_id
        msg.latitude = solution.latitude_deg
        msg.longitude = solution.longitude_deg
        msg.altitude = solution.height_m
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.status.status = (
            NavSatStatus.STATUS_GBAS_FIX
            if solution.quality in {1, 2}
            else NavSatStatus.STATUS_FIX
        )
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_KNOWN
        msg.position_covariance[0] = solution.std_e_m * solution.std_e_m
        msg.position_covariance[4] = solution.std_n_m * solution.std_n_m
        msg.position_covariance[8] = solution.std_u_m * solution.std_u_m
        self._navsat_pub.publish(msg)

    def _publish_odometry(
        self,
        solution: RtkSol,
        stamp,
        map_x: float,
        map_y: float,
        map_z: float,
    ) -> None:
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = map_x
        odom.pose.pose.position.y = map_y
        odom.pose.pose.position.z = map_z

        qx, qy, qz, qw = quaternion_from_yaw(self.yaw_rad)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.pose.covariance[0] = solution.std_e_m * solution.std_e_m
        odom.pose.covariance[1] = solution.cov_ne_m2
        odom.pose.covariance[6] = solution.cov_ne_m2
        odom.pose.covariance[7] = solution.std_n_m * solution.std_n_m
        odom.pose.covariance[14] = solution.std_u_m * solution.std_u_m
        odom.pose.covariance[35] = 999.0
        self._odom_pub.publish(odom)

        if self._tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.frame_id
            transform.child_frame_id = self.child_frame_id
            transform.transform.translation.x = map_x
            transform.transform.translation.y = map_y
            transform.transform.translation.z = map_z
            transform.transform.rotation = odom.pose.pose.orientation
            self._tf_broadcaster.sendTransform(transform)

    def destroy_node(self) -> bool:
        self._close_socket()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = RtkSolOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
