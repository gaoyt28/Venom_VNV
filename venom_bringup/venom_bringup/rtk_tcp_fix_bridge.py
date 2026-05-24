"""Bridge an external RTK TCP stream into ``/fix`` only.

This node is intentionally conservative: it publishes ``sensor_msgs/NavSatFix``
and leaves global odometry generation to ``navsat_transform_node`` + EKF.
"""

from __future__ import annotations

import json
import re
import socket
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus


FLOAT_PATTERN = re.compile(r'[-+]?\d+(?:\.\d+)?')


def _is_valid_latitude(value: float) -> bool:
    return -90.0 <= value <= 90.0


def _is_valid_longitude(value: float) -> bool:
    return -180.0 <= value <= 180.0


def _pick_altitude(value: float) -> Optional[float]:
    if -1000.0 <= value <= 10000.0:
        return value
    return None


class RtkTcpFixBridge(Node):
    """Read a TCP RTK stream and publish NavSatFix observations."""

    def __init__(self) -> None:
        super().__init__('rtk_tcp_fix_bridge')

        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 9000)
        self.declare_parameter('frame_id', 'gps_link')
        self.declare_parameter('fix_topic', '/fix')
        self.declare_parameter('default_altitude_m', 0.0)
        self.declare_parameter('use_altitude_from_stream', True)
        self.declare_parameter('covariance_m2', 4.0)
        self.declare_parameter('socket_timeout_sec', 2.0)
        self.declare_parameter('reconnect_interval_sec', 1.0)

        self.host = str(self.get_parameter('host').value)
        self.port = int(self.get_parameter('port').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.fix_topic = str(self.get_parameter('fix_topic').value)
        self.default_altitude_m = float(self.get_parameter('default_altitude_m').value)
        self.use_altitude_from_stream = bool(
            self.get_parameter('use_altitude_from_stream').value
        )
        self.covariance_m2 = max(0.01, float(self.get_parameter('covariance_m2').value))
        self.socket_timeout_sec = max(
            0.2,
            float(self.get_parameter('socket_timeout_sec').value),
        )
        self.reconnect_interval_sec = max(
            0.2,
            float(self.get_parameter('reconnect_interval_sec').value),
        )

        self._publisher = self.create_publisher(NavSatFix, self.fix_topic, 10)
        self._stop_event = threading.Event()
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._last_fix_log_time = 0.0
        self._reader_thread.start()

        self.get_logger().info(
            'RTK TCP fix bridge active: '
            f'{self.host}:{self.port} -> {self.fix_topic} ({self.frame_id})'
        )

    def destroy_node(self) -> bool:
        self._stop_event.set()
        if self._reader_thread.is_alive():
            self._reader_thread.join(timeout=1.0)
        return super().destroy_node()

    def _reader_loop(self) -> None:
        while rclpy.ok() and not self._stop_event.is_set():
            try:
                self._read_until_disconnect()
            except Exception as exc:  # pragma: no cover - hardware I/O path
                self.get_logger().warning(
                    f'RTK bridge disconnected from {self.host}:{self.port}: {exc}'
                )
            if not self._stop_event.is_set():
                time.sleep(self.reconnect_interval_sec)

    def _read_until_disconnect(self) -> None:
        with socket.create_connection(
            (self.host, self.port),
            timeout=self.socket_timeout_sec,
        ) as sock:
            sock.settimeout(self.socket_timeout_sec)
            with sock.makefile('r', encoding='utf-8', errors='ignore') as stream:
                self.get_logger().info(
                    f'Connected to RTK TCP source at {self.host}:{self.port}.'
                )
                while rclpy.ok() and not self._stop_event.is_set():
                    line = stream.readline()
                    if not line:
                        raise ConnectionError('RTK TCP stream closed by peer')
                    fix = self._parse_fix(line.strip())
                    if fix is None:
                        continue
                    self._publish_fix(*fix)

    def _parse_fix(self, line: str) -> Optional[tuple[float, float, float]]:
        if not line:
            return None

        json_fix = self._parse_json_fix(line)
        if json_fix is not None:
            return json_fix

        values = [float(token) for token in FLOAT_PATTERN.findall(line)]
        if len(values) < 2:
            return None

        for index in range(len(values) - 1):
            lat = values[index]
            lon = values[index + 1]
            altitude = self.default_altitude_m
            if index + 2 < len(values):
                parsed_altitude = _pick_altitude(values[index + 2])
                if parsed_altitude is not None:
                    altitude = parsed_altitude

            if _is_valid_latitude(lat) and _is_valid_longitude(lon) and not (
                abs(lat) < 1e-9 and abs(lon) < 1e-9
            ):
                return lat, lon, altitude

            if _is_valid_longitude(lat) and _is_valid_latitude(lon) and not (
                abs(lat) < 1e-9 and abs(lon) < 1e-9
            ):
                return lon, lat, altitude

        return None

    def _parse_json_fix(self, line: str) -> Optional[tuple[float, float, float]]:
        if not line.startswith('{'):
            return None
        try:
            payload = json.loads(line)
        except json.JSONDecodeError:
            return None

        latitude = payload.get('latitude', payload.get('lat'))
        longitude = payload.get('longitude', payload.get('lon', payload.get('lng')))
        if latitude is None or longitude is None:
            return None

        try:
            lat = float(latitude)
            lon = float(longitude)
        except (TypeError, ValueError):
            return None

        if not _is_valid_latitude(lat) or not _is_valid_longitude(lon):
            return None

        altitude = self.default_altitude_m
        if self.use_altitude_from_stream:
            for key in ('altitude', 'alt', 'height'):
                value = payload.get(key)
                if value is None:
                    continue
                try:
                    altitude = float(value)
                    break
                except (TypeError, ValueError):
                    continue

        return lat, lon, altitude

    def _publish_fix(self, latitude: float, longitude: float, altitude: float) -> None:
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = latitude
        msg.longitude = longitude
        msg.altitude = altitude if self.use_altitude_from_stream else self.default_altitude_m
        msg.position_covariance = [
            self.covariance_m2,
            0.0,
            0.0,
            0.0,
            self.covariance_m2,
            0.0,
            0.0,
            0.0,
            self.covariance_m2 * 4.0,
        ]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self._publisher.publish(msg)

        now = time.monotonic()
        if now - self._last_fix_log_time >= 5.0:
            self.get_logger().info(
                f'RTK fix lat={latitude:.8f}, lon={longitude:.8f}, alt={msg.altitude:.2f}'
            )
            self._last_fix_log_time = now


def main() -> None:
    rclpy.init()
    node = RtkTcpFixBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
