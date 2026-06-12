"""OpenCV USB camera publisher for quick vehicle camera bringup."""

from __future__ import annotations

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header


class UsbCameraNode(Node):
    """Publish frames from a UVC/OpenCV camera as sensor_msgs/Image."""

    def __init__(self) -> None:
        super().__init__('usb_camera_node')

        self.declare_parameter('camera_index', 1)
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 20.0)
        self.declare_parameter('use_dshow', False)

        self._frame_id = str(self.get_parameter('frame_id').value)
        self._image_topic = str(self.get_parameter('image_topic').value)
        camera_index = int(self.get_parameter('camera_index').value)
        width = int(self.get_parameter('width').value)
        height = int(self.get_parameter('height').value)
        fps = float(self.get_parameter('fps').value)
        use_dshow = bool(self.get_parameter('use_dshow').value)

        backend = cv2.CAP_DSHOW if use_dshow else cv2.CAP_ANY
        self._capture = cv2.VideoCapture(camera_index, backend)
        if not self._capture.isOpened():
            raise RuntimeError(f'Failed to open camera index {camera_index}')

        if width > 0:
            self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        if height > 0:
            self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if fps > 0.0:
            self._capture.set(cv2.CAP_PROP_FPS, fps)

        self._image_pub = self.create_publisher(Image, self._image_topic, 10)
        period = 1.0 / max(fps, 1.0)
        self._timer = self.create_timer(period, self._publish_frame)

        actual_width = self._capture.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_height = self._capture.get(cv2.CAP_PROP_FRAME_HEIGHT)
        actual_fps = self._capture.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(
            f'USB camera {camera_index} publishing {self._image_topic}: '
            f'{actual_width:.0f}x{actual_height:.0f} @ {actual_fps:.1f} FPS'
        )

    def _publish_frame(self) -> None:
        ok, frame_bgr = self._capture.read()
        if not ok or frame_bgr is None:
            self.get_logger().warn('Failed to read USB camera frame', throttle_duration_sec=2.0)
            return

        msg = Image()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.height = int(frame_bgr.shape[0])
        msg.width = int(frame_bgr.shape[1])
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = int(frame_bgr.shape[1] * 3)
        msg.data = frame_bgr.tobytes()
        self._image_pub.publish(msg)

    def destroy_node(self) -> bool:
        if hasattr(self, '_capture') and self._capture is not None:
            self._capture.release()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = UsbCameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
