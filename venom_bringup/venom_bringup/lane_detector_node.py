"""Traditional-vision lane detector for low-speed right-lane following."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32


@dataclass
class LaneFit:
    left: np.ndarray
    right: np.ndarray


class LaneDetectorNode(Node):
    """Detect a lane region and publish center error plus debug image."""

    def __init__(self) -> None:
        super().__init__('lane_detector_node')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('debug_image_topic', '/lane/debug_image')
        self.declare_parameter('center_error_topic', '/lane/center_error')
        self.declare_parameter('valid_topic', '/lane/valid')
        self.declare_parameter('target_width', 1280)
        self.declare_parameter('target_height', 720)
        self.declare_parameter('camera_matrix', [4322.6, 0.0, 938.2, 0.0, 4322.3, 549.5, 0.0, 0.0, 1.0])
        self.declare_parameter('distortion_coefficients', [-0.5656, 0.3260, 0.0, 0.0, -0.2207])
        self.declare_parameter('calibration_width', 1920)
        self.declare_parameter('calibration_height', 1080)
        self.declare_parameter('use_undistort', True)
        self.declare_parameter('use_exposure_adjust', False)
        self.declare_parameter('gamma', 1.4)
        self.declare_parameter('brightness_beta', -15.0)
        self.declare_parameter('perspective_points', [520.0, 330.0, 880.0, 330.0, 180.0, 719.0, 1240.0, 719.0])
        self.declare_parameter('s_thresh_min', 130)
        self.declare_parameter('s_thresh_max', 255)
        self.declare_parameter('sx_thresh_min', 25)
        self.declare_parameter('sx_thresh_max', 220)
        self.declare_parameter('l_thresh_min', 80)
        self.declare_parameter('lane_width_min_px', 300.0)
        self.declare_parameter('lane_width_max_px', 900.0)
        self.declare_parameter('max_center_jump_px', 180.0)
        self.declare_parameter('fit_smoothing_alpha', 0.70)
        self.declare_parameter('horizontal_filter_width_px', 85)
        self.declare_parameter('vertical_connect_height_px', 31)

        self._target_width = int(self.get_parameter('target_width').value)
        self._target_height = int(self.get_parameter('target_height').value)
        self._use_undistort = bool(self.get_parameter('use_undistort').value)
        self._use_exposure_adjust = bool(self.get_parameter('use_exposure_adjust').value)
        self._gamma = float(self.get_parameter('gamma').value)
        self._brightness_beta = float(self.get_parameter('brightness_beta').value)
        self._s_thresh = (
            int(self.get_parameter('s_thresh_min').value),
            int(self.get_parameter('s_thresh_max').value),
        )
        self._sx_thresh = (
            int(self.get_parameter('sx_thresh_min').value),
            int(self.get_parameter('sx_thresh_max').value),
        )
        self._l_thresh_min = int(self.get_parameter('l_thresh_min').value)
        self._lane_width_min_px = float(self.get_parameter('lane_width_min_px').value)
        self._lane_width_max_px = float(self.get_parameter('lane_width_max_px').value)
        self._max_center_jump_px = float(self.get_parameter('max_center_jump_px').value)
        self._fit_smoothing_alpha = float(self.get_parameter('fit_smoothing_alpha').value)
        self._horizontal_filter_width_px = int(self.get_parameter('horizontal_filter_width_px').value)
        self._vertical_connect_height_px = int(self.get_parameter('vertical_connect_height_px').value)

        self._camera_matrix = np.array(self.get_parameter('camera_matrix').value, dtype=np.float32).reshape(3, 3)
        self._distortion = np.array(self.get_parameter('distortion_coefficients').value, dtype=np.float32)
        self._scale_camera_matrix()
        self._perspective_points = np.array(self.get_parameter('perspective_points').value, dtype=np.float32).reshape(4, 2)
        self._perspective_matrix, self._perspective_inverse = self._build_perspective_matrices()
        self._last_fit: Optional[LaneFit] = None

        self._center_pub = self.create_publisher(
            Float32,
            str(self.get_parameter('center_error_topic').value),
            10,
        )
        self._valid_pub = self.create_publisher(
            Bool,
            str(self.get_parameter('valid_topic').value),
            10,
        )
        self._debug_pub = self.create_publisher(
            Image,
            str(self.get_parameter('debug_image_topic').value),
            10,
        )
        self._image_sub = self.create_subscription(
            Image,
            str(self.get_parameter('image_topic').value),
            self._on_image,
            10,
        )
        self.get_logger().info('Lane detector ready')

    def _scale_camera_matrix(self) -> None:
        calibration_width = float(self.get_parameter('calibration_width').value)
        calibration_height = float(self.get_parameter('calibration_height').value)
        if calibration_width <= 0.0 or calibration_height <= 0.0:
            return
        scale_x = self._target_width / calibration_width
        scale_y = self._target_height / calibration_height
        self._camera_matrix[0, 0] *= scale_x
        self._camera_matrix[0, 2] *= scale_x
        self._camera_matrix[1, 1] *= scale_y
        self._camera_matrix[1, 2] *= scale_y

    def _build_perspective_matrices(self) -> tuple[np.ndarray, np.ndarray]:
        img_size = (self._target_width, self._target_height)
        offset_x = 330.0
        dst = np.array(
            [
                [offset_x, 0.0],
                [img_size[0] - offset_x, 0.0],
                [offset_x, img_size[1]],
                [img_size[0] - offset_x, img_size[1]],
            ],
            dtype=np.float32,
        )
        matrix = cv2.getPerspectiveTransform(self._perspective_points, dst)
        inverse = cv2.getPerspectiveTransform(dst, self._perspective_points)
        return matrix, inverse

    def _image_msg_to_bgr(self, msg: Image) -> Optional[np.ndarray]:
        if msg.encoding not in ('bgr8', 'rgb8'):
            self.get_logger().warn(f'Unsupported image encoding: {msg.encoding}', throttle_duration_sec=2.0)
            return None
        channels = 3
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step // channels, channels)
        frame = frame[:, : msg.width, :]
        if msg.encoding == 'rgb8':
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        return frame.copy()

    def _publish_debug(self, stamp, frame_bgr: np.ndarray) -> None:
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = 'camera_link'
        msg.height = int(frame_bgr.shape[0])
        msg.width = int(frame_bgr.shape[1])
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = int(frame_bgr.shape[1] * 3)
        msg.data = frame_bgr.tobytes()
        self._debug_pub.publish(msg)

    def _adjust_exposure(self, frame_bgr: np.ndarray) -> np.ndarray:
        if not self._use_exposure_adjust:
            return frame_bgr
        gamma = max(self._gamma, 0.1)
        table = np.array([((i / 255.0) ** gamma) * 255 for i in range(256)]).astype('uint8')
        adjusted = cv2.LUT(frame_bgr, table)
        return cv2.convertScaleAbs(adjusted, alpha=1.0, beta=self._brightness_beta)

    def _preprocess(self, frame_bgr: np.ndarray) -> np.ndarray:
        resized = cv2.resize(frame_bgr, (self._target_width, self._target_height))
        adjusted = self._adjust_exposure(resized)
        if not self._use_undistort:
            return adjusted
        return cv2.undistort(adjusted, self._camera_matrix, self._distortion, None, self._camera_matrix)

    def _lane_binary(self, frame_bgr: np.ndarray) -> np.ndarray:
        hls = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HLS).astype(np.float32)
        l_channel = hls[:, :, 1]
        s_channel = hls[:, :, 2]
        sobelx = cv2.Sobel(l_channel, cv2.CV_64F, 1, 0)
        abs_sobelx = np.absolute(sobelx)
        max_sobel = np.max(abs_sobelx)
        if max_sobel <= 0.0:
            scaled_sobel = np.zeros_like(abs_sobelx, dtype=np.uint8)
        else:
            scaled_sobel = np.uint8(255 * abs_sobelx / max_sobel)

        sxbinary = np.zeros_like(scaled_sobel, dtype=np.uint8)
        sxbinary[(scaled_sobel >= self._sx_thresh[0]) & (scaled_sobel <= self._sx_thresh[1])] = 1

        s_binary = np.zeros_like(s_channel, dtype=np.uint8)
        s_binary[(s_channel >= self._s_thresh[0]) & (s_channel <= self._s_thresh[1])] = 1

        color_binary = np.zeros_like(sxbinary, dtype=np.uint8)
        color_binary[((sxbinary == 1) | (s_binary == 1)) & (l_channel > self._l_thresh_min)] = 1
        return color_binary

    def _filter_lane_binary(self, binary_img: np.ndarray) -> np.ndarray:
        mask = (binary_img > 0).astype(np.uint8) * 255
        horizontal_kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT,
            (max(3, self._horizontal_filter_width_px), 5),
        )
        horizontal = cv2.morphologyEx(mask, cv2.MORPH_OPEN, horizontal_kernel)
        mask = cv2.subtract(mask, horizontal)

        vertical_kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT,
            (5, max(3, self._vertical_connect_height_px)),
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, vertical_kernel)

        denoise_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, denoise_kernel)
        return (mask > 0).astype(np.uint8)

    @staticmethod
    def _fit_lines(binary_warped: np.ndarray) -> LaneFit:
        histogram = np.sum(binary_warped[:, :], axis=0)
        midpoint = int(histogram.shape[0] / 2)
        leftx_base = int(np.argmax(histogram[:midpoint]))
        rightx_base = int(np.argmax(histogram[midpoint:]) + midpoint)
        nwindows = 9
        window_height = int(binary_warped.shape[0] / nwindows)
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        leftx_current = leftx_base
        rightx_current = rightx_base
        margin = 100
        minpix = 50
        left_lane_inds = []
        right_lane_inds = []

        for window in range(nwindows):
            win_y_low = binary_warped.shape[0] - (window + 1) * window_height
            win_y_high = binary_warped.shape[0] - window * window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin
            good_left_inds = (
                (nonzeroy >= win_y_low)
                & (nonzeroy < win_y_high)
                & (nonzerox >= win_xleft_low)
                & (nonzerox < win_xleft_high)
            ).nonzero()[0]
            good_right_inds = (
                (nonzeroy >= win_y_low)
                & (nonzeroy < win_y_high)
                & (nonzerox >= win_xright_low)
                & (nonzerox < win_xright_high)
            ).nonzero()[0]
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)
            if len(good_left_inds) > minpix:
                leftx_current = int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:
                rightx_current = int(np.mean(nonzerox[good_right_inds]))

        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]
        if len(leftx) == 0 or len(rightx) == 0:
            raise ValueError('expected non-empty vector for lane fit')
        return LaneFit(
            left=np.polyfit(lefty, leftx, 2),
            right=np.polyfit(righty, rightx, 2),
        )

    def _stabilize_fit(self, fit: LaneFit, y_eval: int) -> LaneFit:
        left_x = fit.left[0] * y_eval ** 2 + fit.left[1] * y_eval + fit.left[2]
        right_x = fit.right[0] * y_eval ** 2 + fit.right[1] * y_eval + fit.right[2]
        lane_width = right_x - left_x
        center_x = (left_x + right_x) / 2.0
        valid_fit = self._lane_width_min_px <= lane_width <= self._lane_width_max_px

        if self._last_fit is not None:
            last_left_x = self._last_fit.left[0] * y_eval ** 2 + self._last_fit.left[1] * y_eval + self._last_fit.left[2]
            last_right_x = self._last_fit.right[0] * y_eval ** 2 + self._last_fit.right[1] * y_eval + self._last_fit.right[2]
            last_center_x = (last_left_x + last_right_x) / 2.0
            if abs(center_x - last_center_x) > self._max_center_jump_px:
                valid_fit = False

        if not valid_fit:
            if self._last_fit is None:
                raise ValueError('unstable lane fit')
            return self._last_fit

        if self._last_fit is None:
            self._last_fit = fit
            return fit

        alpha = self._fit_smoothing_alpha
        smoothed = LaneFit(
            left=alpha * self._last_fit.left + (1.0 - alpha) * fit.left,
            right=alpha * self._last_fit.right + (1.0 - alpha) * fit.right,
        )
        self._last_fit = smoothed
        return smoothed

    def _draw_lane(self, base_bgr: np.ndarray, binary_warped: np.ndarray, fit: LaneFit) -> np.ndarray:
        y_max = binary_warped.shape[0]
        out_img = np.dstack((binary_warped, binary_warped, binary_warped)) * 255
        left_points = [
            [fit.left[0] * y ** 2 + fit.left[1] * y + fit.left[2], y]
            for y in range(y_max)
        ]
        right_points = [
            [fit.right[0] * y ** 2 + fit.right[1] * y + fit.right[2], y]
            for y in range(y_max - 1, -1, -1)
        ]
        line_points = np.vstack((left_points, right_points))
        cv2.fillPoly(out_img, np.int_([line_points]), (0, 255, 0))
        lane_overlay = cv2.warpPerspective(
            out_img,
            self._perspective_inverse,
            (self._target_width, self._target_height),
        )
        return cv2.addWeighted(base_bgr, 1.0, lane_overlay, 0.45, 0)

    def _on_image(self, msg: Image) -> None:
        frame_bgr = self._image_msg_to_bgr(msg)
        if frame_bgr is None:
            return

        valid_msg = Bool()
        center_msg = Float32()
        try:
            processed = self._preprocess(frame_bgr)
            binary = self._lane_binary(processed)
            warped = cv2.warpPerspective(
                binary,
                self._perspective_matrix,
                (self._target_width, self._target_height),
            )
            warped = self._filter_lane_binary(warped)
            fit = self._fit_lines(warped)
            fit = self._stabilize_fit(fit, warped.shape[0] - 1)
            debug = self._draw_lane(processed, warped, fit)

            y_eval = warped.shape[0] - 1
            left_x = fit.left[0] * y_eval ** 2 + fit.left[1] * y_eval + fit.left[2]
            right_x = fit.right[0] * y_eval ** 2 + fit.right[1] * y_eval + fit.right[2]
            lane_center = (left_x + right_x) / 2.0
            center_error = float(lane_center - (self._target_width / 2.0))

            valid_msg.data = True
            center_msg.data = center_error
            cv2.putText(
                debug,
                f'center_error: {center_error:.1f}px',
                (30, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.2,
                (255, 255, 255),
                3,
            )
        except Exception as exc:
            debug = cv2.resize(frame_bgr, (self._target_width, self._target_height))
            valid_msg.data = False
            center_msg.data = 0.0
            cv2.putText(debug, 'Lane lost', (40, 80), cv2.FONT_HERSHEY_SIMPLEX, 1.8, (0, 0, 255), 4)
            cv2.putText(debug, str(exc)[:80], (40, 135), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        self._valid_pub.publish(valid_msg)
        self._center_pub.publish(center_msg)
        self._publish_debug(msg.header.stamp, debug)


def main() -> None:
    rclpy.init()
    node = LaneDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
