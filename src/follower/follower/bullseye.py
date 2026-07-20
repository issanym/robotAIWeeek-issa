#!/usr/bin/env python3
"""
ArUco Marker Follower Node for ROS2 (laptop-webcam test version)
------------------------------------------------------------------
Grabs frames directly from a local webcam via cv2.VideoCapture (no camera
driver node, no camera_info topic, no tf tree needed), detects an ArUco
marker, estimates its distance/bearing using the pinhole camera model
(solvePnP), and drives a robot to follow it at a target standoff distance
by publishing geometry_msgs/TwistStamped on /cmd_vel.

Camera intrinsics are approximated from the frame resolution + an assumed
horizontal FOV, since there's no calibration file on hand yet. This is
good enough for testing the detection/control logic; swap in real
calibrated intrinsics before trusting the distance numbers on a real robot.

Dependencies:
    pip install opencv-contrib-python
    ros-<distro>-geometry-msgs

Run:
    ros2 run <your_pkg> aruco_follower.py --ros-args \
        -p camera_index:=0 \
        -p marker_size:=0.15 \
        -p target_distance:=0.6
"""

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class ArucoFollower(Node):
    def __init__(self):
        super().__init__('aruco_follower')

        # ---------------- Parameters ----------------
        self.declare_parameter('camera_index', 0)           # /dev/video0, etc.
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('horizontal_fov_deg', 60.0)   # rough guess for a laptop webcam
        self.declare_parameter('capture_rate_hz', 20.0)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('marker_id', -1)            # -1 = follow first marker seen
        self.declare_parameter('marker_size', 0.15)         # meters, side length of marker
        self.declare_parameter('aruco_dict', 'DICT_5X5_100')
        self.declare_parameter('target_distance', 0.6)      # meters, desired standoff
        self.declare_parameter('kp_linear', 0.6)
        self.declare_parameter('kp_angular', 1.8)
        self.declare_parameter('max_linear_speed', 0.4)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('distance_deadband', 0.05)   # meters
        self.declare_parameter('angle_deadband', 0.03)      # radians
        self.declare_parameter('lost_timeout', 1.0)         # sec before stopping if marker lost
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('show_debug_window', True)

        p = self.get_parameter
        self.camera_index = p('camera_index').value
        self.frame_width = p('frame_width').value
        self.frame_height = p('frame_height').value
        self.horizontal_fov_deg = p('horizontal_fov_deg').value
        self.capture_rate_hz = p('capture_rate_hz').value
        self.cmd_vel_topic = p('cmd_vel_topic').value
        self.marker_id = p('marker_id').value
        self.marker_size = p('marker_size').value
        self.target_distance = p('target_distance').value
        self.kp_linear = p('kp_linear').value
        self.kp_angular = p('kp_angular').value
        self.max_linear_speed = p('max_linear_speed').value
        self.max_angular_speed = p('max_angular_speed').value
        self.distance_deadband = p('distance_deadband').value
        self.angle_deadband = p('angle_deadband').value
        self.lost_timeout = p('lost_timeout').value
        self.frame_id = p('frame_id').value
        self.show_debug = p('show_debug_window').value

        dict_name = p('aruco_dict').value
        aruco_dict_id = getattr(cv2.aruco, dict_name, cv2.aruco.DICT_5X5_100)
        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(aruco_dict_id)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dictionary, self.aruco_params)

        # ---------------- State ----------------
        self.last_seen_time = None

        # ---------------- Webcam capture ----------------
        self.cap = cv2.VideoCapture(self.camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        if not self.cap.isOpened():
            self.get_logger().error(
                f'Could not open webcam at index {self.camera_index}.')

        # Approximate intrinsics from resolution + assumed horizontal FOV.
        # No calibration file needed for laptop testing; fine for tuning
        # detection/control logic, but not precision distance measurement.
        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or self.frame_width
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or self.frame_height
        fx = fy = (actual_w / 2.0) / np.tan(np.radians(self.horizontal_fov_deg / 2.0))
        cx, cy = actual_w / 2.0, actual_h / 2.0
        self.camera_matrix = np.array([
            [fx, 0,  cx],
            [0,  fy, cy],
            [0,  0,  1]
        ], dtype=np.float64)
        self.dist_coeffs = np.zeros((5, 1))  # assume negligible lens distortion
        self.get_logger().info(
            f'Using approximate intrinsics for {actual_w}x{actual_h} @ '
            f'{self.horizontal_fov_deg}deg HFOV (fx=fy={fx:.1f}). '
            'Replace with real calibration before trusting distance values.')

        # ---------------- Debug window ----------------
        if self.show_debug:
            cv2.namedWindow('aruco_follower', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('aruco_follower', actual_w, actual_h)

        # ---------------- Pub/Sub ----------------
        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)

        # Poll the webcam on a timer instead of a topic subscription
        self.capture_timer = self.create_timer(
            1.0 / self.capture_rate_hz, self.capture_callback)

        # Watchdog: stop the robot if the marker hasn't been seen recently
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_callback)

        self.get_logger().info(
            f"Aruco follower started (webcam index {self.camera_index}), "
            f"publishing TwistStamped to '{self.cmd_vel_topic}'.")

    # ------------------------------------------------------------------
    def capture_callback(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn('Failed to read frame from webcam.', throttle_duration_sec=5.0)
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        target_index = None
        if ids is not None and len(ids) > 0:
            ids = ids.flatten()
            if self.marker_id == -1:
                target_index = 0
            else:
                for i, mid in enumerate(ids):
                    if mid == self.marker_id:
                        target_index = i
                        break

        if target_index is None:
            # Nothing to track this frame -- still show the debug window
            # so you can see the feed is alive while searching for a marker.
            if self.show_debug:
                self._draw_debug(frame, found=False)
            return

        marker_corners = corners[target_index]

        # Pose via solvePnP (estimatePoseSingleMarkers was removed from
        # recent OpenCV contrib releases; solvePnP + IPPE_SQUARE is the
        # stable replacement for planar square markers).
        half = self.marker_size / 2.0
        obj_points = np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0]
        ], dtype=np.float64)

        img_points = marker_corners.reshape(4, 2).astype(np.float64)

        pnp_ok, rvec, tvec = cv2.solvePnP(
            obj_points, img_points, self.camera_matrix, self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE)

        if not pnp_ok:
            if self.show_debug:
                self._draw_debug(frame, found=False)
            return

        # tvec = [x, y, z] in camera frame (meters); z is forward distance
        distance = float(tvec[2][0])
        lateral_offset = float(tvec[0][0])          # + = marker to the right
        bearing = np.arctan2(lateral_offset, distance)  # rad, 0 = centered

        self.last_seen_time = self.get_clock().now()
        self.compute_and_publish(distance, bearing)

        if self.show_debug:
            self._draw_debug(frame, found=True, corners=marker_corners,
                              rvec=rvec, tvec=tvec, distance=distance, bearing=bearing)

    # ------------------------------------------------------------------
    def _draw_debug(self, frame, found, corners=None, rvec=None, tvec=None,
                     distance=None, bearing=None):
        """Render the debug popup: bounding box around the marker, pose axes,
        and status text. Called every frame, whether or not a marker is seen,
        so the window always shows a live feed."""
        if found:
            pts = corners.reshape(-1, 2).astype(np.int32)

            # Bounding box for a quick at-a-glance debug indicator
            x, y, w, h = cv2.boundingRect(pts)
            pad = 6
            cv2.rectangle(frame, (x - pad, y - pad), (x + w + pad, y + h + pad),
                          (0, 255, 0), 2)

            # Exact marker outline + ID label
            cv2.polylines(frame, [pts], isClosed=True, color=(0, 200, 255), thickness=2)
            cv2.putText(frame, 'MARKER', (x - pad, y - pad - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # 3D pose axes on the marker
            cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs,
                               rvec, tvec, self.marker_size * 0.5)

            cv2.putText(frame, f'dist={distance:.2f}m  bearing={np.degrees(bearing):.1f}deg',
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(frame, 'searching for marker...', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # Crosshair at image center for a visual reference on "centered"
        h_img, w_img = frame.shape[:2]
        cx, cy = w_img // 2, h_img // 2
        cv2.drawMarker(frame, (cx, cy), (255, 255, 255), cv2.MARKER_CROSS, 20, 1)

        cv2.imshow('aruco_follower', frame)
        cv2.waitKey(1)

    # ------------------------------------------------------------------
    def compute_and_publish(self, distance, bearing):
        distance_error = distance - self.target_distance
        if abs(distance_error) < self.distance_deadband:
            distance_error = 0.0
        if abs(bearing) < self.angle_deadband:
            bearing = 0.0

        linear_speed = self.kp_linear * distance_error
        # Positive bearing = marker to the right -> negative angular.z (turn right, REP-103)
        angular_speed = -self.kp_angular * bearing

        linear_speed = float(np.clip(linear_speed, -self.max_linear_speed, self.max_linear_speed))
        angular_speed = float(np.clip(angular_speed, -self.max_angular_speed, self.max_angular_speed))

        self.publish_cmd(linear_speed, angular_speed)

    # ------------------------------------------------------------------
    def publish_cmd(self, linear_x, angular_z):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.twist.linear.x = linear_x
        msg.twist.angular.z = angular_z
        self.cmd_pub.publish(msg)

    # ------------------------------------------------------------------
    def watchdog_callback(self):
        """Stop the robot if the marker hasn't been seen for lost_timeout seconds."""
        if self.last_seen_time is None:
            return
        elapsed = (self.get_clock().now() - self.last_seen_time).nanoseconds / 1e9
        if elapsed > self.lost_timeout:
            self.publish_cmd(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_cmd(0.0, 0.0)
        node.cap.release()
        if node.show_debug:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
