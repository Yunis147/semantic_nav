"""
Table Detector Node — runs on the PI.

Captures RGB+depth from a locally connected RealSense,
sends RGB frames to the laptop's yolo_server for detection,
and converts detections to map-frame coordinates via TF.

Usage:
    ros2 run nav2 table_detector --ros-args -p server_ip:=<LAPTOP_IP>
"""

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped

import pyrealsense2 as rs
import numpy as np
import cv2
import socket
import struct

SERIAL = "336222071373"


class TableDetectorNode(Node):
    def __init__(self):
        super().__init__('table_detector')

        # --- parameters ---
        self.declare_parameter('server_ip', '192.168.1.100')
        self.declare_parameter('server_port', 5555)
        self.server_ip = self.get_parameter('server_ip').value
        self.server_port = self.get_parameter('server_port').value

        # --- TF setup ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- RealSense setup (local on Pi) ---
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_device(SERIAL)
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
        cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
        self.pipe.start(cfg)
        self.align = rs.align(rs.stream.color)

        # --- socket to yolo_server (laptop) ---
        self.sock = None
        self.connect_to_server()

        # run detection loop at ~5Hz (Pi doesn't need faster)
        self.timer = self.create_timer(0.2, self.detect_loop)

    def connect_to_server(self):
        """Connect (or reconnect) to the laptop YOLO server."""
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5.0)
            self.sock.connect((self.server_ip, self.server_port))
            self.get_logger().info(
                f"Connected to yolo_server at {self.server_ip}:{self.server_port}")
        except Exception as e:
            self.get_logger().warn(f"Cannot connect to yolo_server: {e}")
            self.sock = None

    def recv_exact(self, n):
        """Receive exactly n bytes."""
        buf = b''
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionResetError("server closed connection")
            buf += chunk
        return buf

    def send_frame(self, img):
        """Send an RGB frame and receive detections from yolo_server.

        Returns list of (x1, y1, x2, y2, class_id, confidence) tuples.
        """
        if self.sock is None:
            self.connect_to_server()
            if self.sock is None:
                return []

        try:
            # encode frame as JPEG to reduce bandwidth
            _, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 80])
            jpg_bytes = jpg.tobytes()

            h, w, c = img.shape
            header = struct.pack('<IIII', h, w, c, len(jpg_bytes))
            self.sock.sendall(header + jpg_bytes)

            # receive detections
            num_boxes = struct.unpack('<I', self.recv_exact(4))[0]
            detections = []
            for _ in range(num_boxes):
                data = self.recv_exact(24)  # 5 ints + 1 float = 24 bytes
                x1, y1, x2, y2, cls_id, conf = struct.unpack('<iiiiif', data)
                detections.append((x1, y1, x2, y2, cls_id, conf))
            return detections

        except (ConnectionResetError, BrokenPipeError, socket.timeout, OSError) as e:
            self.get_logger().warn(f"Server connection lost: {e}, reconnecting...")
            self.sock = None
            return []

    def camera_point_to_map(self, x, y, z):
        """Transform a point from camera_link frame to map frame."""
        pt = PointStamped()
        pt.header.frame_id = 'camera_link'
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.point.x = x
        pt.point.y = y
        pt.point.z = z
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'camera_link', rclpy.time.Time())
            map_point = do_transform_point(pt, transform)
            return map_point.point.x, map_point.point.y, map_point.point.z
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None

    def detect_loop(self):
        # grab frames from local RealSense
        frames = self.align.process(self.pipe.wait_for_frames(timeout_ms=15000))
        color = frames.get_color_frame()
        depth = frames.get_depth_frame()
        if not color or not depth:
            return

        img = np.asanyarray(color.get_data())

        # send frame to laptop, get detections back
        detections = self.send_frame(img)

        for (x1, y1, x2, y2, cls_id, conf) in detections:
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            dist = depth.get_distance(cx, cy)
            if dist <= 0:
                continue

            # deproject pixel to 3D point in camera optical frame
            intrinsics = depth.profile.as_video_stream_profile().intrinsics
            point_3d = rs.rs2_deproject_pixel_to_point(intrinsics, [cx, cy], dist)

            # convert optical frame (X=right, Y=down, Z=forward)
            # to ROS camera_link frame (X=forward, Y=left, Z=up)
            ros_x = point_3d[2]   # optical Z -> ROS X (forward)
            ros_y = -point_3d[0]  # optical X -> ROS -Y (left)
            ros_z = -point_3d[1]  # optical Y -> ROS -Z (up)

            map_point = self.camera_point_to_map(ros_x, ros_y, ros_z)
            if map_point:
                self.get_logger().info(
                    f"Table at map: x={map_point[0]:.2f}, "
                    f"y={map_point[1]:.2f}, z={map_point[2]:.2f} "
                    f"(conf={conf:.2f}, depth={dist:.2f}m)")

    def destroy_node(self):
        self.pipe.stop()
        if self.sock:
            self.sock.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = TableDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
