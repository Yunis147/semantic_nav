"""
Table Detector Node — runs on the PI.

Captures RGB+depth from a locally connected RealSense,
sends RGB frames to the laptop's yolo_server for detection,
converts detections to map-frame coordinates via TF,
and stores unique tables (deduplication by distance threshold).

Tables are auto-named table1..table10 and saved to a JSON file.

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
import json
import os
import math

SERIAL = "419522072867"

# two detections closer than this (in meters) are the same table
DEDUP_DISTANCE = 0.5

# stop after this many unique tables
MAX_TABLES = 10

# how many detections to average before committing a table
MIN_HITS = 5

# where to save
TABLE_FILE = os.path.expanduser('~/tables.json')


class TableStore:
    """Accumulates table detections, deduplicates, and saves to JSON."""

    def __init__(self, file_path, dedup_dist, max_tables, min_hits, logger):
        self.file_path = file_path
        self.dedup_dist = dedup_dist
        self.max_tables = max_tables
        self.min_hits = min_hits
        self.logger = logger

        # each entry: {'name': 'table1', 'x': .., 'y': .., 'hits': N,
        #              'sum_x': .., 'sum_y': ..}
        # x, y = averaged detection point on the table surface (NOT true center)
        # true center + width/length will come from LiDAR clustering later
        self.tables = []
        self.load()

    def load(self):
        """Load previously saved tables from disk."""
        if os.path.exists(self.file_path):
            try:
                with open(self.file_path, 'r') as f:
                    saved = json.load(f)
                for t in saved:
                    # restore running sums so averaging can continue
                    t.setdefault('hits', self.min_hits)
                    t.setdefault('sum_x', t['x'] * t['hits'])
                    t.setdefault('sum_y', t['y'] * t['hits'])
                self.tables = saved
                self.logger.info(
                    f"Loaded {len(self.tables)} tables from {self.file_path}")
            except Exception as e:
                self.logger.warn(f"Could not load {self.file_path}: {e}")

    def save(self):
        """Write current tables to disk (only committed ones)."""
        out = []
        for t in self.tables:
            if t['hits'] >= self.min_hits:
                out.append({
                    'name': t['name'],
                    'x': round(t['x'], 3),
                    'y': round(t['y'], 3),
                })
        with open(self.file_path, 'w') as f:
            json.dump(out, f, indent=2)

    def _distance(self, x1, y1, x2, y2):
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def _next_name(self):
        used = {t['name'] for t in self.tables}
        for i in range(1, self.max_tables + 1):
            name = f"table{i}"
            if name not in used:
                return name
        return None

    def add_detection(self, x, y):
        """Process a new table detection at map coords (x, y).

        If it's close to an existing table, update that table's running average.
        Otherwise create a new table entry.
        Returns (table_name, is_new, is_committed) or None if full.
        """
        # find closest existing table
        best_dist = float('inf')
        best_table = None
        for t in self.tables:
            d = self._distance(x, y, t['x'], t['y'])
            if d < best_dist:
                best_dist = d
                best_table = t

        if best_table and best_dist < self.dedup_dist:
            # update running average
            best_table['hits'] += 1
            best_table['sum_x'] += x
            best_table['sum_y'] += y
            best_table['x'] = best_table['sum_x'] / best_table['hits']
            best_table['y'] = best_table['sum_y'] / best_table['hits']

            committed = best_table['hits'] >= self.min_hits
            if best_table['hits'] == self.min_hits:
                # just crossed the threshold — save
                self.save()
                self.logger.info(
                    f"✓ {best_table['name']} CONFIRMED at "
                    f"({best_table['x']:.2f}, {best_table['y']:.2f}) "
                    f"after {self.min_hits} detections")
            return best_table['name'], False, committed

        # new table
        if len(self.tables) >= self.max_tables:
            return None

        name = self._next_name()
        if name is None:
            return None

        entry = {
            'name': name,
            'x': x,
            'y': y,
            'hits': 1,
            'sum_x': x,
            'sum_y': y,
        }
        self.tables.append(entry)
        self.logger.info(
            f"+ New candidate {name} at ({x:.2f}, {y:.2f}) — "
            f"need {self.min_hits} hits to confirm")
        return name, True, False


class TableDetectorNode(Node):
    def __init__(self):
        super().__init__('table_detector')

        # --- parameters ---
        self.declare_parameter('server_ip', '192.168.1.100')
        self.declare_parameter('server_port', 5555)
        self.declare_parameter('table_file', TABLE_FILE)
        self.declare_parameter('dedup_distance', DEDUP_DISTANCE)
        self.declare_parameter('max_tables', MAX_TABLES)
        self.declare_parameter('min_hits', MIN_HITS)

        self.server_ip = self.get_parameter('server_ip').value
        self.server_port = self.get_parameter('server_port').value
        table_file = self.get_parameter('table_file').value
        dedup_dist = self.get_parameter('dedup_distance').value
        max_tables = self.get_parameter('max_tables').value
        min_hits = self.get_parameter('min_hits').value

        # --- table store ---
        self.store = TableStore(
            table_file, dedup_dist, max_tables, min_hits, self.get_logger())

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

        # run detection loop at ~5Hz
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
                # only use x, y for table position (z is height, not useful)
                result = self.store.add_detection(map_point[0], map_point[1])
                if result is None:
                    self.get_logger().info("All 10 tables found — ignoring new detections")

    def destroy_node(self):
        # save whatever we have on shutdown
        self.store.save()
        self.get_logger().info(f"Tables saved to {self.store.file_path}")
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
