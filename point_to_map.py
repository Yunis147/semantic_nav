import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped

import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO

SERIAL = "336222071373"
TABLE_ID = 60

class TableDetectorNode(Node):
    def __init__(self):
        super().__init__('table_detector')

        # TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # YOLO setup
        self.model = YOLO("yolo11n.pt")

        # RealSense setup
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_device(SERIAL)
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
        cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
        self.pipe.start(cfg)
        self.align = rs.align(rs.stream.color)

        # run detection loop on a timer instead of while True
        self.timer = self.create_timer(0.1, self.detect_loop)  # ~10Hz

    def camera_point_to_map(self, x, y, z):
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
            self.get_logger().warn(f'Transform failed: {e}')
            return None

    def detect_loop(self):
        frames = self.align.process(self.pipe.wait_for_frames(timeout_ms=15000))
        color, depth = frames.get_color_frame(), frames.get_depth_frame()
        if not color or not depth:
            return
        img = np.asanyarray(color.get_data())

        res = self.model(img, classes=[TABLE_ID], conf=0.4, verbose=False)[0]
        for b in res.boxes:
            x1, y1, x2, y2 = map(int, b.xyxy[0])
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            dist = depth.get_distance(cx, cy)
            if dist <= 0:
                continue

            intrinsics = depth.profile.as_video_stream_profile().intrinsics
            point_3d = rs.rs2_deproject_pixel_to_point(intrinsics, [cx, cy], dist)

            map_point = self.camera_point_to_map(point_3d[0], point_3d[1], point_3d[2])
            if map_point:
                self.get_logger().info(
                    f"Table at map coords: x={map_point[0]:.2f}, y={map_point[1]:.2f}, z={map_point[2]:.2f}")

            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(img, f"table {b.conf[0]:.2f} {dist:.2f}m", (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        cv2.imshow("table", img)
        cv2.waitKey(1)

    def destroy_node(self):
        self.pipe.stop()
        cv2.destroyAllWindows()
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
        rclpy.rmdir if False else None
        rclpy.shutdown()

if __name__ == '__main__':
    main()
