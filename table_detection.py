import pyrealsense2 as rs, numpy as np, cv2
from ultralytics import YOLO

SERIAL = "419522072867"
model = YOLO("yolo11n.pt")
TABLE_ID = 60

pipe, cfg = rs.pipeline(), rs.config()
cfg.enable_device(SERIAL)
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
pipe.start(cfg)
align = rs.align(rs.stream.color)

try:
    while True:
        frames = align.process(pipe.wait_for_frames(timeout_ms=15000))
        color, depth = frames.get_color_frame(), frames.get_depth_frame()
        if not color or not depth: continue
        img = np.asanyarray(color.get_data())

        res = model(img, classes=[TABLE_ID], conf=0.4, verbose=False)[0]
        for b in res.boxes:
            x1, y1, x2, y2 = map(int, b.xyxy[0])
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            dist = depth.get_distance(cx, cy)
            intrinsics = depth.profile.as_video_stream_profile().intrinsics
            point_3d = rs.rs2_deproject_pixel_to_point(intrinsics, [cx, cy], dist)
            print(f"table 3D point (camera frame): X={point_3d[0]:.2f}, Y={point_3d[1]:.2f}, Z={point_3d[2]:.2f}")
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(img, f"table {b.conf[0]:.2f} {dist:.2f}m", (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        cv2.imshow("table", img)
        if cv2.waitKey(1) == 27: break
finally:
    pipe.stop(); cv2.destroyAllWindows()
