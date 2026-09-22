"""
YOLO Detection Server — runs on the LAPTOP (GPU).

Listens on a TCP socket for RGB frames from the Pi,
runs YOLO inference on GPU, and sends back detections.

Usage:
    python3 yolo_server.py                     # default 0.0.0.0:5555
    python3 yolo_server.py --port 5555         # custom port
    python3 yolo_server.py --model yolo11n.pt  # custom model
"""

import socket
import struct
import argparse
import numpy as np
import cv2
from ultralytics import YOLO

TABLE_ID = 60  # COCO class: dining table


def run_server(host, port, model_path):
    model = YOLO(model_path)
    print(f"[yolo_server] model '{model_path}' loaded on GPU")

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(1)
    print(f"[yolo_server] listening on {host}:{port}")

    while True:
        conn, addr = srv.accept()
        print(f"[yolo_server] client connected: {addr}")
        try:
            handle_client(conn, model)
        except (ConnectionResetError, BrokenPipeError):
            print(f"[yolo_server] client {addr} disconnected")
        finally:
            conn.close()


def recv_exact(conn, n):
    """Receive exactly n bytes from socket."""
    buf = b''
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            raise ConnectionResetError("connection closed")
        buf += chunk
    return buf


def handle_client(conn, model):
    while True:
        # --- receive frame ---
        # protocol: [height:4][width:4][channels:4][jpeg_len:4][jpeg_data]
        header = recv_exact(conn, 16)
        h, w, c, jpg_len = struct.unpack('<IIII', header)
        jpg_data = recv_exact(conn, jpg_len)

        # decode JPEG back to numpy
        img = cv2.imdecode(np.frombuffer(jpg_data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            # send zero detections
            conn.sendall(struct.pack('<I', 0))
            continue

        # --- run YOLO ---
        results = model(img, classes=[TABLE_ID], conf=0.4, verbose=False, device=0)[0]

        # --- send detections ---
        # protocol: [num_boxes:4] then per box: [x1:4][y1:4][x2:4][y2:4][class_id:4][conf:f]
        boxes = results.boxes
        num = len(boxes)
        conn.sendall(struct.pack('<I', num))

        for b in boxes:
            x1, y1, x2, y2 = map(int, b.xyxy[0])
            cls_id = int(b.cls[0])
            conf = float(b.conf[0])
            conn.sendall(struct.pack('<iiiiif', x1, y1, x2, y2, cls_id, conf))


def main():
    parser = argparse.ArgumentParser(description='YOLO detection server')
    parser.add_argument('--host', default='0.0.0.0', help='bind address')
    parser.add_argument('--port', type=int, default=5555, help='listen port')
    parser.add_argument('--model', default='yolo11n.pt', help='YOLO model path')
    args = parser.parse_args()
    run_server(args.host, args.port, args.model)


if __name__ == '__main__':
    main()
