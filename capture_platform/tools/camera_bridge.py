#!/usr/bin/env python3
"""相机桥：用 Terminal 的相机权限打开所有摄像头，提供 MJPEG 流。

为什么需要它：OpenCode.app 没有 macOS 相机权限，而 Terminal 有。
在 Terminal 里运行本脚本，平台就能通过 http://127.0.0.1:<port>/stream 读画面。

用法：
    cd "/Users/Admin/Documents/Default Project/rebot_capture"
    .venv/bin/python tools/camera_bridge.py            # 自动探测 0..3
    .venv/bin/python tools/camera_bridge.py --index 0 1
端口：8801, 8802, ...（每个可用相机一个）
"""
from __future__ import annotations

import argparse
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2

DEV_W, DEV_H, DEV_FPS = 640, 480, 30
LOG_PATH = Path("/tmp/rebot_camera_bridge.log")


class _Log:
    def __init__(self, path: Path):
        self.path = path

    def write(self, s: str) -> None:
        try:
            with self.path.open("a", encoding="utf-8") as f:
                f.write(s)
        except Exception:  # noqa: BLE001
            pass


LOG = _Log(LOG_PATH)


class Cam:
    def __init__(self, index: int, port: int):
        self.index = index
        self.port = port
        self.jpeg: bytes | None = None
        self.err = ""
        self.frames = 0
        self._stop = threading.Event()
        self._cap = None

    def start(self) -> bool:
        cap = cv2.VideoCapture(self.index)
        if not cap.isOpened():
            self.err = "打不开（权限/占用/索引）"
            return False
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, DEV_W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, DEV_H)
        cap.set(cv2.CAP_PROP_FPS, DEV_FPS)
        ok, frame = cap.read()
        if not ok or frame is None:
            self.err = "读不到帧（权限？）"
            cap.release()
            return False
        self._cap = cap
        threading.Thread(target=self._loop, daemon=True).start()
        return True

    def _loop(self) -> None:
        period = 1.0 / DEV_FPS
        while not self._stop.is_set():
            t0 = time.monotonic()
            ok, frame = self._cap.read()
            if ok and frame is not None:
                okj, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 72])
                if okj:
                    self.jpeg = buf.tobytes()
                    self.frames += 1
            dt = period - (time.monotonic() - t0)
            if dt > 0:
                time.sleep(dt)

    def stop(self) -> None:
        self._stop.set()
        if self._cap is not None:
            self._cap.release()


CAMS: dict[int, Cam] = {}


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):  # 静音
        pass

    def do_GET(self):  # noqa: N802
        try:
            self._do_get()
        except (BrokenPipeError, ConnectionResetError):
            return
        except Exception as e:  # noqa: BLE001
            LOG.write(f"[{time.strftime('%H:%M:%S')}] handler error: {type(e).__name__}: {e}\n")
            try:
                self.send_error(500)
            except Exception:  # noqa: BLE001
                pass

    def _do_get(self) -> None:
        port = self.server.server_address[1]
        if self.path.startswith("/stream"):
            cam = CAMS.get(port)
            if cam is None or cam.jpeg is None:
                self.send_error(503, "no frame")
                return
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            while True:
                jpg = cam.jpeg
                if jpg:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                     + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
                time.sleep(1.0 / 20.0)
        else:
            body = "\n".join(f"cam{c.index} → http://127.0.0.1:{c.port}/stream  ({c.frames}帧)"
                             for c in CAMS.values()).encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--index", type=int, nargs="*", default=None, help="指定相机索引；默认探测 0..3")
    ap.add_argument("--port", type=int, default=8801, help="起始端口")
    args = ap.parse_args()

    indices = args.index if args.index else list(range(4))
    port = args.port
    for idx in indices:
        cam = Cam(idx, port)
        if cam.start():
            CAMS[port] = cam
            print(f"✅ cam{idx} → http://127.0.0.1:{port}/stream")
            port += 1
        else:
            print(f"❌ cam{idx}: {cam.err}")

    if not CAMS:
        print("没有可用相机（请确认 Terminal 有相机权限：系统设置→隐私与安全性→相机→Terminal）")
        return 1

    servers = []
    for p, cam in CAMS.items():
        srv = ThreadingHTTPServer(("127.0.0.1", p), Handler)
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        servers.append(srv)
    print("\n⚠️ 保持这个 Terminal 窗口开着（不要按 Ctrl+C），平台里就能看到画面。")
    print(f"日志: {LOG_PATH}")
    while True:
        time.sleep(1)


if __name__ == "__main__":
    raise SystemExit(main())
