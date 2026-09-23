#!/usr/bin/env python3
"""cam_server.py — 摄像头 + MediaPipe 双手检测 + 手势状态机 + 数据广播 + MJPEG 预览。

必须由「有摄像头权限的进程」运行（Terminal 或 CamView.app）。

用法:
    cd ~/Documents/Default\\ Project/reBotArm_control_py
    ~/.local/bin/uv run python cam_server.py --index 0

输出:
    数据:     tcp://127.0.0.1:9100   每帧一行 JSON
    预览:     http://127.0.0.1:9101/stream   （浏览器打开看骨架+手势）

说明:
    * 画面做**镜像**（自拍视角）→ 手往右移，画面里也往右
    * 左右手标签按当前摄像头的实测结果校准；更换摄像头可用 --no-swap-handedness
    * 同时检测两只手（右手拿笔、左手做手势），控制器只取左手

JSON:
    {"hand": true, "fps": 30.1, "t": ...,
     "hands": [{"handedness":"Left","gesture":"open","px":0.42,"py":0.33,
                "scale":0.13,"pinch":1.02,"n_ext":5}, ...]}
"""
from __future__ import annotations

import argparse
import json
import socket
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

import mediapipe as mp
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision

HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12),
    (9, 13), (13, 14), (14, 15), (15, 16),
    (13, 17), (17, 18), (18, 19), (19, 20),
    (0, 17),
]


class Shared:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.jpeg: bytes | None = None
        self.data: dict = {"hand": False, "hands": [], "fps": 0.0, "t": 0.0, "seq": 0}
        self.hist: dict[str, deque[str]] = {}
        self.stop = False


S = Shared()


def physical_handedness(label: str, swap: bool) -> str:
    """Correct the model label using the camera's observed handedness calibration."""
    if swap and label in ("Left", "Right"):
        return "Right" if label == "Left" else "Left"
    return label


def classify(lm: np.ndarray) -> tuple[str, dict]:
    """lm: (21,3) 归一化坐标（镜像画面）。返回 (gesture, info)"""
    def d(a: int, b: int) -> float:
        return float(np.linalg.norm(lm[a] - lm[b]))

    scale = max(d(0, 9), 1e-6)
    ext = {
        "index": d(8, 0) > d(6, 0) * 1.08,
        "middle": d(12, 0) > d(10, 0) * 1.08,
        "ring": d(16, 0) > d(14, 0) * 1.08,
        "pinky": d(20, 0) > d(18, 0) * 1.08,
    }
    thumb_ext = d(4, 17) > d(2, 17) * 1.05
    n_ext = sum(ext.values()) + (1 if thumb_ext else 0)
    pinch = d(4, 8) / scale

    if pinch < 0.28 and n_ext <= 3:
        g = "pinch"
    elif n_ext <= 1:
        g = "fist"
    elif ext["index"] and ext["middle"] and not ext["ring"] and not ext["pinky"]:
        g = "peace"
    elif n_ext >= 4 and pinch > 0.5:
        g = "open"
    else:
        g = "other"
    return g, {"n_ext": n_ext, "pinch": round(pinch, 3)}


def camera_thread(args: argparse.Namespace) -> None:
    options = vision.HandLandmarkerOptions(
        base_options=mp_python.BaseOptions(model_asset_path=args.model),
        running_mode=vision.RunningMode.VIDEO,
        num_hands=2,
        min_hand_detection_confidence=0.4,
        min_hand_presence_confidence=0.4,
        min_tracking_confidence=0.4,
    )
    landmarker = vision.HandLandmarker.create_from_options(options)

    cap = cv2.VideoCapture(args.index, cv2.CAP_AVFOUNDATION)
    if not cap.isOpened():
        cap = cv2.VideoCapture(args.index)
    if not cap.isOpened():
        print(f"❌ 打不开摄像头 index={args.index}（权限？被占用？）", flush=True)
        S.stop = True
        return
    if args.width:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    ok, frame = cap.read()
    if not ok:
        print("❌ 摄像头无画面（权限未授予？）", flush=True)
        S.stop = True
        return
    h, w = frame.shape[:2]
    print(f"✅ 摄像头已打开 index={args.index}  {w}x{h}（镜像模式）", flush=True)

    t0 = time.time()
    n = 0
    fps = 0.0
    last_log = 0.0
    while not S.stop:
        t_loop = time.perf_counter()
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.01)
            continue
        frame = cv2.flip(frame, 1)          # 镜像：自拍视角
        brightness = float(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).mean())
        n += 1
        fps = n / max(time.time() - t0, 1e-6)

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_img = mp.Image(image_format=mp.ImageFormat.SRGB, data=np.ascontiguousarray(rgb))
        res = landmarker.detect_for_video(mp_img, int(time.time() * 1000))

        hands_out: list[dict] = []
        seen_keys: set[str] = set()
        if res.hand_landmarks:
            for i, hl in enumerate(res.hand_landmarks):
                lm = np.array([[p.x, p.y, p.z] for p in hl], dtype=np.float64)
                raw_handed = ""
                if res.handedness and i < len(res.handedness):
                    raw_handed = res.handedness[i][0].category_name
                handed = physical_handedness(raw_handed, args.swap_handedness)
                key = handed or f"h{i}"
                seen_keys.add(key)
                g_raw, info = classify(lm)
                hist = S.hist.setdefault(key, deque(maxlen=5))
                hist.append(g_raw)
                gl = list(hist)
                gesture = max(set(gl), key=gl.count) if len(gl) >= 3 else g_raw
                palm = lm[[0, 5, 9, 13, 17]].mean(axis=0)
                hands_out.append({
                    "handedness": handed,
                    "raw_handedness": raw_handed,
                    "gesture": gesture,
                    "px": round(float(palm[0]), 4),
                    "py": round(float(palm[1]), 4),
                    "scale": round(float(np.linalg.norm(lm[9] - lm[0])), 4),
                    "pinch": info["pinch"],
                    "n_ext": info["n_ext"],
                })
                # 画骨架（左手=绿色控制手，右手=灰色）
                ctrl = handed == "Left"
                col = (80, 255, 120) if ctrl else (150, 150, 150)
                for a, b in HAND_CONNECTIONS:
                    pa = (int(lm[a][0] * w), int(lm[a][1] * h))
                    pb = (int(lm[b][0] * w), int(lm[b][1] * h))
                    cv2.line(frame, pa, pb, col, 2, cv2.LINE_AA)
                for k in range(21):
                    cv2.circle(frame, (int(lm[k][0] * w), int(lm[k][1] * h)), 4, col, -1, cv2.LINE_AA)
                cv2.circle(frame, (int(palm[0] * w), int(palm[1] * h)), 10,
                           (255, 120, 255), 3, cv2.LINE_AA)
                tag = f"{'★LEFT' if ctrl else 'RIGHT'} {gesture}"
                cv2.putText(frame, tag, (int(lm[0][0] * w) - 20, int(lm[0][1] * h) - 18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, col, 2, cv2.LINE_AA)

        for k in list(S.hist.keys()):
            if k not in seen_keys:
                S.hist.pop(k, None)

        # One sequence number per camera frame. The publisher may send the same
        # snapshot more than once; consumers must not treat a repeat as fresh input.
        with S.lock:
            S.data = {"hand": bool(hands_out), "hands": hands_out,
                      "fps": round(fps, 1), "t": time.time(), "seq": n,
                      "brightness": round(brightness, 1)}

        cv2.putText(frame, f"camera={args.index}  hands={len(hands_out)}  {fps:4.1f}fps  (mirrored)",
                    (14, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2, cv2.LINE_AA)
        if not hands_out and brightness < 45:
            cv2.putText(frame, "LOW LIGHT - show left palm near lens",
                        (14, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 180, 255), 2, cv2.LINE_AA)
        okj, jpg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if okj:
            with S.lock:
                S.jpeg = jpg.tobytes()

        if time.time() - last_log > 2.0:
            last_log = time.time()
            with S.lock:
                d = dict(S.data)
            desc = " | ".join(f"{x['handedness'][:1]}:{x['gesture']}"
                              f"({x['px']:.2f},{x['py']:.2f})" for x in d["hands"]) or "无手"
            print(f"[cam] {desc}  {d['fps']:.0f}fps  light={d['brightness']:.0f}", flush=True)

        # 按目标帧率节流（扣除本帧计算耗时）
        time.sleep(max(0.0, 1.0 / args.fps_cap - (time.perf_counter() - t_loop)))

    cap.release()
    landmarker.close()
    print("[cam] 已停止", flush=True)


def publisher_thread(port: int) -> None:
    def _bind():
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("127.0.0.1", port))
        s.listen(1)
        s.settimeout(0.5)
        return s

    srv = _bind()
    print(f"✅ 数据端口已监听 tcp://127.0.0.1:{port}", flush=True)
    conn: socket.socket | None = None
    while not S.stop:
        if conn is None:
            try:
                conn, addr = srv.accept()
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                print(f"✅ 客户端已连接: {addr}", flush=True)
            except socket.timeout:
                continue
            except OSError as e:
                # 关键：不要退出线程！重建监听套接字后继续（否则端口会永久失联）
                print(f"[pub] accept 出错: {e}，重建监听…", flush=True)
                try:
                    srv.close()
                except Exception:  # noqa: BLE001
                    pass
                time.sleep(0.5)
                try:
                    srv = _bind()
                    print("[pub] 监听已恢复", flush=True)
                except OSError as e2:
                    print(f"[pub] 重建失败: {e2}（1s 后重试）", flush=True)
                    time.sleep(1.0)
                continue
        with S.lock:
            payload = json.dumps(S.data) + "\n"
        try:
            conn.sendall(payload.encode())
        except OSError:
            print("[pub] 客户端断开", flush=True)
            conn.close()
            conn = None
            continue
        time.sleep(0.01)
    if conn:
        conn.close()
    srv.close()


def preview_thread(port: int) -> None:
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, body: bytes, ctype: str) -> None:
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self._send(("<html><head><title>cam_server</title></head>"
                            "<body style='background:#111;color:#eee'>"
                            "<h3>cam_server 预览（镜像视角，★LEFT = 控制手）</h3>"
                            "<p>这里只显示识别结果。机械臂默认冻结；控制窗口按 Space 解锁后，"
                            "左掌需在画面中心张开并保持 0.6 秒才会建立控制锚点。</p>"
                            "<img src='/stream' style='max-width:100%'></body></html>").encode(), "text/html; charset=utf-8")
                return
            if self.path == "/snap":
                with S.lock:
                    jpg = S.jpeg
                if jpg:
                    self._send(jpg, "image/jpeg")
                else:
                    self.send_error(503)
                return
            if self.path == "/data":
                with S.lock:
                    body = (json.dumps(S.data) + "\n").encode()
                self._send(body, "application/json")
                return
            if self.path == "/stream":
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                try:
                    while not S.stop:
                        with S.lock:
                            jpg = S.jpeg
                        if jpg:
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                            self.wfile.write(f"Content-Length: {len(jpg)}\r\n\r\n".encode())
                            self.wfile.write(jpg)
                            self.wfile.write(b"\r\n")
                        time.sleep(1 / 25)
                except (BrokenPipeError, ConnectionResetError):
                    pass
                return
            self.send_error(404)

    httpd = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    print(f"✅ 预览地址 http://127.0.0.1:{port}/stream", flush=True)
    httpd.serve_forever()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--index", type=int, default=0)
    ap.add_argument("--model", default="models/hand_landmarker.task")
    ap.add_argument("--port", type=int, default=9100)
    ap.add_argument("--preview-port", type=int, default=9101)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps-cap", type=float, default=30)
    ap.add_argument("--swap-handedness", action=argparse.BooleanOptionalAction, default=True,
                    help="按当前 0 号摄像头的实测结果反转左右手标签")
    args = ap.parse_args()

    print("=== cam_server 启动 ===", flush=True)
    print(f"  摄像头 index={args.index}  模型={args.model}", flush=True)
    print(f"  左右手标签校准: {'反转' if args.swap_handedness else '原样'}", flush=True)
    threading.Thread(target=publisher_thread, args=(args.port,), daemon=True).start()
    threading.Thread(target=preview_thread, args=(args.preview_port,), daemon=True).start()
    try:
        camera_thread(args)
    except KeyboardInterrupt:
        pass
    finally:
        S.stop = True
        time.sleep(0.3)
        print("=== cam_server 退出 ===", flush=True)


if __name__ == "__main__":
    main()
