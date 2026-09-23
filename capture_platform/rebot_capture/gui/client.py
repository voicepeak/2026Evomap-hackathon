"""GUI 的服务客户端：REST（异步）/ 两个 WebSocket 线程 / MJPEG 拉流线程。

GUI 是采集服务的**客户端**，与浏览器界面等价：
- 笔输入走 /ws/teleop（线程安全、只保留最新样本，避免积压）
- 状态刷新走 /ws/state（10Hz）
- 相机画面走 /api/camera/stream（MJPEG，本地解析，缩到角落小窗）

所有耗时接口（replay / park / camera probe…）都在线程池里跑，不冻结界面。
"""
from __future__ import annotations

import json
import socket
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from typing import Any, Callable

import cv2
import numpy as np
from PySide6.QtCore import QObject, QRunnable, QThread, QThreadPool, QTimer, Signal
from PySide6.QtGui import QImage

try:
    from websockets.sync.client import connect as ws_connect
    HAS_WS = True
except Exception:  # pragma: no cover
    ws_connect = None
    HAS_WS = False


class ApiError(Exception):
    """REST 调用失败（带 HTTP 状态与后端 detail）。"""

    def __init__(self, status: int, detail: str):
        super().__init__(detail or f"HTTP {status}")
        self.status = status
        self.detail = detail or ""


# ──────────────────────────────────────────────────────────────────────── #
# 异步 REST
# ──────────────────────────────────────────────────────────────────────── #
class _Signals(QObject):
    done = Signal(int, object)          # task_id, (result | ApiError)
    state = Signal(dict)                # /ws/state 负载
    ws_ok = Signal(bool)                # 笔通道连通性
    frame = Signal(object, QImage)      # 相机 key（索引或 URL）, 画面


class _Job(QRunnable):
    def __init__(self, fn: Callable[[], Any], sig: _Signals, task_id: int):
        super().__init__()
        self._fn, self._sig, self._id = fn, sig, task_id

    def run(self) -> None:  # noqa: D102
        try:
            self._sig.done.emit(self._id, self._fn())
        except ApiError as e:
            self._sig.done.emit(self._id, e)
        except Exception as e:  # noqa: BLE001
            self._sig.done.emit(self._id, ApiError(0, f"{type(e).__name__}: {e}"))


# ──────────────────────────────────────────────────────────────────────── #
# WS 线程
# ──────────────────────────────────────────────────────────────────────── #
class _StateThread(QThread):
    """持续接收 /ws/state（断线自动重连）。"""

    def __init__(self, base_ws: str, sig: _Signals):
        super().__init__()
        self._url = base_ws + "/ws/state"
        self._sig = sig
        self._stop = threading.Event()

    def stop(self) -> None:
        self._stop.set()

    def run(self) -> None:  # noqa: D102
        while not self._stop.is_set():
            if not HAS_WS:
                self._sig.state.emit({})
                return
            try:
                with ws_connect(self._url, open_timeout=3, max_size=None) as ws:
                    while not self._stop.is_set():
                        try:
                            raw = ws.recv(timeout=1.0)
                        except TimeoutError:
                            continue
                        if raw is None:
                            break
                        if isinstance(raw, (bytes, bytearray)):
                            raw = raw.decode("utf-8", "replace")
                        try:
                            self._sig.state.emit(json.loads(raw))
                        except json.JSONDecodeError:
                            continue
            except Exception:  # noqa: BLE001
                pass
            if not self._stop.is_set():
                self._stop.wait(1.2)


class _PenThread(QThread):
    """把最新笔样本发到 /ws/teleop（只保留最新，绝不积压）。"""

    def __init__(self, base_ws: str, sig: _Signals):
        super().__init__()
        self._url = base_ws + "/ws/teleop"
        self._sig = sig
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._latest: str | None = None
        self._wake = threading.Event()

    def stop(self) -> None:
        self._stop.set()
        self._wake.set()

    def send(self, sample: dict) -> None:
        payload = json.dumps({"type": "pen", "data": sample})
        with self._lock:
            self._latest = payload
        self._wake.set()

    def run(self) -> None:  # noqa: D102
        while not self._stop.is_set():
            if not HAS_WS:
                return
            try:
                # 启动即连（屏幕上立刻能看到"笔"通道状态），再等样本
                with ws_connect(self._url, open_timeout=3) as ws:
                    self._sig.ws_ok.emit(True)
                    while not self._stop.is_set():
                        self._wake.wait(0.2)
                        self._wake.clear()
                        if self._stop.is_set():
                            return
                        with self._lock:
                            payload, self._latest = self._latest, None
                        if payload:
                            ws.send(payload)
            except Exception:  # noqa: BLE001
                pass
            finally:
                self._sig.ws_ok.emit(False)
            if not self._stop.is_set():
                self._stop.wait(1.2)


# ──────────────────────────────────────────────────────────────────────── #
# MJPEG 拉流
# ──────────────────────────────────────────────────────────────────────── #
class _MjpegThread(QThread):
    """从 /api/camera/stream 读 MJPEG，节流后发出缩小的 QImage（按相机 key 路由）。"""

    def __init__(self, base_url: str, key: Any, path: str, sig: _Signals,
                 width: int = 320, max_fps: float = 12.0):
        super().__init__()
        self._key = key
        self._sig = sig
        self._width = int(width)
        self._interval = 1.0 / max(1.0, max_fps)
        self._stop = threading.Event()
        self._sock: socket.socket | None = None
        self._parsed = urllib.parse.urlparse(base_url)
        self._path = path

    def stop(self) -> None:
        self._stop.set()
        s = self._sock
        if s is not None:
            try:
                s.close()
            except Exception:  # noqa: BLE001
                pass

    # ------------------------------------------------------------------ #
    def _read_jpeg(self, buf: bytearray) -> bytes | None:
        """从缓冲区里取一帧 JPEG（解析 Content-Length），不足则返回 None。"""
        marker = b"\r\n\r\n"
        end = buf.find(marker)
        if end < 0:
            if len(buf) > 65536:      # 异常流，丢掉防止无限增长
                del buf[:-1024]
            return None
        head = bytes(buf[:end])
        length = 0
        for line in head.split(b"\r\n"):
            if line.lower().startswith(b"content-length:"):
                try:
                    length = int(line.split(b":", 1)[1].strip())
                except ValueError:
                    length = 0
        if length <= 0:
            del buf[: end + 4]
            return None
        start = end + 4
        if len(buf) < start + length:
            return None
        frame = bytes(buf[start:start + length])
        del buf[: start + length]
        return frame

    def run(self) -> None:  # noqa: D102
        host = self._parsed.hostname or "127.0.0.1"
        port = self._parsed.port or 80
        buf = bytearray()
        while not self._stop.is_set():
            try:
                sock = socket.create_connection((host, port), timeout=2.0)
                self._sock = sock
                sock.sendall(
                    f"GET {self._path} HTTP/1.1\r\nHost: {host}:{port}\r\n"
                    "Accept: multipart/x-mixed-replace\r\nConnection: close\r\n\r\n".encode()
                )
                sock.settimeout(0.5)
                # 吃掉响应头
                while not self._stop.is_set():
                    try:
                        chunk = sock.recv(4096)
                    except socket.timeout:
                        continue
                    if not chunk:
                        break
                    buf.extend(chunk)
                    end = buf.find(b"\r\n\r\n")
                    if end >= 0:
                        del buf[: end + 4]
                        break
                buf.clear()
                last = 0.0
                while not self._stop.is_set():
                    try:
                        chunk = sock.recv(16384)
                    except socket.timeout:
                        continue
                    if not chunk:
                        break
                    buf.extend(chunk)
                    while True:
                        jpg = self._read_jpeg(buf)
                        if jpg is None:
                            break
                        now = time.monotonic()
                        if now - last < self._interval:
                            continue          # 丢弃多余帧，保持界面轻快
                        last = now
                        arr = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                        if arr is None:
                            continue
                        h, w = arr.shape[:2]
                        if w > self._width:
                            h = max(1, int(h * self._width / w))
                            arr = cv2.resize(arr, (self._width, h), interpolation=cv2.INTER_AREA)
                        arr = np.ascontiguousarray(arr)
                        img = QImage(arr.data, arr.shape[1], arr.shape[0],
                                     arr.strides[0], QImage.Format.Format_BGR888).copy()
                        self._sig.frame.emit(self._key, img)
            except Exception:  # noqa: BLE001
                pass
            finally:
                if self._sock is not None:
                    try:
                        self._sock.close()
                    except Exception:  # noqa: BLE001
                        pass
                    self._sock = None
            if not self._stop.is_set():
                self._stop.wait(1.0)


# ──────────────────────────────────────────────────────────────────────── #
# 客户端
# ──────────────────────────────────────────────────────────────────────── #
class ServiceClient(QObject):
    state = Signal(dict)
    ws_ok = Signal(bool)
    frame = Signal(object, QImage)

    def __init__(self, base_url: str = "http://127.0.0.1:8787", parent: QObject | None = None):
        super().__init__(parent)
        self.base_url = base_url.rstrip("/")
        self._sig = _Signals()
        self._sig.done.connect(self._on_done)
        self._sig.state.connect(self.state)
        self._sig.ws_ok.connect(self.ws_ok)
        self._sig.frame.connect(self.frame)
        self._pool = QThreadPool.globalInstance()
        self._callbacks: dict[int, Callable[[Any], None]] = {}
        self._next_id = 0
        self._threads: list[QThread] = []
        self._streams: dict[int, _MjpegThread] = {}
        self._state_thread: _StateThread | None = None
        self._pen_thread: _PenThread | None = None

    # ------------------------------------------------------------------ #
    # 同步 REST（只用于启动探测／健康检查）
    # ------------------------------------------------------------------ #
    def request(self, method: str, path: str, body: dict | None = None, timeout: float = 8.0) -> Any:
        url = self.base_url + path
        data = json.dumps(body).encode() if body is not None else None
        req = urllib.request.Request(
            url, data=data, method=method,
            headers={"Content-Type": "application/json"} if data else {},
        )
        try:
            with urllib.request.urlopen(req, timeout=timeout) as resp:
                raw = resp.read()
                return json.loads(raw) if raw else None
        except urllib.error.HTTPError as e:
            detail = ""
            try:
                detail = json.loads(e.read()).get("detail", "")
            except Exception:  # noqa: BLE001
                pass
            raise ApiError(e.code, str(detail)) from None
        except urllib.error.URLError as e:
            raise ApiError(0, f"连接失败：{e.reason}") from None

    def get(self, path: str) -> Any:
        return self.request("GET", path)

    def post(self, path: str, body: dict | None = None) -> Any:
        return self.request("POST", path, body if body is not None else {})

    # ------------------------------------------------------------------ #
    # 异步 REST
    # ------------------------------------------------------------------ #
    def call(self, method: str, path: str, body: dict | None, done: Callable[[Any], None]) -> None:
        """在后台线程调用 REST；done 收到结果（或 ApiError）。"""
        self._next_id += 1
        task_id = self._next_id
        self._callbacks[task_id] = done
        self._pool.start(_Job(lambda: self.request(method, path, body), self._sig, task_id))

    def post_async(self, path: str, body: dict | None, done: Callable[[Any], None]) -> None:
        self.call("POST", path, body, done)

    def get_async(self, path: str, done: Callable[[Any], None]) -> None:
        self.call("GET", path, None, done)

    def _on_done(self, task_id: int, result: Any) -> None:
        cb = self._callbacks.pop(task_id, None)
        if cb is not None:
            cb(result)

    # ------------------------------------------------------------------ #
    # WS / 视频流
    # ------------------------------------------------------------------ #
    def start(self) -> None:
        base_ws = self.base_url.replace("http://", "ws://").replace("https://", "wss://")
        self._state_thread = _StateThread(base_ws, self._sig)
        self._pen_thread = _PenThread(base_ws, self._sig)
        self._state_thread.start()
        self._pen_thread.start()
        self._threads += [self._state_thread, self._pen_thread]

    def stop(self) -> None:
        for t in self._threads:
            t.stop()  # type: ignore[attr-defined]
        for t in self._threads:
            t.wait(1500)
        for s in list(self._streams.values()):
            s.stop()
        for s in list(self._streams.values()):
            s.wait(1500)
        self._streams.clear()

    def send_pen(self, sample: dict) -> None:
        if self._pen_thread is not None:
            self._pen_thread.send(sample)

    def open_stream(self, key: Any, path: str, width: int = 320) -> None:
        if key in self._streams:
            return
        th = _MjpegThread(self.base_url, key, path, self._sig, width=width)
        self._streams[key] = th
        th.start()

    def close_stream(self, key: Any) -> None:
        th = self._streams.pop(key, None)
        if th is not None:
            th.stop()
            th.wait(1500)

    def close_all_streams(self) -> None:
        for index in list(self._streams):
            self.close_stream(index)
