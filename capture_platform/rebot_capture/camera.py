"""相机模块：MJPEG 预览 + 逐 episode 录像（可选，无相机/无权限时优雅降级）。

- 每个相机一个后台线程，按 fps 读帧
- 预览：编码 JPEG 供 /api/camera/stream（MJPEG）
- 录像：episode 期间写 MP4（cv2.VideoWriter），停止后返回文件信息
"""
from __future__ import annotations

import threading
import time
from pathlib import Path

from typing import Any

import numpy as np

try:
    import cv2

    HAS_CV2 = True
except Exception:  # pragma: no cover
    cv2 = None
    HAS_CV2 = False


class Camera:
    def __init__(self, index: int | None = None, width: int = 640, height: int = 480, fps: int = 30,
                 jpeg_quality: int = 70, name: str | None = None, url: str | None = None):
        self.index = int(index) if index is not None else None
        self.url = url
        self.name = name or (f"cam{index}" if index is not None else (url or "cam"))
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.jpeg_quality = int(jpeg_quality)

        self._cap = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._jpeg: bytes | None = None
        self._frame: np.ndarray | None = None
        self._err = ""
        self._opened = False
        self._frames = 0
        self._t0 = time.monotonic()

        self._writer = None
        self._rec_path: Path | None = None
        self._rec_frames = 0
        self._rec_started = 0.0

    # ------------------------------------------------------------------ #
    def open(self) -> bool:
        if not HAS_CV2:
            self._err = "未安装 opencv（pip install 'rebot-capture[camera]'）"
            return False
        try:
            cap = cv2.VideoCapture(self.url if self.url else self.index)
            try:
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                cap.set(cv2.CAP_PROP_FPS, self.fps)
            except Exception:  # noqa: BLE001
                pass
            if not cap.isOpened():
                cap.release()
                self._err = "打开失败（设备被占用 / 无权限 / 索引不对）"
                return False
            ok, frame = cap.read()
            if not ok or frame is None:
                cap.release()
                self._err = "已打开但读不到帧（macOS 相机权限？系统设置→隐私与安全性→相机→勾选 OpenCode）"
                return False
            self._cap = cap
            self._opened = True
            self._err = ""
            self._stop.clear()
            self._thread = threading.Thread(target=self._loop, name=f"camera-{self.index}", daemon=True)
            self._thread.start()
            return True
        except Exception as e:  # noqa: BLE001
            self._err = f"{type(e).__name__}: {e}"
            return False

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.5)
            self._thread = None
        with self._lock:
            if self._writer is not None:
                try:
                    self._writer.release()
                except Exception:  # noqa: BLE001
                    pass
                self._writer = None
            if self._cap is not None:
                try:
                    self._cap.release()
                except Exception:  # noqa: BLE001
                    pass
                self._cap = None
        self._opened = False

    # ------------------------------------------------------------------ #
    def _loop(self) -> None:
        period = 1.0 / max(1, self.fps)
        while not self._stop.is_set():
            t0 = time.monotonic()
            cap = self._cap
            if cap is None:
                break
            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.05)
                continue
            self._frames += 1
            with self._lock:
                self._frame = frame
                if HAS_CV2:
                    q = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
                    try:
                        okj, buf = cv2.imencode(".jpg", frame, q)
                        if okj:
                            self._jpeg = buf.tobytes()
                    except Exception:  # noqa: BLE001
                        pass
                if self._writer is not None:
                    try:
                        self._writer.write(frame)
                        self._rec_frames += 1
                    except Exception:  # noqa: BLE001
                        pass
            dt = period - (time.monotonic() - t0)
            if dt > 0:
                time.sleep(dt)

    # ------------------------------------------------------------------ #
    def start_recording(self, path: Path, fps: int | None = None) -> bool:
        if not self._opened or not HAS_CV2:
            return False
        with self._lock:
            frame = self._frame
        if frame is None:
            return False
        h, w = frame.shape[:2]
        path.parent.mkdir(parents=True, exist_ok=True)
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(str(path), fourcc, float(fps or self.fps), (w, h))
        if not writer.isOpened():
            return False
        with self._lock:
            self._writer = writer
            self._rec_path = Path(path)
            self._rec_frames = 0
            self._rec_started = time.monotonic()
        return True

    def stop_recording(self) -> dict | None:
        with self._lock:
            writer = self._writer
            path = self._rec_path
            frames = self._rec_frames
            dur = time.monotonic() - self._rec_started if self._rec_started else 0.0
            self._writer = None
            self._rec_path = None
        if writer is not None:
            try:
                writer.release()
            except Exception:  # noqa: BLE001
                pass
        if path is None:
            return None
        return {
            "path": str(path),
            "frames": int(frames),
            "duration_s": round(float(dur), 2),
            "fps": round(frames / dur, 1) if dur > 0 else 0.0,
        }

    def snapshot(self) -> bytes | None:
        with self._lock:
            return self._jpeg

    def frame(self) -> np.ndarray | None:
        """最近一帧原始画面（复制一份，供识别线程使用）。"""
        with self._lock:
            return None if self._frame is None else self._frame.copy()

    def status(self) -> dict:
        return {
            "index": self.index,
            "url": self.url,
            "name": self.name,
            "opened": self._opened,
            "error": self._err,
            "frames": self._frames,
            "fps_actual": round(self._frames / max(1e-6, time.monotonic() - self._t0), 1),
            "recording": self._writer is not None,
            "recording_frames": self._rec_frames,
            "resolution": [self.width, self.height],
        }


class CameraHub:
    """相机集合/选择/录制协调。支持 USB 相机（索引）与 MJPEG 桥（URL）。"""

    def __init__(self, width: int = 640, height: int = 480, fps: int = 30, max_index: int = 4,
                 bridge_ports: tuple[int, ...] = (8801, 8802, 8803, 8804)):
        self.width, self.height, self.fps = width, height, fps
        self.max_index = max_index
        self.bridge_ports = tuple(bridge_ports)
        self.cameras: dict[Any, Camera] = {}
        self.selected: Any | None = None
        self.aliases: dict[Any, str] = {}   # key → 数据集里的名字（wrist / scene ...）
        self.last_error = ""

    # ------------------------------------------------------------------ #
    def probe(self) -> list[Any]:
        """探测：先试 USB 相机索引，再试本机相机桥端口（Terminal 权限方案）。"""
        found: list[Any] = []
        if not HAS_CV2:
            self.last_error = "未安装 opencv"
            return found
        for i in range(self.max_index):
            if i in self.cameras:
                found.append(i)
                continue
            cam = Camera(i, self.width, self.height, self.fps)
            if cam.open():
                self.cameras[i] = cam
                found.append(i)
            else:
                self.last_error = cam.status()["error"]
        for port in self.bridge_ports:
            url = f"http://127.0.0.1:{port}/stream"
            if url in self.cameras:
                found.append(url)
                continue
            cam = Camera(None, self.width, self.height, self.fps, name=f"bridge{port}", url=url)
            if cam.open():
                self.cameras[url] = cam
                found.append(url)
        if found and self.selected is None:
            self.selected = found[0]
        return found

    def open(self, key: Any) -> dict:
        """key 可以是相机索引（int）或 MJPEG URL（str）。"""
        if key not in self.cameras:
            if isinstance(key, str) and key.startswith("http"):
                cam = Camera(None, self.width, self.height, self.fps, url=key, name=key.split("//")[-1].split("/")[0])
            else:
                cam = Camera(int(key), self.width, self.height, self.fps)
            if not cam.open():
                self.last_error = cam.status()["error"]
                return {"ok": False, "error": self.last_error}
            self.cameras[key] = cam
        self.selected = key
        return {"ok": True, "source": key}

    def open_url(self, url: str, name: str | None = None) -> dict:
        cam = Camera(None, self.width, self.height, self.fps, url=url, name=name)
        if not cam.open():
            self.last_error = cam.status()["error"]
            return {"ok": False, "error": self.last_error}
        self.cameras[url] = cam
        self.selected = url
        return {"ok": True, "source": url}

    def close(self, key: Any | None = None) -> dict:
        k = self.selected if key is None else key
        cam = self.cameras.pop(k, None)
        if cam is not None:
            cam.close()
        if self.selected == k:
            self.selected = next(iter(self.cameras), None)
        return {"ok": True, "closed": k}

    def close_all(self) -> None:
        for cam in list(self.cameras.values()):
            cam.close()
        self.cameras.clear()
        self.selected = None

    # ------------------------------------------------------------------ #
    def set_alias(self, key: Any, name: str) -> dict:
        """给某路相机起数据集里的名字（wrist / scene / camN）。"""
        safe = "".join(ch for ch in str(name).strip() if ch.isalnum() or ch in "_-")
        if not safe:
            return {"ok": False, "error": "名字只能是字母/数字/_/-"}
        if key in self.cameras:
            self.aliases[key] = safe
        elif key is None and self.selected is not None:
            self.aliases[self.selected] = safe
        else:
            return {"ok": False, "error": f"没有这个相机: {key}"}
        self.cameras.get(key if key in self.cameras else self.selected).name = safe
        return {"ok": True, "key": key, "alias": safe}

    def current_alias(self) -> str:
        if self.selected is None:
            return "cam0"
        return self.aliases.get(self.selected, self.cameras[self.selected].name if self.selected in self.cameras else "cam0")

    # ------------------------------------------------------------------ #
    @property
    def current(self) -> Camera | None:
        return self.cameras.get(self.selected) if self.selected is not None else None

    def start_recording(self, path: Path, fps: int | None = None) -> bool:
        cam = self.current
        return bool(cam and cam.start_recording(path, fps))

    def stop_recording(self) -> dict | None:
        cam = self.current
        return cam.stop_recording() if cam else None

    def get(self, key: Any | None = None) -> Camera | None:
        return self.cameras.get(self.selected if key is None else key)

    def snapshot(self, key: Any | None = None) -> bytes | None:
        cam = self.get(key)
        return cam.snapshot() if cam else None

    def frame(self, key: Any | None = None) -> np.ndarray | None:
        cam = self.get(key)
        return cam.frame() if cam else None

    def status(self) -> dict:
        return {
            "has_opencv": HAS_CV2,
            "selected": self.selected,
            "selected_alias": self.current_alias() if self.selected is not None else None,
            "cameras": [
                {**c.status(), "alias": self.aliases.get(k, c.name)}
                for k, c in self.cameras.items()
            ],
            "error": self.last_error,
        }
