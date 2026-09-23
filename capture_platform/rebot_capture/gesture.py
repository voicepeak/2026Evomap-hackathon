"""电脑摄像头手势 → 夹爪行程（张开手 = 张开行程，握拳 = 闭合行程；不区分左右手）。

只做感知：后台线程读相机最新帧 → MediaPipe GestureRecognizer → 稳定判定后发布
"目标行程"；实际运动由控制循环（CaptureService.tick）交给 teleop_core 下发，
本模块不直接向电机发命令。

模型：capture_platform/models/vision/gesture_recognizer.task
（MediaPipe Gesture Recognizer，缺失时给出清晰错误并保持空闲）
"""
from __future__ import annotations

import threading
import time
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

# MediaPipe 类别 → 本模块动作
ACTION_MAP = {
    "Open_Palm": "open",
    "Closed_Fist": "fist",
}
TARGET_BY_ACTION = {
    "open": 1.0,    # 夹爪张开行程
    "fist": 0.0,    # 夹爪闭合行程
}


@dataclass
class GestureState:
    enabled: bool = False
    gesture: str = ""        # 已稳定生效的动作："open" / "fist" / ""
    raw: str = ""            # 最近一帧识别到的类别名
    score: float = 0.0
    handedness: str = ""
    target: float | None = None   # 1.0=张开行程 0.0=闭合行程 None=未给出
    updates: int = 0         # 生效次数（每次"手势变化"或"手离开后再次出现"加一）
    updated_s: float = 0.0   # 距最近一次稳定判定的秒数
    frames: int = 0
    fps: float = 0.0
    error: str = ""
    camera: int | None = None

    def to_dict(self) -> dict:
        return asdict(self)


def default_recognizer(model_path: Path):
    """构造 MediaPipe 手势识别器；返回可调用对象 (bgr_frame, ts_ms) -> [(name, score, handedness)]。"""
    import mediapipe as mp
    from mediapipe.tasks import python as mp_python
    from mediapipe.tasks.python import vision

    options = vision.GestureRecognizerOptions(
        base_options=mp_python.BaseOptions(model_asset_path=str(model_path)),
        running_mode=vision.RunningMode.VIDEO,
        num_hands=2,
    )
    rec = vision.GestureRecognizer.create_from_options(options)

    def recognize(bgr: np.ndarray, ts_ms: int):
        rgb = bgr[:, :, ::-1].copy()  # BGR → RGB
        image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        result = rec.recognize_for_video(image, int(ts_ms))
        out: list[tuple[str, float, str]] = []
        for i, categories in enumerate(result.gestures or []):
            if not categories:
                continue
            top = categories[0]
            hand = ""
            if result.handedness and i < len(result.handedness) and result.handedness[i]:
                hand = result.handedness[i][0].category_name or ""
            out.append((top.category_name or "", float(top.score or 0.0), hand))
        return out

    return recognize


class GestureWorker:
    """后台手势识别线程（只发布状态，不发运动命令）。"""

    def __init__(
        self,
        frame_getter,
        model_path: Path | str | None = None,
        *,
        camera: int | None = None,
        hold_s: float = 0.3,
        interval_s: float = 0.08,
        min_score: float = 0.45,
        recognizer=None,
    ):
        self._get_frame = frame_getter
        self.model_path = Path(model_path) if model_path else None
        self._recognizer = recognizer
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.hold_s = float(hold_s)
        self.interval_s = float(interval_s)
        self.min_score = float(min_score)
        self._state = GestureState(camera=camera)
        self._committed = ""      # 已生效的手势（同一手势不重复触发）
        self._updates = 0
        self._gone_at: float | None = None   # 手离开画面的起始时间

    # ------------------------------------------------------------------ #
    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        with self._lock:
            self._state.enabled = True
            self._state.error = ""
        self._thread = threading.Thread(target=self._loop, name="gesture", daemon=True)
        self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout)
            self._thread = None
        with self._lock:
            self._state.enabled = False

    close = stop

    def state(self) -> dict:
        with self._lock:
            st = GestureState(**self._state.__dict__)
        if st.updated_s > 0:
            st.updated_s = round(time.monotonic() - st.updated_s, 2)
        return st.to_dict()

    # ------------------------------------------------------------------ #
    def _publish(self, **kw) -> None:
        with self._lock:
            for k, v in kw.items():
                setattr(self._state, k, v)
            self._state.enabled = True

    def _loop(self) -> None:
        rec = self._recognizer
        if rec is None:
            if self.model_path is None or not self.model_path.exists():
                self._publish(enabled=True, error=f"缺少手势模型：{self.model_path}")
                return
            try:
                rec = default_recognizer(self.model_path)
            except Exception as e:  # noqa: BLE001
                self._publish(enabled=True, error=f"{type(e).__name__}: {e}")
                return

        t0 = time.monotonic()
        frames = 0
        candidate = ""
        candidate_since = 0.0
        while not self._stop.is_set():
            tick = time.monotonic()
            frame = self._get_frame()
            if frame is None:
                self._publish(error="相机没有画面（未打开 / 无权限）")
                time.sleep(0.25)
                continue
            try:
                dets = rec(frame, int((tick - t0) * 1000))
            except Exception as e:  # noqa: BLE001
                self._publish(error=f"识别失败：{type(e).__name__}: {e}")
                time.sleep(0.5)
                continue
            frames += 1
            best = max(dets, key=lambda d: d[1], default=None)
            name, score, hand = best if best else ("", 0.0, "")
            if score < self.min_score:
                name, score = "", 0.0
            action = ACTION_MAP.get(name, "")
            now = time.monotonic()

            if action and action == candidate:
                if now - candidate_since >= self.hold_s and action != self._committed:
                    self._committed = action
                    self._updates += 1
                    self._gone_at = None
                    self._publish(gesture=action, raw=name, score=round(score, 3), handedness=hand,
                                  target=TARGET_BY_ACTION[action], updates=self._updates,
                                  updated_s=now, error="")
                    candidate = ""
            elif action:
                if action != self._committed:      # 与已生效手势相同 → 不重复触发
                    candidate = action
                    candidate_since = now
                    self._gone_at = None
                else:
                    candidate = ""
            else:
                candidate = ""
                # 手离开画面一段时间 → 允许"再次出现同一手势"算一次新的触发
                if self._committed:
                    if self._gone_at is None:
                        self._gone_at = now
                    elif now - self._gone_at > 0.8:
                        self._committed = ""
                        self._gone_at = None

            with self._lock:
                self._state.raw = name
                self._state.score = round(float(score), 3)
                self._state.handedness = hand
                self._state.frames = frames
                self._state.fps = round(frames / max(1e-6, now - t0), 1)
                self._state.enabled = True
                self._state.error = ""

            dt = self.interval_s - (time.monotonic() - tick)
            if dt > 0:
                time.sleep(dt)
