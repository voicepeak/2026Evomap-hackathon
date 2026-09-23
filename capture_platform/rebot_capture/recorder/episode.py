"""Episode 数据模型与录制器。

一个 episode = 一次"抓取-放置"级别的演示：连续关节状态 + 动作 + 笔输入 + 成功标记。
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from ..device.backends import JointState
from ..teleop.samples import PenSample


@dataclass
class Episode:
    index: int
    task_id: str | None
    started_at: float
    ended_at: float
    success: bool
    t: np.ndarray
    pos: np.ndarray  # (N,6)
    vel: np.ndarray  # (N,6)
    tau: np.ndarray  # (N,6)
    grip: np.ndarray  # (N,)
    action: np.ndarray  # (N,7)
    pen_pressure: np.ndarray  # (N,)
    pen_touching: np.ndarray  # (N,) bool
    qc: list[dict[str, Any]] = field(default_factory=list)
    score: dict[str, Any] = field(default_factory=dict)
    files: dict[str, str] = field(default_factory=dict)
    note: str = ""

    # ------------------------------------------------------------------ #
    @property
    def n_frames(self) -> int:
        return int(self.t.size)

    @property
    def duration_s(self) -> float:
        return float(self.ended_at - self.started_at)

    @property
    def fps_effective(self) -> float:
        return self.n_frames / self.duration_s if self.duration_s > 0 else 0.0

    @property
    def grade(self) -> str:
        return str(self.score.get("grade", "?"))

    @property
    def score_value(self) -> float:
        return float(self.score.get("total", 0.0))

    def state_vector(self) -> np.ndarray:
        """(N,7) 观测：6 关节 + 夹爪。"""
        return np.concatenate([self.pos, self.grip.reshape(-1, 1)], axis=1)

    def to_dict(self, with_details: bool = False) -> dict[str, Any]:
        d: dict[str, Any] = {
            "index": self.index,
            "task_id": self.task_id,
            "duration_s": round(self.duration_s, 2),
            "frames": self.n_frames,
            "fps_effective": round(self.fps_effective, 1),
            "success": self.success,
            "grade": self.grade,
            "score": round(self.score_value, 1),
            "note": self.note,
            "files": dict(self.files),
        }
        if with_details:
            d["qc"] = list(self.qc)
            d["score_detail"] = dict(self.score)
        return d


class EpisodeRecorder:
    """在内存里攒一集数据，stop() 时产出 Episode。"""

    def __init__(self, index: int, task_id: str | None, fps: int):
        self.index = index
        self.task_id = task_id
        self.fps = fps
        self.started_at = time.time()
        self._t: list[float] = []
        self._pos: list[np.ndarray] = []
        self._vel: list[np.ndarray] = []
        self._tau: list[np.ndarray] = []
        self._grip: list[float] = []
        self._action: list[np.ndarray] = []
        self._pen_p: list[float] = []
        self._pen_touch: list[bool] = []

    # ------------------------------------------------------------------ #
    def add(self, state: JointState, pen: PenSample | None, action: np.ndarray) -> None:
        self._t.append(time.time())
        self._pos.append(np.asarray(state.pos, dtype=float))
        self._vel.append(np.asarray(state.vel, dtype=float))
        self._tau.append(np.asarray(state.tau, dtype=float))
        self._grip.append(float(state.grip))
        self._action.append(np.asarray(action, dtype=float).reshape(-1))
        self._pen_p.append(float(pen.pressure) if pen else 0.0)
        self._pen_touch.append(bool(pen.touching) if pen else False)

    def stop(self, success: bool, note: str = "") -> Episode:
        ended = time.time()
        n = len(self._t)
        if n == 0:
            raise ValueError("空 episode：没有任何样本")
        return Episode(
            index=self.index,
            task_id=self.task_id,
            started_at=self.started_at,
            ended_at=ended,
            success=success,
            t=np.asarray(self._t, dtype=float),
            pos=np.stack(self._pos),
            vel=np.stack(self._vel),
            tau=np.stack(self._tau),
            grip=np.asarray(self._grip, dtype=float),
            action=np.stack(self._action),
            pen_pressure=np.asarray(self._pen_p, dtype=float),
            pen_touching=np.asarray(self._pen_touch, dtype=bool),
            note=note,
        )
