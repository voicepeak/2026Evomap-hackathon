"""机械臂后端抽象。

采集端只依赖这个协议；真机实现（motorbridge / reBotArm_control_py）与
Mock 实现可以互换，保证"没有硬件也能开发/演示"。
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Protocol, Sequence, runtime_checkable

import numpy as np


class BackendError(RuntimeError):
    pass


@dataclass
class JointState:
    """一帧关节状态（单位：rad、rad/s、N·m；夹爪为归一化开度 0-1）。"""

    t: float
    pos: np.ndarray
    vel: np.ndarray
    tau: np.ndarray
    grip: float = 0.0

    @property
    def vector(self) -> np.ndarray:
        """6 关节 + 夹爪 = 7 维观测向量（数据集用）。"""
        return np.concatenate([self.pos, np.array([self.grip], dtype=float)])


@dataclass
class ArmStatus:
    connected: bool = False
    calibrated: bool = False
    motor_count: int = 0
    voltage: float = 0.0
    temps_c: list[float] = field(default_factory=list)
    faults: dict[str, str] = field(default_factory=dict)
    error: str | None = None

    def to_dict(self) -> dict:
        return {
            "connected": self.connected,
            "calibrated": self.calibrated,
            "motor_count": self.motor_count,
            "voltage": round(self.voltage, 2),
            "temps_c": [round(t, 1) for t in self.temps_c],
            "faults": dict(self.faults),
            "error": self.error,
        }


@runtime_checkable
class ArmBackend(Protocol):
    """所有机械臂后端必须实现的最小接口。"""

    name: str

    def connect(self) -> None: ...

    def close(self) -> None: ...

    def status(self) -> ArmStatus: ...

    def read(self) -> JointState: ...

    def command(self, q_target: Sequence[float]) -> None:
        """下发 6 关节目标角 + 夹爪开度（长度为 7 的向量）。"""

    def park(self) -> None:
        """平滑回到 park 位（零位附近自锁姿态）。"""

    def disable(self) -> None:
        """失能（仅允许在 park 位附近由上层调用）。"""


def now() -> float:
    return time.monotonic()
