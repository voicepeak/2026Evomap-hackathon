"""Mock 机械臂后端 —— 没有硬件也能开发 / 演示 / 自检。

一阶惯性近似 + 限位裁剪 + 温度/电压模拟；接口与真机后端完全一致。
"""
from __future__ import annotations

import math
from typing import Sequence

import numpy as np

from .backends import ArmStatus, JointState, now
from .profile import DeviceProfile


class MockArm:
    name = "mock"

    def __init__(self, profile: DeviceProfile, seed: int = 7):
        self.profile = profile
        self.n = profile.n_joints
        self._rng = np.random.default_rng(seed)

        self._limits = np.asarray(profile.limits_array(), dtype=float)  # (n,2)
        self._q = np.zeros(self.n)
        self._qd = np.zeros(self.n)
        self._tau = np.zeros(self.n)
        self._grip = 0.0

        self._target = np.zeros(self.n)
        self._grip_target = 0.0

        self._connected = False
        self._enabled = False
        self._last_t = now()
        self._temps = np.full(self.n + 1, 42.0)
        self._limit_hits = 0
        self._samples = 0

    # ------------------------------------------------------------------ #
    def connect(self) -> None:
        self._connected = True
        self._enabled = True
        self._last_t = now()

    def close(self) -> None:
        self._connected = False
        self._enabled = False

    def disable(self) -> None:
        self._enabled = False

    def park(self) -> None:
        # park 位 = 折叠零位（J2/J3 的零位即下限，自锁姿态）
        self._target = np.zeros(self.n)
        self._target[0] = 0.0
        self._grip_target = 0.0

    # ------------------------------------------------------------------ #
    def command(self, q_target: Sequence[float]) -> None:
        q = np.asarray(q_target, dtype=float).reshape(-1)
        if q.size < self.n + 1:
            raise ValueError(f"mock command 需要 {self.n + 1} 维向量，收到 {q.size}")
        clipped = np.clip(q[: self.n], self._limits[:, 0], self._limits[:, 1])
        self._limit_hits += int(np.sum(clipped != q[: self.n]))
        self._target = clipped
        self._grip_target = float(np.clip(q[self.n], 0.0, 1.0))

    # ------------------------------------------------------------------ #
    def grip_rad(self) -> float:
        """夹爪绝对角度（rad）——mock 用 GRIP_LO/HI 反归一化。"""
        lo, hi = math.radians(3.0), math.radians(328.0)
        return lo + float(self._grip) * (hi - lo)

    def send_mit(self, q6, kp, kd, tau, grip_rad: float | None = None) -> None:
        """原始 MIT 下发（teleop_core 路径）。"""
        q = np.asarray(q6, dtype=float).reshape(-1)[: self.n]
        clipped = np.clip(q, self._limits[:, 0], self._limits[:, 1])
        self._limit_hits += int(np.sum(clipped != q))
        self._target = clipped
        if grip_rad is not None:
            lo, hi = math.radians(3.0), math.radians(328.0)
            frac = (float(grip_rad) - lo) / max(1e-6, hi - lo)
            self._grip_target = float(np.clip(frac, 0.0, 1.0))

    # ------------------------------------------------------------------ #
    def read(self) -> JointState:
        t = now()
        dt = max(1e-4, min(0.2, t - self._last_t))
        self._last_t = t

        if self._enabled:
            # 一阶惯性趋近目标（比真实伺服慢一些，便于肉眼观察）
            alpha = 1.0 - np.exp(-dt / 0.12)
            dq = (self._target - self._q) * alpha
            self._q = np.clip(self._q + dq, self._limits[:, 0], self._limits[:, 1])
            self._grip += (self._grip_target - self._grip) * alpha

            # 速度与力矩（粗略模拟：力矩 ~ 跟踪误差，叠加摩擦）
            self._qd = dq / dt
            self._tau = (self._target - self._q) * 8.0 + 0.6
        else:
            self._qd = np.zeros(self.n)
            self._tau = np.zeros(self.n)

        self._q = np.clip(
            self._q + self._rng.normal(0.0, 2e-4, self.n),
            self._limits[:, 0],
            self._limits[:, 1],
        )
        self._samples += 1

        load = np.concatenate([np.abs(self._tau) / 10.0, [abs(self._grip_target - self._grip) * 2.0]])
        self._temps = self._temps + (42.0 + load * 6.0 - self._temps) * 0.001
        return JointState(t=t, pos=self._q.copy(), vel=self._qd.copy(), tau=self._tau.copy(), grip=float(self._grip))

    # ------------------------------------------------------------------ #
    def status(self) -> ArmStatus:
        temps = self._temps[: self.n].tolist()
        faults = {}
        if not self._connected:
            faults["connection"] = "MockArm 未连接"
        return ArmStatus(
            connected=self._connected,
            calibrated=True,
            motor_count=self.profile.n_motors,
            voltage=48.1 + float(self._rng.normal(0, 0.05)),
            temps_c=temps,
            faults=faults,
        )

    # ------------------------------------------------------------------ #
    def diagnostics(self) -> dict:
        return {
            "backend": self.name,
            "samples": self._samples,
            "limit_hits": self._limit_hits,
            "enabled": self._enabled,
        }
