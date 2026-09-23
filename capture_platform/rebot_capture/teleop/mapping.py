"""输入 → 机械臂目标的映射层。

- `MockPenMapper`：演示/自检用，把笔面位移映射成关节角增量（不依赖 IK）。
- `IKMapper`：真机用，接 reBotArm_control_py 的 solve_ik（下一步接入）。

约定：映射层输出长度为 7 的向量（6 关节 + 夹爪 0-1），单位 rad。
"""
from __future__ import annotations

from typing import Protocol

import numpy as np

from ..device.profile import DeviceProfile
from .samples import PenSample


class Mapper(Protocol):
    def map(self, pen: PenSample | None, base_q: np.ndarray) -> np.ndarray: ...


class MockPenMapper:
    """v0 演示映射：落笔离合 + 位移增量 → 关节角；笔压 → 夹爪。

    行为与 tablet_teleop 的"落笔跟手 / 抬笔悬停"一致：
    - 落笔瞬间记录锚点，之后只按位移增量驱动；
    - 抬笔保持最后目标（悬停）；
    - 输出始终裁剪到关节限位内。
    """

    def __init__(self, profile: DeviceProfile):
        self.profile = profile
        self._limits = np.asarray(profile.limits_array(), dtype=float)
        self._anchor: tuple[float, float] | None = None
        self._anchor_q: np.ndarray | None = None
        self._target = np.zeros(profile.n_joints)
        self._grip = 0.0

    def reset(self, q: np.ndarray) -> None:
        """把内部目标重置到指定关节角（warmup / 交接时用，避免跳变）。"""
        q = np.asarray(q, dtype=float).reshape(-1)[: self.profile.n_joints]
        self._target = np.clip(q.copy(), self._limits[:, 0], self._limits[:, 1])
        self._grip = 0.0
        self._anchor = None
        self._anchor_q = None

    def map(self, pen: PenSample | None, base_q: np.ndarray) -> np.ndarray:
        q = np.asarray(base_q, dtype=float).reshape(-1)[: self.profile.n_joints]

        if pen is not None and pen.touching:
            if self._anchor is None:
                self._anchor = (pen.x, pen.y)
                self._anchor_q = q.copy()
            dx = pen.x - self._anchor[0]
            dy = pen.y - self._anchor[1]
            t = self._anchor_q if self._anchor_q is not None else q
            self._target = np.array(
                [
                    t[0] + dx * 0.9,
                    t[1] + dy * 1.1,
                    t[2] - dy * 0.8,
                    t[3] + dy * 0.3,
                    t[4] + dx * 0.2,
                    t[5] + pen.twist * 0.01,
                ][: self.profile.n_joints],
                dtype=float,
            )
            self._grip = float(np.clip(pen.pressure, 0.0, 1.0))
        elif pen is None or not pen.touching:
            # 抬笔：保持目标（悬停），但清空锚点以便下次落笔重新取锚
            self._anchor = None
            self._anchor_q = None
            if self._target.size == 0:
                self._target = q.copy()

        self._target = np.clip(self._target, self._limits[:, 0], self._limits[:, 1])
        return np.concatenate([self._target, [self._grip]])


class IKMapper:
    """真机映射：笔面 → 末端位姿 → IK → 关节角。

    需要 reBotArm_control_py 的 kinematics.solve_ik + 重力前馈；
    在真机后端接入时实现（见《采集平台方案.md》§12）。
    """

    def __init__(self, *_, **__):
        raise NotImplementedError(
            "IKMapper 尚未接入 reBotArm_control_py —— 当前请使用 MockPenMapper（--backend mock）"
        )

    def map(self, pen: PenSample | None, base_q: np.ndarray) -> np.ndarray:  # pragma: no cover
        raise NotImplementedError
