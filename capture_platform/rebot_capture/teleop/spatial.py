"""速度级空间遥操映射（对齐 spatial_teleop.py 的"位置模式"）。

数据流：
    笔位移 → 末端速度 v_cmd → 雅可比阻尼最小二乘 → 关节速度 qd
          → 零空间（限位排斥/构型正则/J6 自转）→ 限幅积分 → 关节目标
          → 关节软限位硬夹；夹爪由笔压映射（限速 + 目标回退）

安全：
    - 笛卡尔安全盒（前瞻 0.25s）
    - 关节软限位余量（默认 1.5°）
    - 关节速度硬上限 1.0 rad/s

笔坐标：自动识别归一化（0-1，用 travel=0.25）或像素（用 travel=200px）。
"""
from __future__ import annotations

import math
import time

import numpy as np

from ..device.profile import DeviceProfile
from ..integrations.rebotarm import RebotArmRepo
from .samples import PenSample

# ── 上游已验证常量（spatial_teleop.py） ──────────────────────────────
VMAX = 0.08                       # 满速 m/s
QD_MAX = 1.0                      # 关节速度硬上限 rad/s
LIMIT_SOFT = 0.12                 # 零空间限位排斥范围 rad
LIMIT_K = 0.35                    # 排斥强度 rad/s
DAMPING = 0.02                    # 阻尼最小二乘 λ0
ORI_K = 1.5                       # 姿态保持增益 1/s
ORI_VMAX = 0.4                    # 姿态修正速度上限 rad/s
EXPO = 1.4                        # 手感曲线
TRAVEL_PX = 200.0                 # 像素输入：满速位移
DEAD_PX = 6.0                     # 像素输入：死区
TRAVEL_NORM = 0.25                # 归一化输入：满速位移
DEAD_NORM = 0.008                 # 归一化输入：死区
PEN_TIMEOUT = 0.4

# 笛卡尔安全盒（沿用上游已验证的工作空间）
BX = (0.14, 0.46)
BY = (-0.30, 0.30)
BZ = (0.16, 0.52)
RMAX = 0.48

# 夹爪（第七电机）归一化目标：0=闭合(lo) 1=张开(hi)
GRIP_INVERT_PRESSURE = True       # True：笔压越大越闭合


class SpatialVelocityMapper:
    def __init__(
        self,
        profile: DeviceProfile,
        repo: RebotArmRepo | str | None = None,
        limit_margin_deg: float = 1.5,
        vmax: float = VMAX,
        box: bool = True,
        box_scale: float = 1.0,
    ):
        self.profile = profile
        self.repo = repo if isinstance(repo, RebotArmRepo) else RebotArmRepo(repo)
        repo = self.repo.load()

        self.model = repo.load_robot_model()
        self.data = self.model.createData()
        self.pin = repo.pin
        self.fid = repo.get_end_effector_frame_id(self.model)
        lo, hi = repo.joint_limits(self.model, 6)
        lm = math.radians(limit_margin_deg)
        self._lo = lo + lm
        self._hi = hi - lm

        self.vmax = vmax
        self.box = box
        self.box_scale = box_scale
        self.speed_scale = 1.0
        self.ori_weight = 0.0          # >0 时启用姿态软保持

        self._q_cmd: np.ndarray | None = None
        self._seed: np.ndarray | None = None
        self._rpy_ref: np.ndarray | None = None
        self._anchor: tuple[float, float] | None = None
        self._t = time.monotonic()
        self._grip_frac = 0.0
        self._last_pressure = 0.0
        self._last_v = np.zeros(3)
        self._last_qd = np.zeros(6)

    # ------------------------------------------------------------------ #
    def reset(self, q: np.ndarray) -> None:
        q = np.asarray(q, dtype=float).reshape(-1)[:6]
        self._q_cmd = np.clip(q.copy(), self._lo, self._hi)
        self._seed = self._q_cmd.copy()
        self._rpy_ref = np.asarray(self.repo.joint_to_pose(self._q_cmd)[1], float).copy()
        self._anchor = None

    # ------------------------------------------------------------------ #
    def _vel_from_disp(self, d: float, vmax: float) -> float:
        """位移 → 速度（死区 + 幂曲线），自动识别归一化/像素。"""
        a = abs(d)
        if a <= self._dead:
            return 0.0
        f = min((a - self._dead) / max(self._travel - self._dead, 1e-6), 1.0)
        return math.copysign((f ** EXPO) * vmax, d)

    def _limit_repulse(self, q: np.ndarray) -> np.ndarray:
        z = np.zeros(6)
        for i in range(6):
            m = min(LIMIT_SOFT, 0.5 * (self._hi[i] - self._lo[i]))
            if q[i] < self._lo[i] + m:
                z[i] = LIMIT_K * (1.0 - (q[i] - self._lo[i]) / m)
            elif q[i] > self._hi[i] - m:
                z[i] = -LIMIT_K * (1.0 - (self._hi[i] - q[i]) / m)
        return np.clip(z, -LIMIT_K, LIMIT_K)

    # ------------------------------------------------------------------ #
    def map(self, pen: PenSample | None, base_q: np.ndarray) -> np.ndarray:
        now = time.monotonic()
        dt = float(np.clip(now - self._t, 1e-4, 0.05))
        self._t = now

        q_meas = np.clip(np.asarray(base_q, dtype=float).reshape(-1)[:6], self._lo, self._hi)
        if self._q_cmd is None:
            self.reset(q_meas)
        assert self._q_cmd is not None and self._seed is not None

        # 归一化 / 像素自动识别（取首帧笔坐标判断）
        if not hasattr(self, "_travel"):
            if pen is not None and max(abs(pen.x), abs(pen.y)) <= 1.5:
                self._travel, self._dead = TRAVEL_NORM, DEAD_NORM
            else:
                self._travel, self._dead = TRAVEL_PX, DEAD_PX

        touching = pen is not None and bool(pen.touching)
        vmax = self.vmax * self.speed_scale
        v_cmd = np.zeros(3)

        if touching and pen is not None:
            if self._anchor is None:
                self._anchor = (pen.x, pen.y)
                self._rpy_ref = np.asarray(self.repo.joint_to_pose(self._q_cmd)[1], float).copy()
                self._seed = self._q_cmd.copy()
            dx = pen.x - self._anchor[0]
            dy = pen.y - self._anchor[1]
            if getattr(pen, "shift", False):
                # 升降模式：上下 = Z，左右 = Y
                v_cmd = np.array([0.0, self._vel_from_disp(dx, vmax), self._vel_from_disp(-dy, vmax)])
            else:
                # 默认：上下 = X（前后），左右 = Y
                v_cmd = np.array([self._vel_from_disp(-dy, vmax), self._vel_from_disp(dx, vmax), 0.0])
        else:
            self._anchor = None

        # ── 安全盒（前瞻 0.25s） ──────────────────────────────────────
        p_cur, rpy_cur = self.repo.joint_to_pose(self._q_cmd)
        p_cur = np.asarray(p_cur, float)
        if self.box:
            look = 0.25
            p_next = p_cur + v_cmd * look
            for k, (b, s) in enumerate(((BX, self.box_scale), (BY, self.box_scale), (BZ, self.box_scale))):
                c = 0.5 * (b[0] + b[1])
                h = 0.5 * (b[1] - b[0]) * s
                if abs(p_next[k] - c) > h:
                    v_cmd[k] = 0.0
            r_next = float(np.hypot(p_next[0], p_next[1]))
            if r_next > RMAX * self.box_scale and (v_cmd[0] or v_cmd[1]):
                v_cmd[0] = 0.0
                v_cmd[1] = 0.0

        # ── 雅可比阻尼最小二乘 + 零空间 ───────────────────────────────
        pin = self.pin
        q_pad = self.repo.pad_q_for_model(self.model, self._q_cmd, 6)
        pin.computeJointJacobians(self.model, self.data, q_pad)
        J = np.asarray(
            pin.getFrameJacobian(self.model, self.data, self.fid, pin.LOCAL_WORLD_ALIGNED), float
        )[:, :6]
        Jv, Jo = J[:3, :], J[3:, :]

        try:
            smin = float(np.linalg.svd(Jv, compute_uv=False)[-1])
        except np.linalg.LinAlgError:
            smin = 1e-3
        lam = DAMPING * (1.0 + 4.0 * max(0.0, 0.06 - smin) / 0.06)

        B = Jv @ Jv.T + (lam ** 2) * np.eye(3)
        try:
            Jpinv = Jv.T @ np.linalg.inv(B)
        except np.linalg.LinAlgError:
            Jpinv = np.zeros((6, 3))
        qd = Jpinv @ v_cmd
        N = np.eye(6) - Jpinv @ Jv

        # 姿态软保持（可选，默认关闭）
        if self.ori_weight > 0 and self._rpy_ref is not None:
            rpy_now = np.asarray(self.repo.joint_to_pose(self._q_cmd)[1], float)
            Rr = pin.rpy.rpyToMatrix(float(self._rpy_ref[0]), float(self._rpy_ref[1]), float(self._rpy_ref[2]))
            Rc = pin.rpy.rpyToMatrix(float(rpy_now[0]), float(rpy_now[1]), float(rpy_now[2]))
            domega = self.ori_weight * ORI_K * np.asarray(pin.log3(Rr @ Rc.T), float)
            n_om = float(np.linalg.norm(domega))
            if n_om > ORI_VMAX:
                domega *= ORI_VMAX / n_om
            JoN = Jo @ N
            B2 = JoN @ JoN.T + (lam ** 2) * np.eye(3)
            try:
                JoN_pinv = JoN.T @ np.linalg.inv(B2)
                qd = qd + JoN_pinv @ (domega - Jo @ qd)
                N = N - JoN_pinv @ JoN
            except np.linalg.LinAlgError:
                pass

        # 零空间：限位排斥 + 构型正则 + J6 自转
        z = self._limit_repulse(self._q_cmd) - 0.05 * (self._q_cmd - self._seed)
        if pen is not None and touching and abs(getattr(pen, "twist", 0.0)) > 1e-6:
            z[5] += float(pen.twist) * math.radians(35.0)
        qd = qd + N @ z

        # ── 限幅 + 积分 + 软限位硬夹 ──────────────────────────────────
        qd = np.clip(qd, -QD_MAX, QD_MAX)
        self._q_cmd = np.clip(self._q_cmd + qd * dt, self._lo, self._hi)
        self._last_v, self._last_qd = v_cmd.copy(), qd.copy()

        # ── 夹爪：笔压 → 开合（限速） ─────────────────────────────────
        if touching and pen is not None:
            self._last_pressure = float(np.clip(pen.pressure, 0.0, 1.0))
        p = self._last_pressure
        target_frac = (1.0 - p) if GRIP_INVERT_PRESSURE else p
        rate = 2.0 * dt  # 归一化开合速度（约 1s 全行程）
        self._grip_frac = float(
            np.clip(self._grip_frac + float(np.clip(target_frac - self._grip_frac, -rate, rate)), 0.0, 1.0)
        )

        return np.concatenate([self._q_cmd, [self._grip_frac]])

    # ------------------------------------------------------------------ #
    def diagnostics(self) -> dict:
        return {
            "q_cmd": np.round(self._q_cmd, 3).tolist() if self._q_cmd is not None else None,
            "v_cmd": np.round(self._last_v, 4).tolist(),
            "qd": np.round(self._last_qd, 3).tolist(),
            "limits": [np.round(self._lo, 3).tolist(), np.round(self._hi, 3).tolist()],
            "travel_units": getattr(self, "_travel", None),
            "speed_scale": self.speed_scale,
            "box": self.box,
        }
