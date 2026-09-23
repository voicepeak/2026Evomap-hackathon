"""真机后端：reBotArm_control_py（RobStride 电机 + Pinocchio 重力前馈）。

行为对齐上游 `spatial_teleop.py` 的已验证做法：
- 启动安全门：仅当接近折叠零位（默认 ≤0.05 rad）才允许使能，且拒绝时**不失能**
- MIT 模式 + 重力前馈 tau，使能瞬间不跳
- 关节软限位余量（默认 1.5°）
- 夹爪：kp=20 / kd=2 / 限速 1.2 rad/s / 力矩保护 1 N·m（超限回退目标）
- 归零：0.25 rad/s 限速斜坡（等价 tools/go_zero.py）
- 失能：仅在 park 位附近允许（唯一推荐断电方式）
"""
from __future__ import annotations

import math
import time
from typing import Sequence

import numpy as np

from ..integrations.rebotarm import RebotArmRepo
from .backends import ArmStatus, BackendError, JointState, now
from .profile import DeviceProfile

# ── 上游已验证常量（spatial_teleop.py） ──────────────────────────────
GRIP_KP = 20.0
GRIP_KD = 2.0
GRIP_RATE = 1.20                    # rad/s（约 69°/s）
GRIP_LO = math.radians(3.0)         # 程序可用下界
GRIP_HI = math.radians(328.0)       # 程序可用上界
GRIP_TAU_LIMIT = 1.0                # N·m
PARK_RATE = 0.25                    # rad/s
START_GATE_RAD = 0.05
DISABLE_GATE_RAD = 0.05


class RealArm:
    name = "rebot"

    def __init__(
        self,
        profile: DeviceProfile,
        repo: RebotArmRepo | str | None = None,
        limit_margin_deg: float = 1.5,
        allow_nonzero_start: bool = False,
        fps: int = 100,
    ):
        self.profile = profile
        self.repo = repo if isinstance(repo, RebotArmRepo) else RebotArmRepo(repo)
        self.limit_margin_deg = limit_margin_deg
        self.allow_nonzero_start = allow_nonzero_start
        self.fps = fps

        self._arm = None
        self._model = None
        self._data = None
        self._fid = None
        self._lo = np.zeros(6)
        self._hi = np.zeros(6)
        self._kp = np.zeros(6)
        self._kd = np.zeros(6)
        self._q_cmd = np.zeros(6)
        self._grip = None
        self._connected = False

    # ================================================================== #
    def _wait_feedback(self, arm, timeout: float = 3.0) -> int:
        """等待所有电机上报一次反馈，返回就绪数量。

        注意：刚 connect 时缓存可能为空或陈旧，**必须先拿到完整反馈**再判断姿态，
        否则"首帧 0 值"会误通过安全门。
        """
        n_total = len(arm._motor_map)
        t0 = time.monotonic()
        good = 0
        while time.monotonic() - t0 < timeout:
            for m in arm._motor_map.values():
                try:
                    m.request_feedback()
                except Exception:  # noqa: BLE001
                    pass
            for c in arm._ctrl_map.values():
                try:
                    c.poll_feedback_once()
                except Exception:  # noqa: BLE001
                    pass
            good = sum(1 for m in arm._motor_map.values() if m.get_state() is not None)
            if good >= n_total:
                break
            time.sleep(0.05)
        return good

    def connect(self) -> None:
        if self._connected:
            return
        repo = self.repo.load()

        arm = repo.RebotArm()
        arm.connect()

        # 1) 总线健康：必须能收到全部电机反馈
        n_total = len(arm._motor_map)
        good = self._wait_feedback(arm)
        if n_total == 0 or good < n_total:
            raise BackendError(
                f"CAN 反馈不完整：{good}/{n_total} 个电机有数据；"
                "请检查适配器（gs_usb/PCAN）/ 波特率 1Mbps / 终端电阻 / 接线。"
            )

        # 2) 启动安全门（用刚拿到的完整反馈）
        g = arm.arm
        q0 = np.asarray(g.get_positions(), float)[:6]
        if not self.allow_nonzero_start and float(np.max(np.abs(q0))) > START_GATE_RAD:
            # 拒绝使能：不动、不失能（与上游一致）
            raise BackendError(
                f"启动姿态距折叠零位 {float(np.max(np.abs(q0))):.3f} rad > {START_GATE_RAD} rad；"
                "拒绝使能（机械臂维持原状态）。请先用工具归零，或加 --allow-nonzero-start。"
            )

        model = repo.load_robot_model()
        data = model.createData()
        fid = repo.get_end_effector_frame_id(model)
        lo, hi = repo.joint_limits(model, 6)
        lm = math.radians(self.limit_margin_deg)
        lo = lo + lm
        hi = hi - lm

        kp, kd = g._mit_kp.copy(), g._mit_kd.copy()
        g.mode_mit(kp=kp, kd=kd)
        g.enable()
        tau = repo.compute_gravity(model, repo.pad_q_for_model(model, q0, 6), data)[:6]
        g.send_mit(q0, vel=np.zeros(6), kp=kp, kd=kd, tau=tau)

        grip = None
        if arm.has_gripper:
            try:
                gg = arm.gripper
                gg.mode_mit()
                gg.enable()
                gpos = float(np.asarray(gg.get_positions(), float).reshape(-1)[0])
                grip = {
                    "group": gg,
                    "send": gpos,
                    "held": gpos,
                    "last": time.monotonic(),
                    "tau": 0.0,
                }
            except Exception:  # noqa: BLE001
                grip = None

        self._arm = arm
        self._model = model
        self._data = data
        self._fid = fid
        self._lo, self._hi = lo, hi
        self._kp, self._kd = kp, kd
        self._q_cmd = q0.copy()
        self._grip = grip
        self._connected = True

    # ================================================================== #
    def read(self) -> JointState:
        if not self._connected:
            raise BackendError("RealArm 未连接")
        pos, vel, torq = self._arm.get_state()
        pos = np.asarray(pos, float)
        grip_rad = float(pos[6]) if pos.size > 6 else 0.0
        grip_norm = (grip_rad - GRIP_LO) / max(1e-6, GRIP_HI - GRIP_LO)
        if self._grip is not None:
            self._grip["held"] = grip_rad
        return JointState(
            t=now(),
            pos=pos[:6].copy(),
            vel=np.asarray(vel, float)[:6].copy(),
            tau=np.asarray(torq, float)[:6].copy(),
            grip=float(np.clip(grip_norm, 0.0, 1.0)),
        )

    # ================================================================== #
    def command(self, q_target: Sequence[float]) -> None:
        if not self._connected:
            raise BackendError("RealArm 未连接")
        q = np.asarray(q_target, float).reshape(-1)
        qa = np.clip(q[:6], self._lo, self._hi)
        tau = self.repo.compute_gravity(
            self._model, self.repo.pad_q_for_model(self._model, qa, 6), self._data
        )[:6]
        self._arm.arm.send_mit(qa, vel=np.zeros(6), kp=self._kp, kd=self._kd, tau=tau)
        self._q_cmd = qa
        if q.size > 6:
            self._command_gripper(float(q[6]))

    # ------------------------------------------------------------------ #
    def _command_gripper(self, frac: float) -> None:
        g = self._grip
        if g is None:
            return
        t = time.monotonic()
        dt = float(np.clip(t - g["last"], 1e-3, 0.05))
        g["last"] = t

        frac = float(np.clip(frac, 0.0, 1.0))
        target = GRIP_LO + frac * (GRIP_HI - GRIP_LO)
        step = GRIP_RATE * 1.5 * dt
        g["send"] = float(np.clip(g["send"] + float(np.clip(target - g["send"], -step, step)), GRIP_LO, GRIP_HI))

        g["group"].send_mit(
            np.array([g["send"]]),
            vel=np.zeros(1),
            kp=np.array([GRIP_KP]),
            kd=np.array([GRIP_KD]),
            tau=np.zeros(1),
        )

        # 力矩保护：超过阈值 → 目标回退到实测位置（不再顶住）
        try:
            tau_all = np.asarray(self._arm.get_state(request_feedback=False)[2], float)
            g["tau"] = float(tau_all[-1]) if tau_all.size else 0.0
        except Exception:  # noqa: BLE001
            g["tau"] = 0.0
        if abs(g["tau"]) > GRIP_TAU_LIMIT:
            g["send"] = float(g["held"])

    # ------------------------------------------------------------------ #
    def grip_rad(self) -> float:
        """夹爪绝对角度（rad，上游 teleop_core 的坐标）。"""
        if self._grip is None:
            return 0.0
        return float(self._grip["held"])

    def send_mit(self, q6, kp, kd, tau, grip_rad: float | None = None) -> None:
        """原始 MIT 下发（teleop_core 路径）：core 已做限速/积分，这里只做软限位与发送。"""
        if not self._connected:
            raise BackendError("RealArm 未连接")
        qa = np.clip(np.asarray(q6, float)[:6], self._lo, self._hi)
        self._arm.arm.send_mit(
            qa,
            vel=np.zeros(6),
            kp=np.asarray(kp, float),
            kd=np.asarray(kd, float),
            tau=np.asarray(tau, float),
        )
        self._q_cmd = qa
        if grip_rad is None or self._grip is None:
            return
        g = self._grip
        g["send"] = float(np.clip(float(grip_rad), GRIP_LO, GRIP_HI))
        g["group"].send_mit(
            np.array([g["send"]]),
            vel=np.zeros(1),
            kp=np.array([GRIP_KP]),
            kd=np.array([GRIP_KD]),
            tau=np.zeros(1),
        )
        try:
            tau_all = np.asarray(self._arm.get_state(request_feedback=False)[2], float)
            g["tau"] = float(tau_all[-1]) if tau_all.size else 0.0
        except Exception:  # noqa: BLE001
            g["tau"] = 0.0
        if abs(g["tau"]) > GRIP_TAU_LIMIT:
            g["send"] = float(g["held"])  # 力矩保护：目标回退到实测（与上游一致）

    # ================================================================== #
    def park(self, timeout: float = 25.0) -> None:
        """0.25 rad/s 限速斜坡回折叠零位（保持悬停，不失能）。"""
        if not self._connected:
            raise BackendError("RealArm 未连接")
        q = np.asarray(self._arm.arm.get_positions(), float)[:6].copy()
        dt = 1.0 / max(1, self.fps)
        t0 = time.monotonic()
        while True:
            step = PARK_RATE * dt
            q = q + np.clip(-q, -step, step)
            self.command(np.concatenate([q, [0.0]]))  # 归零顺带闭合夹爪
            time.sleep(dt)
            if float(np.max(np.abs(q))) < 1e-4 or time.monotonic() - t0 > timeout:
                break
        # 收尾：保持 0.5s
        for _ in range(int(0.5 * self.fps)):
            self.command(np.zeros(7))
            time.sleep(dt)

    # ================================================================== #
    def disable(self) -> None:
        """失能（唯一推荐断电方式）：仅允许在 park 位附近。"""
        q = self.read().pos
        dev = float(np.max(np.abs(q)))
        if dev > DISABLE_GATE_RAD:
            raise BackendError(
                f"非 park 位（最大偏差 {dev:.3f} rad > {DISABLE_GATE_RAD} rad）拒绝失能；请先 park()。"
            )
        self._arm.arm.disable()
        if self._grip is not None:
            try:
                self._grip["group"].disable()
            except Exception:  # noqa: BLE001
                pass

    # ================================================================== #
    def status(self) -> ArmStatus:
        return ArmStatus(
            connected=self._connected,
            calibrated=True,
            motor_count=self.profile.n_motors if self._connected else 0,
            voltage=0.0,  # 上游未暴露母线电压
            temps_c=[],
            faults={},
            error=None,
        )

    def diagnostics(self) -> dict:
        d = {"backend": self.name, "repo": str(self.repo.path), "connected": self._connected}
        if self._connected:
            d["q_cmd"] = np.round(self._q_cmd, 3).tolist()
            d["limits"] = [np.round(self._lo, 3).tolist(), np.round(self._hi, 3).tolist()]
            if self._grip is not None:
                d["gripper"] = {
                    "send_deg": round(math.degrees(self._grip["send"]), 1),
                    "held_deg": round(math.degrees(self._grip["held"]), 1),
                    "tau": round(self._grip["tau"], 3),
                }
        return d

    # ================================================================== #
    def close(self, force: bool = False) -> None:
        """与上游一致：退出时**不失能**（保持悬停，之后由通信看门狗接管）。

        失能只走 `disable()`（等价上游 `disable_arm.py`，需在 park 位附近）。
        """
        self._connected = False
