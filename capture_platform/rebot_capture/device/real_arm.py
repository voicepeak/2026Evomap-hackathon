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

# ── 上游已验证常量（spatial_teleop.py）+ 平台夹爪力控 ──────────────
GRIP_KP = 20.0
GRIP_KD = 2.0
GRIP_RATE = 1.20                    # rad/s（约 69°/s）
GRIP_LO = math.radians(3.0)         # 程序可用下界（可被机型配置 gripper.lo_deg 覆盖）
GRIP_HI = math.radians(328.0)       # 程序可用上界（可被机型配置 gripper.hi_deg 覆盖）
GRIP_TAU_LIMIT = 2.0                # N·m：与 teleop_core.CoreArgs.grip_tau_limit 对齐
GRIP_ERR_MAX = GRIP_TAU_LIMIT / GRIP_KP   # 0.1 rad：偏差 × kp = 力矩 → 夹住偏差就限住了力矩
GRIP_RECOVER_ERR = math.radians(15.0)   # 命令与实测偏差超过它 + 无力矩 → 认为电机掉了使能/锁存故障
GRIP_RECOVER_WAIT = 2.0                 # 持续这么久才自愈
GRIP_RECOVER_COOLDOWN = 5.0             # 自愈重试间隔                # N·m
PARK_RATE = 0.25                    # rad/s
START_GATE_RAD = 0.05
DISABLE_GATE_RAD = 0.05


def grip_force_limited(desired: float, held: float, lo: float, hi: float,
                       err_max: float = GRIP_ERR_MAX) -> float:
    """夹爪实际下发值：只允许比实测位置超前/落后 err_max。

    位置环里 `tau ≈ kp·(目标−实测)`，所以把偏差夹住 = 把力矩夹住：
    到限位或夹住东西时，目标不会越积越远，电机不会顶着几十 N·m 堵转。
    （RobStride 堵转后可能锁存故障：位置不跟随、iq=0，整个夹爪"死掉"。）
    """
    err = float(np.clip(float(desired) - float(held), -abs(float(err_max)), abs(float(err_max))))
    return float(np.clip(float(held) + err, float(lo), float(hi)))


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
        # 夹爪可用行程/力矩/速度（机型配置可改；与 teleop_core.CoreArgs.grip_* 保持一致）
        gp = profile.gripper
        self._g_lo = math.radians(float(getattr(gp, "lo_deg", 3.0)))
        self._g_hi = math.radians(float(getattr(gp, "hi_deg", 328.0)))
        self._g_tau_limit = float(getattr(gp, "tau_limit", GRIP_TAU_LIMIT))
        self._g_err_max = self._g_tau_limit / max(GRIP_KP, 1e-6)
        self._g_rate = float(getattr(gp, "rate", GRIP_RATE))

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
                    "sent": gpos,
                    "held": gpos,
                    "last": time.monotonic(),
                    "tau": 0.0,
                }
                # 上次若是异常退出（堵转/失能），先把故障锁存清掉，保证一上来就是活的
                try:
                    self._grip_recover(grip)      # clear_error + mode_mit + enable（幂等）
                except Exception:  # noqa: BLE001
                    pass
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
        grip_norm = (grip_rad - self._g_lo) / max(1e-6, self._g_hi - self._g_lo)
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
        target = self._g_lo + frac * (self._g_hi - self._g_lo)
        step = self._g_rate * dt
        g["send"] = float(np.clip(g["send"] + float(np.clip(target - g["send"], -step, step)),
                                  self._g_lo, self._g_hi))
        self._send_gripper(g, g["send"])

    # ------------------------------------------------------------------ #
    def _send_gripper(self, g: dict, desired: float) -> None:
        """按"限偏差"下发夹爪（力矩有上限），并读力矩、做自愈。"""
        sent = grip_force_limited(desired, g["held"], self._g_lo, self._g_hi, self._g_err_max)
        g["sent"] = sent
        g["group"].send_mit(
            np.array([sent]),
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
        # 自愈：命令与实测长期偏差大、但几乎没有力矩 → 电机多半掉了使能或者被锁存故障
        # （RobStride 堵转/写参数后可能锁存：位置不跟随、iq=0）。先 clear_error 再使能。
        # 力矩大时不触发，避免"顶住东西还反复使能硬顶"。
        err = abs(float(desired) - float(g["held"]))
        t_g = time.monotonic()
        if err > GRIP_RECOVER_ERR and abs(g["tau"]) < 0.2:
            g["stuck_since"] = g.get("stuck_since") or t_g
            if t_g - g["stuck_since"] > GRIP_RECOVER_WAIT and t_g - g.get("recover_at", 0.0) > GRIP_RECOVER_COOLDOWN:
                try:
                    self._grip_recover(g)
                    g["recover_at"] = t_g
                    g["stuck_since"] = None
                    print(f"[grip] 夹爪未跟随（偏差 {math.degrees(err):.0f}°，力矩≈0）→ "
                          f"clear_error + 重新使能", flush=True)
                except Exception:  # noqa: BLE001
                    pass
        else:
            g["stuck_since"] = None

    def _grip_recover(self, g: dict) -> None:
        """清故障锁存 + 回到 MIT + 使能（夹爪这一路，不动机械臂）。"""
        group = g.get("group")
        if group is None:
            return
        motors = getattr(group, "_mm", {}) or {}
        for name in list(getattr(group, "_jn", []) or []):
            mot = motors.get(name)
            if mot is not None:
                try:
                    mot.clear_error()
                except Exception:  # noqa: BLE001
                    pass
        group.mode_mit()
        group.enable()

    # ------------------------------------------------------------------ #
    def grip_target(self) -> float:
        """返回最近下发的归一化夹爪目标，供动作录制使用。"""
        if self._grip is None:
            return 0.0
        return float(np.clip((self._grip["send"] - self._g_lo) / (self._g_hi - self._g_lo), 0.0, 1.0))

    def grip_rad(self) -> float:
        """夹爪绝对角度（rad，上游 teleop_core 的坐标）。"""
        if self._grip is None:
            return 0.0
        return float(self._grip["held"])

    def grip_tau(self) -> float:
        """夹爪电机力矩（N·m）——给 teleop_core 的夹爪力控用。"""
        if self._grip is None:
            return 0.0
        return float(self._grip.get("tau", 0.0))

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
        g["send"] = float(np.clip(float(grip_rad), self._g_lo, self._g_hi))   # 期望目标（记录/自愈用）
        self._send_gripper(g, g["send"])

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
