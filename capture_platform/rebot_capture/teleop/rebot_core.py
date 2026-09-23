"""平台侧适配器：把 reBotArm_control_py/teleop_core.py 接进采集端。

- 复用上游 `TeleopCore`（与 spatial_teleop.py 同逻辑：速度档/冻结/姿态模式/漂浮/
  关节直控/预设/对齐/安全盒/限位/夹爪直控）
- 平台只做：浏览器输入（笔坐标像素）、录制、质检、打包
- 笔坐标：浏览器 pad 归一化 0-1 → 虚拟窗口 960×620 像素（与上游窗口一致）
"""
from __future__ import annotations

import time
from typing import Any

import numpy as np

from ..device.profile import DeviceProfile
from ..integrations.rebotarm import RebotArmRepo
from .samples import PenSample

# 与上游窗口一致，保证 TRAVEL_PX=200 的手感不变
VIRTUAL_W = 960.0
VIRTUAL_H = 620.0


class RebotCoreMapper:
    """持有 TeleopCore；输入来自浏览器，输出是 CoreCommand（原样交给后端发送）。"""

    is_core = True

    def __init__(self, profile: DeviceProfile, repo: RebotArmRepo | str | None = None):
        self.profile = profile
        self.repo = repo if isinstance(repo, RebotArmRepo) else RebotArmRepo(repo)
        TeleopCore, CoreArgs, CoreCommand = self.repo.load_teleop_core()

        self.model = self.repo.load_robot_model()
        self.data = self.model.createData()
        self.fid = self.repo.get_end_effector_frame_id(self.model)
        self.core = TeleopCore(self.model, self.data, self.fid, profile.kp_array(), profile.kd_array(), CoreArgs())

        self._last_t = time.monotonic()
        self._last_cmd = None

    # ------------------------------------------------------------------ #
    # 生命周期
    # ------------------------------------------------------------------ #
    def prime(self, q: np.ndarray, grip_rad: float | None = None) -> None:
        self.core.prime(np.asarray(q, dtype=float)[:6], grip_pos=grip_rad)

    def reset(self, q: np.ndarray) -> None:
        """平台 goto/park 结束后同步（等价上游 q_cmd = S.q）。"""
        self.prime(np.asarray(q, dtype=float)[:6])

    # ------------------------------------------------------------------ #
    # 主步进：输入笔样本 → CoreCommand
    # ------------------------------------------------------------------ #
    def step(self, pen: PenSample | None, q_meas: np.ndarray, tau_meas: np.ndarray | None = None,
             grip_rad: float | None = None, vel_meas: np.ndarray | None = None) -> Any:
        now = time.monotonic()
        dt = now - self._last_t
        self._last_t = now

        twist = float(getattr(self, "_twist_cmd", 0.0))
        if pen is not None:
            # 归一化 0-1 → 上游窗口像素
            if twist == 0.0:
                twist = float(getattr(pen, "twist", 0.0))
            self.core.set_pen(pen.x * VIRTUAL_W, pen.y * VIRTUAL_H, bool(pen.touching),
                              shift=bool(getattr(pen, "shift", False)),
                              twist=twist)
        else:
            self.core.set_pen(0.0, 0.0, False, shift=False, twist=twist)

        cmd = self.core.step(dt, np.asarray(q_meas, dtype=float)[:6], tau_meas, vel_meas=vel_meas)
        self.core.update_feedback(np.asarray(q_meas, dtype=float)[:6], tau_meas, grip_pos=grip_rad)
        self._last_cmd = cmd
        return cmd

    # ------------------------------------------------------------------ #
    # 控制接口（对应上游键盘/按钮）
    # ------------------------------------------------------------------ #
    def toggle_freeze(self) -> bool:
        return self.core.toggle_freeze()

    def set_freeze(self, on: bool) -> bool:
        self.core.freeze = bool(on)
        return self.core.freeze

    def cycle_speed(self) -> int:
        return self.core.cycle_speed()

    def set_mode(self, mode: str) -> None:
        self.core.set_mode(mode)

    def select_motor(self, index: int) -> None:
        """平台扩展：选中电机并进入直控（笔左键循环 / 点 J 按钮）。"""
        self.core.select_motor(int(index))

    def set_float(self, on: bool) -> None:
        self.core.set_float(on)

    def set_joint_hold(self, v: float) -> None:
        self.core.joint_hold(v)

    def select_joint(self, i: int) -> None:
        self.core.select_joint(i)

    def align(self) -> None:
        self.core.request_align()

    def goto_preset(self, i: int) -> bool:
        return self.core.goto_preset(i)

    def record_preset(self, i: int) -> dict:
        return self.core.record_preset(i)

    def set_tau_limit(self, v: float) -> None:
        self.core.set_tau_limit(v)

    def set_twist(self, v: float) -> None:
        """J6 自转速度指令（rad/s；0=松开）。"""
        self._twist_cmd = float(v)

    # ------------------------------------------------------------------ #
    def state(self) -> dict:
        c = self.core
        return {
            "mode": c.mode,
            "freeze": bool(c.freeze),
            "float": bool(c.float_mode),
            "speed_index": int(c.speed_i),
            "speed_scale": [0.5, 1.0, 2.0][int(c.speed_i)],
            "pen_pressed": bool(c.pressed),
            "joint_index": int(c.j_sel),
            "joint_req": float(c.j_req),
            "gripper_ready": bool(c.grip_ready),
            "gripper_send_deg": round(float(np.degrees(c.grip_send)), 1) if c.grip_ready else None,
            "gripper_pos_deg": round(float(np.degrees(c.grip_pos)), 1) if c.grip_ready else None,
            "gripper_tau": round(float(c.grip_tau), 3),
            "tau_limit": float(c.tau_limit),
            "posture_k": float(c.args.posture_k),
            "box": bool(c.args.box),
            "status": c.status,
            "alarm": c.alarm,
            "msg": c.msg,
            "rate_hz": round(float(c.rate_hz), 1),
            "damp": round(float(c.damp), 4),
            "stale_ori": bool(c.stale_ori),
            "presets": sorted(c.presets.keys()),
        }

    def diagnostics(self) -> dict:
        c = self.core
        return {
            **self.state(),
            "q_cmd_deg": [round(float(np.degrees(x)), 1) for x in (c.q_cmd if c.q_cmd is not None else np.zeros(6))],
            "v_cmd": [round(float(x), 4) for x in c.v_cmd],
            "qd_cmd": [round(float(x), 3) for x in c.qd_cmd],
            # 关节直控速度环（调参用）：参考 / 实测 / 修正量（rad/s）
            "joint_vel_ref": round(float(c._jvel_ref), 3),
            "joint_vel_meas": round(float(c._jvel_meas), 3),
            "joint_vel_corr": round(float(c._jvel_corr), 3),
        }
