#!/usr/bin/env python3
"""reBot 空间遥操控制核心（无 GUI 版）。

**从 `spatial_teleop.py` 的 `control_loop()` 原样抽出**，不改控制律：
同一套常量、同一套顺序（冻结/对齐/回放/漂浮/直控 → 速度指令 → 安全盒 →
雅可比阻尼最小二乘 → 零空间 → 限幅积分 → 夹爪 → 力矩保护）。

`spatial_teleop.py` 保持不动；本文件供平台 / 无头脚本复用。

用法::

    core = TeleopCore(model, data, fid, kp, kd)
    core.pen_press(x_px, y_px); core.pen_move(x_px, y_px)
    cmd = core.step(dt, q_meas, tau_meas)
    arm.arm.send_mit(cmd.q, vel=np.zeros(6), kp=cmd.kp, kd=cmd.kd, tau=cmd.tau)
    if cmd.grip_send is not None:
        grip.send_mit(np.array([cmd.grip_send]), vel=np.zeros(1),
                      kp=np.array([cmd.grip_kp]), kd=np.array([cmd.grip_kd]), tau=np.zeros(1))
"""
from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import pinocchio as pin

from reBotArm_control_py.dynamics import compute_generalized_gravity
from reBotArm_control_py.kinematics import joint_to_pose, pad_q_for_model, pos_rot_to_se3

# ── 常量（与 spatial_teleop.py 一致） ────────────────────────────────────────
RATE = 200
PEN_TIMEOUT = 0.4
TRAVEL_PX = 200.0
DEAD_PX = 6.0
EXPO = 1.4
VMAX = 0.08
QD_MAX = 1.0
TWIST_RATE = math.radians(35.0)
LIMIT_MARGIN = math.radians(1.5)
# 平台调整（原上游 0.12 rad / 0.35 rad/s）：限位附近只轻微回推，
# 避免"顶不进去 / 过去了又被弹回来"的死区与反弹感。
LIMIT_SOFT = math.radians(3.0)
LIMIT_K = 0.12
ORI_K = 1.5
ORI_VMAX = 0.4
REPLAY_KP = 1.2
ORI_POS_K = 6.0
GRIP_KP = 20.0
GRIP_KD = 2.0
GRIP_RATE = 1.20
GRIP_RANGE = 3.00
GRIP_LO = math.radians(3.0)
GRIP_HI = math.radians(328.0)
GRIP_TAU_LIMIT = 1.0
DAMPING = 0.02
BX = (0.14, 0.46)
BY = (-0.30, 0.30)
BZ = (0.16, 0.52)
RMAX = 0.48
SPEED_PRESETS = [("慢", 0.5), ("中", 1.0), ("快", 2.0)]
PRESET_FILE = Path(__file__).resolve().parent / "config" / "poses.json"

# ── 平台扩展：电机直控（mode="joint"）的笔轴映射 ──────────────────────────────
# axis="y"：笔上下象限（俯仰类电机）；axis="x"：笔左右象限（回转/偏航类电机）。
# sign：笔向上（y）/向右（x）为正时，对应的关节正方向；个别方向反了改这里即可。
JOINT_PEN_MAP: dict[int, tuple[str, float]] = {
    0: ("x", +1.0),   # J1 肩部水平回转
    1: ("y", +1.0),   # J2 肩部俯仰
    2: ("y", +1.0),   # J3 肘部俯仰
    3: ("y", +1.0),   # J4 腕部俯仰（+ = 抬头）
    4: ("x", +1.0),   # J5 腕部偏航
    5: ("x", +1.0),   # J6 腕部自转
    # 6 = 夹爪：计划由摄像头检测控制，暂不由笔驱动
}
JOINT_PEN_RATE = math.radians(60.0)   # 笔满偏时的关节速度（与 Q/A 直控一致）
MOTOR_NAMES = ("J1 肩部水平", "J2 肩部俯仰", "J3 肘部俯仰",
               "J4 腕部俯仰", "J5 腕部偏航", "J6 腕部自转")


# ── 工具函数（原样） ────────────────────────────────────────────────────────
def vel_from_px(v_px: float, vmax: float, travel: float = TRAVEL_PX, dead: float = DEAD_PX) -> float:
    a = abs(v_px)
    if a <= dead:
        return 0.0
    f = min((a - dead) / max(travel - dead, 1.0), 1.0)
    return math.copysign((f ** EXPO) * vmax, v_px)


def limit_repulse(q: np.ndarray, lo: np.ndarray, hi: np.ndarray,
                  band: float = LIMIT_SOFT, k: float = LIMIT_K) -> np.ndarray:
    z = np.zeros(6)
    for i in range(6):
        m = min(float(band), 0.5 * (hi[i] - lo[i]))
        if m <= 0:
            continue
        if q[i] < lo[i] + m:
            z[i] = float(k) * (1.0 - (q[i] - lo[i]) / m)
        elif q[i] > hi[i] - m:
            z[i] = -float(k) * (1.0 - (hi[i] - q[i]) / m)
    return np.clip(z, -float(k), float(k))


def soft_box_velocity(v_cmd: np.ndarray, p_cur: np.ndarray, *,
                      box_scale: float = 1.0, band: float = 0.04,
                      bx=BX, by=BY, bz=BZ, rmax: float = RMAX) -> np.ndarray:
    """安全盒软边界：接近边界时按剩余空间成比例减速（可以慢慢贴边），越界方向直接禁止。

    与旧行为（一旦预测越界就把该方向速度置 0）相比：不再有"推不动"的死区，
    边界附近仍有安全约束（速度随剩余空间线性趋 0，不会越界）。
    """
    v = np.asarray(v_cmd, float).copy()
    p = np.asarray(p_cur, float)[:3]
    for axis, b in enumerate((bx, by, bz)):
        center = 0.5 * (b[0] + b[1])
        half = 0.5 * (b[1] - b[0]) * float(box_scale)
        lo_, hi_ = center - half, center + half
        if v[axis] > 0:
            room = hi_ - p[axis]
        elif v[axis] < 0:
            room = p[axis] - lo_
        else:
            continue
        if room <= 0:                     # 已在界外：只允许往界内走
            v[axis] = 0.0
        elif room < band * box_scale:
            v[axis] *= room / (band * box_scale)
    # 半径上限：只削掉"向外"的径向分量，切向保留（可以沿弧线滑动，不卡死）
    r = float(np.hypot(p[0], p[1]))
    if r > 1e-9:
        radial = (p[0] * v[0] + p[1] * v[1]) / r
        if radial > 0:
            room_r = rmax * float(box_scale) - r
            band_r = band * float(box_scale)
            scale = 0.0 if room_r <= 0 else (1.0 if room_r >= band_r else room_r / band_r)
            if scale < 1.0:
                drop = radial * (1.0 - scale)
                v[0] -= drop * p[0] / r
                v[1] -= drop * p[1] / r
    return v


def rotvec_err(rpy_ref: np.ndarray, rpy_cur: np.ndarray) -> np.ndarray:
    Rr = pos_rot_to_se3(np.zeros(3), roll=float(rpy_ref[0]), pitch=float(rpy_ref[1]),
                        yaw=float(rpy_ref[2])).rotation
    Rc = pos_rot_to_se3(np.zeros(3), roll=float(rpy_cur[0]), pitch=float(rpy_cur[1]),
                        yaw=float(rpy_cur[2])).rotation
    return np.asarray(pin.log3(Rr @ Rc.T), float)


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def smoothstep(a: float) -> float:
    a = min(max(a, 0.0), 1.0)
    return 10 * a**3 - 15 * a**4 + 6 * a**5


def load_presets(path: Path | str = PRESET_FILE) -> dict:
    try:
        p = Path(path)
        return json.loads(p.read_text()) if p.exists() else {}
    except Exception:  # noqa: BLE001
        return {}


def save_presets(d: dict, path: Path | str = PRESET_FILE) -> None:
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(d, ensure_ascii=False, indent=2))


# ── 参数与输出 ──────────────────────────────────────────────────────────────
@dataclass
class CoreArgs:
    vmax: float = VMAX
    pen_range: float = TRAVEL_PX
    dead: float = DEAD_PX
    damping: float = DAMPING
    ori_weight: float = 0.0
    posture_k: float = 0.2
    wmax_deg: float = 30.0
    j_rate: float = 60.0
    limit_margin: float = 1.5
    limit_soft: float = LIMIT_SOFT     # 限位排斥作用带（rad）
    limit_k: float = LIMIT_K           # 限位排斥最大速度（rad/s）
    box: bool = True
    box_scale: float = 1.0
    box_band: float = 0.04             # 安全盒软边界缓冲带（m）
    float_kd: float = 2.5
    tau_limit: float = 0.0
    gripper: bool = True
    grip_range: float = GRIP_RANGE
    grip_abs: bool = True
    grip_lo: float = 3.0
    grip_hi: float = 328.0
    grip_rate: float = GRIP_RATE
    u_max_clip: float = 0.0   # >0 时对空间速度做整体限幅（上游未用，保留位）


@dataclass
class CoreCommand:
    q: np.ndarray                      # 6 关节目标（rad）
    kp: np.ndarray
    kd: np.ndarray
    tau: np.ndarray                    # 重力前馈
    grip_send: float | None = None     # 夹爪目标（rad）；None=本帧不控制夹爪
    grip_kp: float = GRIP_KP
    grip_kd: float = GRIP_KD
    status: str = ""
    alarm: str = ""
    msg: str = ""
    speed_scale: float = 1.0
    damp: float = DAMPING
    stale_ori: bool = False
    float_mode: bool = False
    mode: str = "pos"
    freeze: bool = False
    joint_index: int = 3
    pen_pressed: bool = False


class TeleopCore:
    """无 GUI 的空间遥操核心（与 spatial_teleop.py 的 control_loop 同逻辑）。"""

    def __init__(
        self,
        model: pin.Model,
        data: pin.Data,
        fid: int,
        kp: np.ndarray,
        kd: np.ndarray,
        args: CoreArgs | None = None,
        presets: dict | None = None,
    ):
        self.model = model
        self.data = data
        self.fid = fid
        self.kp = np.asarray(kp, float).copy()
        self.kd = np.asarray(kd, float).copy()
        self.args = args or CoreArgs()
        self.presets = presets if presets is not None else load_presets()

        lm = math.radians(self.args.limit_margin)
        self.lo = np.asarray(model.lowerPositionLimit[:6], float) + lm
        self.hi = np.asarray(model.upperPositionLimit[:6], float) - lm

        # ── 输入状态（等价 S） ──
        self.pen = (0.0, 0.0)
        self.anchor = (0.0, 0.0)
        self.pressed = False
        self.pen_t = 0.0
        self.shift = False
        self.freeze = False
        self.speed_i = 1
        self.mode = "pos"
        self.float_mode = False
        self.j_req = 0.0
        self.j_sel = 3
        self.align_req = False
        self.replay: dict | None = None
        self.twist = 0.0
        self.selftest_cmd: dict | None = None
        self.tau_limit = float(self.args.tau_limit)

        # ── 内部（等价 S 的指令/参考） ──
        self.q_cmd: np.ndarray | None = None
        self.q_seed: np.ndarray | None = None
        self.p_ref = np.zeros(3)
        self.rpy_ref = np.zeros(3)
        self.rpy_ref_base1 = 0.0
        self.stale_ori = False
        self.v_cmd = np.zeros(3)
        self.qd_cmd = np.zeros(6)
        self.damp = float(self.args.damping)
        self.status = "初始化…"
        self.msg = ""
        self.alarm = ""
        self.rate_hz = 0.0

        # ── 夹爪（等价 S.grip_*） ──
        self.grip_ready = False
        self.grip_pos = 0.0
        self.grip_cmd = 0.0
        self.grip_send = 0.0
        self.grip_tau = 0.0
        self.grip_lim = (0.0, 0.0)
        self.grip_msg = ""
        self.grip_min = 0.0
        self.grip_max = 0.0
        self._grip_ext_target: float | None = None   # 外部（手势）行程目标 0=合 1=开

        self._twist_prev = 0.0
        self._was_float = False
        self._t_prev: float | None = None
        self._t0: float | None = None
        self._n = 0

    # ====================================================================== #
    # 初始化（使能后调用一次，等价 control_loop 的启动段）
    # ====================================================================== #
    def prime(self, q0: np.ndarray, grip_pos: float | None = None) -> None:
        q0 = np.asarray(q0, float)[:6].copy()
        p0, rpy0 = joint_to_pose(q0)
        self.q_cmd = q0.copy()
        self.q_seed = q0.copy()
        self.p_ref = np.asarray(p0, float).copy()
        self.rpy_ref = np.asarray(rpy0, float).copy()
        self.rpy_ref_base1 = float(self.rpy_ref[1])
        self.status = "就绪（笔=空间运动）"
        if grip_pos is not None:
            self.grip_ready = True
            self.grip_pos = float(grip_pos)
            self.grip_cmd = float(grip_pos)
            self.grip_send = float(grip_pos)
            if self.args.grip_abs:
                self.grip_lim = (math.radians(self.args.grip_lo), math.radians(self.args.grip_hi))
            else:
                r = float(self.args.grip_range)
                self.grip_lim = (self.grip_pos - r, self.grip_pos + r)
            self.grip_min = self.grip_max = float(grip_pos)

    # ====================================================================== #
    # 输入接口（等价 GUI 事件）
    # ====================================================================== #
    def pen_press(self, x: float, y: float) -> None:
        self.pen = (float(x), float(y))
        self.anchor = (float(x), float(y))
        self.pressed = True
        self.pen_t = time.perf_counter()

    def pen_move(self, x: float, y: float) -> None:
        self.pen = (float(x), float(y))
        if self.pressed:
            self.pen_t = time.perf_counter()

    def pen_release(self) -> None:
        self.pressed = False

    def set_pen(self, x: float, y: float, touching: bool, shift: bool = False, twist: float = 0.0) -> None:
        """平台用的合并入口：touching=True 且此前未按 → 等价 pen_press。"""
        if touching:
            if not self.pressed:
                self.pen_press(x, y)
            else:
                self.pen_move(x, y)
        else:
            if self.pressed:
                self.pen_release()
        self.shift = bool(shift)
        self.twist = float(twist)

    def toggle_freeze(self) -> bool:
        self.freeze = not self.freeze
        return self.freeze

    def cycle_speed(self) -> int:
        self.speed_i = (self.speed_i + 1) % len(SPEED_PRESETS)
        return self.speed_i

    def set_mode(self, mode: str) -> None:
        if mode not in ("pos", "ori", "joint"):
            raise ValueError("mode 只能是 'pos'、'ori' 或 'joint'")
        self.mode = mode
        # 与 spatial_teleop.py 的 set_mode()/按键 O 一致：切换模式时把位置保持点
        # 同步到当前指令位置，并把笔锚点重置到当前笔位。
        # 否则姿态模式的"末端位置保持"会把机械臂持续拉回切换前的旧参考点
        # （表现为某个关节一直转，例如 J4 不断上仰）；远处按住笔切换也会立即跳速。
        if self.q_cmd is not None:
            self.p_ref = np.asarray(joint_to_pose(self.q_cmd)[0], float).copy()
        self.anchor = self.pen
        if mode == "joint":
            self.msg = self.motor_msg()
        elif mode == "ori":
            self.msg = "姿态模式：笔上下=俯仰 左右=摆头"
        else:
            self.msg = "位置模式：笔上下=前后 左右=横移"

    def select_motor(self, i: int) -> None:
        """平台扩展：选中电机并进入直控模式（笔左键循环调用 / 点 J 按钮）。"""
        self.j_sel = int(np.clip(i, 0, 6))
        self.set_mode("joint")

    def set_gripper_target(self, frac: float) -> None:
        """平台扩展：外部（摄像头手势）设定夹爪行程目标，0 = 闭合、1 = 张开。

        与上次相同的请求直接忽略：到限位/有阻力被力矩保护退回后，不会反复顶；
        等行程下一次变化（换手势）再继续。真正下发仍走 step() 的限速与力矩保护。
        """
        if not self.grip_ready or not self.args.gripper:
            return
        frac = float(np.clip(frac, 0.0, 1.0))
        if self._grip_ext_target is not None and abs(frac - self._grip_ext_target) < 1e-6:
            return
        self._grip_ext_target = frac
        g_lo, g_hi = self.grip_lim
        self.grip_cmd = float(g_lo + frac * (g_hi - g_lo))

    def motor_msg(self) -> str:
        if self.j_sel >= 6 or self.j_sel not in JOINT_PEN_MAP:
            return "电机直控：夹爪（计划由摄像头检测，暂不动作）"
        axis = JOINT_PEN_MAP[self.j_sel][0]
        return f"电机直控：{MOTOR_NAMES[self.j_sel]}（笔{'上下' if axis == 'y' else '左右'}）"

    def set_float(self, on: bool) -> None:
        self.float_mode = bool(on)

    def joint_hold(self, v: float) -> None:
        """关节直控速度（rad/s，按住时非零；等价 S.j_req）。"""
        self.j_req = float(v)

    def select_joint(self, i: int) -> None:
        self.j_sel = int(np.clip(i, 0, 6))

    def request_align(self) -> None:
        # 与上游按键 R 一致：重设笔锚点，避免重新对齐后按旧位移继续推动机械臂。
        self.anchor = self.pen
        self.align_req = True

    def record_preset(self, i: int) -> dict:
        q = np.asarray(self.q_cmd, float).copy() if self.q_cmd is not None else np.zeros(6)
        p, rpy = joint_to_pose(q)
        self.presets[str(i)] = {
            "q": [float(x) for x in q],
            "p": [float(x) for x in p],
            "rpy": [float(x) for x in rpy],
        }
        save_presets(self.presets)
        return self.presets[str(i)]

    def goto_preset(self, i: int) -> bool:
        d = self.presets.get(str(i))
        if not d:
            self.msg = f"槽位{i} 还没记录"
            return False
        self.replay = {
            "p": np.array(d["p"], float),
            "rpy": np.array(d["rpy"], float),
            "t_end": time.perf_counter() + 20.0,
        }
        self.msg = f"→ 槽位{i}"
        return True

    def set_tau_limit(self, v: float) -> None:
        self.tau_limit = float(v)

    def set_joint_rate_deg(self, deg_s: float) -> None:
        self._j_rate = math.radians(deg_s)

    # ====================================================================== #
    # 主步进（等价 control_loop 一次迭代）
    # ====================================================================== #
    def step(self, dt_raw: float, q_meas: np.ndarray, tau_meas: np.ndarray | None = None) -> CoreCommand:
        now = time.perf_counter()
        if self._t0 is None:
            self._t0 = now
        dt = max(1e-4, min(float(dt_raw), 0.05))
        q_meas = np.asarray(q_meas, float)[:6]
        if self.q_cmd is None:
            self.prime(q_meas)

        freeze = self.freeze
        pressed = self.pressed
        pen = self.pen
        anchor = self.anchor
        twist = self.twist
        speed = SPEED_PRESETS[self.speed_i][1]
        mode = self.mode
        replay = self.replay
        align = self.align_req
        self.align_req = False

        # 手腕自转刚结束：把新姿势记为姿态参考
        if abs(self._twist_prev) > 1e-6 and abs(twist) < 1e-6 and not freeze:
            self.rpy_ref = np.asarray(joint_to_pose(self.q_cmd)[1], float).copy()
            self.rpy_ref_base1 = float(self.rpy_ref[1])
            self.msg = "手腕自转结束 → 参考姿态已跟随"
        self._twist_prev = twist

        # 刚退出漂浮：指令同步到实测，避免跳变
        if self._was_float and not self.float_mode:
            self.q_cmd = q_meas.copy()
            self.rpy_ref = np.asarray(joint_to_pose(q_meas)[1], float).copy()
            self.rpy_ref_base1 = float(self.rpy_ref[1])
            self.q_seed = self.q_cmd.copy()
            self.p_ref = np.asarray(joint_to_pose(q_meas)[0], float).copy()
            self.msg = "退出悬停模式（指令已同步到当前姿态）"

        # 重新对齐
        if align:
            self.rpy_ref = np.asarray(joint_to_pose(q_meas)[1], float).copy()
            self.rpy_ref_base1 = float(self.rpy_ref[1])
            self.q_seed = q_meas.copy()
            self.p_ref = np.asarray(joint_to_pose(q_meas)[0], float).copy()
            self.msg = "已重新对齐"

        # 笔超时
        if pressed and (now - self.pen_t) > PEN_TIMEOUT:
            self.pressed = False
            pressed = False

        # 0. 姿势回放
        ori_w = float(self.args.ori_weight)
        rpy_ref_use = self.rpy_ref
        if replay is not None:
            vmax = float(self.args.vmax) * speed
            p_des = np.asarray(replay["p"], float)
            p_now_c = np.asarray(joint_to_pose(self.q_cmd)[0], float)
            err_p = p_des - p_now_c
            v_cmd = np.clip(REPLAY_KP * err_p, -vmax, vmax)
            twist_cmd = 0.0
            rpy_ref_use = np.asarray(replay["rpy"], float)
            ori_w = max(ori_w, 0.8)
            if float(np.linalg.norm(err_p)) < 0.004 or now > float(replay["t_end"]) or pressed:
                self.replay = None
                self.msg = "回放取消（笔按下）" if pressed else "已到预设位姿 ✅"
                v_cmd = np.zeros(3)
        else:
            v_cmd = np.zeros(3)
            twist_cmd = 0.0

        # 当前位置/姿态
        p_cur0, rpy_cur0 = joint_to_pose(self.q_cmd)
        p_cur0 = np.asarray(p_cur0, float)
        rpy_cur0 = np.asarray(rpy_cur0, float)

        # ── 0a. 悬停（漂浮）：kp=0 + 重力补偿 ──
        if self.float_mode:
            q_fl = q_meas.copy()
            tau_fl = compute_generalized_gravity(self.model, pad_q_for_model(self.model, q_fl, 6), self.data)[:6]
            self.q_cmd = q_fl.copy()
            self.q_seed = q_fl.copy()
            p_fl, rpy_fl = joint_to_pose(q_fl)
            self.p_ref = np.asarray(p_fl, float).copy()
            self.rpy_ref = np.asarray(rpy_fl, float).copy()
            self._was_float = True
            return CoreCommand(
                q=q_fl,
                kp=np.zeros(6),
                kd=np.full(6, float(self.args.float_kd)),
                tau=tau_fl,
                grip_send=None,
                status="悬停（漂浮）中：可以用手推动，松手停在原地",
                speed_scale=speed, damp=self.damp, float_mode=True, mode=mode,
                freeze=freeze, joint_index=self.j_sel, pen_pressed=pressed,
                msg=self.msg,
            )
        self._was_float = False

        # ── 0a2. 电机直控（平台扩展）：笔按电机功能象限驱动所选关节 ──
        j_direct = 0.0
        if (mode == "joint" and pressed and not freeze
                and replay is None and self.selftest_cmd is None
                and int(self.j_sel) in JOINT_PEN_MAP):
            axis, sign = JOINT_PEN_MAP[int(self.j_sel)]
            dx = pen[0] - anchor[0]
            dy = pen[1] - anchor[1]
            disp = (-dy if axis == "y" else dx) * sign
            j_direct = vel_from_px(disp, JOINT_PEN_RATE, self.args.pen_range, self.args.dead)

        # ── 0b. 关节直控：参考跟随，避免笛卡尔任务拉扯 ──
        j_cmd = 0.0 if freeze else self.j_req
        if mode == "joint" and abs(j_direct) > 1e-9:
            j_cmd = j_direct                      # 笔控优先于 Q/A 保持
        j_sel = int(self.j_sel)
        if abs(j_cmd) > 1e-6:
            self.p_ref = p_cur0.copy()
            self.rpy_ref = rpy_cur0.copy()
            rpy_ref_use = self.rpy_ref
            self.q_seed = self.q_cmd.copy()

        # ── 1. 笔/自检 → 空间速度指令（位置模式）/ 角速度指令（姿态模式）──
        vmax = float(self.args.vmax) * speed
        w_cmd = np.zeros(3)
        if replay is not None:
            pass
        elif self.selftest_cmd is not None:
            v_cmd = np.array([self.selftest_cmd.get("vx", 0.0), self.selftest_cmd.get("vy", 0.0),
                              self.selftest_cmd.get("vz", 0.0)], float)
            twist_cmd = float(self.selftest_cmd.get("twist", 0.0))
            w_cmd = np.array([self.selftest_cmd.get("wx", 0.0), self.selftest_cmd.get("wy", 0.0),
                              self.selftest_cmd.get("wz", 0.0)], float)
        elif mode == "joint":
            # 电机直控：笔只驱动所选关节（已在 0a2 算出），不产生笛卡尔运动
            v_cmd = np.zeros(3)
            twist_cmd = 0.0
        elif pressed and not freeze and mode == "ori":
            dx = pen[0] - anchor[0]
            dy = pen[1] - anchor[1]
            wmax = math.radians(float(self.args.wmax_deg))
            w_cmd = np.array([vel_from_px(-dy, wmax, self.args.pen_range, self.args.dead) if self.shift else 0.0,
                              vel_from_px(-dy, wmax, self.args.pen_range, self.args.dead) if not self.shift else 0.0,
                              vel_from_px(dx, wmax, self.args.pen_range, self.args.dead)])
            v_cmd = np.zeros(3)
            twist_cmd = 0.0
        elif pressed and not freeze:
            dx = pen[0] - anchor[0]
            dy = pen[1] - anchor[1]
            if self.shift:
                v_cmd = np.array([0.0,
                                  vel_from_px(dx, vmax, self.args.pen_range, self.args.dead),
                                  vel_from_px(-dy, vmax, self.args.pen_range, self.args.dead)])
            else:
                v_cmd = np.array([vel_from_px(-dy, vmax, self.args.pen_range, self.args.dead),
                                  vel_from_px(dx, vmax, self.args.pen_range, self.args.dead),
                                  0.0])
            twist_cmd = twist
        else:
            v_cmd = np.zeros(3)
            twist_cmd = 0.0
        if freeze:
            v_cmd = np.zeros(3)
            w_cmd = np.zeros(3)
            twist_cmd = 0.0

        # ── 2. 安全盒（软边界：接近边界按剩余空间成比例减速，可慢慢贴边）──
        if self.args.box:
            v_cmd = soft_box_velocity(v_cmd, p_cur0, box_scale=self.args.box_scale,
                                      band=self.args.box_band)

        # ── 3. 雅可比 → 关节速度 ──
        q_pad = pad_q_for_model(self.model, self.q_cmd, 6)
        pin.computeJointJacobians(self.model, self.data, q_pad)
        J = np.asarray(pin.getFrameJacobian(self.model, self.data, self.fid,
                                            pin.LOCAL_WORLD_ALIGNED), float)[:, :6]
        Jv = J[:3, :]
        Jo = J[3:, :]
        try:
            smin = float(np.linalg.svd(Jv, compute_uv=False)[-1])
        except np.linalg.LinAlgError:
            smin = 1e-3
        lam = float(self.args.damping) * (1.0 + 4.0 * max(0.0, 0.06 - smin) / 0.06)
        self.damp = lam

        z = limit_repulse(self.q_cmd, self.lo, self.hi,
                          band=float(self.args.limit_soft), k=float(self.args.limit_k))
        if float(self.args.posture_k) > 0:
            z = z + (-float(self.args.posture_k)) * (self.q_cmd - self.q_seed)
        if abs(twist_cmd) > 1e-6:
            z[5] += twist_cmd

        if mode == "ori":
            A_o = Jo @ Jo.T + (lam ** 2) * np.eye(3)
            try:
                Jo_pinv = Jo.T @ np.linalg.inv(A_o)
            except np.linalg.LinAlgError:
                Jo_pinv = np.zeros((6, 3))
            qd = Jo_pinv @ w_cmd
            N1 = np.eye(6) - Jo_pinv @ Jo
            JvN = Jv @ N1
            B_p = JvN @ JvN.T + (lam ** 2) * np.eye(3)
            try:
                JvN_pinv = JvN.T @ np.linalg.inv(B_p)
            except np.linalg.LinAlgError:
                JvN_pinv = np.zeros((6, 3))
            v_hold = np.clip(-ORI_POS_K * (p_cur0 - self.p_ref), -0.08, 0.08)
            qd = qd + JvN_pinv @ (v_hold - Jv @ qd)
            N2 = N1 - JvN_pinv @ JvN
            qd = qd + N2 @ z
        else:
            A = Jv @ Jv.T + (lam ** 2) * np.eye(3)
            try:
                Jpinv = Jv.T @ np.linalg.inv(A)
            except np.linalg.LinAlgError:
                Jpinv = np.zeros((6, 3))
            qd = Jpinv @ v_cmd
            N = np.eye(6) - Jpinv @ Jv
            if ori_w > 0 and abs(twist_cmd) < 1e-6:
                rpy_cur = np.asarray(joint_to_pose(self.q_cmd)[1], float)
                domega = ori_w * ORI_K * rotvec_err(rpy_ref_use, rpy_cur)
                n_om = float(np.linalg.norm(domega))
                if n_om > ORI_VMAX:
                    domega *= ORI_VMAX / n_om
                JoN = Jo @ N
                B = JoN @ JoN.T + (lam ** 2) * np.eye(3)
                try:
                    JoN_pinv = JoN.T @ np.linalg.inv(B)
                    qd = qd + JoN_pinv @ (domega - Jo @ qd)
                    N = N - JoN_pinv @ JoN
                except np.linalg.LinAlgError:
                    pass
            qd = qd + N @ z

        self.stale_ori = bool(np.linalg.norm(
            [wrap_pi(self.rpy_ref[i] - joint_to_pose(self.q_cmd)[1][i]) for i in range(3)]) > 0.5)

        if abs(j_cmd) > 1e-6 and j_sel < 6:
            qd[j_sel] += j_cmd

        # ── 4. 关节速度限幅 + 积分 + 软限位硬夹 ──
        qd = np.clip(qd, -QD_MAX, QD_MAX)
        q_new = np.clip(self.q_cmd + qd * dt, self.lo, self.hi)
        self.q_cmd = q_new.copy()
        self.v_cmd = v_cmd.copy()
        self.qd_cmd = qd.copy()

        tau = compute_generalized_gravity(self.model, pad_q_for_model(self.model, q_new, 6), self.data)[:6]

        # ── 夹爪直控 + 力矩保护 ──
        grip_send_out: float | None = None
        if self.grip_ready and self.args.gripper:
            g_lo, g_hi = self.grip_lim
            if abs(j_cmd) > 1e-6 and j_sel == 6:
                self.grip_cmd = float(np.clip(
                    self.grip_cmd + math.copysign(float(self.args.grip_rate) * dt, j_cmd), g_lo, g_hi))
            step_g = float(self.args.grip_rate) * 1.5 * dt
            self.grip_send = float(np.clip(
                self.grip_send + float(np.clip(self.grip_cmd - self.grip_send, -step_g, step_g)), g_lo, g_hi))
            grip_send_out = self.grip_send
            # 力矩保护：只要还在往目标走（目标 ≠ 实测）且力矩超限，就退回实测并停下本次行程
            # （含手势/外部目标；退回后相同目标不会再顶，直到行程变化）
            driving_g = abs(self.grip_cmd - self.grip_pos) > math.radians(2.0)
            if driving_g and abs(self.grip_tau) > GRIP_TAU_LIMIT:
                self.j_req = 0.0
                self.grip_cmd = float(self.grip_pos)
                self.grip_send = float(self.grip_pos)
                self.grip_msg = (f"⚠️ 到限位/有阻力（{self.grip_tau:+.2f}N·m）已松手，"
                                 f"位置 {math.degrees(self.grip_pos):+.1f}°")

        # ── 反馈与力矩保护 ──
        self._n += 1
        self.rate_hz = self._n / max(now - self._t0, 1e-6)
        if self.tau_limit > 0 and not freeze and tau_meas is not None:
            over = np.where(np.abs(np.asarray(tau_meas, float)[:6]) > self.tau_limit)[0]
            if len(over):
                self.freeze = True
                self.alarm = ("⚠️ 力矩超限自动冻结: " +
                              ", ".join(f"J{i+1}={np.asarray(tau_meas, float)[i]:+.1f}" for i in over))

        if freeze:
            self.status = "冻结中（保持悬停）"
        elif mode == "joint":
            self.status = self.motor_msg()
        else:
            self.status = "就绪（笔=空间运动）"

        return CoreCommand(
            q=q_new,
            kp=self.kp,
            kd=self.kd,
            tau=tau,
            grip_send=grip_send_out,
            status=self.status,
            alarm=self.alarm,
            msg=self.msg,
            speed_scale=speed,
            damp=lam,
            stale_ori=self.stale_ori,
            float_mode=False,
            mode=mode,
            freeze=freeze,
            joint_index=j_sel,
            pen_pressed=pressed,
        )

    # ====================================================================== #
    # 反馈更新（等价 control_loop 里 n%4 的刷新）
    # ====================================================================== #
    def update_feedback(self, q_meas: np.ndarray, tau_meas: np.ndarray | None = None,
                        grip_pos: float | None = None, grip_tau: float | None = None) -> None:
        if grip_pos is not None:
            self.grip_pos = float(grip_pos)
            self.grip_min = min(self.grip_min, self.grip_pos)
            self.grip_max = max(self.grip_max, self.grip_pos)
        if grip_tau is not None:
            self.grip_tau = float(grip_tau)
        _ = (q_meas, tau_meas)

    # ====================================================================== #
    # 退出：最小 jerk 归零轨迹（等价 finally 里的 clean 分支）
    # ====================================================================== #
    def finish_steps(self, duration: float = 3.0, hold: float = 0.5, rate: int = RATE):
        """生成 (q_t, tau) 序列：最小 jerk 从当前 q_cmd 回零，随后保持。"""
        q_from = np.asarray(self.q_cmd if self.q_cmd is not None else np.zeros(6), float).copy()
        steps = max(1, int(duration * rate))
        for i in range(1, steps + 1):
            s = smoothstep(i / steps)
            q_t = q_from * (1.0 - s)
            tau = compute_generalized_gravity(self.model, pad_q_for_model(self.model, q_t, 6), self.data)[:6]
            yield q_t, tau
        tau0 = compute_generalized_gravity(self.model, np.zeros(self.model.nq), self.data)[:6]
        for _ in range(int(hold * rate)):
            yield np.zeros(6), tau0
