#!/usr/bin/env python3
"""reBot 空间遥操控制核心（无 GUI 版）。

**从 `spatial_teleop.py` 的 `control_loop()` 原样抽出**，同一套常量、同一套顺序
（冻结/对齐/回放/漂浮/直控 → 速度指令 → 安全盒 → 雅可比阻尼最小二乘 → 零空间 →
限幅积分 → 夹爪 → 力矩保护）。

平台在此基础上补了四处（都已参数化在 `CoreArgs` / 机型配置，可关掉回到上游行为）：

1. **速度指令整形**：末端速度/角速度/J6 自转/关节直控速度都先过
   "一阶低通 + 加速度限幅"再做控制。原来这些指令都是阶跃（落笔、松手、
   换向、按住 Q/A 的瞬间直接从 0 跳到满速），表现就是"顿一下 / 窜一下"。
   整形只削瞬间突变，**额定速度不变**（低通 30ms、加速度上限见 CoreArgs）。
2. **关节直控 PID 速度环**（电机直控 / Q/A）：用实测关节速度闭环，
   参考速度 ≈ 实速（`vel_meas` 传电机反馈；不传则退化为指令差分）。
   P/I 补摩擦与负载造成的稳态速度差，D 抑制过冲；输出修正量限幅 + 抗饱和。
3. **姿态模式速度档**：角速度上限随 慢/中/快 缩放（原来三档完全一样），
   并把笔的死区/曲线调得更跟手（`DEAD_PX` 3px、`EXPO` 1.2）。
4. **夹爪**（三个平台扩展，参数见 `CoreArgs.grip_*` 与机型配置 `gripper`）：
   - 笔左右拉动是**增量跟手**：笔移多少、开度按比例动多少（`grip_pen_range` 像素 = 全行程），
     笔停/抬笔**立刻停**（不会"笔结束了还在动"）；按住 Q/A 仍是"按住一直走"的兜底；
   - 力控/堵转保护：下发值夹在"实测 ± τ上限/kp"内（力矩有上限），推着不动
     0.15~0.6s 就停手 + 半力保持，同方向不再顶（反向/换手势继续）；
   - 方向/行程可配：`gripper.direction`（+1 = 角度增大是张开；−1 = 反过来）、
     `lo_deg`/`hi_deg` 两端都留余量（开口端默认留得更多，避免"卡在外面"）。

`spatial_teleop.py` 保持不动；本文件供平台 / 无头脚本复用。

用法::

    core = TeleopCore(model, data, fid, kp, kd)
    core.pen_press(x_px, y_px); core.pen_move(x_px, y_px)
    cmd = core.step(dt, q_meas, tau_meas, vel_meas=qd_meas)
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

# ── 常量（与 spatial_teleop.py 一致；★ = 平台调过手感） ───────────────────────
RATE = 200
PEN_TIMEOUT = 0.4
TRAVEL_PX = 200.0
DEAD_PX = 3.0              # ★ 死区（原 6.0）：小位移也跟手，不再"推半天不动"
EXPO = 1.2                 # ★ 手感曲线（原 1.4）：更早提速（原来小位移太"肉"）
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
# ★ 平台调过：1.0 → 2.0 N·m。1.0 太小：位置环里 τ ≈ kp·偏差，而 kd 阻尼在
#   1.2 rad/s 时就要 2.4 N·m，偏差还没到限就已经"有阻力"了 → 夹爪走不动/走走停停。
#   2.0 N·m 能让夹爪按额定速度走（0.85 rad/s 量级），又远低于电机峰值 14 N·m；
#   真正防夹伤靠"限偏差 = 限力矩"+ 堵转停手（见 CoreArgs.grip_*），不是靠这个阈值。
GRIP_TAU_LIMIT = 2.0
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
# 夹爪（6）用**左右拉动**：笔右划 = 张开、左划 = 闭合。
# 与关节不同，夹爪是"增量跟手"：笔动多少、开度按比例动多少，笔停/抬笔就停
# （见 step() 夹爪段的 grip_pen_range / grip_track_rate；不会按住一直走）。
JOINT_PEN_MAP: dict[int, tuple[str, float]] = {
    0: ("x", +1.0),   # J1 肩部水平回转
    1: ("y", +1.0),   # J2 肩部俯仰
    2: ("y", +1.0),   # J3 肘部俯仰
    3: ("y", +1.0),   # J4 腕部俯仰（+ = 抬头）
    4: ("x", +1.0),   # J5 腕部偏航
    5: ("x", +1.0),   # J6 腕部自转
    6: ("x", +1.0),   # 夹爪：左划 = 闭合、右划 = 张开（拖动开度；手势之外的手动兜底）
}
JOINT_PEN_RATE = math.radians(60.0)   # 笔满偏时的关节速度（与 Q/A 直控一致）
MOTOR_NAMES = ("J1 肩部水平", "J2 肩部俯仰", "J3 肘部俯仰",
               "J4 腕部俯仰", "J5 腕部偏航", "J6 腕部自转")


# ── 工具函数（原样） ────────────────────────────────────────────────────────
def vel_from_px(v_px: float, vmax: float, travel: float = TRAVEL_PX, dead: float = DEAD_PX,
                expo: float = EXPO) -> float:
    a = abs(v_px)
    if a <= dead:
        return 0.0
    f = min((a - dead) / max(travel - dead, 1.0), 1.0)
    return math.copysign((f ** expo) * vmax, v_px)


def shape_step(cur: float, target: float, acc: float, tau: float, dt: float) -> float:
    """一维速度指令整形：一阶低通 + 加速度限幅（不降额定速度，只削突变）。

    - `tau`：低通时间常数（s），把笔的抖动/离散采样抹平；0 = 关
    - `acc`：加速度上限（单位/s²），起停、反向都走斜坡；0 = 关
    """
    x = target if tau <= 0 else cur + (target - cur) * (dt / (tau + dt))
    lim = max(float(acc), 0.0) * dt
    d = x - cur
    if lim > 0 and abs(d) > lim:
        d = math.copysign(lim, d)
    return cur + d


def shape_vec(cur: np.ndarray, target: np.ndarray, acc: float, tau: float, dt: float) -> np.ndarray:
    """三维速度指令整形（逐分量，见 shape_step）。"""
    cur = np.asarray(cur, float)
    target = np.asarray(target, float)
    x = target if tau <= 0 else cur + (target - cur) * (dt / (tau + dt))
    lim = max(float(acc), 0.0) * dt
    d = x - cur
    if lim > 0:
        d = np.clip(d, -lim, lim)
    return cur + d


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
    expo: float = EXPO                     # ★ 笔 → 速度手感曲线（越小越跟手）
    damping: float = DAMPING
    ori_weight: float = 0.0
    posture_k: float = 0.2
    wmax_deg: float = 40.0                 # ★ 姿态模式满偏角速度（原 30；随速度档缩放）
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
    # ── 夹爪力控（★ 平台新增）：别再"顶死/卡住" ──
    grip_tau_limit: float = GRIP_TAU_LIMIT      # 电机力矩上限（N·m）：到它就判定"有阻力/到限位"
    grip_stall_s: float = 0.6                   # 想走但实测不动超过这么久 → 认定被挡住（无力矩信号时的兜底）
    grip_stall_fast_s: float = 0.15             # 力矩已超限时的快速判定（有反馈时反应更快）
    grip_hold_frac: float = 0.5                 # 停手后保留的夹持力比例（0.5 → 约 0.75N·m，能抱住东西）
    # ── 夹爪笔控（左右拉动）：增量跟手，笔停夹爪停 ──
    grip_pen_range: float = 500.0               # 笔走这么多像素 = 走完整个行程（越小越灵敏）
    grip_pen_dead: float = 2.5                  # 像素死区：累积超过它才动（滤掉笔的抖动）
    grip_track_rate: float = 3.0                # 拖动时目标最大变化速度（rad/s，防"甩笔"一下到底）
    # ── 速度指令整形（★ 平台新增）：只削"起停/反向的瞬间跳变"，不降额定速度 ──
    v_acc: float = 1.2         # 末端加速度上限（m/s²）
    v_tau: float = 0.03        # 末端速度一阶低通（s）
    w_acc: float = 10.0        # 角加速度上限（rad/s²）
    w_tau: float = 0.03        # 角速度一阶低通（s）
    twist_acc: float = 8.0     # J6 自转角加速度上限（rad/s²）
    twist_tau: float = 0.03
    j_acc: float = 14.0        # 关节直控加速度上限（rad/s²）：0→60°/s 约 75ms
    j_tau: float = 0.02        # 关节直控速度一阶低通（s）
    # ── 关节直控速度环（PID，★ 平台新增）：让电机真的跑在指令速度上 ──
    jvel_kp: float = 0.20      # 比例
    jvel_ki: float = 2.00      # 积分（消除摩擦/负载造成的稳态速度差）
    jvel_kd: float = 0.02      # 微分（作用在实测速度上，抑制过冲）
    jvel_tau: float = 0.02     # 测速低通（s）
    jvel_corr_max: float = 0.25  # 速度环修正量上限（rad/s），防止异常大修正


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
        self.grip_blocked = False        # ★ 本次行程被挡住（到限位/有阻力），同方向不再顶
        self._grip_block_dir = 0.0       # 挡住的方向（+1 = 往大开，−1 = 往闭合）
        self._grip_stall_t: float | None = None
        self._grip_stall_pos = 0.0
        self._grip_pen_start = 0.0       # ★ 笔按下的那一刻的开度（位置式的起点，兼容保留）
        self._grip_pen_last_x = 0.0      # 上一帧的笔 x（增量跟手）
        self._grip_pen_acc = 0.0         # 未超过死区的累计像素
        self._grip_pen_active = False    # 这一笔是否正在拖夹爪（抬笔要立刻停）
        self._grip_ext_target: float | None = None   # 外部（手势）行程目标 0=合 1=开

        self._twist_prev = 0.0
        self._was_float = False
        self._t_prev: float | None = None
        self._t0: float | None = None
        self._n = 0

        # ── 速度整形 / 关节速度环状态（★ 平台新增） ──
        self._v_sm = np.zeros(3)     # 整形后的末端速度（m/s）
        self._w_sm = np.zeros(3)     # 整形后的角速度（rad/s）
        self._tw_sm = 0.0            # 整形后的 J6 自转速度（rad/s）
        self._j_sm = 0.0             # 整形后的关节直控速度（rad/s）
        self._jvel_i = 0.0           # 速度环积分项
        self._jvel_d = 0.0           # 速度环微分项（低通后）
        self._jvel_meas = 0.0        # 低通后的实测速度
        self._jvel_meas_prev: float | None = None
        self._jvel_corr = 0.0        # 最近一次速度环修正量（诊断用）
        self._jvel_ref = 0.0         # 最近一次关节速度参考（诊断用）
        self._q_meas: np.ndarray | None = None
        self._q_meas_prev: np.ndarray | None = None
        self._qd_fb: np.ndarray | None = None   # 电机反馈速度（rad/s）

    # ====================================================================== #
    # 速度整形 / 关节直控速度环（PID）
    # ====================================================================== #
    def _reset_shapers(self) -> None:
        """命令源切换（模式/电机/回放/冻结）时清空整形与速度环状态。

        否则上一段运动的速度会"漏"到新目标上（例如换电机瞬间被带动）。
        """
        self._v_sm[:] = 0.0
        self._w_sm[:] = 0.0
        self._tw_sm = 0.0
        self._j_sm = 0.0
        self._jvel_reset()

    def _jvel_reset(self) -> None:
        self._jvel_i = 0.0
        self._jvel_d = 0.0
        self._jvel_meas = 0.0
        self._jvel_meas_prev = None
        self._jvel_corr = 0.0
        self._jvel_ref = 0.0

    def _measured_vel(self, i: int, dt: float) -> float:
        """关节 i 的实测角速度：优先用电机反馈，没有就差分；再做一阶低通。"""
        if self._qd_fb is not None:
            raw = float(np.asarray(self._qd_fb, float)[i])
        elif self._q_meas is not None and self._q_meas_prev is not None:
            raw = float((self._q_meas[i] - self._q_meas_prev[i]) / max(dt, 1e-4))
        else:
            raw = 0.0
        tau = max(0.0, float(self.args.jvel_tau))
        a = 1.0 if tau <= 0 else dt / (tau + dt)
        self._jvel_meas += (raw - self._jvel_meas) * a
        return self._jvel_meas

    def _joint_vel_pid(self, ref: float, i: int, dt: float) -> float:
        """关节直控速度环（PID，串级在位置积分外环上）。

        `ref` 已做过整形（限加速 + 低通），这里用实测速度闭环：
        - P/I 修正摩擦、负载、重力前馈误差造成的速度偏差（速度更准，不是更慢）
        - D 作用在实测速度上（避免参考阶跃时的微分冲击），并再低通一次
        - 输出修正量限幅 + 积分抗饱和
        """
        a = self.args
        meas = self._measured_vel(i, dt)
        err = ref - meas
        i_lim = float(a.jvel_corr_max) / max(float(a.jvel_ki), 1e-6)
        self._jvel_i = float(np.clip(self._jvel_i + err * dt, -i_lim, i_lim))
        d_raw = 0.0 if self._jvel_meas_prev is None else -(meas - self._jvel_meas_prev) / max(dt, 1e-4)
        tau_d = max(1e-3, float(a.jvel_tau))
        self._jvel_d += (d_raw - self._jvel_d) * (dt / (tau_d + dt))
        self._jvel_meas_prev = meas
        corr = float(a.jvel_kp) * err + float(a.jvel_ki) * self._jvel_i + float(a.jvel_kd) * self._jvel_d
        corr = float(np.clip(corr, -float(a.jvel_corr_max), float(a.jvel_corr_max)))
        self._jvel_corr = corr
        self._jvel_ref = float(ref)
        return float(ref) + corr

    # ====================================================================== #
    # 夹爪：限力矩 + 堵转停手（★ 平台）
    # ====================================================================== #
    def grip_err_max(self) -> float:
        """位置环的"安全偏差"：偏差 × kp = 电机力矩，因此偏差夹住 = 力矩有上限。

        tau = kp·(目标−实测)（堵转时速度项为 0），所以
        `err_max = grip_tau_limit / GRIP_KP`（1.5N·m / 20 ≈ 0.075 rad ≈ 4.3°）。
        """
        return max(0.02, float(self.args.grip_tau_limit) / max(float(GRIP_KP), 1e-6))

    def grip_hold_pos(self, direction: float) -> float:
        """停手后的"半力保持点"：自锁机构靠它抱住东西，又不会一直磨。"""
        g_lo, g_hi = self.grip_lim
        d = math.copysign(1.0, direction) if direction else 0.0
        return float(np.clip(self.grip_pos + d * float(self.args.grip_hold_frac) * self.grip_err_max(),
                             g_lo, g_hi))

    def grip_clear_block(self) -> None:
        """放开"被挡住"的标记：换手势 / 反向 / 重新对齐时调用。"""
        self.grip_blocked = False
        self._grip_block_dir = 0.0
        self._grip_stall_t = None
        self._grip_stall_pos = self.grip_pos

    def grip_blocked_towards(self, direction: float) -> bool:
        """这个方向是否已经被挡住（挡住就不许再顶，反向可以）。"""
        return bool(self.grip_blocked) and direction != 0.0 and \
            math.copysign(1.0, direction) == self._grip_block_dir

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
        self._grip_pen_start = float(self.grip_cmd)
        self.grip_clear_block()

    # ====================================================================== #
    # 输入接口（等价 GUI 事件）
    # ====================================================================== #
    def pen_press(self, x: float, y: float) -> None:
        self.pen = (float(x), float(y))
        self.anchor = (float(x), float(y))
        self.pressed = True
        self.pen_t = time.perf_counter()
        # 夹爪是"增量跟手"：记下起笔位置，之后按每帧位移累计开度
        self._grip_pen_start = float(self.grip_cmd)
        self._grip_pen_last_x = float(x)
        self._grip_pen_acc = 0.0
        self._grip_pen_active = False
        self.grip_clear_block()          # 重新落笔 = 允许再试一次

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
        self._reset_shapers()      # 换模式 = 换指令源：上一段速度不许漏到新模式
        self._grip_pen_start = float(self.grip_cmd)   # 拉动起点跟着重锚
        self._grip_pen_last_x = float(self.pen[0])
        self._grip_pen_acc = 0.0
        if mode == "joint":
            self.grip_clear_block()   # 显式选中夹爪 = 允许再推一次
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

        与上次相同的请求直接忽略：到限位/有阻力停手后，不会反复顶；
        等行程下一次变化（换手势）再继续。真正下发仍走 step() 的限速 + 限力矩。
        """
        if not self.grip_ready or not self.args.gripper:
            return
        frac = float(np.clip(frac, 0.0, 1.0))
        if self._grip_ext_target is not None and abs(frac - self._grip_ext_target) < 1e-6:
            return
        self._grip_ext_target = frac
        g_lo, g_hi = self.grip_lim
        self.grip_cmd = float(g_lo + frac * (g_hi - g_lo))
        self.grip_clear_block()      # 新的行程目标 = 新的尝试

    def motor_msg(self) -> str:
        if self.j_sel >= 6 or self.j_sel not in JOINT_PEN_MAP:
            return "电机直控：夹爪（笔左划=闭合 / 右划=张开，拉动多少开多少）"
        axis = JOINT_PEN_MAP[self.j_sel][0]
        return f"电机直控：{MOTOR_NAMES[self.j_sel]}（笔{'上下' if axis == 'y' else '左右'}）"

    def set_float(self, on: bool) -> None:
        self.float_mode = bool(on)

    def joint_hold(self, v: float) -> None:
        """关节直控速度（rad/s，按住时非零；等价 S.j_req）。"""
        self.j_req = float(v)

    def select_joint(self, i: int) -> None:
        i = int(np.clip(i, 0, 6))
        if i != self.j_sel:
            self._reset_shapers()   # 换关节：上一段的整形/速度环状态不串到新关节
        self.j_sel = i

    def request_align(self) -> None:
        # 与上游按键 R 一致：重设笔锚点，避免重新对齐后按旧位移继续推动机械臂。
        self.anchor = self.pen
        self.align_req = True
        self._grip_pen_last_x = float(self.pen[0])
        self._grip_pen_acc = 0.0
        self.grip_clear_block()

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
    def step(self, dt_raw: float, q_meas: np.ndarray, tau_meas: np.ndarray | None = None,
             vel_meas: np.ndarray | None = None) -> CoreCommand:
        """一步控制。

        `vel_meas`：关节实测角速度（rad/s，可选）。给了就用电机反馈做直控速度环；
        没给就退化成"指令差分"，行为与上游一致（测试/无反馈场景）。
        """
        now = time.perf_counter()
        if self._t0 is None:
            self._t0 = now
        dt = max(1e-4, min(float(dt_raw), 0.05))
        q_meas = np.asarray(q_meas, float)[:6]
        self._q_meas_prev = None if self._q_meas is None else self._q_meas.copy()
        self._q_meas = q_meas.copy()
        self._qd_fb = None if vel_meas is None else np.asarray(vel_meas, float)[:6]
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
            self._reset_shapers()   # 漂浮期间手工推臂，退出后从 0 起速
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
        # 夹爪（6）例外：笔左右是**增量跟手**（笔动多少、开度按比例动多少；抬笔立刻停），
        # 逻辑写在下面的夹爪段里（要按帧累计位移），这里不产生"位置目标"。
        j_direct = 0.0
        if (mode == "joint" and pressed and not freeze
                and replay is None and self.selftest_cmd is None
                and int(self.j_sel) in JOINT_PEN_MAP and int(self.j_sel) != 6):
            axis, sign = JOINT_PEN_MAP[int(self.j_sel)]
            dx = pen[0] - anchor[0]
            dy = pen[1] - anchor[1]
            disp = (-dy if axis == "y" else dx) * sign
            j_direct = vel_from_px(disp, JOINT_PEN_RATE, self.args.pen_range, self.args.dead,
                                   self.args.expo)

        # ── 0b. 关节直控：参考跟随，避免笛卡尔任务拉扯 ──
        j_cmd = 0.0 if freeze else self.j_req
        if mode == "joint" and abs(j_direct) > 1e-9:
            j_cmd = j_direct                      # 笔控优先于 Q/A 保持
        # 速度指令整形：起停/换向走斜坡 + 低通（额定速度不变，只去掉"瞬间跳变"）。
        # 之前是直接从 0 跳到满速（Q/A 按住、笔甩动、松手都是阶跃）→ 冲击感很大。
        if freeze or replay is not None or self.selftest_cmd is not None:
            self._j_sm = 0.0
        else:
            self._j_sm = shape_step(self._j_sm, j_cmd, self.args.j_acc, self.args.j_tau, dt)
        j_ref = 0.0 if freeze else float(self._j_sm)
        j_sel = int(self.j_sel)
        if abs(j_ref) > 1e-6:
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
            # ★ 角速度上限随速度档缩放（原来慢/中/快完全一样，姿态模式调档没反应）
            wmax = math.radians(float(self.args.wmax_deg)) * speed
            w_cmd = np.array([vel_from_px(-dy, wmax, self.args.pen_range, self.args.dead, self.args.expo) if self.shift else 0.0,
                              vel_from_px(-dy, wmax, self.args.pen_range, self.args.dead, self.args.expo) if not self.shift else 0.0,
                              vel_from_px(dx, wmax, self.args.pen_range, self.args.dead, self.args.expo)])
            v_cmd = np.zeros(3)
            twist_cmd = 0.0
        elif pressed and not freeze:
            dx = pen[0] - anchor[0]
            dy = pen[1] - anchor[1]
            if self.shift:
                v_cmd = np.array([0.0,
                                  vel_from_px(dx, vmax, self.args.pen_range, self.args.dead, self.args.expo),
                                  vel_from_px(-dy, vmax, self.args.pen_range, self.args.dead, self.args.expo)])
            else:
                v_cmd = np.array([vel_from_px(-dy, vmax, self.args.pen_range, self.args.dead, self.args.expo),
                                  vel_from_px(dx, vmax, self.args.pen_range, self.args.dead, self.args.expo),
                                  0.0])
            twist_cmd = twist
        else:
            v_cmd = np.zeros(3)
            twist_cmd = 0.0

        # ── 1b. 速度指令整形（★ 平台新增）：限加速 + 低通 ──
        # 位置/姿态模式原来也是"阶跃速度"：落笔/松手/反向的瞬间都在硬切换，
        # 手感是"一顿一顿"；这里把指令磨成连续斜坡（额定速度不变，只是能到的速度更快）。
        if freeze or replay is not None or self.selftest_cmd is not None:
            self._reset_shapers()
        else:
            self._v_sm = shape_vec(self._v_sm, v_cmd, self.args.v_acc, self.args.v_tau, dt)
            self._w_sm = shape_vec(self._w_sm, w_cmd, self.args.w_acc, self.args.w_tau, dt)
            self._tw_sm = shape_step(self._tw_sm, twist_cmd, self.args.twist_acc, self.args.twist_tau, dt)
            v_cmd = self._v_sm.copy()
            w_cmd = self._w_sm.copy()
            twist_cmd = float(self._tw_sm)
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

        # ── 3b. 关节直控：PID 速度环 ──
        # 原来是"指令速度直接当关节速度"（qd += j_cmd）：电机没跟上就永远差着，
        # 有摩擦/负载时实际速度达不到指令，且任何抖动都原样传到电机。
        # 现在用实测速度闭环（参考已整形）：P/I 补掉稳态速度差，D 抑制过冲。
        if abs(j_ref) > 1e-6 and j_sel < 6:
            qd[j_sel] += self._joint_vel_pid(j_ref, j_sel, dt)
        else:
            self._jvel_reset()

        # ── 4. 关节速度限幅 + 积分 + 软限位硬夹 ──
        qd = np.clip(qd, -QD_MAX, QD_MAX)
        q_new = np.clip(self.q_cmd + qd * dt, self.lo, self.hi)
        self.q_cmd = q_new.copy()
        self.v_cmd = v_cmd.copy()
        self.qd_cmd = qd.copy()

        tau = compute_generalized_gravity(self.model, pad_q_for_model(self.model, q_new, 6), self.data)[:6]

        # ── 夹爪：行程跟目标 + 限力矩 + 堵转停手（★ 平台重写） ──
        # 原来两个问题：
        # 1) 下发值按 1.5× 速度追目标 → 目标跑到前面，位置环只能靠"多出力矩"追速；
        # 2) 力矩保护在平台这条链路上是失效的：real_arm 把目标拉回实测后，
        #    下一帧又被这里的目标覆盖 → 电机顶着 14N·m 堵转 → RobStride 锁存故障
        #    → 夹爪彻底不动（数据里能看到：命令在动、位置一动不动）。
        # 现在：目标与下发 1:1 限速；堵转（推着不动）就停手并保留半力夹持；
        # 同方向不再顶，反向/换手势才重新尝试。
        grip_send_out: float | None = None
        if self.grip_ready and self.args.gripper:
            g_lo, g_hi = self.grip_lim
            err_max = self.grip_err_max()
            rate = float(self.args.grip_rate)
            # 笔在拖夹爪？（电机直控 + 选中夹爪 + 笔按下；回放/自检/冻结时不算）
            pen_drive = bool(mode == "joint" and j_sel == 6 and pressed and not freeze
                             and replay is None and self.selftest_cmd is None)
            if pen_drive:
                # 增量跟手：这一帧笔动了多少像素 → 开度按比例动多少
                # （不像关节那样"按住一直走"；笔停就不动，抬笔立刻停）
                dx_px = float(pen[0]) - float(self._grip_pen_last_x)
                self._grip_pen_last_x = float(pen[0])
                self._grip_pen_acc += dx_px
                if abs(self._grip_pen_acc) >= float(self.args.grip_pen_dead):
                    full = max(float(self.args.grip_pen_range), 1.0)
                    want_step = (self._grip_pen_acc / full) * (g_hi - g_lo)
                    lim = float(self.args.grip_track_rate) * dt     # 防"甩笔"一下到底
                    step = float(np.clip(want_step, -lim, lim))
                    d = math.copysign(1.0, step)
                    if self.grip_blocked_towards(d):
                        self._grip_pen_acc = 0.0                     # 这个方向被挡住：别再累计
                    else:
                        new = float(np.clip(self.grip_cmd + step, g_lo, g_hi))
                        if abs((new - self.grip_cmd) - step) > 1e-12:
                            self._grip_pen_acc = 0.0                 # 到端了：余量丢掉
                        else:
                            # 被"最大速度"截掉的部分留回累计器，下一帧继续走（不丢行程）
                            self._grip_pen_acc = (want_step - step) / (g_hi - g_lo) * full
                        self.grip_cmd = new
                        self._grip_pen_start = float(self.grip_cmd)
                self._grip_pen_active = True
            else:
                # 抬笔 / 笔超时 / 换模式：夹爪立刻停在当前开度（不追剩余的行程）
                self._grip_pen_last_x = float(pen[0]) if pressed else self._grip_pen_last_x
                self._grip_pen_acc = 0.0
                if self._grip_pen_active:
                    self.grip_cmd = float(self.grip_pos)
                    self.grip_send = float(self.grip_pos)
                    self._grip_pen_active = False
                # 键盘兜底（Q/A 按住）：按速度走
                if abs(j_ref) > 1e-6 and j_sel == 6:
                    d = math.copysign(1.0, j_ref)
                    if not self.grip_blocked_towards(d):
                        self.grip_cmd = float(np.clip(self.grip_cmd + d * rate * dt, g_lo, g_hi))
            step_g = rate * dt
            self.grip_send = float(np.clip(
                self.grip_send + float(np.clip(self.grip_cmd - self.grip_send, -step_g, step_g)), g_lo, g_hi))
            grip_send_out = self.grip_send

            want = self.grip_send - self.grip_pos
            pushing = abs(want) > 0.6 * err_max          # 偏差已到限力区：电机在出力
            if not pushing:
                self._grip_stall_t = None
                self._grip_stall_pos = self.grip_pos
                # 已经退到保持点 / 目标换方向 → 解除"挡住"标记
                if self.grip_blocked and (abs(want) < 1e-9
                                          or math.copysign(1.0, want) != self._grip_block_dir):
                    self.grip_clear_block()
            elif abs(self.grip_pos - self._grip_stall_pos) > math.radians(0.5):
                self._grip_stall_pos = self.grip_pos  # 还在走：计时重来
                self._grip_stall_t = 0.0
            else:
                # 推着不动：按控制周期累加（不依赖墙钟，仿真/自检里也一致）
                self._grip_stall_t = dt if self._grip_stall_t is None else self._grip_stall_t + dt
                # 力矩已经贴近上限 → 快速判定（位置环堵转时实测力矩 ≈ 偏差×kp ≈ 上限，
                # 所以用 0.8× 而不是严格大于，否则永远差一点点不触发）
                fast = abs(self.grip_tau) >= 0.8 * float(self.args.grip_tau_limit)
                limit_s = float(self.args.grip_stall_fast_s) if fast else float(self.args.grip_stall_s)
                if self._grip_stall_t > limit_s:
                    d = math.copysign(1.0, want)
                    hold = self.grip_hold_pos(d)
                    self.grip_cmd = hold
                    self.grip_send = hold
                    self.grip_blocked = True
                    self._grip_block_dir = d
                    self._grip_stall_t = None
                    self._grip_stall_pos = self.grip_pos
                    self.j_req = 0.0
                    self.grip_msg = (
                        f"夹爪到限位/有阻力（{self.grip_tau:+.2f}N·m）：停在 "
                        f"{math.degrees(self.grip_pos):+.1f}°，半力保持；同方向不再顶，"
                        f"反向/换手势可继续")
                    self.msg = self.grip_msg

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
