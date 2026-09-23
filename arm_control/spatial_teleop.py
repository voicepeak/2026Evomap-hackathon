#!/usr/bin/env python3
"""spatial_teleop.py — 数位板驱动的空间协调运动（速度级雅可比伺服）

和旧版的根本区别：
    旧版 = 笔位移 → 绝对目标点 → 解 IK（会累积、会 FAIL、姿态锁死）
    新版 = 笔位移 → 空间速度指令 → 雅可比 → 关节速度（松笔即停，永不卡死）

笔（唯一运动输入）:
    按住 + 上/下偏   →  末端前伸 / 后收   (X)
    按住 + 左/右偏   →  末端左移 / 右移   (Y)
    按住 + Shift+上/下 →  末端升 / 降      (Z)
    松开            →  速度立即归零（停住保持，不失能）

键盘（模式）:
    ,  /  .     手腕 J6 自转（按住，注意是按住不放）
    F           速度档：慢 / 中 / 快
    Space       冻结 / 解冻
    R           重新对齐（记录当前姿态为参考姿态 + 笔重新锚定）
    Esc         退出（先平滑归零，再保持悬停）

协调由雅可比完成（阻尼最小二乘 + 零空间借位 + 姿态软保持），
所以是"一组关节配合完成一个空间运动"，不是一个个拧电机。

安全: 逐关节软限位、逐关节限速、笛卡尔安全盒、冻结、退出平滑归零、绝不断力矩。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import threading
import time
import tkinter as tk

import numpy as np
import pinocchio as pin

REPO = "/Users/Admin/Desktop/reBotArm_control_py"
sys.path.insert(0, REPO)

from reBotArm_control_py.actuator import RebotArm  # noqa: E402
from reBotArm_control_py.dynamics import compute_generalized_gravity, load_dynamics_model  # noqa: E402
from reBotArm_control_py.kinematics import (  # noqa: E402
    get_end_effector_frame_id,
    pos_rot_to_se3,
    joint_to_pose,
    load_robot_model,
    pad_q_for_model,
)

# ── 配置 ──────────────────────────────────────────────────────────────────────
RATE = 200
PEN_TIMEOUT = 0.4
TRAVEL_PX = 200.0          # 笔偏离多少像素 = 满速（--pen-range 覆盖）
DEAD_PX = 6.0              # 死区
EXPO = 1.4                 # 手感曲线
VMAX = 0.08                # 满速 m/s
QD_MAX = 1.0               # 关节速度硬上限 rad/s
TWIST_RATE = math.radians(35.0)   # J6 自转速度 rad/s
LIMIT_MARGIN = math.radians(1.5)  # 关节软限位余量（--limit-margin 可改）
LIMIT_SOFT = 0.12          # 零空间限位排斥的作用范围 rad（只在贴边时才动）
LIMIT_K = 0.35             # 排斥强度 rad/s（调小，避免干扰位置/姿态保持）
ORI_K = 1.5                # 姿态保持增益 1/s
ORI_VMAX = 0.4             # 姿态修正速度上限 rad/s（防止和平移打架）
REPLAY_KP = 1.2            # 姿势回放：位置闭环增益 1/s
ORI_POS_K = 6.0            # 姿态模式：末端位置保持增益 1/s
PRESET_FILE = REPO + "/config/poses.json"   # 姿势预设（教学记录）
J4_RATE = math.radians(45.0)
# 夹爪（第 7 个电机）保守参数
GRIP_KP = 20.0             # 比关节软（防撞死）
GRIP_KD = 2.0
GRIP_RATE = 1.20           # rad/s（约 69°/s）
GRIP_RANGE = 3.00          # （备用）以启动位置为中心的活动半径
GRIP_LO = math.radians(3.0)      # 实测机械下端 -11.9° + 5° 余量
GRIP_HI = math.radians(328.0)    # 实测机械上端 +335.4° - 7° 余量
GRIP_TAU_LIMIT = 1.0       # 力矩超过它就自动停+回退 (N·m)   # J4 直控速度 °/s（按住时；实际到手约 1/5~1/3）
DAMPING = 0.02             # 阻尼最小二乘 λ0

# 笛卡尔安全盒（沿用已验证的工作空间）
BX = (0.14, 0.46)
BY = (-0.30, 0.30)
BZ = (0.16, 0.52)
RMAX = 0.48

CW, CH = 960, 620
BG, FG, DIM, ACC = "#0b1220", "#e5e7eb", "#8aa0c6", "#38bdf8"
OKC, WARN, RED = "#34d399", "#fbbf24", "#ef4444"

SPEED_PRESETS = [("慢", 0.5), ("中", 1.0), ("快", 2.0)]

VERBOSE = False     # --verbose 时输出全部细节；默认只输出重要信息


def log(msg: str, *, force: bool = False, **_ignored) -> None:
    if VERBOSE or force:
        print(msg, flush=True)


class State:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.stop = False
        self.error: str | None = None
        self.status = "初始化…"
        self.ready = False
        # 反馈
        self.q = np.zeros(6)
        self.qd = np.zeros(6)          # 实测速度
        self.tau = np.zeros(6)
        self.p = np.zeros(3)           # 末端位置
        self.rpy = np.zeros(3)
        # 指令
        self.q_cmd = np.zeros(6)
        self.v_cmd = np.zeros(3)       # 世界系空间速度指令
        self.qd_cmd = np.zeros(6)
        self.twist = 0.0               # J6 自转速度指令
        # 笔
        self.pen = (0.0, 0.0)
        self.anchor = (0.0, 0.0)
        self.pressed = False
        self.pen_t = 0.0
        self.shift = False
        # 模式
        self.freeze = False
        self.speed_i = 1
        self.preset = 1.0
        self.ori_on = True
        self.stale_ori = False
        self.align_req = False
        self.selftest_cmd: dict | None = None   # 自检时强制速度
        self.rate_hz = 0.0
        self.damp = DAMPING
        self.alarm = ""
        self.quit_req = False
        self.float_mode = False    # 悬停/漂浮模式：kp=0 + 重力补偿，可用手推动
        self.mode = "pos"          # pos=空间位置 | ori=工具姿态
        self.p_ref = np.zeros(3)   # 姿态模式下的位置保持点
        self.replay: dict | None = None
        self.msg = ""
        self.presets: dict = {}
        self.esc_t = 0.0
        self.j_req = 0.0        # 关节直控速度指令（rad/s，按住时非零）
        self.j_sel = 3          # 直控选中的电机 0..6（3=J4 俯仰，6=夹爪）
        self.grip_ready = False
        self.grip_pos = 0.0
        self.grip_cmd = 0.0
        self.grip_tau = 0.0
        self.grip_lim = (0.0, 0.0)
        self.grip_msg = ""
        self.grip_send = 0.0
        self.grip_min = 0.0
        self.grip_max = 0.0
        self.j_rng = math.radians(60.0)


S = State()


def load_presets() -> dict:
    try:
        with open(PRESET_FILE) as f:
            return json.load(f)
    except Exception:  # noqa: BLE001
        return {}


def save_presets(d: dict) -> None:
    try:
        os.makedirs(os.path.dirname(PRESET_FILE), exist_ok=True)
        with open(PRESET_FILE, "w") as f:
            json.dump(d, f, ensure_ascii=False, indent=1)
    except Exception as e:  # noqa: BLE001
        print(f"[preset] 保存失败: {e}", flush=True)


def record_preset(i: int) -> None:
    with S.lock:
        q = S.q.copy(); p = S.p.copy(); rpy = S.rpy.copy()
    S.presets[str(i)] = {"q": [float(x) for x in q], "p": [float(x) for x in p],
                         "rpy": [float(x) for x in rpy]}
    save_presets(S.presets)
    log(f"[preset] 已记录 槽位{i}: q(deg)={np.round(np.degrees(q), 1)} "
          f"p={np.round(p, 3)}", force=True)


def goto_preset(i: int) -> None:
    d = S.presets.get(str(i))
    if not d:
        log(f"[preset] 槽位{i} 还没记录（Shift+{i} 记录当前姿势）", flush=True)
        return
    with S.lock:
        S.replay = {"p": np.array(d["p"], float), "rpy": np.array(d["rpy"], float),
                    "t_end": time.monotonic() + 25.0}
        S.msg = f"前往预设 {i}…"
    log(f"[preset] → 槽位{i}  p={np.round(d['p'], 3)}", force=True)


def vel_from_px(v_px: float, vmax: float) -> float:
    a = abs(v_px)
    if a <= DEAD_PX:
        return 0.0
    f = min((a - DEAD_PX) / max(TRAVEL_PX - DEAD_PX, 1.0), 1.0)
    return math.copysign((f ** EXPO) * vmax, v_px)


def limit_repulse(q: np.ndarray, lo: np.ndarray, hi: np.ndarray) -> np.ndarray:
    z = np.zeros(6)
    for i in range(6):
        m = min(LIMIT_SOFT, 0.5 * (hi[i] - lo[i]))
        if q[i] < lo[i] + m:
            z[i] = LIMIT_K * (1.0 - (q[i] - lo[i]) / m)
        elif q[i] > hi[i] - m:
            z[i] = -LIMIT_K * (1.0 - (hi[i] - q[i]) / m)
    return np.clip(z, -LIMIT_K, LIMIT_K)


def rotvec_err(rpy_ref, rpy_cur) -> np.ndarray:
    """参考姿态与当前姿态之间的旋转向量（世界系，弧度）——避免 RPY 大角度耦合。"""
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


# ── 控制线程 ──────────────────────────────────────────────────────────────────


def control_loop(args) -> None:
    arm = None
    clean = False
    model = data = fid = None
    try:
        arm = RebotArm()
        arm.connect()
        g = arm.arm
        q0 = np.asarray(g.get_positions(), float)[:6]
        if float(np.max(np.abs(q0))) > 0.05:
            raise RuntimeError("启动姿态距折叠零位超过 0.05 rad；拒绝使能"
                               "（不 disconnect / 不失能，机械臂维持原状态）")

        model = load_robot_model()
        data = model.createData()
        fid = get_end_effector_frame_id(model)
        lm = math.radians(args.limit_margin)
        lo = np.asarray(model.lowerPositionLimit[:6], float) + lm
        hi = np.asarray(model.upperPositionLimit[:6], float) - lm
        kp, kd = g._mit_kp.copy(), g._mit_kd.copy()

        g.mode_mit(kp=kp, kd=kd)
        g.enable()
        tau = compute_generalized_gravity(model, pad_q_for_model(model, q0, 6), data)[:6]
        g.send_mit(q0, vel=np.zeros(6), kp=kp, kd=kd, tau=tau)

        grip_g = None
        if args.gripper:
            try:
                grip_g = arm.gripper
                grip_g.mode_mit()
                grip_g.enable()
                gpos0 = float(np.asarray(grip_g.get_positions(), float).reshape(-1)[0])
                if args.grip_abs:
                    glo, ghi = GRIP_LO, GRIP_HI
                else:
                    glo, ghi = gpos0 - args.grip_range, gpos0 + args.grip_range
                with S.lock:
                    S.grip_ready = True
                    S.grip_pos = gpos0
                    S.grip_cmd = gpos0
                    S.grip_send = gpos0
                    S.grip_lim = (glo, ghi)
                log(f"[grip] 已使能  当前={math.degrees(gpos0):+.1f}°  "
                    f"可用范围 {math.degrees(glo):+.1f}° ~ {math.degrees(ghi):+.1f}°"
                    f"（实测机械端 -11.9° / +335.4°，已留余量）", force=True)
            except Exception as e:  # noqa: BLE001
                log(f"[grip] 不可用: {type(e).__name__}: {e}", force=True)
                grip_g = None

        p0, rpy0 = joint_to_pose(q0)
        with S.lock:
            S.q = q0.copy()
            S.q_cmd = q0.copy()
            S.p = np.asarray(p0, float).copy()
            S.rpy = np.asarray(rpy0, float).copy()
            S.status = "就绪（笔=空间运动）"
            S.p_ref = np.asarray(p0, float).copy()
            S.ready = True
        rpy_ref = np.asarray(rpy0, float).copy()
        rpy_ref_base1 = float(rpy_ref[1])
        q_seed = q0.copy()
        log(f"[spatial] 起始末端 X={p0[0]:+.3f} Y={p0[1]:+.3f} Z={p0[2]:+.3f}", force=True)

        q_cmd = q0.copy()          # ← 本地累加器（只在启动/对齐时初始化一次）
        twist_prev = 0.0
        was_float = False
        dt_nom = 1.0 / RATE
        t_prev = time.perf_counter()
        t0 = t_prev
        n = 0
        while not S.stop:
            now = time.perf_counter()
            dt = max(1e-4, min(now - t_prev, 0.05))
            t_prev = now

            with S.lock:
                freeze = S.freeze
                pressed = S.pressed
                pen_t = S.pen_t
                shift = S.shift
                speed = SPEED_PRESETS[S.speed_i][1]
                pen = S.pen
                anchor = S.anchor
                twist = S.twist
                stale_ori = S.stale_ori
                st_cmd = S.selftest_cmd
                align = S.align_req
                S.align_req = False
                mode = S.mode
                replay = S.replay
                float_now = S.float_mode

            if abs(twist_prev) > 1e-6 and abs(twist) < 1e-6 and not freeze:
                # 拧完手腕：把新姿势记为姿态参考（否则姿态控制器会把它掰回去）
                rpy_ref = np.asarray(S.rpy, float).copy()
                rpy_ref_base1 = float(rpy_ref[1])
                log("[spatial] 手腕自转结束 → 参考姿态已跟随")
            twist_prev = twist

            if was_float and not S.float_mode:
                # 刚退出漂浮：把指令同步到实测，避免跳变
                q_cmd = np.asarray(S.q, float).copy()
                rpy_ref = np.asarray(S.rpy, float).copy()
                rpy_ref_base1 = float(rpy_ref[1])
                q_seed = q_cmd.copy()
                with S.lock:
                    S.p_ref = np.asarray(S.p, float).copy()
                    S.q_cmd = q_cmd.copy()
                log("[float] 退出悬停模式（指令已同步到当前姿态）")

            if align:
                # 重新对齐：把当前姿态/构型记为新参考，笔锚点已在按键时重置
                rpy_ref = np.asarray(joint_to_pose(np.asarray(S.q, float))[1], float).copy()
                rpy_ref_base1 = float(rpy_ref[1])
                q_seed = np.asarray(S.q, float).copy()
                with S.lock:
                    S.p_ref = np.asarray(S.p, float).copy()
                log("[spatial] 已重新对齐（姿态参考 + 构型正则 + 位置保持点）", flush=True)

            if pressed and (now - pen_t) > PEN_TIMEOUT:
                with S.lock:
                    S.pressed = False
                pressed = False

            # ── 0. 姿势回放（教学）：用目标位姿闭环引导，笔一按就取消 ──
            ori_w = args.ori_weight
            rpy_ref_use = rpy_ref
            if replay is not None:
                p_des = np.asarray(replay["p"], float)
                p_now_c = np.asarray(joint_to_pose(q_cmd)[0], float)
                err_p = p_des - p_now_c
                v_cmd = np.clip(REPLAY_KP * err_p, -vmax, vmax)
                twist_cmd = 0.0
                rpy_ref_use = np.asarray(replay["rpy"], float)
                ori_w = max(ori_w, 0.8)
                if float(np.linalg.norm(err_p)) < 0.004 or now > float(replay["t_end"]) or pressed:
                    with S.lock:
                        S.replay = None
                        S.msg = "回放取消（笔按下）" if pressed else "已到预设位姿 ✅"
                    log(f"[spatial] {S.msg}", flush=True)
                    v_cmd = np.zeros(3)

            # 当前位置/姿态（直控参考、安全盒、姿态任务共用）
            p_cur0, rpy_cur0 = joint_to_pose(q_cmd)
            p_cur0 = np.asarray(p_cur0, float)
            rpy_cur0 = np.asarray(rpy_cur0, float)

            # ── 0a. 悬停（漂浮）模式：kp=0 + 重力补偿 → 手可推动、松手停在原地 ──
            if S.float_mode:
                q_fl = np.asarray(arm.get_positions(), float)[:6]
                tau_fl = compute_generalized_gravity(model, pad_q_for_model(model, q_fl, 6), data)[:6]
                g.send_mit(q_fl, vel=np.zeros(6), kp=np.zeros(6), kd=float(args.float_kd),
                           tau=tau_fl)
                n += 1
                if n % 4 == 0:
                    p_fl, rpy_fl = joint_to_pose(q_fl)
                    with S.lock:
                        S.q = q_fl.copy()
                        S.p = np.asarray(p_fl, float).copy()
                        S.rpy = np.asarray(rpy_fl, float).copy()
                        S.q_cmd = q_fl.copy()
                        S.rate_hz = n / max(now - t0, 1e-6)
                        S.status = "悬停（漂浮）中：可以用手推动，松手停在原地"
                    if n % (RATE * 10) == 0:
                        log(f"[float] 悬停中 末端Z={p_fl[2]:+.3f}  "
                            f"q(deg)={np.round(np.degrees(q_fl), 1)}")
                time.sleep(max(0.0, dt_nom - (time.perf_counter() - now)))
                continue

            # ── 0b. 关节直控（选中电机，原始模式）：位置/姿态参考跟随，避免笛卡尔任务拉扯 ──
            j_cmd = 0.0 if freeze else S.j_req
            j_sel = int(S.j_sel)
            if abs(j_cmd) > 1e-6:
                with S.lock:
                    S.p_ref = p_cur0.copy()
                rpy_ref = rpy_cur0.copy()
                rpy_ref_use = rpy_ref
                q_seed = q_cmd.copy()

            # ── 1. 笔/自检 → 空间速度指令（位置模式）/ 角速度指令（姿态模式）──
            vmax = args.vmax * speed
            w_cmd = np.zeros(3)
            if replay is not None:
                pass
            elif st_cmd is not None:
                v_cmd = np.array([st_cmd.get("vx", 0.0), st_cmd.get("vy", 0.0),
                                  st_cmd.get("vz", 0.0)], float)
                twist_cmd = float(st_cmd.get("twist", 0.0))
                w_cmd = np.array([st_cmd.get("wx", 0.0), st_cmd.get("wy", 0.0),
                                  st_cmd.get("wz", 0.0)], float)
            elif pressed and not freeze and mode == "ori":
                dx = pen[0] - anchor[0]
                dy = pen[1] - anchor[1]
                wmax = math.radians(args.wmax_deg)
                # 上下=俯仰(世界Y)  左右=摆头(世界Z)  Shift+上下=侧倾(世界X)
                w_cmd = np.array([vel_from_px(-dy, wmax) if shift else 0.0,
                                  vel_from_px(-dy, wmax) if not shift else 0.0,
                                  vel_from_px(dx, wmax)])
                v_cmd = np.zeros(3)
                twist_cmd = 0.0
            elif pressed and not freeze:
                dx = pen[0] - anchor[0]
                dy = pen[1] - anchor[1]
                if shift:
                    v_cmd = np.array([0.0, vel_from_px(dx, vmax), vel_from_px(-dy, vmax)])
                else:
                    v_cmd = np.array([vel_from_px(-dy, vmax), vel_from_px(dx, vmax), 0.0])
                twist_cmd = twist
            else:
                v_cmd = np.zeros(3)
                twist_cmd = 0.0
            if freeze:
                v_cmd = np.zeros(3)
                w_cmd = np.zeros(3)
                twist_cmd = 0.0

            # ── 2. 安全盒（可用 --no-box 关闭 / --box-scale 放大）──
            if args.box:
                look = 0.25                               # 0.25s 前瞻
                p_next = p_cur0 + v_cmd * look
                cx = 0.5 * (BX[0] + BX[1]); cy = 0.5 * (BY[0] + BY[1]); cz = 0.5 * (BZ[0] + BZ[1])
                hx = 0.5 * (BX[1] - BX[0]) * args.box_scale
                hy = 0.5 * (BY[1] - BY[0]) * args.box_scale
                hz = 0.5 * (BZ[1] - BZ[0]) * args.box_scale
                for k, (c_, h_) in enumerate(((cx, hx), (cy, hy), (cz, hz))):
                    if abs(p_next[k] - c_) > h_:
                        v_cmd[k] = 0.0
                r_next = float(np.hypot(p_next[0], p_next[1]))
                if r_next > RMAX * args.box_scale and (v_cmd[0] or v_cmd[1]):
                    v_cmd[0] = 0.0
                    v_cmd[1] = 0.0

            # ── 3. 雅可比 → 关节速度（阻尼最小二乘 + 零空间）──
            q_pad = pad_q_for_model(model, q_cmd, 6)
            pin.computeJointJacobians(model, data, q_pad)
            J = np.asarray(pin.getFrameJacobian(model, data, fid, pin.LOCAL_WORLD_ALIGNED), float)[:, :6]
            Jv = J[:3, :]
            Jo = J[3:, :]

            # 自适应阻尼（接近奇异位形时加大）
            try:
                smin = float(np.linalg.svd(Jv, compute_uv=False)[-1])
            except np.linalg.LinAlgError:
                smin = 1e-3
            lam = args.damping * (1.0 + 4.0 * max(0.0, 0.06 - smin) / 0.06)
            with S.lock:
                S.damp = lam

            # 零空间：关节限位排斥 + 构型正则 + J6 自转
            z = limit_repulse(q_cmd, lo, hi)
            if args.posture_k > 0:
                z += -args.posture_k * (q_cmd - q_seed)
            if abs(twist_cmd) > 1e-6:
                z[5] += twist_cmd                 # J6 自转（保持末端位置）

            if mode == "ori":
                # ── 姿态模式：主=角速度，次=末端位置保持，末=限位/构型（严格零空间）──
                A_o = Jo @ Jo.T + (lam ** 2) * np.eye(3)
                try:
                    Jo_pinv = Jo.T @ np.linalg.inv(A_o)
                except np.linalg.LinAlgError:
                    Jo_pinv = np.zeros((6, 3))
                qd = Jo_pinv @ w_cmd
                N1 = np.eye(6) - Jo_pinv @ Jo
                with S.lock:
                    p_ref = np.asarray(S.p_ref, float)
                JvN = Jv @ N1
                B_p = JvN @ JvN.T + (lam ** 2) * np.eye(3)
                try:
                    JvN_pinv = JvN.T @ np.linalg.inv(B_p)
                except np.linalg.LinAlgError:
                    JvN_pinv = np.zeros((6, 3))
                v_hold = np.clip(-ORI_POS_K * (p_cur0 - p_ref), -0.08, 0.08)
                qd = qd + JvN_pinv @ (v_hold - Jv @ qd)
                N2 = N1 - JvN_pinv @ JvN
                qd = qd + N2 @ z
            else:
                # ── 位置模式：主=空间速度，次=姿态软保持，末=限位/构型 ──
                A = Jv @ Jv.T + (lam ** 2) * np.eye(3)
                try:
                    Jpinv = Jv.T @ np.linalg.inv(A)
                except np.linalg.LinAlgError:
                    Jpinv = np.zeros((6, 3))
                qd = Jpinv @ v_cmd
                N = np.eye(6) - Jpinv @ Jv
                if ori_w > 0 and abs(twist_cmd) < 1e-6:
                    rpy_cur = np.asarray(joint_to_pose(q_cmd)[1], float)
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
            with S.lock:
                S.stale_ori = bool(np.linalg.norm(
                    [wrap_pi(rpy_ref[i] - joint_to_pose(q_cmd)[1][i]) for i in range(3)]) > 0.5)

            # 直控（原始）：直接叠加到关节速度，不做零空间折扣 → 能走到限位
            if abs(j_cmd) > 1e-6 and j_sel < 6:
                qd[j_sel] += j_cmd

            # ── 4. 关节速度限幅 + 积分 + 软限位硬夹 ──
            qd = np.clip(qd, -QD_MAX, QD_MAX)
            q_new = np.clip(q_cmd + qd * dt, lo, hi)
            q_cmd = q_new.copy()               # ← 累加器放本地：不能再从共享状态重读（否则只走 1/4）

            tau = compute_generalized_gravity(model, pad_q_for_model(model, q_new, 6), data)[:6]
            g.send_mit(q_new, vel=np.zeros(6), kp=kp, kd=kd, tau=tau)

            # ── 夹爪直控（选中第 7 个电机时）+ 力矩保护 ──
            if grip_g is not None and S.grip_ready:
                g_lo, g_hi = S.grip_lim
                if abs(j_cmd) > 1e-6 and j_sel == 6:
                    S.grip_cmd = float(np.clip(S.grip_cmd + math.copysign(GRIP_RATE * dt, j_cmd),
                                               g_lo, g_hi))
                step_g = GRIP_RATE * 1.5 * dt
                S.grip_send = float(np.clip(S.grip_send + float(np.clip(
                    S.grip_cmd - S.grip_send, -step_g, step_g)), g_lo, g_hi))
                grip_g.send_mit(np.array([S.grip_send]), vel=np.zeros(1),
                                kp=np.array([GRIP_KP]), kd=np.array([GRIP_KD]), tau=np.zeros(1))
                if abs(j_cmd) > 1e-6 and j_sel == 6 and abs(S.grip_tau) > GRIP_TAU_LIMIT:
                    with S.lock:
                        S.j_req = 0.0
                        S.grip_cmd = float(S.grip_pos)      # 目标拉回实测 → 不再顶住
                        S.grip_send = float(S.grip_pos)
                        S.grip_msg = (f"⚠️ 到限位/有阻力（{S.grip_tau:+.2f}N·m）已松手，"
                                      f"位置 {math.degrees(S.grip_pos):+.1f}°")
                    log(f"[grip] {S.grip_msg}", force=True)

            n += 1
            if n % 4 == 0:
                pos, vel, torq = (np.asarray(x, float)[:6] for x in arm.get_state())
                p_now, rpy_now = joint_to_pose(pos)
                with S.lock:
                    S.q = pos.copy()
                    S.qd = vel.copy()
                    S.tau = torq.copy()
                    S.p = np.asarray(p_now, float).copy()
                    S.rpy = np.asarray(rpy_now, float).copy()
                    S.q_cmd = q_cmd.copy()      # 仅供显示/收尾使用
                    S.v_cmd = v_cmd.copy()
                    S.qd_cmd = qd.copy()
                    S.rate_hz = n / max(now - t0, 1e-6)
                    S.status = "冻结中（保持悬停）" if S.freeze else "就绪（笔=空间运动）"
                if grip_g is not None:
                    try:
                        S.grip_pos = float(np.asarray(grip_g.get_positions(), float).reshape(-1)[0])
                        S.grip_min = min(S.grip_min, S.grip_pos)
                        S.grip_max = max(S.grip_max, S.grip_pos)
                        gt = np.asarray(grip_g.get_state()[2] if hasattr(grip_g, "get_state")
                                        else [0.0], float).reshape(-1)
                        S.grip_tau = float(gt[0]) if gt.size else 0.0
                    except Exception:  # noqa: BLE001
                        pass
                if args.tau_limit > 0 and not freeze:
                    over = np.where(np.abs(torq) > args.tau_limit)[0]
                    if len(over):
                        with S.lock:
                            S.freeze = True
                            S.alarm = ("⚠️ 力矩超限自动冻结: " +
                                       ", ".join(f"J{i+1}={torq[i]:+.1f}" for i in over))
                        log(f"[tau] {S.alarm}", force=True)

            if n % (RATE * 10) == 0:
                with S.lock:
                    p, fr, jn = S.p.copy(), S.freeze, S.j_sel + 1
                log(f"[spatial] t={now-t0:6.0f}s P=({p[0]:+.3f},{p[1]:+.3f},{p[2]:+.3f}) "
                    f"直控J{jn} {'冻结' if fr else ''}")

            time.sleep(max(0.0, dt_nom - (time.perf_counter() - now)))

        clean = True
    except Exception as e:  # noqa: BLE001
        with S.lock:
            S.error = f"{type(e).__name__}: {e}"
            S.status = "错误（保持悬停）"
        log(f"[spatial] 异常: {S.error} —— 不失能，2 秒后关闭窗口", force=True)
        time.sleep(2.0)
        with S.lock:
            S.quit_req = True
    finally:
        if arm is not None and getattr(arm, "arm", None) is not None and model is not None:
            g = arm.arm
            kp, kd = g._mit_kp.copy(), g._mit_kd.copy()
            try:
                if clean:
                    with S.lock:
                        S.status = "平滑归零中…"
                    log("[spatial] 平滑归零（最小 jerk）…", force=True)
                    steps = int(3.0 * RATE)
                    q_from = np.asarray(S.q_cmd, float).copy()
                    for i in range(1, steps + 1):
                        a = i / steps
                        s = 10 * a**3 - 15 * a**4 + 6 * a**5
                        q_t = q_from * (1.0 - s)
                        tau = compute_generalized_gravity(model, pad_q_for_model(model, q_t, 6), data)[:6]
                        g.send_mit(q_t, vel=np.zeros(6), kp=kp, kd=kd, tau=tau)
                        time.sleep(1.0 / RATE)
                    tau0 = compute_generalized_gravity(model, np.zeros(model.nq), data)[:6]
                    for _ in range(int(0.5 * RATE)):
                        g.send_mit(np.zeros(6), vel=np.zeros(6), kp=kp, kd=kd, tau=tau0)
                        time.sleep(1.0 / RATE)
                    log("[spatial] 已归零并保持悬停（未失能）", force=True)
                else:
                    # 异常退出：不直接松手——先尝试温和归零（带力矩监测），失败才原地保持
                    print("[spatial] 异常退出 → 先温和归零（避免高处掉落）…", flush=True)
                    homed = False
                    try:
                        q_from = np.asarray(S.q_cmd, float).copy()
                        steps = int(6.0 * RATE)
                        for i in range(1, steps + 1):
                            a = i / steps
                            s_ = 10 * a**3 - 15 * a**4 + 6 * a**5
                            q_t = q_from * (1.0 - s_)
                            tau = compute_generalized_gravity(model, pad_q_for_model(model, q_t, 6),
                                                              data)[:6]
                            g.send_mit(q_t, vel=np.zeros(6), kp=kp, kd=kd, tau=tau)
                            time.sleep(1.0 / RATE)
                            if i % 40 == 0:
                                tqm = float(np.max(np.abs(np.asarray(arm.get_state()[2], float)[:6])))
                                if tqm > 25.0:
                                    print(f"[spatial] 归零中力矩 {tqm:.1f}N·m 过大 → 停止归零", flush=True)
                                    break
                        else:
                            homed = True
                            print("[spatial] 已归零并保持（未失能）", flush=True)
                    except Exception as e2:  # noqa: BLE001
                        print(f"[spatial] 异常归零失败: {e2}", flush=True)
                    if not homed:
                        for _ in range(400):
                            try:
                                tau = compute_generalized_gravity(
                                    model, pad_q_for_model(model, np.asarray(S.q_cmd, float), 6), data)[:6]
                                g.send_mit(np.asarray(S.q_cmd, float), vel=np.zeros(6),
                                           kp=kp, kd=kd, tau=tau)
                            except Exception:  # noqa: BLE001
                                break
                            time.sleep(0.005)
            except Exception as e:  # noqa: BLE001
                log(f"[spatial] 收尾发送失败（电机仍保持，不会掉）: {e}", force=True)
        with S.lock:
            S.ready = False
            S.status = "已归零·悬停中（未失能）"


# ── GUI ───────────────────────────────────────────────────────────────────────

root = tk.Tk()
root.title("reBot 空间协调运动 — 笔=移动  键盘=模式")
root.geometry(f"{CW}x{CH}+40+40")
root.configure(bg=BG)
root.attributes("-topmost", True)
cv = tk.Canvas(root, width=CW, height=CH, bg=BG, highlightthickness=0)
cv.pack(fill="both", expand=True)

ROW_Y0, ROW_H = 132, 34
BAR_X0, BAR_X1 = 150, 430

rows = []
for i in range(7):
    y = ROW_Y0 + i * ROW_H
    rows.append({
        "sel": cv.create_text(28, y + 13, anchor="w", text="", fill=ACC, font=("Menlo", 15, "bold")),
        "name": cv.create_text(84, y + 13, anchor="w", text="", fill=FG, font=("Menlo", 13)),
        "bar_bg": cv.create_rectangle(BAR_X0, y + 3, BAR_X1, y + 19, outline="#334155", fill="#111a2e"),
        "bar": cv.create_rectangle(BAR_X0, y + 4, BAR_X0, y + 18, outline="", fill=OKC),
        "txt": cv.create_text(BAR_X1 + 16, y + 11, anchor="w", text="", fill=FG, font=("Menlo", 12)),
    })

title = cv.create_text(24, 26, anchor="w", text="", fill=FG, font=("Menlo", 16, "bold"))
l2 = cv.create_text(24, 56, anchor="w", text="", fill=DIM, font=("Menlo", 12))
l3 = cv.create_text(24, 80, anchor="w", text="", fill=DIM, font=("Menlo", 12))
l4 = cv.create_text(24, 104, anchor="w", text="", fill=DIM, font=("Menlo", 12))
hints = cv.create_text(24, 530, anchor="nw", fill=DIM, font=("Menlo", 12),
                       text=("  笔：按住拖动 = 移动（松开即停）        Shift + 上下 = 升 / 降\n"
                             "  关节直控：点 J1~J6 / 夹爪 选电机，按住 Q 正转 / A 反转   ·   J6 自转：, / .\n"
                             "  模式：O（位置/姿态）   速度：F   冻结：Space   退出：Esc 两下   （按钮也可以直接用笔点）"))
status = cv.create_text(24, CH - 20, anchor="w", text="", fill=OKC, font=("Menlo", 13, "bold"))

# ── 屏幕按钮（用笔点，不用键盘）──
BUTTONS: list[tuple] = []
BTN_FILL, BTN_FILL_ON = "#1b2742", "#1d4ed8"


HELD: dict = {"btn": None}


def add_button(tag: str, x0: float, y0: float, x1: float, y1: float, label: str,
               action=None, down=None, up=None) -> None:
    r = cv.create_rectangle(x0, y0, x1, y1, fill=BTN_FILL, outline="#2b3a5c", width=2)
    cv.create_text((x0 + x1) / 2, (y0 + y1) / 2, text=label, fill=FG,
                   font=("Menlo", 13, "bold"))
    BUTTONS.append({"tag": tag, "box": (x0, y0, x1, y1), "rect": r,
                    "action": action, "down": down, "up": up})


def hit_button(x: float, y: float) -> bool:
    for b in BUTTONS:
        x0, y0, x1, y1 = b["box"]
        if x0 <= x <= x1 and y0 <= y <= y1:
            if b["down"] is not None:          # 按住型（俯仰）
                b["down"]()
                HELD["btn"] = b
            elif b["action"] is not None:
                b["action"]()
            return True
    return False


banner = cv.create_rectangle(0, 0, 0, 0, fill="#7f1d1d", outline=RED, width=3, state="hidden")
banner_t = cv.create_text(CW / 2, 0, text="", fill="#fecaca", font=("Menlo", 19, "bold"), state="hidden")


def draw() -> None:
    with S.lock:
        q = S.q.copy(); qd = S.qd.copy(); tau = S.tau.copy(); qcm = S.q_cmd.copy()
        p = S.p.copy(); v = S.v_cmd.copy(); qd_cmd = S.qd_cmd.copy()
        frz = S.freeze; st = S.status; hz = S.rate_hz; err = S.error
        preset = SPEED_PRESETS[S.speed_i][0]; speed = SPEED_PRESETS[S.speed_i][1]
        pressed = S.pressed; pen = S.pen; anchor = S.anchor; shift = S.shift
        twist = S.twist; lam = S.damp; alarm = S.alarm
        mode = S.mode; msg = S.msg; replaying = S.replay is not None; float_mode = S.float_mode
        j4_req = S.j_req; j_sel = S.j_sel; j_rate = S.j_rng; rpy_now = S.rpy.copy()
        grip_ready = S.grip_ready; grip_pos = S.grip_pos; grip_cmd = S.grip_cmd
        grip_tau = S.grip_tau; grip_lim = S.grip_lim; grip_msg = S.grip_msg
        grip_min = S.grip_min; grip_max = S.grip_max
    names = ["J1", "J2", "J3", "J4", "J5", "J6", "夹爪"]
    for i, r in enumerate(rows):
        if i < 6:
            lo, hi = LIM_LO[i], LIM_HI[i]
            cur, tg = q[i], qcm[i]
            marg = min(cur - lo, hi - cur)
            barf = (cur - lo) / max(hi - lo, 1e-6)
            warn = marg < 0.12
            txt = (f"{math.degrees(cur):+7.2f}°  目标{math.degrees(tg):+7.2f}°  "
                   f"v{qd[i]:+5.2f}  ω_cmd{qd_cmd[i]:+5.2f}  τ{tau[i]:+5.2f}  余量{math.degrees(marg):+5.1f}°")
        elif grip_ready:
            lo, hi = grip_lim
            cur, tg = grip_pos, grip_cmd
            marg = min(cur - lo, hi - cur)
            barf = (cur - lo) / max(hi - lo, 1e-6)
            warn = False
            txt = (f"{math.degrees(cur):+7.2f}°  目标{math.degrees(tg):+7.2f}°  τ{grip_tau:+5.2f}  "
                   f"已探到 {math.degrees(grip_min):+.0f}°~{math.degrees(grip_max):+.0f}°  {grip_msg}")
        else:
            for v_ in r.values():
                cv.itemconfig(v_, state="hidden")
            continue
        f = min(max(barf, 0.0), 1.0)
        cv.coords(r["bar"], BAR_X0, ROW_Y0 + i * ROW_H + 4,
                  BAR_X0 + f * (BAR_X1 - BAR_X0), ROW_Y0 + i * ROW_H + 18)
        cv.itemconfig(r["bar"], fill=(RED if frz else (WARN if warn else OKC)))
        cv.itemconfig(r["name"], text=names[i], fill=(ACC if i == j_sel else FG))
        cv.itemconfig(r["sel"], text="▶" if i == j_sel else "")
        cv.itemconfig(r["txt"], text=txt, fill=(WARN if i == j_sel else FG))

    cv.itemconfig(title, text=f"🦾 空间协调运动    {'🕊 悬停(漂浮)' if float_mode else ('位置模式' if mode == 'pos' else '姿态模式')}    "
                              f"{preset}速 {VMAX*speed*100:.0f}cm/s    "
                              f"{'❄ 冻结' if frz else '▶ 运行'}    {hz:.0f}Hz")
    cv.itemconfig(l2, text=f"末端    X {p[0]:+.3f}    Y {p[1]:+.3f}    Z {p[2]:+.3f} m"
                           f"        半径 {math.hypot(p[0],p[1]):.3f} m")
    cv.itemconfig(l3, text=f"速度    X {v[0]*100:+6.2f}    Y {v[1]*100:+6.2f}    Z {v[2]*100:+6.2f} cm/s"
                           f"        阻尼 {lam:.3f}")
    cv.itemconfig(status, text=st + (f"   ❌ {err}" if err else ""), fill=(RED if err else OKC))
    cv.itemconfig(l4, text=f"直控 J{j_sel+1}  {'▶ 正转中' if j4_req > 0 else ('▶ 反转中' if j4_req < 0 else '待命')}"
                           f"      J6 自转: {(', / .' if not twist else ('正转' if twist > 0 else '反转'))}"
                           f"      工具姿态: rpy=({math.degrees(rpy_now[0]):+5.1f},"
                           f"{math.degrees(rpy_now[1]):+5.1f},{math.degrees(rpy_now[2]):+5.1f})°   {msg}")
    for b in BUTTONS:
        on = ((b["tag"] == "mode_pos" and mode == "pos")
              or (b["tag"] == "mode_ori" and mode == "ori")
              or (b["tag"] == "float" and S.float_mode)
              or (b["tag"] == "freeze" and frz))
        if b["tag"].startswith("jsel"):
            on = on or (b["tag"] == f"jsel{j_sel}")
        cv.itemconfig(b["rect"], fill=(BTN_FILL_ON if on else BTN_FILL))
    if frz or alarm:
        cv.coords(banner, 0, CH / 2 - 40, CW, CH / 2 + 40)
        cv.itemconfig(banner, state="normal")
        cv.coords(banner_t, CW / 2, CH / 2)
        cv.itemconfig(banner_t, text=alarm or "❄ 冻结中 —— 按 Space / 笔侧键解除", state="normal")
    else:
        cv.itemconfig(banner, state="hidden")
        cv.itemconfig(banner_t, state="hidden")
    if S.quit_req:
        quit_app()
        return
    root.after(50, draw)


LIM_LO = np.array([-2.80, 0.0, 0.0, -1.57, -1.57, -3.14])
LIM_HI = np.array([2.80, 3.14, 3.14, 1.57, 1.57, 3.14])


# ── 事件 ──────────────────────────────────────────────────────────────────────


def set_mode(m: str) -> None:
    with S.lock:
        S.mode = m
        S.p_ref = np.asarray(S.p, float).copy()
        S.anchor = S.pen
        S.msg = ("姿态模式：笔上下=俯仰 左右=摆头 Shift+上下=侧倾"
                 if m == "ori" else "位置模式：笔上下=前后 左右=横移 Shift+上下=升降")
    log(f"[btn] {S.msg}", flush=True)


def build_buttons() -> None:
    y1a, y1b = 388, 426
    y2a, y2b = 434, 472
    w1, gap, x = 172, 10, 24
    add_button("mode_pos", x, y1a, x + w1, y1b, "位置模式", lambda: set_mode("pos"))
    x += w1 + gap
    add_button("mode_ori", x, y1a, x + w1, y1b, "姿态模式", lambda: set_mode("ori"))
    x += w1 + gap
    add_button("float", x, y1a, x + w1, y1b, "悬停模式", toggle_float)
    x += w1 + gap
    add_button("freeze", x, y1a, x + w1, y1b, "冻结 / 解冻", toggle_freeze)
    x += w1 + gap
    add_button("speed", x, y1a, x + w1, y1b, "速度档", cycle_speed)
    w2, gap2, x = 126, 6, 24
    for i in range(7):
        add_button(f"jsel{i}", x, y2a, x + w2, y2b, (f"J{i+1}" if i < 6 else "夹爪"),
                   action=lambda i=i: _select_joint(i))
        x += w2 + gap2


def toggle_float() -> None:
    with S.lock:
        S.float_mode = not S.float_mode
        v = S.float_mode
    log(f"[float] 悬停模式 {'ON —— 可以用手推动机械臂，松手停在原地' if v else 'off —— 恢复位置控制'}"
        + ("  ⚠️ 注意：此时位置任务暂停，笔暂时无效" if v else ""), force=True)


def _select_joint(i: int) -> None:
    with S.lock:
        S.j_sel = i
    log(f"[btn] 直控目标 → J{i+1}", flush=True)


def cycle_speed() -> None:
    with S.lock:
        S.speed_i = (S.speed_i + 1) % len(SPEED_PRESETS)
        n = SPEED_PRESETS[S.speed_i][0]
    log(f"[btn] 速度档 → {n}", flush=True)


def on_press(e) -> None:
    if e.num == 3:
        toggle_freeze()
        return
    if hit_button(float(e.x), float(e.y)):     # 点在按钮上 → 执行动作，不当拖动
        return
    with S.lock:
        S.pressed = True
        S.pen = (float(e.x), float(e.y))
        S.anchor = (float(e.x), float(e.y))
        S.pen_t = time.monotonic()
    log(f"[pen] 按下 x={e.x} y={e.y}", flush=True)


def on_release(e) -> None:
    if HELD["btn"] is not None:
        up = HELD["btn"].get("up")
        HELD["btn"] = None
        if up is not None:
            up()
        return
    with S.lock:
        S.pressed = False
    log("[pen] 松开（停）", flush=True)


def on_motion(e) -> None:
    with S.lock:
        S.pen = (float(e.x), float(e.y))
        if S.pressed:
            S.pen_t = time.monotonic()



_DIGIT = {"1": 1, "2": 2, "3": 3, "4": 4, "exclam": 1, "at": 2, "numbersign": 3,
          "dollar": 4}


def on_key_press(e) -> None:
    k = e.keysym
    if k == "Escape":
        now_ = time.monotonic()
        with S.lock:
            first = not (now_ - S.esc_t < 3.0)
            S.esc_t = now_
            S.msg = "再按一次 Esc 确认退出（3 秒内）" if first else "退出中…"
        log(f"[key] {S.msg}", flush=True)
        if not first:
            quit_app()
    elif k.lower() == "f":
        with S.lock:
            S.speed_i = (S.speed_i + 1) % len(SPEED_PRESETS)
            n = SPEED_PRESETS[S.speed_i][0]
        log(f"[key] 速度档 → {n}", flush=True)
    elif k == "space":
        toggle_freeze()
    elif k.lower() == "r":
        with S.lock:
            S.anchor = S.pen
            S.align_req = True
        log("[key] 重新对齐（姿态参考 + 笔锚点）", flush=True)
    elif k.lower() == "h":
        toggle_float()
    elif k.lower() == "o":
        with S.lock:
            S.mode = "ori" if S.mode == "pos" else "pos"
            S.p_ref = np.asarray(S.p, float).copy()
            S.anchor = S.pen
            m = S.mode
            S.msg = "姿态模式：笔上下=俯仰 左右=摆头 Shift+上下=侧倾" if m == "ori" else "位置模式"
        log(f"[key] {S.msg}")
    elif k in _DIGIT:
        i = _DIGIT[k]
        if e.state & 0x0001:            # Shift 按住 = 记录（macOS 会给出 exclam 等）
            record_preset(i)
        else:
            goto_preset(i)
    elif k.lower() == "q":
        with S.lock:
            S.j_req = S.j_rng
    elif k.lower() == "a":
        with S.lock:
            S.j_req = -S.j_rng
    elif k in ("comma", "less", "greater", "period"):
        with S.lock:
            S.twist = -TWIST_RATE if k in ("comma", "less") else TWIST_RATE
    else:
        with S.lock:
            if k in ("Shift_L", "Shift_R"):
                S.shift = True


def on_key_release(e) -> None:
    k = e.keysym
    if k.lower() in ("q", "a"):
        with S.lock:
            S.j_req = 0.0
        return
    if k in ("comma", "less", "greater", "period"):
        with S.lock:
            S.twist = 0.0
    elif k in ("Shift_L", "Shift_R"):
        with S.lock:
            S.shift = False


_quitting = False


def toggle_freeze() -> None:
    with S.lock:
        S.freeze = not S.freeze
        if S.freeze:
            S.pressed = False
            S.alarm = ""
        v = S.freeze
    log(f"[key] 冻结 {'ON' if v else 'off'}", force=True)


def quit_app() -> None:
    global _quitting
    if _quitting:
        return
    _quitting = True
    S.stop = True
    try:
        root.destroy()
    except Exception:  # noqa: BLE001
        pass


# ── 自检：脚本化空间运动，验证"协调 + 精度 + 安全盒"──────────────────────────


def selftest(args) -> None:
    def ee():
        with S.lock:
            return S.p.copy()

    def run(name, secs, **v):
        p0 = ee()
        with S.lock:
            q0_ = S.q.copy()
            qc0 = S.q_cmd.copy()
            rpy0_ = S.rpy.copy()
        t_end = time.time() + secs
        while time.time() < t_end:
            with S.lock:
                S.selftest_cmd = v
            time.sleep(0.02)
        with S.lock:
            S.selftest_cmd = None
        time.sleep(0.35)
        p1 = ee()
        with S.lock:
            q1_ = S.q.copy()
            qc1 = S.q_cmd.copy()
        d = (p1 - p0) * 100
        with S.lock:
            rpy1_ = S.rpy.copy()
        dq = np.degrees(q1_ - q0_)
        dqc = np.degrees(qc1 - qc0)
        drpy = np.degrees(rpy1_ - rpy0_)
        print(f"[selftest] {name:14s} Δ=({d[0]:+6.2f},{d[1]:+6.2f},{d[2]:+6.2f}) cm   "
              f"Δ姿态(r,p,y)=({drpy[0]:+5.1f},{drpy[1]:+5.1f},{drpy[2]:+5.1f})°", flush=True)
        print(f"            指令关节Δ=[{dqc[0]:+6.1f} {dqc[1]:+6.1f} {dqc[2]:+6.1f} {dqc[3]:+6.1f} "
              f"{dqc[4]:+6.1f} {dqc[5]:+6.1f}]  实测关节Δ=[{dq[0]:+6.1f} {dq[1]:+6.1f} "
              f"{dq[2]:+6.1f} {dq[3]:+6.1f} {dq[4]:+6.1f} {dq[5]:+6.1f}]  (deg)", flush=True)

    time.sleep(1.0)
    print("[selftest] 空间协调运动测试开始", flush=True)
    run("前伸 +3cm", 1.6, vx=0.02)
    run("抬升 +2cm", 1.5, vz=0.015)
    run("左移 +3cm", 1.6, vy=0.02)
    run("右移 -3cm", 1.6, vy=-0.02)
    run("下降 -2cm", 1.5, vz=-0.015)
    run("后收 -3cm", 1.6, vx=-0.02)
    run("J6 自转 +30°", 1.0, twist=math.radians(30))

    # ── 独立俯仰测试（J4：抬头/低头，末端位置应保持）──
    with S.lock:
        S.j_req = math.radians(20.0)
    run("J4 直控 抬头 2s", 2.0)
    with S.lock:
        S.j_req = math.radians(-20.0)
    run("J4 直控 低头 2s", 2.0)
    with S.lock:
        S.j_req = 0.0

    # ── 姿态模式测试 ──
    with S.lock:
        S.mode = "ori"
        S.p_ref = np.asarray(S.p, float).copy()
    print("[selftest] 切到姿态模式（末端位置应保持不动）", flush=True)
    run("姿态: 俯仰", 2.0, wy=-0.12)
    run("姿态: 摆头", 2.0, wz=0.12)
    with S.lock:
        S.mode = "pos"

    # ── 姿势预设：记录 → 移开 → 一键回位 ──
    time.sleep(0.5)
    record_preset(1)
    run("移开 +5cm", 2.5, vx=0.02)
    with S.lock:
        p_before = S.p.copy()
    goto_preset(1)
    t_end = time.time() + 25.0
    while time.time() < t_end:
        with S.lock:
            if S.replay is None:
                break
        time.sleep(0.05)
    time.sleep(1.5)
    with S.lock:
        p_after = S.p.copy()
    with S.lock:
        d = S.presets.get("1", {})
    p_tgt = np.array(d.get("p", [0, 0, 0]), float)
    err = np.linalg.norm(p_after - p_tgt) * 100
    print(f"[selftest] 预设回位: 目标={np.round(p_tgt,3)} 实际={np.round(p_after,3)} "
          f"误差={err:.2f}cm（移开时在 {np.round(p_before,3)}）", flush=True)

    # ── 夹爪测试 ──
    if S.grip_ready:
        with S.lock:
            S.j_sel = 6
            S.j_req = 1.0
        p0g = S.grip_pos
        time.sleep(1.5)
        with S.lock:
            S.j_req = 0.0
        time.sleep(0.6)
        p1g = S.grip_pos
        with S.lock:
            S.j_req = -1.0
        time.sleep(1.5)
        with S.lock:
            S.j_req = 0.0
        time.sleep(0.6)
        p2g = S.grip_pos
        print(f"[selftest] 夹爪 正向={math.degrees(p1g-p0g):+.2f}°  反向回={math.degrees(p2g-p1g):+.2f}°  "
              f"力矩={S.grip_tau:+.2f}N·m  范围={tuple(round(math.degrees(x),1) for x in S.grip_lim)}",
              flush=True)
        with S.lock:
            S.j_sel = 3

    print("[selftest] 完成，3 秒后退出（会平滑归零）", flush=True)
    time.sleep(3.0)
    with S.lock:
        S.quit_req = True            # 交给主线程退出（Tk 不能跨线程碰）


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="数位板空间协调运动（速度级雅可比伺服）")
    ap.add_argument("--vmax", type=float, default=VMAX, help="满速 m/s")
    ap.add_argument("--pen-range", type=float, default=TRAVEL_PX, help="笔偏离多少像素=满速")
    ap.add_argument("--dead", type=float, default=DEAD_PX, help="死区像素")
    ap.add_argument("--sweep-deg", type=float, default=45.0, help="横向满窗拖动 = 多少度（粗调）")
    ap.add_argument("--fine-factor", type=float, default=10.0, help="细调倍数")
    ap.add_argument("--damping", type=float, default=DAMPING, help="阻尼最小二乘 λ0")
    ap.add_argument("--ori-weight", type=float, default=0.0, help="姿态软保持权重（0=不管，默认）")
    ap.add_argument("--posture-k", type=float, default=0.2, help="零空间构型正则增益（0=关）")
    ap.add_argument("--wmax-deg", type=float, default=30.0, help="姿态模式角速度上限 °/s")
    ap.add_argument("--j-rate", type=float, default=60.0, help="关节直控速度 °/s（按住时）")
    ap.add_argument("--limit-margin", type=float, default=1.5, help="软限位余量（度，0=顶到机械限位）")
    ap.add_argument("--box", action=argparse.BooleanOptionalAction, default=True,
                    help="笛卡尔安全盒（--no-box 完全关闭）")
    ap.add_argument("--box-scale", type=float, default=1.0, help="安全盒放大倍数")
    ap.add_argument("--float-kd", type=float, default=2.5, help="悬停(漂浮)模式的阻尼 kd")
    ap.add_argument("--tau-limit", type=float, default=0.0, help="力矩保护 N·m（0=关）")
    ap.add_argument("--verbose", action="store_true", help="终端输出全部细节（默认只显示重要信息）")
    ap.add_argument("--gripper", action=argparse.BooleanOptionalAction, default=True,
                    help="启用夹爪（第 7 个电机，默认启用）")
    ap.add_argument("--grip-range", type=float, default=GRIP_RANGE, help="夹爪活动半径 rad（相对模式）")
    ap.add_argument("--grip-abs", action=argparse.BooleanOptionalAction, default=True,
                    help="用实测绝对限位（默认开）：3° ~ 328°")
    ap.add_argument("--grip-lo", type=float, default=3.0, help="绝对下限（度）")
    ap.add_argument("--grip-hi", type=float, default=328.0, help="绝对上限（度）")
    ap.add_argument("--grip-rate", type=float, default=GRIP_RATE, help="夹爪速度 rad/s")
    ap.add_argument("--selftest", action="store_true", help="自检：脚本化空间运动")
    ap.add_argument("--dump-layout", action="store_true", help="打印界面元素坐标后退出（排版检查）")
    args = ap.parse_args()
    VERBOSE = bool(args.verbose)   # 模块级变量，无需 global
    J4_RATE = math.radians(args.j_rate)

    TRAVEL_PX = args.pen_range
    DEAD_PX = args.dead
    GRIP_RANGE = args.grip_range
    GRIP_RATE = args.grip_rate
    GRIP_LO = math.radians(args.grip_lo)
    GRIP_HI = math.radians(args.grip_hi)
    with S.lock:
        S.j_rng = math.radians(args.j_rate)
        S.sweep_deg = args.sweep_deg
        S.fine_factor = args.fine_factor
        S.margin_deg = args.limit_margin

    signal.signal(signal.SIGINT, lambda *_: quit_app())
    signal.signal(signal.SIGTERM, lambda *_: quit_app())   # pkill/kill 也能走平滑归零
    S.presets = load_presets()
    if S.presets:
        print(f"[preset] 已载入槽位: {sorted(S.presets.keys())}（{PRESET_FILE}）", flush=True)

    build_buttons()
    cv.bind("<ButtonPress>", on_press)
    cv.bind("<ButtonRelease>", on_release)
    cv.bind("<Motion>", on_motion)
    root.bind("<KeyPress>", on_key_press)
    root.bind("<KeyRelease>", on_key_release)
    root.protocol("WM_DELETE_WINDOW", quit_app)
    root.focus_force()

    th = threading.Thread(target=control_loop, args=(args,), daemon=True)
    th.start()
    if args.selftest:
        threading.Thread(target=selftest, args=(args,), daemon=True).start()

    log("─" * 56, force=True)
    log("  机械臂已就绪（全程悬停，不会失能）", force=True)
    log("  笔   : 按住拖动 = 按当前模式移动（松开即停）", force=True)
    log("  关节 : 点 J1~J6（或夹爪）选电机 → 按住 Q 正转 / A 反转", force=True)
    log("  J6自转: , / .    模式: O    速度: F    冻结: Space    退出: Esc 两下", force=True)
    log("─" * 56, force=True)
    root.after(100, draw)

    if args.dump_layout:
        def dump() -> None:
            print("── 画布元素（y 坐标从上往下）──", flush=True)
            items = []
            for it in cv.find_all():
                c = cv.coords(it)
                if len(c) >= 4:
                    x0, y0 = min(c[0], c[2]), min(c[1], c[3])
                    x1, y1 = max(c[0], c[2]), max(c[1], c[3])
                elif len(c) == 2:
                    x0 = x1 = c[0]; y0 = y1 = c[1]
                else:
                    continue
                txt = cv.itemcget(it, "text")
                items.append((y0, x0, x1, y1, cv.type(it), txt))
            for y0, x0, x1, y1, t, txt in sorted(items):
                print(f"  y={y0:6.1f}..{y1:6.1f}  x={x0:6.1f}..{x1:6.1f}  {t:9s} {txt[:60]}", flush=True)
            print("── 按钮清单 ──", flush=True)
            for b in BUTTONS:
                x0, y0, x1, y1 = b["box"]
                print(f"  {b['tag']:9s} x={x0:6.1f}..{x1:6.1f} y={y0:6.1f}..{y1:6.1f}", flush=True)
            root.quit()
        root.after(900, dump)

    root.mainloop()
    th.join(timeout=15.0)
    print(f"[spatial] 结束 status={S.status} error={S.error}", flush=True)
