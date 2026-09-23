#!/usr/bin/env python3
"""hybrid_teleop.py — 右手数位板 + 左手手势 联合遥操作

分工（已与用户确认）:
    右手数位板  →  前后深度 X（笔上下拖动）+ 左右 Y 大范围（笔左右拖动）
                    Shift+上下 = 直接调整高度 Z（应急/微调）
    左手手势    →  上下 Z（掌心相对位置）+ 左右 Y 微调（掌心左右，±8cm）
                    ✊ 握拳 = 夹爪夹紧 / ✋ 张开 = 夹爪松开 / 🤏 捏合 = 全局冻结
    手机姿态    →  前后倾 = 高度（速度式）+ 左右倾 = J6 自转（速度式）
                    可校准回中 / 反向 / 手机上一键冻结（--phone）

数据来源:  tcp://127.0.0.1:9100  (cam_server.py, 只取左手)
          http://<本机IP>:9210/ (phone_ctrl.py, 手机浏览器)
安全: 全程悬停；退出时平滑归零 + 不断力矩；工作空间/限速/关节限位统一在最后执行。
"""
from __future__ import annotations

import argparse
import json
import math
import signal
import socket
import sys
import threading
import time
import tkinter as tk

import numpy as np

REPO = "/Users/Admin/Desktop/reBotArm_control_py"
sys.path.insert(0, REPO)

from reBotArm_control_py.actuator import RebotArm  # noqa: E402
from reBotArm_control_py.dynamics import compute_generalized_gravity, load_dynamics_model  # noqa: E402
from reBotArm_control_py.kinematics import (  # noqa: E402
    get_end_effector_frame_id,
    joint_to_pose,
    load_robot_model,
    pad_q_for_model,
    pos_rot_to_se3,
)
from reBotArm_control_py.kinematics.inverse_kinematics import IKParams, solve_ik  # noqa: E402
from palm_safety import PalmSafetyGate  # noqa: E402

# ── 配置 ──────────────────────────────────────────────────────────────────────
RATE = 200
SLEW = 1.2
PALM_JOINT_SLEW = 0.45     # 手势控制时关节最大变化速度 (rad/s)
SCALE_PEN = 0.00105        # m/px
Z_GAIN = 0.42              # 手心纵向位移 → 目标高度 (m / 归一化画面高度)
Z_OFFSET_MAX = 0.06        # 单次锚定允许的最大高度偏移 (m)
Z_DEAD_PY = 0.015          # 手心纵向死区（归一化画面高度）
Z_TAU = 0.18               # 高度偏移低通时间常数 (s)
Z_RATE_MAX = 0.03          # 高度目标最大变化速度 (m/s)
SCALE_REF = 0.13           # 参考手部尺寸（远近灵敏度补偿基准）
Y_GAIN = 0.35              # 手势 px → 左右微调 (m / 归一化单位)
Y_FINE_MAX = 0.05          # 左右微调上限 ±5cm
Y_RATE_MAX = 0.03          # 左右微调最大变化速度 (m/s)
G_TAU = 0.25               # 左右微调低通 (s)
Y_DEAD = 0.004
FRAME_TIMEOUT = 0.30       # 相机连续停帧超过此时间，立即停止手势位移 (s)
GRIP_KP = 7.0
GRIP_SLEW = 1.2
CTRL_HAND = "Left"

# ── 手机姿态遥控（--phone）────────────────────────────────────────────────────
YAW_OFF_MAX = math.radians(120.0)  # J6 自转偏置上限 ±120°
PHONE_STALE = 0.5                  # 手机数据超过这个时间没更新 → 速度归零 (s)
PHONE_DZ_MAX = 0.02                # 手机高度速度的单帧变化上限 (m)
PHONE_JOINT_SLEW = 0.8             # 手机控制时关节最大变化速度 (rad/s)

Q_READY = np.array([0.0, 0.6, 0.9, 0.0, 0.0, 0.0])
BX = (0.14, 0.46)
BY = (-0.30, 0.30)
BZ = (0.16, 0.52)
RMAX = 0.48
JMIN = np.array([-2.80, 0.02, 0.02, -1.55, -1.55, -3.10])
JMAX = np.array([2.80, 3.10, 3.10, 1.55, 1.55, 3.10])


def clamp_ws(p: np.ndarray) -> np.ndarray:
    p = np.array(p, float)
    p[0] = min(max(p[0], BX[0]), BX[1])
    p[1] = min(max(p[1], BY[0]), BY[1])
    p[2] = min(max(p[2], BZ[0]), BZ[1])
    r = float(np.hypot(p[0], p[1]))
    if r > RMAX:
        k = RMAX / r
        p[0] *= k
        p[1] *= k
    return p


class State:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.pen_xy = np.zeros(2)      # 数位板给出的 X(深度)/Y(左右) 目标
        self.z_abs = 0.0               # 绝对高度目标
        self.z_base = 0.0              # 手势高度锚定基准
        self.y_fine = 0.0              # 手势左右微调
        self.target = np.zeros(3)
        self.ee = np.zeros(3)
        self.q = np.zeros(6)
        self.q_ref = np.zeros(6)
        self.status = "连接中…"
        self.ik = "—"
        self.engaged = False
        self.freeze = True           # 启动后需 Space 解锁
        self.ready = False
        self.ready_req = 0.0
        self.rate_hz = 0.0
        self.stop = False
        self.error: str | None = None
        # 手势
        self.g_connected = False
        self.g_left = False
        self.g_gesture = "none"
        self.g_px = 0.5
        self.g_py = 0.5
        self.g_fps = 0.0
        self.g_t = 0.0
        self.g_frame_age = float("inf")
        self.g_handed = "—"
        self.g_nhands = 0
        self.g_scale = SCALE_REF
        self.g_anchor: tuple[float, float] | None = None
        self.g_armed = False
        # 夹爪
        self.grip = 0.0
        self.grip_ready = False
        self.grip_open = 0.0
        self.grip_close = 0.0
        # 手机姿态遥控
        self.ph_on = False
        self.ph_url = ""
        self.ph_connected = False
        self.ph_age = float("inf")
        self.ph_beta = 0.0
        self.ph_gamma = 0.0
        self.ph_z_vel = 0.0
        self.ph_yaw_vel = 0.0
        self.ph_n = 0
        self.yaw_off = 0.0            # J6 自转偏置 (rad)


S = State()


def gesture_client(port: int) -> None:
    while not S.stop:
        sock = None
        last_seq = None
        try:
            sock = socket.create_connection(("127.0.0.1", port), timeout=2.0)
            sock.settimeout(0.2)
            print(f"[gesture] 已连接 cam_server 127.0.0.1:{port}", flush=True)
            with S.lock:
                S.g_connected = True
            buf = b""
            while not S.stop:
                try:
                    chunk = sock.recv(8192)
                except socket.timeout:
                    with S.lock:
                        if time.monotonic() - S.g_t > FRAME_TIMEOUT:
                            S.g_left = False
                            S.g_gesture = "none"
                    continue
                if not chunk:
                    break
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    try:
                        d = json.loads(line.decode())
                    except Exception:  # noqa: BLE001
                        continue
                    seq = d.get("seq", d.get("t"))
                    if seq is None or seq == last_seq:
                        with S.lock:
                            if time.monotonic() - S.g_t > FRAME_TIMEOUT:
                                S.g_left = False
                                S.g_gesture = "none"
                        continue
                    last_seq = seq
                    try:
                        frame_age = time.time() - float(d["t"])
                    except (KeyError, TypeError, ValueError):
                        continue
                    if not 0 <= frame_age <= FRAME_TIMEOUT:
                        with S.lock:
                            S.g_left = False
                            S.g_gesture = "none"
                            S.g_frame_age = frame_age
                        continue
                    hands = d.get("hands") or []
                    ctrl = next((x for x in hands if x.get("handedness") == CTRL_HAND), None)
                    with S.lock:
                        S.g_nhands = len(hands)
                        S.g_fps = float(d.get("fps", 0.0))
                        S.g_t = time.monotonic()
                        S.g_frame_age = frame_age
                        if ctrl is not None:
                            S.g_left = True
                            S.g_gesture = str(ctrl.get("gesture", "none"))
                            S.g_px = float(ctrl.get("px", 0.5))
                            S.g_py = float(ctrl.get("py", 0.5))
                            S.g_scale = float(ctrl.get("scale", SCALE_REF))
                            S.g_handed = CTRL_HAND
                        else:
                            S.g_left = False
                            S.g_gesture = "none"
        except OSError as e:
            with S.lock:
                S.g_connected = False
                S.g_left = False
            if not S.stop:
                print(f"[gesture] 未连接/断开: {e}（2s 后重试）", flush=True)
                time.sleep(2.0)
        finally:
            with S.lock:
                S.g_connected = False
                S.g_left = False
                S.g_gesture = "none"
                S.g_nhands = 0
            if sock is not None:
                try:
                    sock.close()
                except Exception:  # noqa: BLE001
                    pass


def control_loop(args, ph_state=None) -> None:
    arm = None
    g = None
    grip_g = None
    clean = False
    q_cmd = np.zeros(6)
    grip_cmd = 0.0
    model = None
    try:
        arm = RebotArm()
        arm.connect()
        g = arm.arm
        q_start = np.asarray(g.get_positions(), float)[:6]
        q_cmd = q_start.copy()
        if float(np.max(np.abs(q_start))) > 0.05:
            g = None
            raise RuntimeError("启动姿态距折叠零位超过 0.05 rad；拒绝使能"
                               "（不 disconnect / 不失能，机械臂维持原状态）")
        model = load_robot_model()
        data = model.createData()
        fid = get_end_effector_frame_id(model)
        kp, kd = g._mit_kp.copy(), g._mit_kd.copy()
        g.mode_mit(kp=kp, kd=kd)
        g.enable()
        tau_hold = compute_generalized_gravity(model, pad_q_for_model(model, q_start, 6), data)[:6]
        g.send_mit(q_start, vel=np.zeros(6), kp=kp, kd=kd, tau=tau_hold)

        if args.gripper:
            try:
                grip_g = arm.gripper
                grip_g.mode_mit()
                grip_g.enable()
                grip_cmd = float(np.asarray(grip_g.get_positions(), float).reshape(-1)[0])
                S.grip_open = grip_cmd + args.grip_range
                S.grip_close = grip_cmd - args.grip_range * 0.5
                S.grip_ready = True
                print(f"[grip] 使能, 当前={grip_cmd:+.3f} 开={S.grip_open:+.3f} 合={S.grip_close:+.3f}", flush=True)
            except Exception as e:  # noqa: BLE001
                print(f"[grip] 不可用: {e}", flush=True)
                grip_g = None

        q_meas = np.asarray(g.get_positions(), float)[:6]
        p, rpy_lock = joint_to_pose(np.asarray(arm.get_state()[0], float))
        rpy_lock = np.asarray(rpy_lock, float)
        q_cmd = q_meas.copy()
        with S.lock:
            S.q = q_meas.copy()
            S.ee = np.asarray(p, float)
            S.pen_xy = np.array([p[0], p[1]])
            S.z_abs = float(p[2])
            S.z_base = float(p[2])
            S.target = np.asarray(p, float).copy()
            S.q_ref = q_meas.copy()
            S.grip = grip_cmd
            S.ready = True
            S.status = "就绪"

        dt_nom = 1.0 / RATE
        t_prev = time.perf_counter()
        t0 = t_prev
        n = 0
        goal_until = 0.0
        was_eng = False
        z_off_s = 0.0
        y_fine_s = 0.0
        y_base = 0.0
        g_anchor: tuple[float, float] | None = None    # (py, px)
        palm_gate = PalmSafetyGate()
        grip_hist: list[str] = []
        pinch_latch = False
        pinch_since: float | None = None
        yaw_off = 0.0                 # J6 自转偏置 (rad)，由手机驱动
        ph_estop_seen = 0             # 手机"冻结/解除"请求计数器
        ph_resume_seen = 0

        while not S.stop:
            now = time.perf_counter()
            dt = max(1e-4, min(now - t_prev, 0.05))
            t_prev = now

            q_meas = np.asarray(g.get_positions(request_feedback=False), float)[:6]
            tau = compute_generalized_gravity(model, pad_q_for_model(model, q_meas, 6), data)[:6]

            with S.lock:
                eng = S.engaged and not S.freeze
                rq = S.ready_req
                S.ready_req = 0.0
                S.q = q_meas.copy()
                hand = S.g_left and (time.monotonic() - S.g_t < FRAME_TIMEOUT)
                gesture = S.g_gesture
                px, py = S.g_px, S.g_py
                frame_time = S.g_t
                manual_freeze = S.freeze

            # 捏合需要持续 0.4s 才生效（防止误判）
            if gesture == "pinch":
                if pinch_since is None:
                    pinch_since = now
            else:
                pinch_since = None
            pinch = gesture == "pinch" and pinch_since is not None and (now - pinch_since) > 0.4
            if pinch and not pinch_latch:
                pinch_latch = True
                print("[hybrid] 🤏 捏合 → 全局冻结", flush=True)
            if not pinch:
                pinch_latch = False

            with S.lock:
                z_cmd = S.z_abs
                z_base = S.z_base
                pen = S.pen_xy.copy()
                sc = S.g_scale
            was_palm_active = palm_gate.armed
            palm_active = palm_gate.update(
                detected=hand, gesture=gesture, px=px, py=py,
                scale=sc, frame_time=frame_time,
                frozen=manual_freeze or pinch or (eng and not was_palm_active), now=now,
            )
            if palm_active and not was_palm_active:
                # A newly visible hand must never resume an old IK target. Start
                # from the measured end-effector pose and anchor the palm there.
                current_ee, current_rpy = joint_to_pose(np.asarray(arm.get_state()[0], float))
                current_ee = np.asarray(current_ee, float)
                if np.linalg.norm(clamp_ws(current_ee) - current_ee) > 0.005:
                    palm_gate.reset()
                    palm_active = False
                    print("[hybrid] 当前位置在手势工作空间外，拒绝手势使能", flush=True)
                else:
                    rpy_lock = np.asarray(current_rpy, float)
                    with S.lock:
                        S.ee = current_ee.copy()
                        # 数位板正在拖动时，不要抢它的 X/Y 目标（否则会"拖了没反应"）
                        pen_busy = bool(S.engaged)
                        if not pen_busy:
                            S.pen_xy = current_ee[:2].copy()
                        S.z_abs = float(current_ee[2])
                        S.z_base = float(current_ee[2])
                        S.y_fine = 0.0
                    if not pen_busy:
                        pen = current_ee[:2].copy()
                    z_cmd = float(current_ee[2])
                    z_base = z_cmd
                    y_fine_s = 0.0
                    y_base = 0.0
                    g_anchor = None
            with S.lock:
                S.g_armed = palm_active

            # 手的大小 → 位移增益补偿（离镜头远近手感一致）
            comp = float(np.clip(sc / SCALE_REF, 0.6, 1.5))

            # ── 手心相对位置 → 高度/左右目标 ──
            if palm_active:
                if g_anchor is None:
                    g_anchor = (py, px)
                    z_base = z_cmd
                    z_off_s = 0.0
                    y_base = y_fine_s
                    with S.lock:
                        S.z_base = z_base
                    print(f"[hybrid] 左手(重)锚定 py={py:.3f} px={px:.3f} z_base={z_base:.3f}", flush=True)
                dy = (g_anchor[0] - py) / comp   # 手抬起为正
                if abs(dy) <= Z_DEAD_PY:
                    dy = 0.0
                else:
                    dy -= np.copysign(Z_DEAD_PY, dy)
                z_off_raw = float(np.clip(dy * Z_GAIN, -Z_OFFSET_MAX, Z_OFFSET_MAX))
                z_alpha = min(1.0, dt / max(Z_TAU, 1e-3))
                z_off_s += z_alpha * (z_off_raw - z_off_s)
                z_target = float(np.clip(z_base + z_off_s, BZ[0], BZ[1]))
                z_cmd += float(np.clip(z_target - z_cmd, -Z_RATE_MAX * dt, Z_RATE_MAX * dt))

                # 左右微调同样以重新识别时的位置为零点，保留已有偏移。
                y_raw = float(np.clip(y_base + (px - g_anchor[1]) * Y_GAIN / comp,
                                      -Y_FINE_MAX, Y_FINE_MAX))
                if abs(y_raw - y_base) < Y_DEAD:
                    y_raw = y_base
                alpha = min(1.0, dt / max(G_TAU, 1e-3))
                y_fine_s += float(np.clip(alpha * (y_raw - y_fine_s),
                                          -Y_RATE_MAX * dt, Y_RATE_MAX * dt))
            else:
                g_anchor = None
                z_off_s = 0.0

            # ── 手机姿态：高度（速度式）+ J6 自转（速度式）──
            ph_active = False
            if ph_state is not None:
                snap = ph_state.snapshot()
                if snap["estop_req"] != ph_estop_seen:
                    ph_estop_seen = snap["estop_req"]
                    with S.lock:
                        S.freeze = True
                        S.engaged = False
                    engaged = False
                    print("[phone] ⛔ 手机请求冻结（按 Space / 笔侧键 / 手机「解除」恢复）", flush=True)
                if snap["resume_req"] != ph_resume_seen:
                    ph_resume_seen = snap["resume_req"]
                    with S.lock:
                        S.freeze = False
                    print("[phone] ✅ 手机请求解除冻结", flush=True)
                fresh = bool(snap["fresh"])
                if fresh and not (manual_freeze or pinch):
                    z_vel = float(snap["z_vel"])
                    yaw_vel = float(snap["yaw_vel"])
                    if z_vel:
                        dz = float(np.clip(z_vel * dt, -PHONE_DZ_MAX, PHONE_DZ_MAX))
                        z_cmd = float(np.clip(z_cmd + dz, BZ[0], BZ[1]))
                        with S.lock:      # 手势锚定基准一起平移，避免和手势互相拉扯
                            S.z_base = float(np.clip(S.z_base + dz, BZ[0], BZ[1]))
                    if yaw_vel:
                        yaw_off = float(np.clip(yaw_off + math.radians(yaw_vel) * dt,
                                                -YAW_OFF_MAX, YAW_OFF_MAX))
                    ph_active = bool(abs(z_vel) > 1e-4 or abs(yaw_vel) > 1e-2)
                with S.lock:
                    S.ph_connected = fresh
                    S.ph_age = float(snap["age"])
                    S.ph_beta = float(snap["beta"])
                    S.ph_gamma = float(snap["gamma"])
                    S.ph_z_vel = float(snap["z_vel"]) if fresh else 0.0
                    S.ph_yaw_vel = float(snap["yaw_vel"]) if fresh else 0.0
                    S.ph_n = int(snap["n"])
                    S.yaw_off = yaw_off

            new_z = float(np.clip(z_cmd, BZ[0], BZ[1]))

            with S.lock:
                S.z_abs = new_z
                S.g_anchor = g_anchor
                S.y_fine = y_fine_s
                tgt = np.array([pen[0], pen[1] + y_fine_s, new_z], float)
            tgt = clamp_ws(tgt)
            with S.lock:
                S.target = tgt

            if now < goal_until:
                q_goal = Q_READY.copy()
            elif eng or palm_active or ph_active:
                res = solve_ik(
                    model, data, fid,
                    pos_rot_to_se3(tgt, roll=float(rpy_lock[0]), pitch=float(rpy_lock[1]),
                                   yaw=float(rpy_lock[2]) + yaw_off),
                    pad_q_for_model(model, q_cmd, 6),
                    IKParams(max_iter=120, tolerance=1e-4, step_size=0.5, damping=1e-6),
                    controlled_joints=6,
                )
                ok = bool(res.success) and float(res.error) < 5e-3
                if ok:
                    q_goal = np.clip(np.asarray(res.q[:6], float), JMIN, JMAX)
                    with S.lock:
                        S.ik = f"OK ({res.error:.1e})"
                else:
                    q_goal = q_cmd.copy()
                    with S.lock:
                        S.ik = f"FAIL {res.error:.3f}"
                        # 目标解不到（限位/奇异）时，把高度基准拉回实测，避免目标越积越远
                        cur_z = float(np.clip(S.ee[2], BZ[0], BZ[1]))
                        S.z_abs = cur_z
                        S.z_base = cur_z
            else:
                q_goal = q_meas.copy()  # 冻结/无输入时保持实测姿态，不拉向软限位

            step = (PHONE_JOINT_SLEW if ph_active else (PALM_JOINT_SLEW if palm_active else SLEW)) * dt
            q_cmd = q_cmd + np.clip(q_goal - q_cmd, -step, step)
            g.send_mit(q_cmd, vel=np.zeros(6), kp=kp, kd=kd, tau=tau)

            # ── 夹爪 ──
            if grip_g is not None and S.grip_ready:
                g_target = grip_cmd
                if hand and not manual_freeze and gesture == "fist":
                    grip_hist.append("fist")
                elif hand and not manual_freeze and gesture == "open":
                    grip_hist.append("open")
                else:
                    grip_hist.clear()
                if len(grip_hist) >= 6:
                    g_target = S.grip_close if grip_hist[-1] == "fist" else S.grip_open
                    print(f"[grip] → {'夹紧' if grip_hist[-1]=='fist' else '松开'} ({g_target:+.3f})", flush=True)
                    grip_hist.clear()
                gs = GRIP_SLEW * dt
                grip_cmd = grip_cmd + float(np.clip(g_target - grip_cmd, -gs, gs))
                try:
                    grip_g.send_mit(np.array([grip_cmd]), vel=np.zeros(1),
                                    kp=np.array([GRIP_KP]), kd=np.array([1.0]), tau=np.zeros(1))
                except Exception:  # noqa: BLE001
                    pass
                with S.lock:
                    S.grip = grip_cmd

            n += 1
            if n % 10 == 0:
                p_now, _ = joint_to_pose(np.asarray(arm.get_state()[0], float))
                with S.lock:
                    S.ee = np.asarray(p_now, float)
                    S.rate_hz = n / max(now - t0, 1e-6)
                    if S.freeze:
                        S.status = "手动冻结"
                    elif pinch:
                        S.status = "捏合冻结"
                    elif not hand:
                        S.status = "无左手（高度冻结）"
                    elif gesture == "fist":
                        S.status = "握拳（高度冻结+夹紧）"
                    else:
                        S.status = "跟随中"
                if eng and not was_eng:
                    with S.lock:
                        S.q_ref = q_meas.copy()
                    print("[hybrid] 落笔", flush=True)
                was_eng = eng
            if n % 100 == 0:
                print(f"[hybrid] t={now-t0:6.1f}s 左手={int(hand)} {gesture:5s} py={py:.3f} px={px:.3f} "
                      f"z={new_z:+.3f} y_fine={y_fine_s:+.3f} 自转={math.degrees(yaw_off):+6.1f}° "
                      f"夹爪={grip_cmd:+.3f} q={np.round(q_meas,3)}",
                      flush=True)

            time.sleep(max(0.0, dt_nom - (time.perf_counter() - now)))

        clean = True
    except Exception as e:  # noqa: BLE001
        with S.lock:
            S.error = f"{type(e).__name__}: {e}"
            S.status = "错误（保持悬停）"
        print(f"[hybrid] 异常: {S.error} —— 不失能", flush=True)
    finally:
        if g is not None:
            try:
                if clean:
                    with S.lock:
                        S.status = "平滑归零中…"
                    print("[hybrid] 平滑归零（最小 jerk）…", flush=True)
                    steps = int(3.0 * RATE)
                    for i in range(1, steps + 1):
                        a = i / steps
                        s = 10 * a**3 - 15 * a**4 + 6 * a**5
                        q_t = q_cmd + (np.zeros(6) - q_cmd) * s
                        tau = compute_generalized_gravity(model, pad_q_for_model(model, q_t, 6), data)[:6]
                        g.send_mit(q_t, vel=np.zeros(6), kp=kp, kd=kd, tau=tau)
                        time.sleep(1.0 / RATE)
                    tau0 = compute_generalized_gravity(model, np.zeros(model.nq), data)[:6]
                    for _ in range(int(0.5 * RATE)):
                        g.send_mit(np.zeros(6), vel=np.zeros(6), kp=kp, kd=kd, tau=tau0)
                        time.sleep(1.0 / RATE)
                    print("[hybrid] 已归零并保持悬停（未失能）", flush=True)
                else:
                    for _ in range(50):
                        try:
                            g.send_mit(q_cmd, vel=np.zeros(6), kp=kp, kd=kd, tau=np.zeros(6))
                        except Exception:  # noqa: BLE001
                            break
                        time.sleep(0.01)
            except Exception as e:  # noqa: BLE001
                print(f"[hybrid] 收尾发送失败（电机仍保持，不会掉）: {e}", flush=True)
        with S.lock:
            S.ready = False
            S.status = "已归零·悬停中（未失能）"


# ── GUI ───────────────────────────────────────────────────────────────────────
W, H = 1400, 840
CW, CH = 740, 650
MX, MY = 60, 60

root = tk.Tk()
root.title("reBot 混合遥操 — 右手数位板(深度/左右) + 左手手势(高度/左右微调/夹爪)")
root.geometry(f"{W}x{H}+30+24")
root.attributes("-topmost", True)
cv = tk.Canvas(root, width=CW, height=CH, bg="#111827", highlightthickness=0)
cv.place(x=20, y=20)
panel = tk.Canvas(root, width=580, height=CH, bg="#0b1220", highlightthickness=0)
panel.place(x=CW + 40, y=20)

cv.create_text(CW / 2, 22, text="俯视：笔上下→深度 X，笔左右→左右 Y  |  左手掌心上下→高度 Z，左右→左右微调",
               fill="#9ca3af", font=("Helvetica", 13))

pen_dot = cv.create_oval(0, 0, 0, 0, outline="#f472b6", width=2)
tgt_dot = cv.create_oval(0, 0, 0, 0, fill="#facc15", outline="")
ee_dot = cv.create_oval(0, 0, 0, 0, fill="#4ade80", outline="")
trail_line = cv.create_line(0, 0, 0, 0, fill="#38bdf8", width=2)
pen_trail: list[tuple[float, float]] = []
panel_text = panel.create_text(20, 20, anchor="nw", fill="#e5e7eb", font=("Menlo", 13), text="")
panel.create_text(20, CH - 200, anchor="nw", fill="#9ca3af", font=("Menlo", 12),
                  text=("操作\n"
                        "  Space → 解锁/冻结；右键只会冻结\n"
                        "  右手落笔拖动 → 深度 X + 左右 Y（大范围）\n"
                        "  Shift+上下   → 直接调整高度 Z\n"
                        "  ✋ 左掌在画面中心停 0.6 秒 → 锚定\n"
                        "  掌心上下移动 → 相对锚点控制高度\n"
                        "                  停手后保持对应高度\n"
                        "                  左右移动 = 左右微调\n"
                        "  ✊ 左手握拳    → 高度冻结 + 夹爪夹紧\n"
                        "  🤏 捏合 0.4s   → 全局冻结\n"
                        "  Esc → 平滑归零保持"))


def world_to_px(x: float, y: float) -> tuple[float, float]:
    px = MX + (y - BY[0]) / (BY[1] - BY[0]) * (CW - 2 * MX)
    py = CH - MY - (x - BX[0]) / (BX[1] - BX[0]) * (CH - 2 * MY)
    return px, py


def redraw_static() -> None:
    cv.delete("static")
    x0, y0 = world_to_px(BX[0], BY[0])
    x1, y1 = world_to_px(BX[1], BY[1])
    cv.create_rectangle(x0, y1, x1, y0, outline="#374151", width=2, tags="static")
    cv.create_text(x1 - 110, y1 - 14, text="X 深度 →", fill="#6b7280", font=("Helvetica", 11), tags="static")
    cv.create_text(x0 + 6, y0 - 16, text="← Y 左右 →", fill="#6b7280", font=("Helvetica", 11), tags="static")


redraw_static()
anchor_pen = [0.0, 0.0]
anchor_xy = np.zeros(2)
anchor_z = 0.0
engaged = False


def pen_anchor(e) -> None:
    global engaged, anchor_z          # ← 必须声明 anchor_z，否则 Shift 调高度会跳到 0
    print(f"[pen] 落笔事件 x={e.x} y={e.y} ready={S.ready} freeze={S.freeze}", flush=True)
    with S.lock:
        if not S.ready or S.freeze:
            if S.freeze:
                print("[pen] ⚠️ 落笔被拒绝：手动冻结开着（按 Space 或笔侧键解除）", flush=True)
            return
        anchor_xy[:] = S.pen_xy
        anchor_z = S.z_abs
        engaged = True
        S.engaged = True
    anchor_pen[0], anchor_pen[1] = float(e.x), float(e.y)


def pen_release() -> None:
    global engaged
    with S.lock:
        S.engaged = False
        engaged = False


def toggle_freeze() -> None:
    global engaged
    with S.lock:
        S.freeze = not S.freeze
        if S.freeze:
            S.engaged = False
            engaged = False


def on_press(e) -> None:
    if e.num == 1:
        pen_anchor(e)
    elif e.num == 3:
        toggle_freeze()          # 侧键：冻结/解冻都能切换（原来只能冻不能解）


def on_release(e) -> None:
    if e.num == 1:
        pen_release()


_motion_n = 0


def on_motion(e) -> None:
    global _motion_n
    _motion_n += 1
    if _motion_n % 200 == 1:
        print(f"[pen] 移动事件 #{_motion_n} x={e.x} y={e.y} engaged={engaged}", flush=True)
    pen_trail.append((e.x, e.y))
    if len(pen_trail) > 600:
        del pen_trail[:200]
    if len(pen_trail) >= 4:
        cv.coords(trail_line, *[c for p in pen_trail[-200:] for c in p])
    cv.coords(pen_dot, e.x - 6, e.y - 6, e.x + 6, e.y + 6)
    if not engaged:
        return
    dx = e.x - anchor_pen[0]
    dy = e.y - anchor_pen[1]
    shift = bool(e.state & 0x0001)
    with S.lock:
        if not S.ready or S.freeze:
            return
        if shift:
            # 应急：直接调高度（同时平移手势基准，避免跳变）
            dz = -dy * SCALE_PEN
            new_z = float(np.clip(anchor_z + dz, BZ[0], BZ[1]))
            S.z_base += (new_z - S.z_abs)
            S.z_abs = new_z
        else:
            x = float(np.clip(anchor_xy[0] - dy * SCALE_PEN, BX[0], BX[1]))
            y = float(np.clip(anchor_xy[1] + dx * SCALE_PEN, BY[0], BY[1]))
            S.pen_xy = np.array([x, y])


def on_key(e) -> None:
    if e.keysym == "Escape":
        quit_app()
    elif e.keysym.lower() == "r":
        with S.lock:
            S.ready_req = time.perf_counter()
    elif e.keysym == "space":
        toggle_freeze()


_quitting = False


def quit_app() -> None:
    global _quitting
    if _quitting:
        return                      # 重复的停止信号直接忽略，避免打断归零
    _quitting = True
    S.stop = True
    try:
        root.destroy()
    except Exception:               # noqa: BLE001
        pass


cv.bind("<ButtonPress>", on_press)
cv.bind("<ButtonRelease>", on_release)
cv.bind("<Motion>", on_motion)
root.bind("<Key>", on_key)
root.protocol("WM_DELETE_WINDOW", quit_app)
root.force_focus = root.focus_force()


def tick() -> None:
    with S.lock:
        ee = S.ee.copy(); tgt = S.target.copy(); q = S.q.copy(); q_ref = S.q_ref.copy()
        st = S.status; ik = S.ik; eng = S.engaged; frz = S.freeze; hz = S.rate_hz; err = S.error
        gconn = S.g_connected; gleft = S.g_left; gges = S.g_gesture; garmed = S.g_armed
        g_age = time.monotonic() - S.g_t if S.g_t else float("inf")
        gpx, gpy, gfps = S.g_px, S.g_py, S.g_fps
        anchor = S.g_anchor; gsc = S.g_scale; yfine = S.y_fine; grip = S.grip; nhands = S.g_nhands
        zabs = S.z_abs; zbase = S.z_base
        ph_on = S.ph_on; ph_conn = S.ph_connected; ph_age = S.ph_age; ph_beta = S.ph_beta
        ph_gamma = S.ph_gamma; ph_z = S.ph_z_vel; ph_yaw = S.ph_yaw_vel; ph_n = S.ph_n
        ph_url = S.ph_url; yaw_deg = math.degrees(S.yaw_off)
    px, py = world_to_px(float(ee[0]), float(ee[1]))
    tx, ty = world_to_px(float(tgt[0]), float(tgt[1]))
    cv.coords(ee_dot, px - 7, py - 7, px + 7, py + 7)
    cv.coords(tgt_dot, tx - 5, ty - 5, tx + 5, ty + 5)
    panel.itemconfig(panel_text, text=(
        f"状态     : {st}\n"
        f"落笔     : {'按住' if eng else '松开'}\n"
        f"手动冻结 : {'ON' if frz else 'off'}\n"
        f"控制频率 : {hz:5.1f} Hz     IK: {ik}\n"
        f"── 左手手势 ──\n"
        f"cam_server: {'✔ 实时' if gconn and g_age < FRAME_TIMEOUT else ('⚠ 停帧' if gconn else '✘ 未连接')}\n"
        f"画面中的手: {nhands} 只   左手: {'有' if gleft and g_age < FRAME_TIMEOUT else '无'}\n"
        f"手势控制  : {'已解锁·已锚定' if garmed else ('冻结中（Space 解锁）' if frz else '等待左掌在中心停 0.6 秒')}\n"
        f"手势      : {gges}\n"
        f"掌心 px/py: {gpx:.3f} / {gpy:.3f}   ({gfps:.0f} fps)\n"
        f"掌心锚点  : {f'{anchor[1]:.3f} / {anchor[0]:.3f}' if anchor else '未锚定'}\n"
        f"高度指令  : {zabs:+.3f} m\n"
        f"相对高度  : {zabs-zbase:+.3f} m  （手心位置映射）\n"
        f"手部尺寸  : {gsc:.3f}  远近补偿 ×{float(np.clip(gsc / SCALE_REF, 0.6, 1.5)):.2f}\n"
        f"左右微调  : {yfine:+.3f} m\n"
        f"夹爪      : {grip:+.3f} rad  {'(已启用)' if S.grip_ready else '(未启用)'}\n"
        + (f"── 手机姿态 ──\n"
           f"连接     : {'✅ 实时' if ph_conn else '❌ 无数据'}  ({ph_age:.1f}s 前 / 收到 {ph_n} 帧)\n"
           f"俯仰/横滚: {ph_beta:+.1f}° / {ph_gamma:+.1f}°\n"
           f"高度速度 : {ph_z:+.3f} m/s\n"
           f"自转速度 : {ph_yaw:+.1f} °/s   偏置 {yaw_deg:+.1f}°\n"
           f"手机页面 : {ph_url}\n" if ph_on else "")
        + f"── 末端 ──\n"
        f"位置     : X={ee[0]:+.3f} Y={ee[1]:+.3f} Z={ee[2]:+.3f}\n"
        f"目标     : X={tgt[0]:+.3f} Y={tgt[1]:+.3f} Z={tgt[2]:+.3f}\n"
        f"半径     : {np.hypot(ee[0], ee[1]):.3f} m\n"
        f"── 关节角 (Δ相对落笔) ──\n"
        + "\n".join(f"  J{i+1}: {q[i]:+.3f}  Δ{q[i]-q_ref[i]:+.3f}" + (" ●" if abs(q[i]-q_ref[i]) > 0.02 else "")
                    for i in range(6))
        + (f"\n\n⚠️ {err}" if err else "")
    ))
    # 冻结时在画布中央显示醒目横幅
    if frz:
        if not cv.find_withtag("frozenbanner"):
            cv.create_rectangle(0, CH / 2 - 46, CW, CH / 2 + 46,
                                fill="#7f1d1d", outline="#ef4444", width=3, tags="frozenbanner")
            cv.create_text(CW / 2, CH / 2,
                           text="⚠️ 手动冻结中 —— 按 Space 或笔侧键解除",
                           fill="#fecaca", font=("Helvetica", 21, "bold"), tags="frozenbanner")
    else:
        cv.delete("frozenbanner")
    root.after(50, tick)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--gesture-port", type=int, default=9100)
    ap.add_argument("--gripper", action="store_true", help="启用夹爪（需先确认行程）")
    ap.add_argument("--grip-range", type=float, default=0.35)
    ap.add_argument("--phone", action="store_true", help="开启手机姿态遥控（浏览器页面）")
    ap.add_argument("--phone-port", type=int, default=9210)
    ap.add_argument("--phone-token", default="rebot", help="手机页面令牌（URL 里的 ?t=）")
    ap.add_argument("--phone-https", action="store_true", help="自签名 HTTPS（iOS 传感器可能要）")
    ap.add_argument("--phone-z-max", type=float, default=0.08, help="手机高度速度上限 m/s")
    ap.add_argument("--phone-yaw-max", type=float, default=35.0, help="手机 J6 自转速度上限 °/s")
    ap.add_argument("--phone-dead", type=float, default=6.0, help="手机死区角度 °")
    args = ap.parse_args()

    ph_state = None
    if args.phone:
        from phone_ctrl import PhoneConfig, PhoneServer
        pcfg = PhoneConfig(port=args.phone_port, token=args.phone_token, https=args.phone_https,
                           dead_deg=args.phone_dead, z_max=args.phone_z_max,
                           yaw_max=args.phone_yaw_max)
        psrv = PhoneServer(pcfg)
        url = psrv.start()
        ph_state = psrv.state
        with S.lock:
            S.ph_on = True
            S.ph_url = url
        print(f"[phone] 📱 手机遥控页面: {url}", flush=True)
        print("[phone]    手机上打开 → ① 启用传感器 ② 校准回中", flush=True)

    signal.signal(signal.SIGINT, lambda *_: quit_app())
    threading.Thread(target=gesture_client, args=(args.gesture_port,), daemon=True).start()
    th = threading.Thread(target=control_loop, args=(args, ph_state), daemon=True)
    th.start()
    root.after(100, tick)
    print("[gui] 窗口已创建，等待事件（用笔在窗口内移动/落笔）", flush=True)
    root.mainloop()
    th.join(timeout=12.0)
    print(f"[hybrid] 结束 status={S.status} error={S.error}", flush=True)
