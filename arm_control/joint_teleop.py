#!/usr/bin/env python3
"""joint_teleop.py — 数位板逐电机直控（无 IK）

分工（已与用户确认）:
    笔      = 唯一的运动输入：按住拖动 → 当前选中关节的角度增量（锚点式，按下不跳）
    键盘    = 全部模式与选择：
                1~6     选 J1~J6（7 = 夹爪，需 --gripper）
                F       粗调 / 细调 切换
                Shift   按住 = 临时细调（对笔和方向键都生效）
                ← →     选中关节 ∓/±0.1°（Shift = ±0.01°）
                G       选中关节平滑回 0
                Z       全部关节平滑回 0（保持悬停，不失能）
                A       对齐：目标角 ← 实测角（消除偏差）
                X / Y   切换笔的控制轴（横向 / 纵向）
                Space   冻结 / 解冻（笔侧键同）
                Esc     退出（先平滑归零，再保持悬停）

安全:
    - 逐关节软限位（URDF 限位 + 余量），硬夹
    - 逐关节限速（默认 0.6 rad/s，可逐关节覆盖）
    - 力矩监控（--tau-limit > 0 时启用，超限自动冻结）
    - 退出走最小 jerk 归零，绝不断力矩（断电请用 disable_arm.py）
"""
from __future__ import annotations

import argparse
import math
import signal
import sys
import threading
import time
import tkinter as tk

import numpy as np

REPO = "/Users/Admin/Desktop/reBotArm_control_py"
sys.path.insert(0, REPO)

from reBotArm_control_py.actuator import RebotArm  # noqa: E402
from reBotArm_control_py.dynamics import compute_generalized_gravity, load_dynamics_model  # noqa: E402
from reBotArm_control_py.kinematics import load_robot_model, pad_q_for_model  # noqa: E402

# ── 配置 ──────────────────────────────────────────────────────────────────────
RATE = 200                       # 控制频率 Hz
SLEW_DEFAULT = 0.6               # 默认关节限速 rad/s
PEN_TIMEOUT = 0.4                # 笔事件超时（视为松笔）s
NUDGE_COARSE = 0.1               # 方向键步长（度）
NUDGE_FINE = 0.01                # Shift+方向键步长（度）
TAU_WARN = 8.0                   # 力矩显示告警阈值 (N·m)，仅显示颜色

CW, CH = 940, 640                # 窗口尺寸
BG = "#0b1220"
FG = "#e5e7eb"
DIM = "#8aa0c6"
ACC = "#38bdf8"
OKC = "#34d399"
WARN = "#fbbf24"
RED = "#ef4444"

# URDF 限位（权威值，启动时用模型再确认一次）
LIM_LO = np.array([-2.80, 0.0, 0.0, -1.57, -1.57, -3.14])
LIM_HI = np.array([2.80, 3.14, 3.14, 1.57, 1.57, 3.14])


class State:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.stop = False
        self.error: str | None = None
        self.status = "初始化…"
        self.q = np.zeros(6)          # 实测角
        self.vel = np.zeros(6)
        self.tau = np.zeros(6)
        self.q_tgt = np.zeros(6)      # 操作者目标角
        self.q_send = np.zeros(6)     # 已下发角（限速后）
        self.q_at_press = 0.0         # 按下瞬间的目标角（锚点基准）
        self.anchor = (0.0, 0.0)      # 按下瞬间的笔位置
        self.pen_t = 0.0              # 最近笔事件时间
        self.grip_send = 0.0
        self.sel = 0                  # 选中关节 0..5（6 = 夹爪，若启用）
        self.n_joints = 6
        self.fine = False
        self.axis = "x"
        self.freeze = False
        self.alarm = ""
        self.quit_req = False
        self.rate_hz = 0.0
        self.pen = (0.0, 0.0)
        self.pressed = False
        self.sweep_deg = 45.0
        self.fine_factor = 10.0
        self.margin_deg = 1.5
        self.grip_enabled = False
        self.grip = 0.0               # 夹爪实测
        self.grip_tgt = 0.0
        self.grip_lim = (0.0, 0.0)
        self.ready = False


S = State()


# ── 控制线程 ──────────────────────────────────────────────────────────────────


def control_loop(args) -> None:
    arm = None
    grip_g = None
    clean = False
    try:
        arm = RebotArm()
        arm.connect()
        g = arm.arm
        q_start = np.asarray(g.get_positions(), float)[:6]
        if float(np.max(np.abs(q_start))) > 0.05:
            g = None
            raise RuntimeError("启动姿态距折叠零位超过 0.05 rad；拒绝使能"
                               "（不 disconnect / 不失能，机械臂维持原状态）")

        model = load_robot_model()
        data = model.createData()
        kp, kd = g._mit_kp.copy(), g._mit_kd.copy()

        # 用 URDF 限位覆盖硬编码值（+余量）
        lim_lo = np.asarray(model.lowerPositionLimit[:6], float).copy()
        lim_hi = np.asarray(model.upperPositionLimit[:6], float).copy()
        if not np.all(np.isfinite(lim_lo)) or not np.all(np.isfinite(lim_hi)):
            lim_lo, lim_hi = LIM_LO.copy(), LIM_HI.copy()
        margin = math.radians(args.limit_margin)
        lo = lim_lo + margin
        hi = lim_hi - margin

        # 逐关节限速
        vlim = np.full(6, float(args.max_vel))
        for item in (args.max_vel_joint or []):
            name, val = item.split("=")
            idx = int(name.upper().replace("J", "")) - 1
            vlim[idx] = float(val)

        g.mode_mit(kp=kp, kd=kd)
        g.enable()
        tau_hold = compute_generalized_gravity(model, pad_q_for_model(model, q_start, 6), data)[:6]
        g.send_mit(q_start, vel=np.zeros(6), kp=kp, kd=kd, tau=tau_hold)

        if args.gripper:
            try:
                grip_g = arm.gripper
                grip_g.mode_mit()
                grip_g.enable()
                gpos = float(np.asarray(grip_g.get_positions(), float).reshape(-1)[0])
                gl, gh = gpos - args.grip_range, gpos + args.grip_range
                with S.lock:
                    S.grip_enabled = True
                    S.n_joints = 7
                    S.grip = gpos
                    S.grip_tgt = gpos
                    S.grip_lim = (gl, gh)
                print(f"[grip] 已使能 当前={gpos:+.3f}  范围=({gl:+.3f}, {gh:+.3f})", flush=True)
            except Exception as e:  # noqa: BLE001
                print(f"[grip] 不可用: {e}", flush=True)
                grip_g = None

        pos, vel, torq = (np.asarray(x, float)[:6] for x in arm.get_state())
        with S.lock:
            S.q = pos.copy()
            S.vel = vel.copy()
            S.tau = torq.copy()
            S.q_tgt = pos.copy()
            S.q_send = pos.copy()
            S.status = "就绪（笔=运动 / 键盘=模式）"
            S.ready = True

        q_send = pos.copy()        # ← 本地累加器（限速器状态不能每拍重读）
        dt_nom = 1.0 / RATE
        t_prev = time.perf_counter()
        t0 = t_prev
        n = 0
        tau_alarm_since = 0.0
        while not S.stop:
            now = time.perf_counter()
            dt = max(1e-4, min(now - t_prev, 0.05))
            t_prev = now

            with S.lock:
                sel = S.sel
                freeze = S.freeze
                q_tgt = S.q_tgt.copy()
                pressed = S.pressed
                pen_t = S.pen_t
                gtgt = S.grip_tgt

            # 笔超时 → 视为松笔（位置模式，目标不变，安全）
            if pressed and (now - pen_t) > PEN_TIMEOUT:
                with S.lock:
                    S.pressed = False

            # 目标夹到软限位（含余量）
            for i in range(6):
                q_tgt[i] = min(max(q_tgt[i], lo[i]), hi[i])

            if freeze:
                # 冻结：目标同步到当前下发值，谁也别想动（解冻后必须重新落笔才动）
                q_tgt = q_send.copy()
            else:
                # 限速下发
                for i in range(6):
                    step = vlim[i] * dt
                    q_send[i] += min(max(q_tgt[i] - q_send[i], -step), step)
            with S.lock:
                S.q_tgt = q_tgt.copy()

            tau = compute_generalized_gravity(model, pad_q_for_model(model, q_send, 6), data)[:6]
            g.send_mit(q_send, vel=np.zeros(6), kp=kp, kd=kd, tau=tau)

            if grip_g is not None:
                gl, gh = S.grip_lim
                gtgt = min(max(gtgt, gl), gh)
                gsend_prev = S.grip_send if hasattr(S, "grip_send") else gtgt
                gsend = gsend_prev + float(np.clip(gtgt - gsend_prev, -args.grip_slew * dt,
                                                   args.grip_slew * dt))
                S.grip_send = gsend

            n += 1
            if n % 4 == 0:      # 50 Hz 读反馈，别把总线压死
                pos, vel, torq = (np.asarray(x, float)[:6] for x in arm.get_state())
                gpos = float(np.asarray(grip_g.get_positions(), float).reshape(-1)[0]) if grip_g else 0.0
                with S.lock:
                    S.q = pos.copy()
                    S.vel = vel.copy()
                    S.tau = torq.copy()
                    S.q_send = q_send.copy()    # 仅供显示/收尾使用
                    S.rate_hz = n / max(now - t0, 1e-6)
                    if grip_g is not None:
                        S.grip = gpos
                        if not freeze:
                            grip_g.send_mit(np.array([S.grip_send]), vel=np.zeros(1),
                                            kp=np.array([args.grip_kp]), kd=np.array([args.grip_kd]),
                                            tau=np.zeros(1))
                    # 力矩监控
                    if args.tau_limit > 0:
                        over = np.where(np.abs(torq) > args.tau_limit)[0]
                        if len(over) and not freeze:
                            if tau_alarm_since == 0.0:
                                tau_alarm_since = now
                            elif now - tau_alarm_since > 0.2:
                                S.freeze = True
                                S.alarm = ("⚠️ 力矩超限自动冻结: " +
                                           ", ".join(f"J{i+1}={torq[i]:+.1f}N·m" for i in over))
                                print(f"[tau] {S.alarm}", flush=True)
                                tau_alarm_since = 0.0
                        else:
                            tau_alarm_since = 0.0
                    S.status = "冻结中（保持悬停）" if S.freeze else "就绪（笔=运动 / 键盘=模式）"

            if n % (RATE * 2) == 0:
                with S.lock:
                    sel_i, tgt_i, q_i = S.sel, S.q_tgt[S.sel], S.q[S.sel]
                    tau_i = S.tau[S.sel]
                print(f"[joint] t={now-t0:6.1f}s 选中=J{sel_i+1} 目标={math.degrees(tgt_i):+7.2f}° "
                      f"实测={math.degrees(q_i):+7.2f}° 力矩={tau_i:+5.2f}N·m "
                      f"{'❄冻结' if freeze else ''} {'细调' if S.fine else '粗调'}", flush=True)

            time.sleep(max(0.0, dt_nom - (time.perf_counter() - now)))

        clean = True
    except Exception as e:  # noqa: BLE001
        with S.lock:
            S.error = f"{type(e).__name__}: {e}"
            S.status = "错误（保持悬停）"
        print(f"[joint] 异常: {S.error} —— 不失能，2 秒后关闭窗口", flush=True)
        time.sleep(2.0)
        with S.lock:
            S.quit_req = True
    finally:
        if arm is not None and getattr(arm, "arm", None) is not None:
            g = arm.arm
            kp, kd = g._mit_kp.copy(), g._mit_kd.copy()
            try:
                if clean:
                    with S.lock:
                        S.status = "平滑归零中…"
                    print("[joint] 平滑归零（最小 jerk）…", flush=True)
                    steps = int(3.0 * RATE)
                    q_from = np.asarray(S.q_send, float).copy()
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
                    print("[joint] 已归零并保持悬停（未失能）", flush=True)
                else:
                    for _ in range(50):
                        try:
                            g.send_mit(np.asarray(S.q_send, float), vel=np.zeros(6), kp=kp, kd=kd,
                                       tau=np.zeros(6))
                        except Exception:  # noqa: BLE001
                            break
                        time.sleep(0.01)
            except Exception as e:  # noqa: BLE001
                print(f"[joint] 收尾发送失败（电机仍保持，不会掉）: {e}", flush=True)
        with S.lock:
            S.ready = False
            S.status = "已归零·悬停中（未失能）"


# ── GUI ───────────────────────────────────────────────────────────────────────

root = tk.Tk()
root.title("reBot 关节直控 — 笔=运动  键盘=模式")
root.geometry(f"{CW}x{CH}+40+40")
root.configure(bg=BG)
root.attributes("-topmost", True)
cv = tk.Canvas(root, width=CW, height=CH, bg=BG, highlightthickness=0)
cv.pack(fill="both", expand=True)

ROW_Y0 = 108
ROW_H = 42
BAR_X0, BAR_X1 = 120, 400

# 关节行元素
rows: list[dict] = []
for i in range(7):
    y = ROW_Y0 + i * ROW_H
    rows.append({
        "sel": cv.create_text(28, y + 14, anchor="w", text="", fill=ACC, font=("Menlo", 15, "bold")),
        "name": cv.create_text(78, y + 14, anchor="w", text="", fill=FG, font=("Menlo", 13)),
        "bar_bg": cv.create_rectangle(BAR_X0, y + 5, BAR_X1, y + 24, outline="#334155", fill="#111a2e"),
        "bar": cv.create_rectangle(BAR_X0, y + 6, BAR_X0, y + 23, outline="", fill=OKC),
        "mark": cv.create_line(BAR_X0, y + 2, BAR_X0, y + 27, fill=WARN, width=3),
        "txt": cv.create_text(BAR_X1 + 14, y + 14, anchor="w", text="", fill=FG, font=("Menlo", 13)),
    })

title = cv.create_text(24, 26, anchor="w", text="", fill=FG, font=("Menlo", 16, "bold"))
sub = cv.create_text(24, 56, anchor="w", text="", fill=DIM, font=("Menlo", 12))
peninfo = cv.create_text(24, 80, anchor="w", text="", fill=DIM, font=("Menlo", 12))
hints = cv.create_text(24, CH - 74, anchor="nw", text="", fill=DIM, font=("Menlo", 12))
status = cv.create_text(24, CH - 22, anchor="w", text="", fill=OKC, font=("Menlo", 13, "bold"))

banner = cv.create_rectangle(0, 0, 0, 0, fill="#7f1d1d", outline=RED, width=3, state="hidden")
banner_t = cv.create_text(CW / 2, 0, text="", fill="#fecaca", font=("Menlo", 19, "bold"), state="hidden")

HINT = ("1~6 选关节 (7=夹爪)   F 粗/细调   Shift 按住=细调   ←→ 微调 0.1°/0.01°   "
        "G 该关节回零   Z 全部回零   A 对齐   X/Y 换轴   Space 冻结   Esc 退出")

cv.itemconfig(hints, text=HINT)


def draw() -> None:
    with S.lock:
        q = S.q.copy(); tau = S.tau.copy(); vel = S.vel.copy()
        tgt = S.q_tgt.copy(); sel = S.sel; fine = S.fine; axis = S.axis
        frz = S.freeze; st = S.status; alarm = S.alarm; hz = S.rate_hz
        pen = S.pen; pressed = S.pressed; ready = S.ready; err = S.error
        sweep = S.sweep_deg; ff = S.fine_factor; nj = S.n_joints
        grip = S.grip; gtgt = S.grip_tgt; glim = S.grip_lim
        margin = S.margin_deg

    names = ["J1", "J2", "J3", "J4", "J5", "J6"] + (["夹爪"] if nj == 7 else [])
    for i, r in enumerate(rows):
        if i >= nj:
            for k, v in r.items():
                cv.itemconfig(v, state="hidden")
            continue
        for k, v in r.items():
            cv.itemconfig(v, state="normal")
        if i < 6:
            lo, hi = LIM_LO[i], LIM_HI[i]
            cur, tg = q[i], tgt[i]
            txt = (f"{math.degrees(cur):+7.2f}°  目标{math.degrees(tg):+7.2f}°  "
                   f"Δ{math.degrees(tg-cur):+5.2f}°  v{vel[i]:+5.2f}  τ{tau[i]:+5.2f}")
            f = (cur - lo) / max(hi - lo, 1e-6)
            ft = (tg - lo) / max(hi - lo, 1e-6)
            near = min(tg - lo, hi - tg)
        else:
            lo, hi = glim
            txt = (f"{grip:+7.3f}  目标{gtgt:+7.3f}  "
                   f"范围({lo:+.2f}, {hi:+.2f})")
            f = (grip - lo) / max(hi - lo, 1e-6)
            ft = (gtgt - lo) / max(hi - lo, 1e-6)
            near = 9.9
        f = min(max(f, 0.0), 1.0)
        ft = min(max(ft, 0.0), 1.0)
        cv.coords(r["bar"], BAR_X0, ROW_Y0 + i * ROW_H + 6,
                  BAR_X0 + f * (BAR_X1 - BAR_X0), ROW_Y0 + i * ROW_H + 23)
        cv.itemconfig(r["bar"], fill=(RED if frz else (WARN if near < math.radians(margin * 2) else OKC)))
        cv.coords(r["mark"], BAR_X0 + ft * (BAR_X1 - BAR_X0), ROW_Y0 + i * ROW_H + 2,
                  BAR_X0 + ft * (BAR_X1 - BAR_X0), ROW_Y0 + i * ROW_H + 27)
        cv.itemconfig(r["name"], text=names[i], fill=(ACC if i == sel else FG))
        cv.itemconfig(r["sel"], text="▶" if i == sel else "")
        cv.itemconfig(r["txt"], text=txt, fill=(WARN if i == sel else FG))

    dpp = deg_per_px()
    cv.itemconfig(title, text=f"🦾 关节直控  ▶ {names[sel]}   "
                              f"{'细调' if fine else '粗调'} ({dpp:.3f}°/px   "
                              f"拖 100px = {dpp*100:.1f}°)   轴: {'横向 X' if axis == 'x' else '纵向 Y'}")
    cv.itemconfig(sub, text=f"控制 {hz:5.1f} Hz   软限位余量 {margin:.1f}°   "
                            f"{'❄ 冻结' if frz else '▶ 可动'}   {'✅ 就绪' if ready else '⏳ 启动中'}")
    cv.itemconfig(peninfo, text=f"笔: x={pen[0]:6.0f} y={pen[1]:6.0f}   "
                                f"{'按住(控制中)' if pressed else '松开(停)'}   "
                                f"选中 {names[sel]}")
    cv.itemconfig(status, text=st + (f"   ❌ {err}" if err else ""), fill=(RED if err else OKC))

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


# ── 笔 / 键盘事件 ─────────────────────────────────────────────────────────────


def deg_per_px() -> float:
    with S.lock:
        sweep = S.sweep_deg
        fine = S.fine
        ff = S.fine_factor
    span = max(BAR_X1 - BAR_X0, 1)
    d = sweep / span
    return d / ff if fine else d


def clamp_target(i: int) -> None:
    lo, hi = LIM_LO[i], LIM_HI[i]
    m = math.radians(S.margin_deg)
    S.q_tgt[i] = min(max(S.q_tgt[i], lo + m), hi - m)


def on_press(e) -> None:
    if e.num == 3:                      # 笔侧键 = 冻结切换
        toggle_freeze()
        return
    with S.lock:
        S.pressed = True
        S.pen = (float(e.x), float(e.y))
        S.pen_t = time.monotonic()
        S.anchor = (float(e.x), float(e.y))
        S.q_at_press = float(S.q_tgt[S.sel])
    print(f"[pen] 按下 x={e.x} y={e.y} 选中=J{S.sel+1} 基准={math.degrees(S.q_at_press):+.2f}°", flush=True)


def on_release(e) -> None:
    with S.lock:
        S.pressed = False
    print("[pen] 松开（停住保持）", flush=True)


def on_motion(e) -> None:
    with S.lock:
        S.pen = (float(e.x), float(e.y))
        if not S.pressed:               # 没按下 = 悬停移动，只更新显示
            return
        S.pen_t = time.monotonic()
        ax, ay = S.anchor
        sel = S.sel
        base = S.q_at_press
        axis = S.axis
    d = deg_per_px()
    if axis == "x":
        ddeg = (e.x - ax) * d
    else:
        ddeg = -(e.y - ay) * d
    with S.lock:
        S.q_tgt[sel] = base + math.radians(ddeg)
        clamp_target(sel)


def on_key(e) -> None:
    k = e.keysym
    with S.lock:
        sel = S.sel
        nj = S.n_joints
        shift = bool(e.state & 0x0001)
    if k == "Escape":
        quit_app()
    elif k in "1234567":
        i = int(k) - 1
        if i < nj:
            with S.lock:
                S.sel = i
            print(f"[key] 选中 J{i+1 if i < 6 else '夹爪(7)'}", flush=True)
        else:
            print("[key] 夹爪未启用（--gripper）", flush=True)
    elif k.lower() == "f":
        with S.lock:
            S.fine = not S.fine
        print(f"[key] {'细调' if S.fine else '粗调'}", flush=True)
    elif k == "space":
        toggle_freeze()
    elif k.lower() == "g":
        with S.lock:
            S.q_tgt[sel] = 0.0
        print(f"[key] J{sel+1} → 0（平滑）", flush=True)
    elif k.lower() == "z":
        with S.lock:
            S.q_tgt[:] = 0.0
        print("[key] 全部 → 0（平滑）", flush=True)
    elif k.lower() == "a":
        with S.lock:
            S.q_tgt = S.q.copy()
            S.q_send = S.q.copy()
        print("[key] 对齐：目标 = 实测", flush=True)
    elif k.lower() == "x":
        with S.lock:
            S.axis = "x"
        print("[key] 控制轴 = 横向 X", flush=True)
    elif k.lower() == "y":
        with S.lock:
            S.axis = "y"
        print("[key] 控制轴 = 纵向 Y", flush=True)
    elif k in ("Left", "Right"):
        step = math.radians(NUDGE_FINE if shift else NUDGE_COARSE)
        sign = 1.0 if k == "Right" else -1.0
        with S.lock:
            S.q_tgt[sel] += sign * step
            clamp_target(sel)
            v = math.degrees(S.q_tgt[sel])
        print(f"[key] J{sel+1} {'+' if sign > 0 else '−'}{math.degrees(step):.2f}° → {v:+.2f}°", flush=True)


_quitting = False


def toggle_freeze() -> None:
    with S.lock:
        S.freeze = not S.freeze
        if S.freeze:
            S.pressed = False
            S.alarm = ""
        else:
            # 解冻时重新锚定，避免"冻着的时候笔还按着"造成跳变
            S.anchor = S.pen
            S.q_at_press = float(S.q_tgt[S.sel])
        v = S.freeze
    print(f"[key] 冻结 {'ON（保持悬停）' if v else 'off'}", flush=True)


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


# ── 自检模式（无需人操作，验证链路与精度）────────────────────────────────────


def selftest(args) -> None:
    """脚本化动作：验证 目标/限速/限位/回零。"""
    script = [
        ("选中 J6（自转）", lambda: setattr(S, "sel", 5)),
        ("J6 目标 +5°", lambda: setattr(S, "q_tgt", _set(S.q_tgt, 5, math.radians(5)))),
        ("等 2.5s 让它走到位", None),
        ("J6 目标 -5°", lambda: setattr(S, "q_tgt", _set(S.q_tgt, 5, math.radians(-5)))),
        ("等 2.5s", None),
        ("J1 目标 +3°", lambda: setattr(S, "q_tgt", _set(S.q_tgt, 0, math.radians(3)))),
        ("等 2.5s", None),
        ("全部回 0", lambda: setattr(S, "q_tgt", np.zeros(6))),
        ("等 3.0s", None),
    ]
    for name, act in script:
        print(f"[selftest] {name}", flush=True)
        if act is not None:
            with S.lock:
                act()
        time.sleep(2.5 if "等" in name else 0.6)
    print("[selftest] 完成，退出中…", flush=True)


def _set(arr: np.ndarray, i: int, v: float) -> np.ndarray:
    a = arr.copy()
    a[i] = v
    return a


# ── 主程序 ────────────────────────────────────────────────────────────────────


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="数位板逐电机直控")
    ap.add_argument("--sweep-deg", type=float, default=45.0, help="横向满窗拖动 = 多少度（粗调）")
    ap.add_argument("--fine-factor", type=float, default=10.0, help="细调倍数")
    ap.add_argument("--max-vel", type=float, default=SLEW_DEFAULT, help="关节默认限速 rad/s")
    ap.add_argument("--max-vel-joint", action="append", default=[],
                    help="逐关节限速，如 --max-vel-joint J6=1.0（可多次）")
    ap.add_argument("--limit-margin", type=float, default=1.5, help="软限位余量（度）")
    ap.add_argument("--tau-limit", type=float, default=0.0, help="力矩保护阈值 N·m（0=关闭）")
    ap.add_argument("--gripper", action="store_true", help="启用夹爪通道（7 号）")
    ap.add_argument("--grip-range", type=float, default=0.35, help="夹爪半行程")
    ap.add_argument("--grip-slew", type=float, default=0.6, help="夹爪限速")
    ap.add_argument("--grip-kp", type=float, default=50.0)
    ap.add_argument("--grip-kd", type=float, default=4.0)
    ap.add_argument("--selftest", action="store_true", help="自检：脚本化动 J6/J1 后归零")
    args = ap.parse_args()

    signal.signal(signal.SIGINT, lambda *_: quit_app())
    signal.signal(signal.SIGTERM, lambda *_: quit_app())   # pkill/kill 也能走平滑归零
    with S.lock:
        S.sweep_deg = args.sweep_deg
        S.fine_factor = args.fine_factor
        S.margin_deg = args.limit_margin

    cv.bind("<ButtonPress>", on_press)
    cv.bind("<ButtonRelease>", on_release)
    cv.bind("<Motion>", on_motion)
    root.bind("<KeyPress>", on_key)
    root.protocol("WM_DELETE_WINDOW", quit_app)
    root.focus_force()

    th = threading.Thread(target=control_loop, args=(args,), daemon=True)
    th.start()
    if args.selftest:
        threading.Thread(target=selftest, args=(args,), daemon=True).start()

    print("[gui] 窗口已就绪 —— 笔=运动，键盘=模式", flush=True)
    print("      1~6 选关节 | F 粗细 | Shift 细调 | ←→ 微调 | G 回零 | Z 全回零 | "
          "A 对齐 | X/Y 换轴 | Space 冻结 | Esc 退出", flush=True)
    root.after(100, draw)
    root.mainloop()
    th.join(timeout=15.0)
    print(f"[joint] 结束 status={S.status} error={S.error}", flush=True)
