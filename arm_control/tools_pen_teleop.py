#!/usr/bin/env python3
"""reBot B601-RS pen-tablet teleop (Wacom CTL-672 tested).

Pen tablet drives the end-effector in an absolute-but-clutched way:
  * hover            -> aim preview only (arm keeps hovering / holding torque)
  * press pen tip    -> engage; pen motion since tip-down moves the target
  * release pen tip  -> disengage; arm holds the last target
  * side button 1    -> switch plane (X-Z front view  <->   X-Y top view)
  * side button 2    -> freeze / unfreeze
  * key R            -> slew to the ready pose
  * key Space        -> freeze toggle
  * key Esc / close  -> min-jerk home, then KEEP HOLDING (never disable!)

Safety note: this program never calls disable_all()/disconnect() by itself, so
the arm can not lose torque and drop.  To explicitly cut torque, run
disable_arm.py (or power off while the arm is folded).
"""
from __future__ import annotations

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
from reBotArm_control_py.kinematics import (  # noqa: E402
    get_end_effector_frame_id,
    joint_to_pose,
    load_robot_model,
    pad_q_for_model,
    pos_rot_to_se3,
)
from reBotArm_control_py.kinematics.inverse_kinematics import IKParams, solve_ik  # noqa: E402

# ── tuning ────────────────────────────────────────────────────────────────────
RATE = 200                 # control loop Hz
SLEW = 1.2                 # joint target slew limit (rad/s) ~0.35 m/s EE speed
SCALE = 0.00105            # m per pixel of pen travel
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
        self.target = np.zeros(3)
        self.ee = np.zeros(3)
        self.q = np.zeros(6)
        self.status = "连接中…"
        self.ik = "—"
        self.engaged = False
        self.freeze = False
        self.plane = "XY"          # 默认俯视：笔左右→Y，笔上下→深度 X（Shift 临时切高度 Z）
        self.ready = False
        self.ready_req = 0.0
        self.rate_hz = 0.0
        self.stop = False
        self.q_ref = np.zeros(6)
        self.error: str | None = None


S = State()


def control_loop() -> None:
    arm = None
    g = None
    model = None
    data = None
    fid = None
    kp = kd = None
    q_cmd = np.zeros(6)
    clean = False

    try:
        arm = RebotArm()
        arm.connect()
        g = arm.arm
        model = load_robot_model()
        data = model.createData()
        fid = get_end_effector_frame_id(model)
        kp, kd = g._mit_kp.copy(), g._mit_kd.copy()
        g.mode_mit(kp=kp, kd=kd)
        g.enable()

        q_meas = np.asarray(g.get_positions(), float)[:6]
        p, rpy_lock = joint_to_pose(np.asarray(arm.get_state()[0], float))
        rpy_lock = np.asarray(rpy_lock, float)   # 启动时锁定姿态 → IK 不再乱补偿手腕
        q_cmd = q_meas.copy()
        with S.lock:
            S.q = q_meas.copy()
            S.ee = np.asarray(p, float)
            S.target = np.asarray(p, float).copy()
            S.ready = True
            S.status = "保持悬停（抬笔）"

        dt_nom = 1.0 / RATE
        t_prev = time.perf_counter()
        t0 = t_prev
        n = 0
        goal_until = 0.0
        was_eng = False
        q_ref = q_meas.copy()

        while not S.stop:
            now = time.perf_counter()
            dt = max(1e-4, min(now - t_prev, 0.05))
            t_prev = now

            q_meas = np.asarray(g.get_positions(request_feedback=False), float)[:6]
            tau = compute_generalized_gravity(model, pad_q_for_model(model, q_meas, 6), data)[:6]

            with S.lock:
                tgt = S.target.copy()
                eng = S.engaged and not S.freeze
                rq = S.ready_req
                S.ready_req = 0.0
                S.q = q_meas.copy()
                S.q_ref = q_ref.copy()
            if eng and not was_eng:
                q_ref = q_meas.copy()          # 落笔瞬间记录参考角，用于显示 Δ
                print(f"[teleop] 落笔 q_ref={np.round(q_ref, 3)}", flush=True)
            was_eng = eng
            if rq > 0:
                goal_until = now + 4.0

            if now < goal_until:
                q_goal = Q_READY.copy()
            elif eng:
                res = solve_ik(
                    model, data, fid,
                    pos_rot_to_se3(tgt, roll=float(rpy_lock[0]), pitch=float(rpy_lock[1]), yaw=float(rpy_lock[2])),
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
            else:
                q_goal = np.clip(q_meas, JMIN, JMAX)

            step = SLEW * dt
            q_cmd = q_cmd + np.clip(q_goal - q_cmd, -step, step)
            g.send_mit(q_cmd, vel=np.zeros(6), kp=kp, kd=kd, tau=tau)

            n += 1
            if n % 10 == 0:
                p_now, _ = joint_to_pose(np.asarray(arm.get_state()[0], float))
                with S.lock:
                    S.ee = np.asarray(p_now, float)
                    S.rate_hz = n / max(now - t0, 1e-6)
                    S.status = "跟随笔" if eng else ("冻结（悬停）" if S.freeze else "保持悬停（抬笔）")
            if n % 100 == 0:
                print(
                    f"[teleop] t={now - t0:6.1f}s eng={int(eng)} "
                    f"q={np.round(q_meas, 3)} Δref={np.round(q_meas - q_ref, 3)}",
                    flush=True,
                )

            time.sleep(max(0.0, dt_nom - (time.perf_counter() - now)))

        clean = True

    except Exception as e:  # noqa: BLE001
        with S.lock:
            S.error = f"{type(e).__name__}: {e}"
            S.status = "错误（保持悬停）"
        print(f"[pen-teleop] 异常: {S.error} —— 不执行失能，电机保持最后指令", flush=True)

    finally:
        # ── shutdown: home + KEEP HOLDING.  Never disable, never disconnect. ──
        if g is not None and kp is not None:
            try:
                if clean:
                    with S.lock:
                        S.status = "平滑归零中…"
                    print("[pen-teleop] 平滑归零（最小 jerk）…", flush=True)
                    steps = int(3.0 * RATE)
                    for i in range(1, steps + 1):
                        a = i / steps
                        s = 10 * a**3 - 15 * a**4 + 6 * a**5
                        q_t = q_cmd + (np.zeros(6) - q_cmd) * s
                        tau = compute_generalized_gravity(model, pad_q_for_model(model, q_t, 6), data)[:6]
                        g.send_mit(q_t, vel=np.zeros(6), kp=kp, kd=kd, tau=tau)
                        time.sleep(1.0 / RATE)
                    # final hold at zero for a moment
                    tau0 = compute_generalized_gravity(model, np.zeros(model.nq), data)[:6]
                    for _ in range(int(0.5 * RATE)):
                        g.send_mit(np.zeros(6), vel=np.zeros(6), kp=kp, kd=kd, tau=tau0)
                        time.sleep(1.0 / RATE)
                    print("[pen-teleop] 已归零并保持悬停（未失能）。"
                          "电机将持续保持最后指令；需要断电请先断电或运行 disable_arm.py", flush=True)
                else:
                    # error path: just keep pushing a hold at the last command
                    for _ in range(50):
                        try:
                            g.send_mit(q_cmd, vel=np.zeros(6), kp=kp, kd=kd, tau=np.zeros(6))
                        except Exception:  # noqa: BLE001
                            break
                        time.sleep(0.01)
                    print("[pen-teleop] 异常退出：已保持最后位置，未失能", flush=True)
            except Exception as e:  # noqa: BLE001
                print(f"[pen-teleop] 收尾发送失败（电机仍保持最后指令，不会掉）: {e}", flush=True)
        with S.lock:
            S.ready = False
            S.status = "已归零·悬停中（未失能）"


# ── GUI ───────────────────────────────────────────────────────────────────────
W, H = 1300, 800
CANVAS_W, CANVAS_H = 780, 640
MX, MY = 60, 60

root = tk.Tk()
root.title("reBot 笔式遥操 — Wacom CTL-672")
root.geometry(f"{W}x{H}+60+40")
root.attributes("-topmost", True)

cv = tk.Canvas(root, width=CANVAS_W, height=CANVAS_H, bg="#111827", highlightthickness=0)
cv.place(x=20, y=20)
panel = tk.Canvas(root, width=440, height=CANVAS_H, bg="#0b1220", highlightthickness=0)
panel.place(x=CANVAS_W + 40, y=20)

cv.create_text(CANVAS_W / 2, 24, text="控制区：落笔画动 → 机械臂末端跟随（抬笔保持，全程悬停）",
               fill="#9ca3af", font=("Helvetica", 15))

pen_dot = cv.create_oval(0, 0, 0, 0, outline="#f472b6", width=2)
tgt_dot = cv.create_oval(0, 0, 0, 0, fill="#facc15", outline="")
ee_dot = cv.create_oval(0, 0, 0, 0, fill="#4ade80", outline="")
trail_line = cv.create_line(0, 0, 0, 0, fill="#38bdf8", width=2)
pen_trail: list[tuple[float, float]] = []


def world_to_px(y: float, z: float) -> tuple[float, float]:
    px = MX + (y - BY[0]) / (BY[1] - BY[0]) * (CANVAS_W - 2 * MX)
    py = CANVAS_H - MY - (z - BZ[0]) / (BZ[1] - BZ[0]) * (CANVAS_H - 2 * MY)
    return px, py


def world_to_px_xy(x: float, y: float) -> tuple[float, float]:
    px = MX + (y - BY[0]) / (BY[1] - BY[0]) * (CANVAS_W - 2 * MX)
    py = CANVAS_H - MY - (x - BX[0]) / (BX[1] - BX[0]) * (CANVAS_H - 2 * MY)
    return px, py


def redraw_static(plane: str) -> None:
    cv.delete("static")
    if plane == "XZ":
        x0, y0 = world_to_px(BY[0], BZ[0])
        x1, y1 = world_to_px(BY[1], BZ[1])
        cv.create_text(x1 - 90, y1 - 14, text="Z 上 (m)", fill="#6b7280", font=("Helvetica", 11), tags="static")
        cv.create_text(x0 + 6, y0 - 16, text="← Y 左右 (m) →", fill="#6b7280", font=("Helvetica", 11), tags="static")
        cx, cy = world_to_px(0.0, 0.0)
        rpx = RMAX / (BY[1] - BY[0]) * (CANVAS_W - 2 * MX)
        cv.create_oval(cx - rpx, cy - rpx, cx + rpx, cy + rpx, outline="#1f2937", width=2, tags="static")
    else:
        x0, y0 = world_to_px_xy(BX[0], BY[0])
        x1, y1 = world_to_px_xy(BX[1], BY[1])
        cv.create_text(x1 - 110, y1 - 14, text="X 前 (m)", fill="#6b7280", font=("Helvetica", 11), tags="static")
        cv.create_text(x0 + 6, y0 - 16, text="← Y 左右 (m) →", fill="#6b7280", font=("Helvetica", 11), tags="static")
    cv.create_rectangle(x0, y1, x1, y0, outline="#374151", width=2, tags="static")


redraw_static("XZ")

panel_text = panel.create_text(20, 20, anchor="nw", fill="#e5e7eb", font=("Menlo", 14), text="")
panel.create_text(
    20, CANVAS_H - 230, anchor="nw", fill="#9ca3af", font=("Menlo", 12),
    text=("操作说明\n"
          "  悬停        → 瞄准预览（不动）\n"
          "  落笔拖动    → 末端跟随\n"
          "  抬笔        → 保持当前位置\n"
          "  侧键 1      → 切换平面 XZ / XY\n"
          "  侧键 2      → 冻结 / 解冻\n"
          "  R           → 回到 ready 姿态\n"
          "  Space       → 冻结切换\n"
          "  Esc         → 平滑归零 + 保持悬停\n"
          "                （绝不失能，不会掉落）"),
)

anchor_pen = [0.0, 0.0]
anchor_tgt = np.zeros(3)
engaged = False


def pen_anchor(e) -> None:
    global engaged
    with S.lock:
        if not S.ready:
            return
        anchor_tgt[:] = S.target
        engaged = True
        S.engaged = True
    anchor_pen[0], anchor_pen[1] = float(e.x), float(e.y)


def pen_release() -> None:
    global engaged
    with S.lock:
        S.engaged = False
        engaged = False


def on_press(e) -> None:
    if e.num == 1:
        pen_anchor(e)
    elif e.num == 2:
        with S.lock:
            S.plane = "XY" if S.plane == "XZ" else "XZ"
            redraw_static(S.plane)
    elif e.num == 3:
        with S.lock:
            S.freeze = not S.freeze


def on_release(e) -> None:
    if e.num == 1:
        pen_release()


def on_motion(e) -> None:
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
    shift = bool(e.state & 0x0001)          # 按住 Shift：把"上下"临时切给高度 Z
    with S.lock:
        if not S.ready:
            return
        plane = S.plane
        if plane == "XY":
            if shift:
                d = np.array([0.0, dx * SCALE, -dy * SCALE])     # 左右→Y, 上下→Z
            else:
                d = np.array([-dy * SCALE, dx * SCALE, 0.0])     # 上下→X(深度), 左右→Y
        else:
            if shift:
                d = np.array([-dy * SCALE, dx * SCALE, 0.0])     # 上下→X(深度)
            else:
                d = np.array([0.0, dx * SCALE, -dy * SCALE])     # 上下→Z
        S.target = clamp_ws(anchor_tgt + d)


def on_key(e) -> None:
    if e.keysym == "Escape":
        quit_app()
    elif e.keysym.lower() == "r":
        with S.lock:
            S.ready_req = time.perf_counter()
    elif e.keysym == "space":
        with S.lock:
            S.freeze = not S.freeze


def quit_app() -> None:
    S.stop = True
    root.destroy()


cv.bind("<ButtonPress>", on_press)
cv.bind("<ButtonRelease>", on_release)
cv.bind("<Motion>", on_motion)
root.bind("<Key>", on_key)
root.protocol("WM_DELETE_WINDOW", quit_app)
root.focus_force()


def tick() -> None:
    with S.lock:
        ee = S.ee.copy()
        tgt = S.target.copy()
        q = S.q.copy()
        q_ref = S.q_ref.copy()
        st = S.status
        ik = S.ik
        plane = S.plane
        eng = S.engaged
        frz = S.freeze
        hz = S.rate_hz
        err = S.error

    if plane == "XZ":
        px, py = world_to_px(float(ee[1]), float(ee[2]))
        tx, ty = world_to_px(float(tgt[1]), float(tgt[2]))
    else:
        px, py = world_to_px_xy(float(ee[0]), float(ee[1]))
        tx, ty = world_to_px_xy(float(tgt[0]), float(tgt[1]))
    cv.coords(ee_dot, px - 7, py - 7, px + 7, py + 7)
    cv.coords(tgt_dot, tx - 5, ty - 5, tx + 5, ty + 5)

    panel.itemconfig(
        panel_text,
        text=(
            f"状态      : {st}\n"
            f"平面      : {plane}  ({'前视 X-Z' if plane == 'XZ' else '俯视 X-Y'})\n"
            f"落笔      : {'按住（跟随）' if eng else '松开（保持）'}\n"
            f"冻结      : {'ON' if frz else 'off'}\n"
            f"控制频率  : {hz:5.1f} Hz\n"
            f"IK        : {ik}\n\n"
            f"末端位置  : X={ee[0]:+.3f} Y={ee[1]:+.3f} Z={ee[2]:+.3f} m\n"
            f"目标位置  : X={tgt[0]:+.3f} Y={tgt[1]:+.3f} Z={tgt[2]:+.3f} m\n"
            f"半径      : {np.hypot(ee[0], ee[1]):.3f} m  (上限 {RMAX})\n\n"
            f"关节角 (rad)  [Δ=相对落笔时，●=正在动]:\n"
            + "\n".join(
                f"  J{i + 1}: {q[i]:+.3f}  Δ{q[i] - q_ref[i]:+.3f} "
                + ("●" if abs(q[i] - q_ref[i]) > 0.02 else " ")
                for i in range(6)
            )
            + (f"\n\n⚠️ {err}" if err else "")
        ),
    )
    root.after(50, tick)


signal.signal(signal.SIGINT, lambda *_: quit_app())

th = threading.Thread(target=control_loop, daemon=True)
th.start()

root.after(100, tick)
root.mainloop()

th.join(timeout=12.0)
print(f"[pen-teleop] 结束 status={S.status} error={S.error}", flush=True)
