"""return_home.py — 把机械臂从任意姿态温和带回折叠零位（带力矩/跟随监测）。

与 go_zero.py 的区别：本工具会**一路盯着力矩和跟随误差**，
一旦某关节顶住（力矩超限或指令走了但关节不动）→ 立刻停下并保持，不硬来。

用法:
    uv run python tools/return_home.py                 # 默认 0.4 rad/s，力矩上限 8 N·m
    uv run python tools/return_home.py --speed 0.25 --tau-limit 6
"""
from __future__ import annotations

import argparse
import math
import sys
import time

import numpy as np

REPO = "/Users/Admin/Desktop/reBotArm_control_py"
sys.path.insert(0, REPO)

from reBotArm_control_py.actuator import RebotArm  # noqa: E402
from reBotArm_control_py.dynamics import compute_generalized_gravity, load_dynamics_model  # noqa: E402
from reBotArm_control_py.kinematics import joint_to_pose, load_robot_model, pad_q_for_model  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--speed", type=float, default=0.40, help="关节最大速度 rad/s")
ap.add_argument("--tau-limit", type=float, default=25.0, help="力矩上限 N·m（大伸展姿态需要更大）")
ap.add_argument("--track-tol", type=float, default=0.35, help="跟随误差上限 rad（持续超限=卡住）")
ap.add_argument("--max-secs", type=float, default=40.0)
args = ap.parse_args()

arm = RebotArm()
arm.connect()
g = arm.arm
model = load_robot_model()
data = model.createData()
kp, kd = g._mit_kp.copy(), g._mit_kd.copy()

q = np.asarray(g.get_positions(), float)[:6].copy()
p, _ = joint_to_pose(q)
print(f"当前 q(deg) = {np.round(np.degrees(q), 1)}")
print(f"当前末端 = ({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f})")
print(f"最大偏差 {float(np.max(np.abs(q))):.3f} rad，用 {args.speed} rad/s 归零 "
      f"（预计 {float(np.max(np.abs(q)))/args.speed:.1f}s）\n", flush=True)

g.mode_mit(kp=kp, kd=kd)
g.enable()
tau = compute_generalized_gravity(model, pad_q_for_model(model, q, 6), data)[:6]
g.send_mit(q, vel=np.zeros(6), kp=kp, kd=kd, tau=tau)     # 使能瞬间不跳
time.sleep(0.4)

dt = 1.0 / 200
t0 = time.perf_counter()
bad_since = 0.0
stop_msg = ""
while True:
    step = args.speed * dt
    q = q + np.clip(np.zeros(6) - q, -step, step)
    tau = compute_generalized_gravity(model, pad_q_for_model(model, q, 6), data)[:6]
    g.send_mit(q, vel=np.zeros(6), kp=kp, kd=kd, tau=tau)
    time.sleep(dt)

    now = time.perf_counter()
    if now - t0 > args.max_secs:
        stop_msg = f"超过 {args.max_secs}s 上限"
        break
    if int((now - t0) * 5) % 10 == 0 and int((now - t0) * 200) % 400 == 0:
        qm = np.asarray(g.get_positions(), float)[:6]
        pm, _ = joint_to_pose(qm)
        print(f"  t={now-t0:4.1f}s  q(deg)={np.round(np.degrees(qm), 1)}  "
              f"末端Z={pm[2]:+.3f}  |τ|max={np.max(np.abs(tau)):.1f}N·m", flush=True)

    # 每 20ms 检查一次
    if int((now - t0) * 50) % 1 == 0:
        qm = np.asarray(g.get_positions(), float)[:6]
        tqm = np.asarray(arm.get_state()[2], float)[:6]
        err = float(np.max(np.abs(q - qm)))
        over = float(np.max(np.abs(tqm)))
        if over > args.tau_limit or err > args.track_tol:
            if bad_since == 0.0:
                bad_since = now
            elif now - bad_since > 0.4:
                stop_msg = (f"关节顶住/卡住：力矩 {over:.1f}N·m，跟随误差 {err:.2f}rad "
                            f"(J{int(np.argmax(np.abs(tqm)))+1} 力矩最大)")
                break
        else:
            bad_since = 0.0

    if float(np.max(np.abs(q))) < 1e-3:
        stop_msg = "已到零位"
        break

q_end = np.asarray(g.get_positions(), float)[:6]
print(f"\n⏹ {stop_msg}")
print(f"   结束 q(deg) = {np.round(np.degrees(q_end), 1)}   "
      f"距零位 {float(np.max(np.abs(q_end)))*57.3:.1f}°", flush=True)
if "零位" in stop_msg:
    tau0 = compute_generalized_gravity(model, np.zeros(model.nq), data)[:6]
    for _ in range(int(1.0 * 200) if False else 200):
        g.send_mit(np.zeros(6), vel=np.zeros(6), kp=kp, kd=kd, tau=tau0)
        time.sleep(dt)
    print("✅ 已回到折叠零位并保持（未失能）", flush=True)
else:
    print("⚠️ 中途停下并**持续托住**当前位置（不会松手）——请检查是否被挡住，"
          "或告我继续", flush=True)
    t_last = 0.0
    while True:
        tau = compute_generalized_gravity(model, pad_q_for_model(model, q_end, 6), data)[:6]
        g.send_mit(q_end, vel=np.zeros(6), kp=kp, kd=kd, tau=tau)
        time.sleep(dt)
        if time.perf_counter() - t_last > 3.0:
            t_last = time.perf_counter()
            qm = np.asarray(g.get_positions(), float)[:6]
            tqm = np.asarray(arm.get_state()[2], float)[:6]
            print(f"   [托住中] q(deg)={np.round(np.degrees(qm),1)} "
                  f"|τ|max={np.max(np.abs(tqm)):.1f}N·m", flush=True)
