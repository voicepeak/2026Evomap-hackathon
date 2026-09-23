"""joint_demo.py — 悬停空中，逐个电机小角度转动（用来确认每个电机的作用）。

流程:
    连接 → 在当前位置使能（绝不跳）
      → 平滑抬到"空中悬停位"[0, 34°, 51°, 0, 0, 0]
      → J1..J6 依次：转 +8°（停 1.3s）→ 转回来（停 0.9s）
      → 停在悬停位保持（不失能），每 2 秒打印一次状态
    退出：Ctrl+C / SIGINT → 平滑归零后退出（不会掉）

安全：全局限速、逐关节软限位、重力前馈、使能瞬间不下发跳变。
"""
from __future__ import annotations

import math
import signal
import sys
import time

import numpy as np

REPO = "/Users/Admin/Desktop/reBotArm_control_py"
sys.path.insert(0, REPO)

from reBotArm_control_py.actuator import RebotArm  # noqa: E402
from reBotArm_control_py.dynamics import compute_generalized_gravity, load_dynamics_model  # noqa: E402
from reBotArm_control_py.kinematics import (  # noqa: E402
    joint_to_pose, load_robot_model, pad_q_for_model,
)

RATE = 200
VMAX = 0.45                      # rad/s 运动限速
HOVER = np.array([0.0, 0.60, 0.90, 0.0, 0.0, 0.0])   # 空中悬停位（已验证安全）
STEP = math.radians(8.0)         # 每个电机转的角度
HOLD_MOVE = 1.3
HOLD_BACK = 0.9

stop = False


def _sig(*_):
    global stop
    stop = True


signal.signal(signal.SIGINT, _sig)
signal.signal(signal.SIGTERM, _sig)

arm = RebotArm()
arm.connect()
g = arm.arm
model = load_robot_model()
data = model.createData()
kp, kd = g._mit_kp.copy(), g._mit_kd.copy()
lo = np.asarray(model.lowerPositionLimit[:6], float) + math.radians(1.0)
hi = np.asarray(model.upperPositionLimit[:6], float) - math.radians(1.0)

q = np.asarray(g.get_positions(), float)[:6].copy()
print(f"[demo] 当前关节角(deg) = {np.round(np.degrees(q), 2)}")
g.mode_mit(kp=kp, kd=kd)
g.enable()
tau = compute_generalized_gravity(model, pad_q_for_model(model, q, 6), data)[:6]
g.send_mit(q, vel=np.zeros(6), kp=kp, kd=kd, tau=tau)     # 使能瞬间不跳
time.sleep(0.3)
print("[demo] 已使能并保持（此时应对抗重力，手感有劲）", flush=True)


def go_to(target: np.ndarray, tag: str = "") -> None:
    """限速平滑走到 target，走到就返回。"""
    global q
    dt = 1.0 / RATE
    t0 = time.perf_counter()
    while not stop:
        step = VMAX * dt
        q = q + np.clip(target - q, -step, step)
        qc = np.clip(q, lo, hi)
        tau = compute_generalized_gravity(model, pad_q_for_model(model, qc, 6), data)[:6]
        g.send_mit(qc, vel=np.zeros(6), kp=kp, kd=kd, tau=tau)
        time.sleep(dt)
        if float(np.max(np.abs(qc - target))) < 1e-3:
            break
        if time.perf_counter() - t0 > 30.0:
            break
    if tag:
        p, rpy = joint_to_pose(np.asarray(g.get_positions(), float)[:6])
        print(f"[demo] {tag}  末端=({p[0]:+.3f},{p[1]:+.3f},{p[2]:+.3f}) "
              f"rpy(deg)=({np.degrees(rpy[0]):+.1f},{np.degrees(rpy[1]):+.1f},{np.degrees(rpy[2]):+.1f})",
              flush=True)


print("[demo] ① 抬到空中悬停位…")
go_to(HOVER, "已到悬停位")
time.sleep(0.5)

names = ["J1 底座旋转", "J2 肩(大臂俯仰)", "J3 肘(小臂俯仰)", "J4 ?", "J5 ?", "J6 ?"]
for i in range(6):
    if stop:
        break
    print(f"[demo] ② 现在动 {names[i]}（+8°）", flush=True)
    tgt = HOVER.copy()
    tgt[i] = float(np.clip(HOVER[i] + STEP, lo[i], hi[i]))
    go_to(tgt, f"{names[i]} +8°")
    time.sleep(HOLD_MOVE if i == 0 else 0.2)     # 第一个多停一会，后面靠停顿区分
    print(f"[demo]    {names[i]} 转回原位", flush=True)
    go_to(HOVER.copy(), f"{names[i]} 回位")
    time.sleep(HOLD_BACK)

print("[demo] ③ 演示结束，悬停保持中（不失能）。要停可以说「关闭使能」", flush=True)
t0 = time.perf_counter()
while not stop:
    time.sleep(2.0)
    q_now = np.asarray(g.get_positions(), float)[:6]
    p, rpy = joint_to_pose(q_now)
    tau = np.asarray(arm.get_state()[2], float)[:6]
    print(f"[demo] 悬停 q(deg)={np.round(np.degrees(q_now), 1)} "
          f"末端Z={p[2]:+.3f} 力矩max={np.max(np.abs(tau)):.2f}N·m", flush=True)

# 收尾：平滑归零
print("[demo] 平滑归零…", flush=True)
dt = 1.0 / RATE
q = np.asarray(g.get_positions(), float)[:6].copy()
q_from = q.copy()
steps = int(3.0 * RATE)
for k in range(1, steps + 1):
    a = k / steps
    s = 10 * a**3 - 15 * a**4 + 6 * a**5
    qt = q_from * (1.0 - s)
    tau = compute_generalized_gravity(model, pad_q_for_model(model, qt, 6), data)[:6]
    g.send_mit(qt, vel=np.zeros(6), kp=kp, kd=kd, tau=tau)
    time.sleep(dt)
tau0 = compute_generalized_gravity(model, np.zeros(model.nq), data)[:6]
for _ in range(int(0.5 * RATE)):
    g.send_mit(np.zeros(6), vel=np.zeros(6), kp=kp, kd=kd, tau=tau0)
    time.sleep(dt)
print("[demo] 已归零并保持悬停（未失能）。要断电请运行 disable_arm.py", flush=True)
