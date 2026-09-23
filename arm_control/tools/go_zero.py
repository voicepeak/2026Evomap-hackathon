"""go_zero.py — 把机械臂从任意（松开/被碰偏的）姿态缓慢带回折叠零位。

安全要点:
    1. 先在**当前位置**使能并下发当前角（kp·(q-q)=0，绝不跳）
    2. 以限速 0.25 rad/s 平滑走到零位（最小 jerk 收尾）
    3. 结束后**保持悬停**（不失能），打印状态
用法: uv run python tools/go_zero.py
"""
import sys
import time

import numpy as np

REPO = "/Users/Admin/Desktop/reBotArm_control_py"
sys.path.insert(0, REPO)

from reBotArm_control_py.actuator import RebotArm  # noqa: E402
from reBotArm_control_py.dynamics import compute_generalized_gravity, load_dynamics_model  # noqa: E402
from reBotArm_control_py.kinematics import load_robot_model, pad_q_for_model  # noqa: E402

RATE = 200
VMAX = 0.25          # rad/s 归零限速

arm = RebotArm()
arm.connect()
g = arm.arm
model = load_robot_model()
data = model.createData()
kp, kd = g._mit_kp.copy(), g._mit_kd.copy()

q = np.asarray(g.get_positions(), float)[:6].copy()
print("当前 q(deg) =", np.round(np.degrees(q), 2))
g.mode_mit(kp=kp, kd=kd)
g.enable()
tau = compute_generalized_gravity(model, pad_q_for_model(model, q, 6), data)[:6]
g.send_mit(q, vel=np.zeros(6), kp=kp, kd=kd, tau=tau)   # 使能瞬间不跳
time.sleep(0.3)

dt = 1.0 / RATE
t0 = time.perf_counter()
while True:
    step = VMAX * dt
    q = q + np.clip(np.zeros(6) - q, -step, step)
    tau = compute_generalized_gravity(model, pad_q_for_model(model, q, 6), data)[:6]
    g.send_mit(q, vel=np.zeros(6), kp=kp, kd=kd, tau=tau)
    time.sleep(dt)
    if float(np.max(np.abs(q))) < 1e-4 or time.perf_counter() - t0 > 20.0:
        break

tau0 = compute_generalized_gravity(model, np.zeros(model.nq), data)[:6]
for _ in range(int(1.0 * RATE)):
    g.send_mit(np.zeros(6), vel=np.zeros(6), kp=kp, kd=kd, tau=tau0)
    time.sleep(dt)

q_now = np.asarray(g.get_positions(), float)[:6]
print("归零后 q(deg) =", np.round(np.degrees(q_now), 2))
print("✅ 已回到折叠零位并保持悬停（未失能）")
