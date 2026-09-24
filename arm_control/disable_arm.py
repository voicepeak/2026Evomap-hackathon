#!/usr/bin/env python3
"""安全失能：**先平滑归零，再失能**。

流程:
    连接 → 读位置
      ├─ 已在零位附近（< --tol）→ 直接失能
      └─ 否则 → 使能(MIT保持) → 最小 jerk 归零(带重力前馈) → 保持确认 → 失能

应急:
    --force 直接失能（不归零）。⚠️ 若不在折叠位，松开会塌/掉落，仅限自己确认安全时用。

用法:
    ~/.local/bin/uv run python disable_arm.py            # 推荐：安全归零后失能
    ~/.local/bin/uv run python disable_arm.py --force    # 应急：立刻失能
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

import numpy as np

# 本仓库根（arm_control/）：默认指向本文件所在仓库，可用环境变量覆盖
REPO = os.environ.get("REBOT_ARM_REPO") or str(Path(__file__).resolve().parent)
sys.path.insert(0, REPO)

from reBotArm_control_py.actuator import RebotArm  # noqa: E402
from reBotArm_control_py.dynamics import compute_generalized_gravity, load_dynamics_model  # noqa: E402
from reBotArm_control_py.kinematics import joint_to_pose, load_robot_model, pad_q_for_model  # noqa: E402

RATE = 200
TOL = 0.05          # rad，零位容差


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--force", action="store_true", help="不归零，直接失能（有掉落风险）")
    ap.add_argument("--tol", type=float, default=TOL)
    args = ap.parse_args()

    arm = RebotArm()
    arm.connect()
    g = arm.arm
    model = load_dynamics_model()
    data = model.createData()

    q0 = np.asarray(g.get_positions(), float)[:6]
    p, _ = joint_to_pose(np.asarray(arm.get_state()[0], float))
    dev = float(np.max(np.abs(q0)))
    print(f"当前关节角 : {np.round(q0, 4)}")
    print(f"末端位置   : X={p[0]:+.3f} Y={p[1]:+.3f} Z={p[2]:+.3f}")
    print(f"距零位偏差 : {dev:.4f} rad ({np.degrees(dev):.1f}°)", flush=True)

    if args.force:
        arm.disable_all()
        time.sleep(0.3)
        arm.disconnect()
        print("⚠️ 已直接失能（--force）——如在非折叠位，机械臂可能塌落", flush=True)
        return 0

    if dev < args.tol:
        arm.disable_all()
        time.sleep(0.3)
        arm.disconnect()
        print(f"✅ 已在零位附近（偏差 {dev:.4f} rad < {args.tol}），全部电机已发送失能命令", flush=True)
        return 0

    # ── 先归零，再失能 ──
    print(f"→ 不在零位（{dev:.3f} rad），先平滑归零…", flush=True)
    kp, kd = g._mit_kp.copy(), g._mit_kd.copy()
    g.mode_mit(kp=kp, kd=kd)
    g.enable()
    time.sleep(0.3)

    dur = max(2.0, dev / 0.4 * 2.0)          # 约 0.4 rad/s
    steps = int(dur * RATE)
    for i in range(1, steps + 1):
        a = i / steps
        s = 10 * a**3 - 15 * a**4 + 6 * a**5
        q_t = q0 + (np.zeros(6) - q0) * s
        tau = compute_generalized_gravity(model, pad_q_for_model(model, q_t, 6), data)[:6]
        g.send_mit(q_t, vel=np.zeros(6), kp=kp, kd=kd, tau=tau)
        time.sleep(1 / RATE)

    # 保持 + 确认到位
    tau0 = compute_generalized_gravity(model, np.zeros(model.nq), data)[:6]
    for _ in range(int(0.8 * RATE)):
        g.send_mit(np.zeros(6), vel=np.zeros(6), kp=kp, kd=kd, tau=tau0)
        time.sleep(1 / RATE)
    q1 = np.asarray(g.get_positions(), float)[:6]
    dev1 = float(np.max(np.abs(q1)))
    print(f"归零完成   : {np.round(q1, 4)}  偏差 {dev1:.4f} rad ({np.degrees(dev1):.1f}°)", flush=True)

    arm.disable_all()
    time.sleep(0.3)
    arm.disconnect()
    print("✅ 已归零，全部电机已发送失能命令", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
