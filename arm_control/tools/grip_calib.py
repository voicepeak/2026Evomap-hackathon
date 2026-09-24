"""grip_calib.py — 夹爪行程标定 / 手动驱动小工具。

慢速单向推进，实时打印 位置 / 目标 / 力矩；遇阻力（力矩超阈值）自动停。
行程、速度、方向都可命令行给。

用法（本工作区；注意先停掉采集服务，别两边抢机械臂）:
    cd <工作区根>
    capture_platform/.venv/bin/python arm_control/tools/grip_calib.py --dir +1 --speed 0.5 --max-travel 6
    capture_platform/.venv/bin/python arm_control/tools/grip_calib.py --dir -1 --speed 0.5 --max-travel 6
    capture_platform/.venv/bin/python arm_control/tools/grip_calib.py --dir +1 --release   # 只失能，不驱动

打印的"停止位置"就是这台的机械端（到限位时力矩会超过 --tau-limit）；
把它填进 capture_platform/configs/rebot_b601_rs.json 的 gripper.lo_deg / hi_deg
（留 5~10° 余量），夹爪的程序可用行程就跟真机对齐了。
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

# 本仓库根（arm_control/）：默认就指向本文件所在仓库，可用环境变量覆盖
REPO = os.environ.get("REBOT_ARM_REPO") or str(Path(__file__).resolve().parents[1])
sys.path.insert(0, REPO)

from reBotArm_control_py.actuator import RebotArm  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--dir", type=int, default=1, help="+1 / -1")
ap.add_argument("--speed", type=float, default=0.5, help="rad/s")
ap.add_argument("--max-travel", type=float, default=20.0, help="最大行程 rad")
ap.add_argument("--tau-limit", type=float, default=1.0, help="力矩上限 N·m（超过就停）")
ap.add_argument("--kp", type=float, default=15.0)
ap.add_argument("--kd", type=float, default=2.0)
ap.add_argument("--release", action="store_true", help="只做失能，不驱动")
args = ap.parse_args()

arm = RebotArm()
arm.connect()
g = arm.gripper

if args.release:
    g.disable()
    time.sleep(0.3)
    print("✅ 夹爪已显式失能（不再有任何保持力矩）", flush=True)
    sys.exit(0)

pos0 = float(np.asarray(g.get_positions(), float).reshape(-1)[0])
print(f"起始位置 {pos0:+.4f} rad ({math.degrees(pos0):+.1f}°)   方向 {args.dir:+d}   "
      f"速度 {args.speed} rad/s  最大行程 {args.max_travel} rad "
      f"({math.degrees(args.max_travel):.0f}°)", flush=True)

g.mode_mit()
g.enable()
kp, kd = np.array([args.kp]), np.array([args.kd])
target = pos0
dt = 0.02
t0 = time.perf_counter()
last_log = 0.0
stop_reason = "到达设定行程上限"
while True:
    target += args.dir * args.speed * dt
    if abs(target - pos0) > args.max_travel:
        break
    g.send_mit(np.array([target]), vel=np.zeros(1), kp=kp, kd=kd, tau=np.zeros(1))
    time.sleep(dt)
    now = time.perf_counter()
    if now - last_log > 0.4:
        last_log = now
        pos = float(np.asarray(g.get_positions(), float).reshape(-1)[0])
        tau = float(np.asarray(arm.get_state()[2], float).reshape(-1)[-1])
        print(f"  t={now-t0:5.1f}s  目标={math.degrees(target):+8.1f}°  "
              f"实测={math.degrees(pos):+8.1f}°  力矩={tau:+5.2f}N·m", flush=True)
        if abs(tau) > args.tau_limit:
            stop_reason = f"力矩 {tau:+.2f}N·m 超过阈值 {args.tau_limit} → 判断到头了"
            break

pos_end = float(np.asarray(g.get_positions(), float).reshape(-1)[0])
print(f"\n⏹ 停止原因: {stop_reason}")
print(f"   停止位置 {pos_end:+.4f} rad ({math.degrees(pos_end):+.1f}°)，"
      f"相对起点走了 {math.degrees(pos_end - pos0):+.1f}°")
print("   现在失能释放（人手也掰不动是正常的：这个夹爪是自锁型）", flush=True)
g.disable()
