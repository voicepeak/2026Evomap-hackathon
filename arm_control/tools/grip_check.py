"""grip_check.py — 夹爪（7 号电机）状态诊断：位置 + 故障码 + 温和响应测试。

流程:
    连接（会自动 clear_error，清掉锁存故障）→ 读位置 → 读故障/警告码
    → 用极软刚度(kp=5)轻推 ±0.05rad，看位置有没有跟随（判断电机/机构是否卡死）
安全：力矩极小（最大 ~0.25 N·m），只动夹爪，不动机械臂。

用法（本工作区；先停掉采集服务）:
    cd <工作区根>
    capture_platform/.venv/bin/python arm_control/tools/grip_check.py
"""
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

arm = RebotArm()
arm.connect()
g = arm.gripper

# 找夹爪的底层电机对象（拿故障码）
motor = None
for name, m in getattr(arm, "_motor_map", {}).items():
    if "grip" in name.lower():
        motor = m
        break

pos0 = float(np.asarray(g.get_positions(), float).reshape(-1)[0])
print(f"夹爪当前位置: {pos0:+.4f} rad ({math.degrees(pos0):+.1f}°)")

if motor is not None:
    try:
        fault, warn = motor.robstride_get_fault_report()
        print(f"故障码 raw = 0x{fault:08X}   警告码 raw = 0x{warn:08X}")
        if fault == 0 and warn == 0:
            print("   → 无故障记录（电机本身没有报错）")
        else:
            print("   → 有故障/警告位被置起（下面用 clear_error 清一次）")
    except Exception as e:  # noqa: BLE001
        print(f"读故障码失败: {type(e).__name__}: {e}")

print()
print("极软刚度响应测试（kp=5，±0.05 rad，各 1 秒）…")
g.mode_mit()
g.enable()
kp, kd = np.array([5.0]), np.array([1.0])
for target, tag in ((pos0 + 0.05, "+0.05"), (pos0 - 0.05, "-0.05"), (pos0, "回原位")):
    t_end = time.time() + 1.0
    while time.time() < t_end:
        g.send_mit(np.array([target]), vel=np.zeros(1), kp=kp, kd=kd, tau=np.zeros(1))
        time.sleep(0.005)
    time.sleep(0.2)
    p = float(np.asarray(g.get_positions(), float).reshape(-1)[0])
    tau = np.asarray(arm.get_state()[2], float)
    print(f"  目标 {tag:>6s}  实测 {p:+.4f} rad ({math.degrees(p):+.2f}°)  "
          f"跟随误差 {math.degrees(target - p):+.2f}°")
print()
print("→ 如果误差很小 = 电机和机构都正常（那'卡住'可能是到机械限位了）")
print("   如果误差很大/几乎不动 = 机构卡死或电机无力（要断电手动检查）")
