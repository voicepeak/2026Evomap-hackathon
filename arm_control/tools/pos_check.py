"""pos_check.py — 启动前检查：机械臂当前关节角 + 是否可安全启动。

注意（实测确认）:
    - CAN 总线是**独占**的：如果已有控制脚本在跑（hybrid/spatial/joint），本脚本
      会打不开总线并报错退出，因此它不可能影响正在受控的机械臂。
    - 本脚本用 RebotArm.connect()（会对电机 clear_error，和正式脚本启动时一样），
      读完就退出，不 enable / 不 disable / 不发指令。
    - 想要"绝对不碰电机状态"的读取，目前 motorbridge 的直连方式拿不到反馈，暂时做不到。
"""
import sys
import numpy as np

REPO = "/Users/Admin/Desktop/reBotArm_control_py"
sys.path.insert(0, REPO)

from reBotArm_control_py.actuator import RebotArm  # noqa: E402

arm = RebotArm()
arm.connect()
g = arm.arm
q = np.asarray(g.get_positions(), float)[:6]
print("q(deg) =", np.round(np.degrees(q), 2))
print("|q|max =", round(float(np.max(np.abs(q))), 4), "rad  阈值 0.05")
print("结论:", "✅ 接近折叠零位，可以安全启动" if float(np.max(np.abs(q))) <= 0.05
      else "⚠️ 偏离零位 >0.05 rad，启动保护会拒绝（这不是故障）")
