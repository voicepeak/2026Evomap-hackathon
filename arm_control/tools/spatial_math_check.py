"""复刻 spatial_teleop 实时循环的数学，逐项找出"指令偏小"的原因（离线，不动电机）。"""
import sys
import numpy as np
import pinocchio as pin

REPO = "/Users/Admin/Desktop/reBotArm_control_py"
sys.path.insert(0, REPO)
from reBotArm_control_py.kinematics import (  # noqa: E402
    get_end_effector_frame_id, joint_to_pose, load_robot_model, pad_q_for_model,
)

model = load_robot_model()
data = model.createData()
fid = get_end_effector_frame_id(model)
lo = np.asarray(model.lowerPositionLimit[:6], float) + np.radians(1.5)
hi = np.asarray(model.upperPositionLimit[:6], float) - np.radians(1.5)
q0 = np.array([0.0, 0.006, 0.004, -0.001, 0.0, -0.003])
V = np.array([0.02, 0.0, 0.0])
DT = 1 / 200
LIMIT_SOFT, LIMIT_K, DAMPING, POSTURE_K, QD_MAX = 0.12, 0.6, 0.02, 0.2, 1.0


def repulse(q):
    z = np.zeros(6)
    for i in range(6):
        m = min(LIMIT_SOFT, 0.5 * (hi[i] - lo[i]))
        if q[i] < lo[i] + m:
            z[i] = LIMIT_K * (1.0 - (q[i] - lo[i]) / m)
        elif q[i] > hi[i] - m:
            z[i] = -LIMIT_K * (1.0 - (hi[i] - q[i]) / m)
    return np.clip(z, -LIMIT_K, LIMIT_K)


def run(tag, use_repulse, use_posture, use_clip, use_fk_first):
    q = q0.copy()
    q_seed = q0.copy()
    for _ in range(int(1.6 * DT ** -1)):
        qp = pad_q_for_model(model, q, 6)
        if use_fk_first:
            pin.framesForwardKinematics(model, data, qp)      # ← 关键：先更新帧位姿
        pin.computeJointJacobians(model, data, qp)
        J = np.asarray(pin.getFrameJacobian(model, data, fid, pin.LOCAL_WORLD_ALIGNED), float)[:, :6]
        Jv = J[:3]
        smin = float(np.linalg.svd(Jv, compute_uv=False)[-1])
        lam = DAMPING * (1.0 + 4.0 * max(0.0, 0.06 - smin) / 0.06)
        A = Jv @ Jv.T + lam**2 * np.eye(3)
        Jpinv = Jv.T @ np.linalg.inv(A)
        qd = Jpinv @ V
        N = np.eye(6) - Jpinv @ Jv
        z = np.zeros(6)
        if use_repulse:
            z += repulse(q)
        if use_posture:
            z += -POSTURE_K * (q - q_seed)
        qd = qd + N @ z
        if use_clip:
            qd = np.clip(qd, -QD_MAX, QD_MAX)
        q = np.clip(q + qd * DT, lo, hi)
    p1, _ = joint_to_pose(q)
    dq = np.degrees(q - q0)
    dp = (np.asarray(p1) - np.asarray(joint_to_pose(q0)[0])) * 100
    print(f"{tag:44s} ΔX={dp[0]:+5.2f}cm  关节Δ=[{dq[0]:+5.1f} {dq[1]:+5.1f} {dq[2]:+5.1f} "
          f"{dq[3]:+5.1f} {dq[4]:+5.1f} {dq[5]:+5.1f}]")


print("目标: 前伸 2cm/s × 1.6s = +3.2cm（理想关节Δ 约 J2+22 J3+5 J4+14）")
print()
run("A 纯主任务（无FK更新）", False, False, False, False)
run("B 纯主任务（先做FK更新）", False, False, False, True)
run("C 主+限位排斥（无FK）", True, False, False, False)
run("D 主+构型正则（无FK）", False, True, False, False)
run("E 全部开启（无FK）", True, True, True, False)
run("F 全部开启（先做FK更新）", True, True, True, True)
print()
print("→ 如果 A 和 B 差很多，就是帧位姿未更新导致雅可比错误（实时循环正是缺了 FK 更新）")
