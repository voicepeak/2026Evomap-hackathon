"""离线验证：雅可比约定 + 速度级积分是否真能产生指定位移（不动电机）。"""
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
q0 = np.array([0.0, 0.006, 0.004, -0.001, 0.0, -0.003])

def jac(q, frame=pin.LOCAL_WORLD_ALIGNED):
    pin.computeJointJacobians(model, data, pad_q_for_model(model, q, 6))
    return np.asarray(pin.getFrameJacobian(model, data, fid, frame), float)[:, :6]

p0, _ = joint_to_pose(q0)
print("起始末端:", np.round(p0, 4))
print()

for name, frame in (("LOCAL_WORLD_ALIGNED", pin.LOCAL_WORLD_ALIGNED),
                    ("WORLD", pin.WORLD), ("LOCAL", pin.LOCAL)):
    J = jac(q0, frame)
    print(f"── {name} ──")
    print("  Jv(3x6):")
    for r in J[:3]:
        print("   ", np.round(r, 4))
    # 数值验证：Jv @ w 是否等于 d(position)/dt
    w = np.array([0.0, 0.1, 0.0, 0.0, 0.0, 0.0])   # 只动 J2
    dt = 1e-4
    p1, _ = joint_to_pose(q0 + w * dt)
    dp_dt = (np.asarray(p1) - np.asarray(p0)) / dt
    print("  Jv@w            =", np.round(J[:3] @ w, 4))
    print("  数值 dP/dt      =", np.round(dp_dt, 4))
    print("  一致:", np.allclose(J[:3] @ w, dp_dt, atol=2e-3))
    print()

print("── 速度级积分仿真：v=(0.02,0,0) 走 1.6s，理论位移 +3.2cm ──")
J = jac(q0)
Jv = J[:3]
lam = 0.02
Jpinv = Jv.T @ np.linalg.inv(Jv @ Jv.T + lam**2 * np.eye(3))
q = q0.copy()
dt = 1 / 200
v = np.array([0.02, 0.0, 0.0])
for i in range(int(1.6 * 200)):
    pin.computeJointJacobians(model, data, pad_q_for_model(model, q, 6))
    J = np.asarray(pin.getFrameJacobian(model, data, fid, pin.LOCAL_WORLD_ALIGNED), float)[:, :6]
    qd = J[:3].T @ np.linalg.inv(J[:3] @ J[:3].T + lam**2 * np.eye(3)) @ v
    q = q + qd * dt
p_end, _ = joint_to_pose(q)
print("  仿真位移:", np.round((np.asarray(p_end) - np.asarray(p0)) * 100, 2), "cm")
print("  关节变化(deg):", np.round(np.degrees(q - q0), 2))
