"""回归：电机直控模式（mode="joint"）。

笔侧键左键选择电机后，笔按该电机的功能象限驱动它：
- 俯仰类（J2/J3/J4）用笔上下，回转类（J1/J5/J6）用笔左右；
- 不产生笛卡尔运动；切换电机时重置锚点，避免立即窜动；
- 夹爪（6）暂不由笔驱动（计划用摄像头检测）。
"""
import numpy as np

DT = 1 / 100
# 测试起始位姿：J2/J3 抬离零位限位，避免"关节限位排斥"混入笔控测量
TEST_Q = np.array([0.0, 0.35, 0.35, 0.0, 0.0, 0.0])


def _make_core(core_env):
    _, TeleopCore, CoreArgs, model, fid = core_env
    core = TeleopCore(model, model.createData(), fid,
                      np.full(6, 60.0), np.full(6, 2.0), CoreArgs())
    core.prime(TEST_Q.copy())
    return core


def _drag(core, steps, x=0.0, y=0.0):
    core.set_pen(0.0, 0.0, True)
    core.set_pen(x, y, True)
    for _ in range(steps):
        core.step(DT, core.q_cmd.copy())


def test_select_motor_enters_joint_mode(core_env):
    core = _make_core(core_env)
    core.select_motor(3)
    assert core.mode == "joint"
    assert core.j_sel == 3
    assert "J4" in core.motor_msg()


def test_up_stroke_drives_selected_pitch_joint(core_env):
    core = _make_core(core_env)
    core.select_motor(3)                 # J4 腕部俯仰 → 笔上下
    q0 = core.q_cmd.copy()
    _drag(core, 100, y=-150.0)           # 上划
    assert core.qd_cmd[3] > 0.2, "上划应驱动 J4 正转"
    assert core.q_cmd[3] > q0[3] + 0.1
    assert np.allclose(core.v_cmd, 0.0), "电机直控不应产生笛卡尔速度"


def test_down_stroke_reverses_direction(core_env):
    core = _make_core(core_env)
    core.select_motor(3)
    q0 = core.q_cmd.copy()
    _drag(core, 100, y=+150.0)           # 下划
    assert core.qd_cmd[3] < -0.2
    assert core.q_cmd[3] < q0[3] - 0.1


def test_left_right_axis_for_rotation_joint(core_env):
    core = _make_core(core_env)
    core.select_motor(4)                 # J5 腕部偏航 → 笔左右
    q0 = core.q_cmd.copy()
    _drag(core, 100, x=+150.0)           # 右划
    assert core.qd_cmd[4] > 0.2
    assert core.q_cmd[4] > q0[4] + 0.1
    assert np.allclose(core.qd_cmd[3], 0.0, atol=0.05), "未选中的关节不应被驱动"


def test_motor_switch_reanchors_pen(core_env):
    core = _make_core(core_env)
    core.select_motor(3)
    core.set_pen(0.0, 0.0, True)
    core.set_pen(0.0, -150.0, True)
    for _ in range(30):
        core.step(DT, core.q_cmd.copy())
    q_before = core.q_cmd.copy()
    core.step(DT, core.q_cmd.copy())             # 切换前一步（此时笔正在驱动）
    step_before = float(np.max(np.abs(core.q_cmd - q_before)))

    q_switch = core.q_cmd.copy()
    core.select_motor(4)                         # 换到 J5 时应重置笔锚点
    assert core.anchor == core.pen, "切换电机未重置笔锚点"
    core.step(DT, core.q_cmd.copy())
    immediate = float(np.max(np.abs(core.q_cmd - q_switch)))
    assert step_before > 2e-3, "前置条件不成立：切换前笔未在驱动"
    assert immediate < 5e-3, f"切换电机瞬间窜动 {immediate:.4f} rad"


def test_gripper_not_pen_driven_yet(core_env):
    core = _make_core(core_env)
    core.select_motor(6)                 # 夹爪：待摄像头检测
    q0 = core.q_cmd.copy()
    _drag(core, 100, y=-150.0)
    assert np.allclose(core.q_cmd, q0, atol=1e-6), "夹爪暂不应由笔驱动"
