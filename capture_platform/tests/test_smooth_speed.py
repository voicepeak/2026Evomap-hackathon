"""回归：速度指令整形（去突变）+ 关节直控 PID 速度环。

背景（用户反馈）：单独控制某个电机时"速度突变很大"；位置/姿态两个模式
"不够丝滑和迅速"。原因是所有速度指令都是阶跃：
- 电机直控 / Q / A：指令速度一帧内 0 ↔ 满速（实测 0.69 rad/s 直接跳上去）；
- 位置 / 姿态：落笔、松手、换向的瞬间同样是硬切换。

现在的做法（`arm_control/teleop_core.py`）：
1. 所有速度指令先过"一阶低通 + 加速度限幅"（规格化在 CoreArgs，可关）；
2. 关节直控用实测速度闭环（PID）：参考速度 ≈ 实际速度，摩擦/负载造成的速度差被补回来；
3. 额定速度不变（不是靠限速换平滑）：满偏仍是 60°/s 量级。

这些用例把"平滑"和"不降速"都钉住，防止以后回退。
"""
import numpy as np

DT = 1 / 100
# 测试起始位姿：J2/J3 抬离零位限位，避免"关节限位排斥"混入测量
TEST_Q = np.array([0.0, 0.35, 0.35, 0.0, 0.0, 0.0])


def _core(core_env):
    _, TeleopCore, CoreArgs, model, fid = core_env
    core = TeleopCore(model, model.createData(), fid,
                      np.full(6, 60.0), np.full(6, 2.0), CoreArgs())
    core.prime(TEST_Q.copy())
    return core


def _run(core, n, vel=None):
    for _ in range(n):
        if core.pressed:
            core.set_pen(*core.pen, True)   # 客户端 50Hz 重发；机器忙时别触发 0.4s 笔超时
        core.step(DT, core.q_cmd.copy(), vel_meas=vel)


# ====================================================================== #
# 一、电机直控：不再"一帧满速"
# ====================================================================== #
def test_joint_direct_first_frame_is_not_full_speed(core_env):
    core = _core(core_env)
    core.select_motor(3)                  # J4 腕部俯仰 → 笔上下
    core.set_pen(0.0, 0.0, True)
    core.set_pen(0.0, -150.0, True)       # 一帧内甩出 150px（原来会瞬间到 0.69 rad/s）
    core.step(DT, core.q_cmd.copy())
    first = float(core.qd_cmd[3])
    assert 0.0 < first < 0.35, f"第一帧速度仍然太大：{first:.3f} rad/s"

    _run(core, 60)
    steady = float(core.qd_cmd[3])
    assert steady > 0.6, f"整形把额定速度也降下去了：{steady:.3f} rad/s"
    assert (core.q_cmd[3] - TEST_Q[3]) > 0.3, "关节没有实际转起来"


def test_joint_direct_release_glides_to_zero(core_env):
    core = _core(core_env)
    core.select_motor(3)
    core.set_pen(0.0, 0.0, True)
    core.set_pen(0.0, -150.0, True)
    _run(core, 40)
    core.set_pen(0.0, -150.0, False)      # 松笔

    core.step(DT, core.q_cmd.copy())
    after_release = float(core.qd_cmd[3])
    assert after_release > 0.3, f"松笔瞬间不该直接停死：{after_release:.3f}"

    seq = []
    for _ in range(30):
        core.step(DT, core.q_cmd.copy())
        seq.append(float(core.qd_cmd[3]))
    assert all(seq[i] >= seq[i + 1] - 1e-6 for i in range(len(seq) - 1)), "减速过程应单调"
    assert abs(seq[-1]) < 0.05, f"0.3s 内应收住，实际 {seq[-1]:.3f}"


def test_joint_direct_reversal_passes_through_zero(core_env):
    core = _core(core_env)
    core.select_motor(3)
    core.set_pen(0.0, 0.0, True)
    core.set_pen(0.0, -150.0, True)
    _run(core, 40)
    before = float(core.qd_cmd[3])
    assert before > 0.5

    core.set_pen(0.0, +150.0, True)       # 一帧内反向
    seq = []
    for _ in range(12):
        core.step(DT, core.q_cmd.copy())
        seq.append(float(core.qd_cmd[3]))
    assert seq[0] > 0.0, "反向第一帧不该瞬间变号（会顿挫）"
    assert all(abs(seq[i + 1] - seq[i]) < 0.25 for i in range(len(seq) - 1)), seq
    assert seq[-1] < 0.0, "应在十几帧内平滑换向"


def test_q_a_hold_ramps_up_and_down(core_env):
    core = _core(core_env)
    core.select_joint(3)
    core.joint_hold(1.047)                # 等价 GUI 按住 Q（原来一帧就到 1.0）
    core.step(DT, core.q_cmd.copy())
    first = float(core.qd_cmd[3])
    assert 0.0 < first < 0.45, f"按住第一帧仍太冲：{first:.3f}"
    _run(core, 60)
    assert float(core.qd_cmd[3]) > 0.95, "稳态速度不该被削"

    core.joint_hold(0.0)                  # 松手
    core.step(DT, core.q_cmd.copy())
    assert float(core.qd_cmd[3]) > 0.3, "松手不该瞬间停死"
    _run(core, 40)
    assert abs(float(core.qd_cmd[3])) < 0.05


def test_freeze_stops_immediately(core_env):
    """冻结是安全动作：必须立即停（只有用户指令才走斜坡）。"""
    core = _core(core_env)
    core.select_joint(3)
    core.joint_hold(1.047)
    _run(core, 60)
    assert float(core.qd_cmd[3]) > 0.9
    core.freeze = True
    core.step(DT, core.q_cmd.copy())
    assert abs(float(core.qd_cmd[3])) < 0.05, "冻结后还有速度，不安全"


# ====================================================================== #
# 二、关节直控速度环（PID）
# ====================================================================== #
def _drive_with_motor(core_env, gain: float, tau_p: float, ref: float, frames: int):
    """仿真一个"只能跑到指令 gain 倍、且有一阶迟滞"的电机，看速度环能否补回来。

    返回 (指令速度, 实际速度) 序列。
    """
    core = _core(core_env)
    core.select_joint(3)
    core.joint_hold(ref)
    q_meas = TEST_Q.copy()
    y = 0.0
    hist = []
    for _ in range(frames):
        vel = np.zeros(6)
        vel[3] = y
        core.step(DT, q_meas.copy(), vel_meas=vel)
        u = float(core.qd_cmd[3])
        y = y + (gain * u - y) * (DT / max(tau_p, 1e-4))
        q_meas[3] += y * DT
        hist.append((u, y))
    return core, hist


def test_joint_velocity_pid_compensates_slow_motor(core_env):
    ref = 0.5
    core, hist = _drive_with_motor(core_env, gain=0.85, tau_p=0.03, ref=ref, frames=150)
    u_end, y_end = hist[-1]
    assert abs(y_end - ref) < 0.03, f"实际速度没跟到指令：{y_end:.3f} vs {ref}"
    assert u_end > ref + 0.02, "速度环没有出力补偿（指令应高于额定才能补摩擦）"
    assert float(core._jvel_corr) > 0.0, "PID 修正量应为正"


def test_joint_velocity_pid_is_stable(core_env):
    """测速有噪声/迟滞时不能自激：尾段指令峰峰值要小。"""
    _, hist = _drive_with_motor(core_env, gain=0.9, tau_p=0.06, ref=0.5, frames=150)
    tail = [u for u, _ in hist[-25:]]
    assert max(tail) - min(tail) < 0.05, f"速度环在振荡：{tail}"


# ====================================================================== #
# 三、位置 / 姿态模式：指令也是连续斜坡
# ====================================================================== #
def test_position_mode_command_ramps_not_steps(core_env):
    core = _core(core_env)
    core.set_pen(480, 310, True)          # 落笔（虚拟窗口像素）
    _run(core, 3)
    core.set_pen(480, 160.0, True)        # 一帧内位移 150px
    core.step(DT, core.q_cmd.copy())
    first = float(np.linalg.norm(core.v_cmd))
    assert 0.0 < first < 0.025, f"位置模式第一帧速度仍太大：{first:.4f} m/s"
    _run(core, 80)
    assert float(np.linalg.norm(core.v_cmd)) > 0.05, "满偏速度不该被削（原来 ≈0.056 m/s）"

    core.set_pen(480, 160.0, False)       # 松笔：平滑收速，不是立即归零
    core.step(DT, core.q_cmd.copy())
    assert float(np.linalg.norm(core.v_cmd)) > 0.01, "松笔不应瞬间停死"


def test_ori_mode_speed_follows_preset(core_env):
    """姿态模式原来慢/中/快三档一样快（wmax 没乘速度档），现在必须分档。"""
    peaks = []
    for idx in (0, 1):
        core = _core(core_env)
        core.speed_i = idx
        core.set_mode("ori")
        core.set_pen(480, 310, True)
        _run(core, 3)
        core.set_pen(480, 110.0, True)    # 200px 满偏
        peak = 0.0
        for _ in range(80):
            core.step(DT, core.q_cmd.copy())
            peak = max(peak, float(np.max(np.abs(core.qd_cmd))))
        peaks.append(peak)
    assert peaks[1] > peaks[0] * 1.3, f"姿态模式速度档没生效：慢={peaks[0]:.2f} 中={peaks[1]:.2f}"
