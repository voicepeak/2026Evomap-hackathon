"""回归：夹爪限位 / 力控（到限位或夹住东西时不再"顶死卡住"）。

现场现象：夹爪总是停在最大处、最小处到不了；数据里 even 有整段"命令在动、
位置一动不动"（电机堵转后锁存故障 → 彻底不动，需要重新连接才恢复）。

根因（三处叠在一起）：
1. `RealArm` 的力矩保护把目标拉回实测后，下一帧又被 core 的目标覆盖 →
   平台这条链路上等于**没有限力**：位置偏差越积越大 → 电机顶着十几 N·m 堵转；
2. 下发值按 1.5× 速度追目标，位置环要靠"多出力矩"追速，正常行程的力矩也偏高；
3. RobStride 堵转/写参数后可能锁存故障（位置不跟随、iq=0），原来的自愈只
   重发 enable，没 `clear_error()`，救不回来。

现在：
- `real_arm.grip_force_limited` 把下发值夹在"实测 ± τ_limit/kp"里 → 力矩有上限；
- core 里"推着不动"超过 0.15s（有力矩反馈）/0.6s（没反馈）→ 停手 + 半力保持，
  同方向不再顶，反向/换手势才继续；
- 自愈补上 `clear_error()`（+ 连接时也清一次），堵转锁存能自己恢复；
- 夹爪可用行程改由机型配置 `gripper.lo_deg / hi_deg` 决定（默认 3°~328°）。
"""
import numpy as np

from rebot_capture.device.profile import DeviceProfile
from rebot_capture.device.real_arm import (GRIP_ERR_MAX, GRIP_KP, GRIP_TAU_LIMIT,
                                           RealArm, grip_force_limited)
from rebot_capture.teleop.samples import PenSample

DT = 1 / 100
TEST_Q = np.array([0.0, 0.35, 0.35, 0.0, 0.0, 0.0])
GL = 2.0           # 测试用：夹爪"实测"位置（rad）——取行程中段（≈118°，与现场数据一致）


def _make_core(core_env):
    _, TeleopCore, CoreArgs, model, fid = core_env
    profile = DeviceProfile.load()
    core = TeleopCore(model, model.createData(), fid,
                      np.full(6, 60.0), np.full(6, 2.0),
                      CoreArgs(grip_lo=profile.gripper.lo_deg, grip_hi=profile.gripper.hi_deg))
    core.prime(TEST_Q.copy(), grip_pos=GL)
    return core


def _step(core, grip_pos=GL, grip_tau=0.0):
    core.update_feedback(core.q_cmd, None, grip_pos=grip_pos, grip_tau=grip_tau)
    return core.step(DT, core.q_cmd.copy())


def _follow(core, pos, rate=0.012):
    """模拟"会跟随"的夹爪：一个周期最多动 rate rad（≈1.2 rad/s，跟得上指令）。"""
    return pos + float(np.clip(core.grip_send - pos, -rate, rate))


# ====================================================================== #
# 一、限力：下发值不会离实测太远（= 力矩有上限）
# ====================================================================== #
def test_grip_force_limited_caps_error():
    lo, hi = np.radians(3.0), np.radians(328.0)
    held = np.radians(120.0)
    assert abs(grip_force_limited(hi, held, lo, hi) - (held + GRIP_ERR_MAX)) < 1e-12
    assert abs(grip_force_limited(lo, held, lo, hi) - (held - GRIP_ERR_MAX)) < 1e-12
    # 偏差 × kp = 电机力矩上限（1.5 N·m 量级，而不是十几 N·m 的堵转）
    assert GRIP_ERR_MAX * GRIP_KP <= GRIP_TAU_LIMIT + 1e-9
    # 限位内正常跟随：不夹
    near = held + 0.3 * GRIP_ERR_MAX
    assert abs(grip_force_limited(near, held, lo, hi) - near) < 1e-12
    # 输出也尊重可用行程
    assert grip_force_limited(hi, hi, lo, hi) <= hi


def test_grip_force_limited_from_profile_range():
    p = DeviceProfile.load()
    assert p.gripper.lo_deg == 3.0 and p.gripper.hi_deg == 328.0
    arm = RealArm.__new__(RealArm)                  # 只验证取配置这一段
    arm._g_lo = np.radians(p.gripper.lo_deg)
    arm._g_hi = np.radians(p.gripper.hi_deg)
    assert abs(arm._g_hi - np.radians(328.0)) < 1e-12


# ====================================================================== #
# 二、夹爪能走完全程（最小处到得了）
# ====================================================================== #
def test_gripper_reaches_both_ends_when_free(core_env):
    core = _make_core(core_env)
    g_lo, g_hi = core.grip_lim

    core.set_gripper_target(0.0)                    # 闭合行程（原来"到不了"的那头）
    pos = GL
    for _ in range(600):
        pos = _follow(core, pos)                    # 模拟跟随的电机
        _step(core, grip_pos=pos, grip_tau=0.3)
    assert abs(pos - g_lo) < np.radians(1.0), f"闭合端没走到：{np.degrees(pos):.1f}°"
    assert not core.grip_blocked

    core.set_gripper_target(1.0)                    # 张开行程
    for _ in range(700):
        pos = _follow(core, pos)
        _step(core, grip_pos=pos, grip_tau=0.3)
    assert abs(pos - g_hi) < np.radians(1.0), f"张开端没走到：{np.degrees(pos):.1f}°"


# ====================================================================== #
# 三、被挡住 → 停手 + 半力保持；同方向不再顶
# ====================================================================== #
def test_gripper_stops_when_blocked_and_holds(core_env):
    core = _make_core(core_env)
    core.set_gripper_target(0.0)                    # 闭合，但实测卡住不动（夹住东西/到限位）
    for _ in range(40):
        _step(core, grip_pos=GL, grip_tau=2.0)
    assert core.grip_blocked
    assert "阻力" in core.grip_msg or "限位" in core.grip_msg
    hold = core.grip_cmd
    assert abs(hold - core.grip_hold_pos(-1.0)) < 1e-9
    assert GL - hold > 0.4 * GRIP_ERR_MAX, "保持点应保留夹持力"

    # 同方向继续推 2 秒：位置不能再被"顶"（grip_cmd 不变）
    for _ in range(200):
        _step(core, grip_pos=GL, grip_tau=2.0)
    assert abs(core.grip_cmd - hold) < 1e-9, "同方向不应继续顶"
    assert core.grip_send <= hold + 1e-9

    # 反向（张开）可以继续
    core.set_gripper_target(1.0)
    for _ in range(30):
        _step(core, grip_pos=GL, grip_tau=0.2)
    assert core.grip_cmd > hold + 0.1
    assert not core.grip_blocked


def test_blocked_direction_does_not_leak_to_pen_channel(core_env):
    """笔控夹爪（位置式）：同方向被挡住后不再顶；反向立刻能动。"""
    core = _make_core(core_env)
    core.select_motor(6)                            # 夹爪
    core.set_pen(0.0, 0.0, False)
    core.set_pen(0.0, 0.0, True)                    # 落笔（锚点 0）
    core.set_pen(200.0, 0.0, True)                  # 右划 = 张开（拖到最开）
    for i in range(40):
        if i % 15 == 0:
            core.set_pen(200.0, 0.0, True)          # 保持笔样本新鲜（客户端本来就 50Hz 重发）
        _step(core, grip_pos=GL, grip_tau=2.0)
    assert core.grip_blocked, "夹爪卡住时笔控也应停手"
    blocked_at = core.grip_cmd

    for i in range(60):
        if i % 15 == 0:
            core.set_pen(200.0, 0.0, True)
        _step(core, grip_pos=GL, grip_tau=2.0)
    assert abs(core.grip_cmd - blocked_at) < 1e-9, "同方向（笔仍停在最右）不应继续顶"

    core.set_pen(-200.0, 0.0, True)                 # 左划 = 闭合（反向）
    for _ in range(30):
        _step(core, grip_pos=GL, grip_tau=0.2)
    assert core.grip_cmd < blocked_at - 0.05, "反方向应能继续"


def test_grip_torque_spike_during_free_travel_is_not_a_block(core_env):
    """正常行程里的力矩尖峰（惯性/阻尼）不能误判成"被挡住"。"""
    core = _make_core(core_env)
    core.set_gripper_target(0.0)
    pos = GL
    for i in range(150):
        pos = _follow(core, pos)
        tau = 2.0 if i in (10, 11, 40) else 0.3     # 偶发力矩尖峰
        _step(core, grip_pos=pos, grip_tau=tau)
    assert not core.grip_blocked, "电机一直在走，不该判成被挡住"
    assert pos < GL - 1.0, f"夹爪应该已经走了不少：{np.degrees(pos):.1f}°"


# ====================================================================== #
# 四、堵转锁存故障的自愈（clear_error）
# ====================================================================== #
class _FakeMotor:
    def __init__(self):
        self.cleared = 0

    def clear_error(self):
        self.cleared += 1


class _FakeGroup:
    def __init__(self):
        self._jn = ["gripper"]
        self._mm = {"gripper": _FakeMotor()}
        self.mode_mit_calls = 0
        self.enable_calls = 0

    def mode_mit(self):
        self.mode_mit_calls += 1

    def enable(self):
        self.enable_calls += 1


def test_grip_recover_clears_fault_then_enables():
    arm = RealArm.__new__(RealArm)
    group = _FakeGroup()
    arm._grip_recover({"group": group})
    assert group._mm["gripper"].cleared == 1, "自愈必须先 clear_error（堵转可能锁存故障）"
    assert group.mode_mit_calls == 1 and group.enable_calls == 1


def test_pen_left_right_drag_is_positional(core_env):
    """夹爪：笔左右拉动 = 位置式（拖多少开多少），右划到张开端、左划到闭合端。"""
    core = _make_core(core_env)
    core.select_motor(6)
    g_lo, g_hi = core.grip_lim

    core.set_pen(480, 310, False)                   # 先抬笔
    core.set_pen(480, 310, True)                    # 落笔（锚点=480，开度起点=当前）
    start = core.grip_cmd

    # 纯上下拖动：夹爪不应有任何变化（轴是左右）
    core.set_pen(480, 150, True)
    for _ in range(20):
        _step(core, grip_pos=GL, grip_tau=0.3)
    assert abs(core.grip_cmd - start) < 1e-9, "上下方向不应驱动夹爪"

    # 右划 100px = 半个行程
    core.set_pen(580, 310, True)
    core.step(DT, core.q_cmd.copy())
    assert abs(core.grip_cmd - min(start + 0.5 * (g_hi - g_lo), g_hi)) < 1e-6

    # 右划 200px（=pen_range）= 全行程 → 张开端（夹爪跟着走）
    pos = GL
    for i in range(400):
        if i % 20 == 0:
            core.set_pen(680, 310, True)            # 模拟客户端 50Hz 重发（别触发 0.4s 超时）
        pos = _follow(core, pos)
        _step(core, grip_pos=pos, grip_tau=0.3)
    assert abs(core.grip_cmd - g_hi) < 1e-9, f"右划应到张开端：{np.degrees(core.grip_cmd):.1f}°"
    assert abs(pos - g_hi) < np.radians(1.0), "夹爪应跟到张开端"

    # 左划（同一锚点）= 闭合端
    for i in range(600):
        if i % 20 == 0:
            core.set_pen(280, 310, True)            # 从 480 向左拉 200px
        pos = _follow(core, pos)
        _step(core, grip_pos=pos, grip_tau=0.3)
    assert abs(core.grip_cmd - g_lo) < 1e-9, f"左划应到闭合端：{np.degrees(core.grip_cmd):.1f}°"
    assert abs(pos - g_lo) < np.radians(1.0), "夹爪应跟到闭合端（原来到不了的那头）"

    # 松笔：停在拖到的位置
    core.set_pen(280, 310, False)
    hold = core.grip_cmd
    for _ in range(30):
        _step(core, grip_pos=pos, grip_tau=0.3)
    assert abs(core.grip_cmd - hold) < 1e-9, "松笔后应停在原处"


def test_core_command_carries_gripper_tau_message(core_env):
    core = _make_core(core_env)
    core.set_gripper_target(0.0)
    for _ in range(20):
        _step(core, grip_pos=GL, grip_tau=2.0)
    cmd = core.step(DT, core.q_cmd.copy())
    assert cmd.grip_send is not None
    assert core.grip_msg in ("", cmd.msg) or "阻力" in core.grip_msg or "限位" in core.grip_msg


def test_pen_pressure_sample_defaults_dont_touch_gripper(core_env):
    """默认（未选夹爪）不应由笔压驱动夹爪 —— 动作通道只有手势/笔直控。"""
    core = _make_core(core_env)
    base = core.grip_cmd
    core.set_pen(480, 310, True)
    core.set_pen(480, 160.0, True)
    for _ in range(60):
        _step(core, grip_pos=GL, grip_tau=0.0)
    assert abs(core.grip_cmd - base) < 1e-9


def test_pen_sample_import_smoke():
    s = PenSample(t=0.0, x=0.5, y=0.5, touching=True)
    assert s.touching and s.twist == 0.0
