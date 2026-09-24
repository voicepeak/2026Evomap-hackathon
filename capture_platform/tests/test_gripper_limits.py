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
import time

import numpy as np
import pytest

from rebot_capture.device.profile import DeviceProfile
from rebot_capture.device.real_arm import (GRIP_ERR_MAX, GRIP_KP, GRIP_TAU_LIMIT,
                                           RealArm, grip_force_limited)
from rebot_capture.teleop.samples import PenSample

DT = 1 / 100
TEST_Q = np.array([0.0, 0.35, 0.35, 0.0, 0.0, 0.0])


def _make_core(core_env):
    """真机同样的参数（行程/方向/力矩都从机型配置来），实测位置放在行程中段。"""
    _, TeleopCore, CoreArgs, model, fid = core_env
    p = DeviceProfile.load()
    core = TeleopCore(model, model.createData(), fid,
                      np.full(6, 60.0), np.full(6, 2.0),
                      CoreArgs(grip_lo=p.gripper.lo_deg, grip_hi=p.gripper.hi_deg,
                               grip_tau_limit=p.gripper.tau_limit, grip_rate=p.gripper.rate,
                               grip_pen_range=p.gripper.pen_range_px))
    core.prime(TEST_Q.copy(), grip_pos=0.0)      # 先 prime（grip_lim 这时才按配置建立）
    g_lo, g_hi = core.grip_lim
    mid = 0.5 * (g_lo + g_hi)
    core.grip_pos = mid                          # 实测 / 目标都放在行程中段
    core.grip_cmd = mid
    core.grip_send = mid
    core.grip_min = core.grip_max = mid
    return core


def _step(core, grip_pos=None, grip_tau=0.0):
    if grip_pos is None:
        grip_pos = core.grip_pos
    if core.pressed:
        core.set_pen(*core.pen, True)       # 客户端 50Hz 重发；机器忙时别触发 0.4s 笔超时
    core.update_feedback(core.q_cmd, None, grip_pos=grip_pos, grip_tau=grip_tau)
    return core.step(DT, core.q_cmd.copy())


def _follow(core, pos, rate=0.012):
    """模拟"会跟随"的夹爪：一个周期最多动 rate rad（≈1.2 rad/s，跟得上指令）。"""
    return pos + float(np.clip(core.grip_send - pos, -rate, rate))


# ====================================================================== #
# 一、限力：下发值不会离实测太远（= 力矩有上限）
# ====================================================================== #
def test_grip_force_limited_caps_error():
    lo, hi = np.radians(-331.0), np.radians(-18.0)
    held = np.radians(-120.0)
    assert abs(grip_force_limited(hi, held, lo, hi) - (held + GRIP_ERR_MAX)) < 1e-12
    assert abs(grip_force_limited(lo, held, lo, hi) - (held - GRIP_ERR_MAX)) < 1e-12
    # 偏差 × kp = 电机力矩上限（2N·m 量级，而不是十几 N·m 的堵转）
    assert GRIP_ERR_MAX * GRIP_KP <= GRIP_TAU_LIMIT + 1e-9
    # 限位内正常跟随：不夹
    near = held + 0.3 * GRIP_ERR_MAX
    assert abs(grip_force_limited(near, held, lo, hi) - near) < 1e-12
    # 输出也尊重可用行程
    assert grip_force_limited(hi, hi, lo, hi) <= hi


def test_gripper_profile_geometry_is_sane():
    """机型配置：行程顺序、两端余量、方向标记（换夹爪/重标定后这些必须仍然成立）。

    旧标定（电机原始坐标）：开口机限 −11.9°、闭合机限 +335.4°。
    程序坐标 = direction × 电机坐标；程序里"角度增大 = 张开"。
    """
    p = DeviceProfile.load().gripper
    assert p.lo_deg < p.hi_deg, "lo/hi 顺序反了"
    span = p.hi_deg - p.lo_deg
    assert 120.0 < span < 360.0, f"行程不合理：{span:.0f}°"
    assert p.direction in (1, -1)
    assert 100.0 <= p.pen_range_px <= 2000.0

    sgn = float(p.direction)
    prog_closed = sgn * 335.4          # 机械闭合端（程序坐标）
    prog_open = sgn * (-11.9)          # 机械开口端（程序坐标）
    lo_limit, hi_limit = min(prog_closed, prog_open), max(prog_closed, prog_open)
    assert p.lo_deg >= lo_limit - 0.5, f"闭合端越过机械端：{p.lo_deg} < {lo_limit}"
    assert p.hi_deg <= hi_limit + 0.5, f"开口端越过机械端：{p.hi_deg} > {hi_limit}"
    assert p.lo_deg - lo_limit >= 0.0, "闭合端没留余量"
    assert hi_limit - p.hi_deg >= 10.0, (
        f"开口端余量太小（容易卡在外面）：{hi_limit - p.hi_deg:.1f}°")


# ====================================================================== #
# 二、夹爪能走完全程（最小处到得了）
# ====================================================================== #
def test_gripper_reaches_both_ends_when_free(core_env):
    core = _make_core(core_env)
    mid = core.grip_pos                 # 测试用的"实测"位置（行程中段）
    g_lo, g_hi = core.grip_lim

    core.set_gripper_target(0.0)                    # 闭合行程（原来"到不了"的那头）
    pos = mid
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
    mid = core.grip_pos
    core.set_gripper_target(0.0)                    # 闭合，但实测卡住不动（夹住东西/到限位）
    for _ in range(40):
        _step(core, grip_tau=2.0)
    assert core.grip_blocked
    assert "阻力" in core.grip_msg or "限位" in core.grip_msg
    hold = core.grip_cmd
    assert abs(hold - core.grip_hold_pos(-1.0)) < 1e-9
    assert mid - hold > 0.4 * GRIP_ERR_MAX, "保持点应保留夹持力"

    # 同方向继续推 2 秒：位置不能再被"顶"（grip_cmd 不变）
    for _ in range(200):
        _step(core, grip_tau=2.0)
    assert abs(core.grip_cmd - hold) < 1e-9, "同方向不应继续顶"
    assert core.grip_send <= hold + 1e-9

    # 反向（张开）可以继续
    core.set_gripper_target(1.0)
    for _ in range(30):
        _step(core, grip_tau=0.2)
    assert core.grip_cmd > hold + 0.1
    assert not core.grip_blocked


def test_blocked_direction_does_not_leak_to_pen_channel(core_env):
    """笔控夹爪（增量跟手）：往张开方向拖、被挡住后同方向不再顶；反向立刻能动。"""
    core = _make_core(core_env)
    mid = core.grip_pos
    core.select_motor(6)                            # 夹爪
    core.set_pen(0.0, 0.0, False)
    core.set_pen(0.0, 0.0, True)                    # 落笔
    # 往右拖（张开方向）→ 实测卡住不动 → 会被判"挡住"
    for i in range(40):
        core.set_pen(20.0 * (i + 1), 0.0, True)     # 每帧 +20px（约 13.9°/帧）
        _step(core, grip_tau=2.0)
    assert core.grip_blocked, "夹爪卡住时笔控也应停手"
    blocked_at = core.grip_cmd

    # 同方向继续拖：不再推进目标
    x = 20.0 * 40
    for _ in range(60):
        x += 20.0
        core.set_pen(x, 0.0, True)
        _step(core, grip_tau=2.0)
    assert abs(core.grip_cmd - blocked_at) < 1e-9, "同方向（继续往右拖）不应继续顶"

    # 反向拖（闭合）：可以动
    for i in range(30):
        x -= 20.0
        core.set_pen(x, 0.0, True)
        _step(core, grip_tau=0.2)
    assert core.grip_cmd < blocked_at - 0.05, "反方向应能继续"
    assert core.grip_cmd < mid, "应该往闭合方向走了"


def test_grip_torque_spike_during_free_travel_is_not_a_block(core_env):
    """正常行程里的力矩尖峰（惯性/阻尼）不能误判成"被挡住"。"""
    core = _make_core(core_env)
    mid = core.grip_pos
    core.set_gripper_target(0.0)
    pos = mid
    for i in range(150):
        pos = _follow(core, pos)
        tau = 2.0 if i in (10, 11, 40) else 0.3     # 偶发力矩尖峰
        _step(core, grip_pos=pos, grip_tau=tau)
    assert not core.grip_blocked, "电机一直在走，不该判成被挡住"
    assert pos < mid - 1.0, f"夹爪应该已经走了不少：{np.degrees(pos):.1f}°"


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


# ====================================================================== #
# 五、方向：direction=-1（角度增大 = 夹紧）时，读/发两个边界都要取反
# ====================================================================== #
class _SentGroup:
    def __init__(self):
        self.sent: list[float] = []

    def send_mit(self, arr, vel=None, kp=None, kd=None, tau=None):
        self.sent.append(float(arr[0]))


class _FakeHardware:
    """假的总线：返回给定的一帧状态（pos/vel/torq）。"""

    def __init__(self, pos):
        self._pos = np.asarray(pos, float)
        self._zero6 = np.zeros(6)

    def get_state(self, request_feedback: bool = True):
        p = np.concatenate([self._zero6, [self._pos]])
        z = np.zeros(7)
        return p, z, z

    def command(self, *a, **k):  # pragma: no cover
        pass


def _flipped_arm(held_software_deg: float = 100.0):
    arm = RealArm.__new__(RealArm)
    arm._g_dir = -1.0                                   # 角度增大 = 夹紧
    arm._g_lo, arm._g_hi = np.radians(3.0), np.radians(342.0)
    arm._g_tau_limit = 2.0
    arm._g_err_max = np.radians(5.7)                    # ≈ 2.0N·m / kp
    arm._g_rate = 1.2
    arm._connected = True
    g = {
        "group": _SentGroup(),
        "send": np.radians(held_software_deg),
        "sent": np.radians(held_software_deg),
        "held": np.radians(held_software_deg),
        "held_raw": np.radians(-held_software_deg),
        "tau": 0.0,
        "last": time.monotonic(),
    }
    arm._grip = g
    arm._arm = _FakeHardware(np.radians(-held_software_deg))   # 硬件读数与软件反号
    return arm, g


def test_gripper_direction_flip_read_and_send():
    arm, g = _flipped_arm(held_software_deg=100.0)

    # 读：硬件 −100° → 软件 +100°（角度增大 = 张开）
    st = arm.read()
    assert abs(g["held_raw"] - np.radians(-100.0)) < 1e-9
    assert abs(g["held"] - np.radians(100.0)) < 1e-9
    assert st.grip == pytest.approx((np.radians(100.0) - arm._g_lo) / (arm._g_hi - arm._g_lo))

    # 发：想要软件 150°（更开）→ 电机应是 −150°…但被"限偏差"夹在 ±5.7°
    arm._send_gripper(g, np.radians(150.0))
    expect_motor = -np.radians(100.0 + 5.7)
    assert abs(g["group"].sent[-1] - expect_motor) < 1e-6, g["group"].sent
    assert float(g["sent"]) == pytest.approx(np.radians(105.7), abs=1e-6)

    # 想合一点（软件 95°）→ 电机 −95°（在偏差内，不夹）
    arm._send_gripper(g, np.radians(95.0))
    assert abs(g["group"].sent[-1] - (-np.radians(95.0))) < 1e-6


def test_pen_gripper_drag_is_incremental_and_stops_on_release(core_env):
    """夹爪笔控：增量跟手（笔移多少、开度按比例动多少），抬笔/笔停立刻停。"""
    core = _make_core(core_env)
    mid = core.grip_pos
    core.select_motor(6)
    g_lo, g_hi = core.grip_lim
    span = g_hi - g_lo

    core.set_pen(480, 310, False)                   # 先抬笔
    core.set_pen(480, 310, True)                    # 落笔
    base = core.grip_cmd

    # 纯上下拖动：夹爪不动
    for _ in range(20):
        core.set_pen(480, 150, True)
        _step(core, grip_tau=0.3)
    assert abs(core.grip_cmd - base) < 1e-9, "上下方向不应驱动夹爪"

    # 缓慢右划 100px（分散到 50 帧 = 2px/帧）：按比例动 100/pen_range × 全行程
    pos = mid
    for i in range(50):
        core.set_pen(480 + 2.0 * (i + 1), 310, True)
        pos = _follow(core, pos)
        _step(core, grip_pos=pos, grip_tau=0.3)
    expect = min(base + (100.0 / core.args.grip_pen_range) * span, g_hi)
    assert abs(core.grip_cmd - expect) < np.radians(1.5), \
        f"增量映射不对：{np.degrees(core.grip_cmd):.1f}° vs 期望 {np.degrees(expect):.1f}°"
    assert core.grip_cmd > base, "右划应往张开方向走"

    # 抬笔：立刻停（目标回到实测，不再追剩余行程）
    core.set_pen(580, 310, False)
    core.update_feedback(core.q_cmd, None, grip_pos=mid, grip_tau=0.3)
    core.step(DT, core.q_cmd.copy())
    assert abs(core.grip_cmd - mid) < 1e-9, "抬笔后应停在实测位置，不再动"


def test_pen_gripper_drag_scale_and_clamp(core_env):
    """拖满 pen_range = 全行程；再拖只会夹在端点（不会越界）。"""
    core = _make_core(core_env)
    mid = core.grip_pos
    core.select_motor(6)
    g_lo, g_hi = core.grip_lim
    core.set_pen(0, 310, False)
    core.set_pen(0, 310, True)
    pos = mid
    n = 100
    for i in range(n):                              # 慢慢往右拖满 pen_range
        core.set_pen(core.args.grip_pen_range * (i + 1) / n, 310, True)
        pos = _follow(core, pos)
        _step(core, grip_pos=pos, grip_tau=0.3)
    assert abs(core.grip_cmd - g_hi) < np.radians(2.0), f"应到开口端：{np.degrees(core.grip_cmd):.1f}°"
    for i in range(40):                              # 再拖：不动（夹在端点）
        core.set_pen(core.args.grip_pen_range * 2, 310, True)
        pos = _follow(core, pos)
        _step(core, grip_pos=pos, grip_tau=0.3)
    assert abs(core.grip_cmd - g_hi) < 1e-9


def test_pen_gripper_jitter_does_not_drift(core_env):
    """笔的像素抖动（±1px）不能把夹爪慢慢带跑（死区累计）。"""
    core = _make_core(core_env)
    mid = core.grip_pos
    core.select_motor(6)
    core.set_pen(480, 310, False)
    core.set_pen(480, 310, True)
    core.step(DT, core.q_cmd.copy())
    base = core.grip_cmd
    for i in range(200):
        core.set_pen(480 + (1.0 if i % 2 else -1.0), 310, True)   # 来回抖 1px
        core.update_feedback(core.q_cmd, None, grip_pos=mid, grip_tau=0.3)
        core.step(DT, core.q_cmd.copy())
    assert abs(core.grip_cmd - base) < np.radians(1.0), "抖动把目标带跑了"


def test_core_command_carries_gripper_tau_message(core_env):
    core = _make_core(core_env)
    core.set_gripper_target(0.0)
    for _ in range(20):
        _step(core, grip_tau=2.0)
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
        _step(core, grip_tau=0.0)
    assert abs(core.grip_cmd - base) < 1e-9


def test_pen_sample_import_smoke():
    s = PenSample(t=0.0, x=0.5, y=0.5, touching=True)
    assert s.touching and s.twist == 0.0
