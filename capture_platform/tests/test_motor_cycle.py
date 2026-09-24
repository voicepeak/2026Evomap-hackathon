"""回归：选电机/循环由**服务端**算（客户端状态过期也不卡）。

现场问题：笔右键单击想切到夹爪，切到 J4 后就"卡住"了。
原因是客户端用**自己缓存的 joint_index** 去算"下一个"：只要客户端状态没刷新，
每次单击都发同一个 index，看起来就是切不动。
现在 `POST /api/teleop/motor {step:+1}` / `POST /api/teleop/joint {step:±1}`
由服务端按自己的选择循环；客户端只用返回值显示提示。
"""
import numpy as np
import pytest

from rebot_capture.device.mock_arm import MockArm
from rebot_capture.device.profile import DeviceProfile
from rebot_capture.recorder.session import CaptureService
from rebot_capture.teleop.rebot_core import RebotCoreMapper


@pytest.fixture()
def core_service(core_env):
    """真 teleop_core + MockArm（不接硬件、不开控制线程）。"""
    repo = core_env[0]
    profile = DeviceProfile.load()
    arm = MockArm(profile)
    mapper = RebotCoreMapper(profile, repo=repo)
    service = CaptureService(profile, arm, mapper=mapper)
    service.connect()
    yield service
    try:
        service.stop_loop()
    except Exception:  # noqa: BLE001
        pass


def test_motor_cycle_is_server_side(core_service):
    seen = []
    for _ in range(8):
        seen.append(int(core_service.teleop_motor(None, +1)["joint_index"]))
    assert seen[0] == (3 + 1) % 7          # 初始 J4（3）→ 下一个
    assert seen == [(3 + k + 1) % 7 for k in range(8)]
    assert 6 in seen, "服务端循环必须能走到夹爪"
    assert len(set(seen)) == 7, "一圈要经过全部 7 个电机"


def test_motor_cycle_backwards_and_explicit_index(core_service):
    assert int(core_service.teleop_motor(6)["joint_index"]) == 6
    st = core_service.teleop_motor(None, -1)
    assert int(st["joint_index"]) == 5, "−1 = 上一个"
    assert st["mode"] == "joint"
    assert "夹爪" in str(core_service.teleop_motor(6)["msg"])


def test_joint_step_keeps_mode(core_service):
    core_service.teleop_mode("pos")
    st = core_service.teleop_joint(None, None, -1)      # [ ] 键：只要换选择，不切模式
    assert int(st["joint_index"]) == 2
    assert st["mode"] == "pos", "joint 的 ±1 不应把模式切到 joint"
    st2 = core_service.teleop_joint(6, None, 0)          # 显式 index 仍然可用
    assert int(st2["joint_index"]) == 6 and st2["mode"] == "pos"


def test_schemas_accept_step_only():
    """请求模型：index 可省略、step 默认 +1（用 importlib 直接读文件，避免 import 服务包时
    触发 app.py 的模块级 create_app()——那是已知待办，不该在单测里起服务）。"""
    import importlib.util
    from pathlib import Path

    path = Path(__file__).resolve().parents[1] / "rebot_capture" / "server" / "schemas.py"
    spec = importlib.util.spec_from_file_location("_rebot_schemas_only", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    assert mod.MotorIn().step == 1 and mod.MotorIn().index is None
    assert mod.MotorIn(index=6).index == 6
    assert mod.JointIn(step=-1).step == -1 and mod.JointIn(step=-1).index is None


def test_gripper_selected_by_index_and_pen_drag_axis(core_service):
    """选中夹爪后：笔左右**增量跟手**驱动开度（上下不动），抬笔立刻停；进入 joint 模式。"""
    st = core_service.teleop_motor(6)
    assert st["mode"] == "joint" and int(st["joint_index"]) == 6
    core = core_service.core_mapper.core
    g_lo, g_hi = core.grip_lim
    mid = 0.5 * (g_lo + g_hi)
    core.grip_pos = core.grip_cmd = core.grip_send = mid

    def step(x, y=310.0, tau=0.3):
        if core.pressed:
            core.set_pen(x, y, True)                # 客户端 50Hz 重发；别触发 0.4s 笔超时
        pos = core.grip_pos + float(np.clip(core.grip_send - core.grip_pos, -0.012, 0.012))
        core.update_feedback(core.q_cmd, None, grip_pos=pos, grip_tau=tau)
        core.step(1 / 100, core.q_cmd.copy())

    core.set_pen(480, 310, False)
    core.set_pen(480, 310, True)
    for _ in range(20):
        step(480, 150.0)                           # 纯上下 → 不动
    assert abs(core.grip_cmd - mid) < 1e-9

    for i in range(60):                            # 右划 120px（2px/帧）→ 往张开方向走
        step(480 + 2.0 * (i + 1))
    expect = (120.0 / core.args.grip_pen_range) * (g_hi - g_lo)
    assert core.grip_cmd > mid + 0.5 * expect, f"右划没走开：{np.degrees(core.grip_cmd):.1f}°"
    assert core.grip_cmd <= g_hi + 1e-9

    grip_at = core.grip_cmd
    core.set_pen(600, 310, False)                  # 抬笔 → 停在实测位置
    step(600, tau=0.0)
    assert abs(core.grip_cmd - core.grip_pos) < 1e-9, "抬笔后应立刻停在实测位置"
    if grip_at > core.grip_pos + 1e-9:             # 原来目标超前（还没走完）
        assert core.grip_cmd < grip_at, "抬笔后应丢掉剩余行程，不再追"
