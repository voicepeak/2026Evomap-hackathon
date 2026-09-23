"""回归：电脑摄像头手势 → 夹爪行程。

- teleop_core.set_gripper_target：0=闭合行程、1=张开行程；相同目标不重复顶；
  力矩保护退回后，相同目标不再动作，换行程才继续；
- GestureWorker：手掌张开 / 握拳的稳定判定（注入假识别器，不依赖 mediapipe 模型）。
"""
import math
import time

import numpy as np

from rebot_capture.gesture import GestureWorker

DT = 1 / 100
TEST_Q = np.array([0.0, 0.35, 0.35, 0.0, 0.0, 0.0])


def _make_core(core_env, grip_pos=0.10):
    _, TeleopCore, CoreArgs, model, fid = core_env
    core = TeleopCore(model, model.createData(), fid,
                      np.full(6, 60.0), np.full(6, 2.0), CoreArgs())
    core.prime(TEST_Q.copy(), grip_pos=grip_pos)
    return core


def test_gripper_target_maps_travel_and_ramps(core_env):
    core = _make_core(core_env)
    g_lo, g_hi = core.grip_lim
    core.set_gripper_target(0.0)                 # 握拳 → 闭合行程
    assert abs(core.grip_cmd - g_lo) < 1e-9
    cmd = core.step(DT, core.q_cmd.copy())
    assert cmd.grip_send is not None and cmd.grip_send < 0.10   # 往闭合方向走
    assert abs(cmd.grip_send - 0.10) <= core.args.grip_rate * 1.5 * DT + 1e-9

    core.set_gripper_target(1.0)                 # 张开手 → 张开行程
    assert abs(core.grip_cmd - g_hi) < 1e-9

    core.set_gripper_target(1.0)                 # 相同目标：不重复顶
    assert abs(core.grip_cmd - g_hi) < 1e-9


def test_gripper_target_stops_on_torque_and_same_target_is_noop(core_env):
    core = _make_core(core_env)
    core.update_feedback(core.q_cmd, None, grip_pos=0.10, grip_tau=2.0)   # 阻力
    core.set_gripper_target(0.0)
    core.step(DT, core.q_cmd.copy())
    assert abs(core.grip_cmd - 0.10) < 1e-9, "力矩超限应退回实测位置并停止"
    assert "阻力" in core.grip_msg or "限位" in core.grip_msg
    core.set_gripper_target(0.0)                 # 同一手势不会反复顶
    assert abs(core.grip_cmd - 0.10) < 1e-9
    core.set_gripper_target(1.0)                 # 换行程才继续
    assert core.grip_cmd > 0.10


class _FakeRecognizer:
    """按脚本返回识别结果；列表最后一项会持续返回。"""

    def __init__(self, script):
        self.script = list(script)

    def __call__(self, frame, ts_ms):
        return self.script[0] if len(self.script) == 1 else self.script.pop(0)


def _wait_state(worker, predicate, timeout=3.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        st = worker.state()
        if predicate(st):
            return st
        time.sleep(0.02)
    return worker.state()


def test_gesture_worker_open_and_fist():
    fake = _FakeRecognizer([[("Open_Palm", 0.92, "Left")]])
    worker = GestureWorker(lambda: np.zeros((8, 8, 3), dtype=np.uint8),
                           recognizer=fake, hold_s=0.03, interval_s=0.01)
    worker.start()
    try:
        st = _wait_state(worker, lambda s: s["target"] == 1.0)
        assert st["gesture"] == "open" and st["score"] > 0.5, st
        fake.script = [[("Closed_Fist", 0.88, "Right")]]
        st = _wait_state(worker, lambda s: s["target"] == 0.0)
        assert st["gesture"] == "fist", st
    finally:
        worker.stop()
    assert worker.state()["enabled"] is False


def test_gesture_worker_ignores_weak_or_other_gestures():
    fake = _FakeRecognizer([[("Open_Palm", 0.92, "Left")]])
    worker = GestureWorker(lambda: np.zeros((8, 8, 3), dtype=np.uint8),
                           recognizer=fake, hold_s=0.03, interval_s=0.01)
    worker.start()
    try:
        _wait_state(worker, lambda s: s["target"] == 1.0)
        fake.script = [[("Victory", 0.99, "Left")]]          # 不是开/合 → 不改目标
        time.sleep(0.2)
        assert worker.state()["target"] == 1.0
        fake.script = [[("Closed_Fist", 0.3, "Left")]]       # 置信度不足 → 不判定
        time.sleep(0.2)
        assert worker.state()["gesture"] == "open"
    finally:
        worker.stop()
