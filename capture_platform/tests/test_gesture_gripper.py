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


def test_gripper_stops_when_blocked_and_same_target_is_noop(core_env):
    """被挡住（有阻力 / 到限位）→ 半力保持停手；同一目标不反复顶，换目标才继续。

    与旧行为（一超 1N·m 立刻退回）的区别：判定带 0.15~0.6s 去抖，
    避免正常行程里的力矩尖峰误触发；停手后保留一半夹持力（自锁机构能抱住东西）。
    """
    core = _make_core(core_env)
    core.set_gripper_target(0.0)                 # 握拳 → 闭合行程
    blocked_at = 2.0                             # 测试用实测位置：行程中段（≈118°）
    for _ in range(30):                          # 模拟"夹住东西/顶到限位"：实测卡住 + 力矩 2N·m
        core.update_feedback(core.q_cmd, None, grip_pos=blocked_at, grip_tau=2.0)
        core.step(DT, core.q_cmd.copy())
    assert core.grip_blocked, "挡住后应停手"
    assert "阻力" in core.grip_msg or "限位" in core.grip_msg, core.grip_msg
    hold = core.grip_cmd
    assert abs(hold - core.grip_hold_pos(-1.0)) < 1e-9, "应退到半力保持点"
    assert hold < blocked_at, "保持点应留在实测位置外侧（保留夹持力）"

    core.set_gripper_target(0.0)                 # 同一手势：不重复顶
    for _ in range(50):
        core.update_feedback(core.q_cmd, None, grip_pos=blocked_at, grip_tau=2.0)
        core.step(DT, core.q_cmd.copy())
    assert abs(core.grip_cmd - hold) < 1e-9, f"同一目标不应继续顶（{core.grip_cmd}）"

    core.set_gripper_target(1.0)                 # 换行程（张开手）→ 反向继续
    assert core.grip_cmd > blocked_at
    assert not core.grip_blocked, "换目标应解除挡住标记"


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


def test_pen_drives_gripper_when_gripper_selected(core_env):
    """选中"夹爪"后，笔上划 = 张开行程、下划 = 闭合行程（手势之外的手动兜底）。"""
    core = _make_core(core_env)
    core.select_motor(6)
    base = core.grip_cmd
    core.set_pen(0.0, 0.0, True)
    core.set_pen(0.0, -150.0, True)          # 上划
    for _ in range(30):
        core.step(DT, core.q_cmd.copy())
    assert core.grip_cmd > base + 1e-6, "上划应向张开方向走"

    down = core.grip_cmd
    core.set_pen(0.0, +150.0, True)          # 下划（锚点仍在原点）
    for _ in range(30):
        core.step(DT, core.q_cmd.copy())
    assert core.grip_cmd < down, "下划应向闭合方向走"


def test_same_gesture_does_not_repeat_but_rearms_after_gap():
    fake = _FakeRecognizer([[("Open_Palm", 0.9, "Left")]])
    worker = GestureWorker(lambda: np.zeros((8, 8, 3), dtype=np.uint8),
                           recognizer=fake, hold_s=0.02, interval_s=0.01)
    worker.start()
    try:
        st = _wait_state(worker, lambda s: s["target"] == 1.0)
        n1 = int(st["updates"])
        time.sleep(0.3)
        assert worker.state()["updates"] == n1, "同一手势保持不应反复触发"
        fake.script = [[]]                                  # 手离开画面
        time.sleep(1.0)
        fake.script = [[("Open_Palm", 0.9, "Left")]]        # 同一个手势再次出现
        st2 = _wait_state(worker, lambda s: int(s["updates"]) == n1 + 1, timeout=3.0)
        assert int(st2["updates"]) == n1 + 1, "手离开后再做同一手势应重新生效"
    finally:
        worker.stop()


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
