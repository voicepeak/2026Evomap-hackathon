"""回归：切换 位置/姿态 模式时必须同步"位置保持点"与"笔锚点"。

与上游 spatial_teleop.py 的 set_mode()/按键 O 一致；否则姿态模式的末端位置保持
会把机械臂持续拉回切换前的旧参考点（表现为某个关节一直转，例如 J4 不断上仰）。
"""
import numpy as np

DT = 1 / 100


def _make_core(core_env):
    _, TeleopCore, CoreArgs, model, fid = core_env
    core = TeleopCore(model, model.createData(), fid,
                      np.full(6, 60.0), np.full(6, 2.0), CoreArgs())
    core.prime(np.zeros(6))
    return core


def test_mode_switch_resyncs_position_reference(core_env):
    """位置模式移动后切姿态模式：末端应停在原地，不被拉回旧参考点。"""
    repo = core_env[0]
    core = _make_core(core_env)

    core.set_pen(0.0, 0.0, True)
    core.set_pen(0.0, -160.0, True)
    for _ in range(60):
        core.step(DT, core.q_cmd.copy())
    core.set_pen(0.0, -160.0, False)
    q_at = core.q_cmd.copy()
    p_at = np.asarray(repo.joint_to_pose(q_at)[0], float)

    core.set_mode("ori")
    assert np.allclose(core.p_ref, p_at, atol=1e-9), "切换模式未同步位置保持点"

    for _ in range(200):
        core.step(DT, core.q_cmd.copy())
    drift_deg = float(np.degrees(np.max(np.abs(core.q_cmd - q_at))))
    assert drift_deg < 0.5, f"切换后仍在移动：最大漂移 {drift_deg:.2f}°"


def test_mode_switch_resets_pen_anchor(core_env):
    """按住笔且远离锚点切换模式：第一帧不应立即产生角速度。"""
    core = _make_core(core_env)

    core.set_pen(0.0, 0.0, True)
    core.set_pen(200.0, -100.0, True)   # 远离锚点并保持按住
    for _ in range(5):
        core.step(DT, core.q_cmd.copy())
    q_at = core.q_cmd.copy()

    core.set_mode("ori")
    core.step(DT, core.q_cmd.copy())
    qd_deg_s = np.degrees((core.q_cmd - q_at) / DT)
    assert float(np.max(np.abs(qd_deg_s))) < 5.0, (
        f"切换瞬间速度过大：{np.round(qd_deg_s, 2)} °/s（锚点未重置）")
