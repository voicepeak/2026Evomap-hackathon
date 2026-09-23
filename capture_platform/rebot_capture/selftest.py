"""自检：不接硬件，跑通「遥操 → 录制 → 质检 → 打包」全链路。

    rebot-capture selftest

预期：前 2 个 episode 合格（A/B），第 3 个因顶限位被硬门拦下（F），
打包只包含合格数据，数据集目录校验通过。
"""
from __future__ import annotations

import math

import numpy as np

from .device.mock_arm import MockArm
from .device.profile import DeviceProfile
from .packer.lerobot import verify_dataset
from .recorder.session import CaptureService
from .teleop.samples import PenSample


def _pen_wave(amplitude: float):
    """温和的椭圆轨迹 + 笔压变化（模拟一次抓取演示）。"""

    def feed(i: int) -> PenSample:
        t = i / 100.0
        return PenSample(
            t=t,
            x=0.5 + amplitude * math.sin(2 * math.pi * t / 1.5),
            y=0.5 + amplitude * math.cos(2 * math.pi * t / 1.8),
            pressure=0.5 + 0.3 * math.sin(2 * math.pi * t / 1.0),
            touching=True,
        )

    return feed


def _pen_push_into_limit():
    """故意把 j1 推到限位（用于验证质量门会拦截）。"""

    def feed(i: int) -> PenSample:
        t = i / 100.0
        x = 0.5 + 4.0 * min(1.0, t / 0.5)
        return PenSample(t=t, x=x, y=0.5, pressure=0.8, touching=True)

    return feed


def run_selftest(verbose: bool = True) -> int:
    log = print if verbose else (lambda *a, **k: None)

    profile = DeviceProfile.load()
    arm = MockArm(profile)
    service = CaptureService(profile, arm, fps=profile.fps)
    service.connect()

    log("== rebot-capture 自检 ==")
    log(f"设备档案: {profile.name}  {profile.n_motors} 电机 @ {profile.fps}Hz  CAN {profile.can}")
    log("")

    ready = np.zeros(profile.n_motors)
    ready[1], ready[2] = 0.6, 0.9
    service.warmup(ready, seconds=1.0)
    log("→ warmup: 已开到 ready 位（j2=0.6, j3=0.9）")

    service.start_session(task_id="selftest-grasp", operator="selftest")

    results: list[dict] = []
    for k in (1, 2):
        service.start_episode()
        service.run_steps(2.2, pen_feed=_pen_wave(0.10 + 0.02 * k))
        results.append(service.stop_episode(success=True))
        log(f"→ episode {results[-1]['index']} 录完：{results[-1]['frames']} 帧")

    service.start_episode()
    service.run_steps(1.8, pen_feed=_pen_push_into_limit())
    results.append(service.stop_episode(success=True))
    log(f"→ episode {results[-1]['index']} 录完（故意顶限位）")

    service.stop_session()

    log("")
    log(f"{'#':>2} {'时长s':>6} {'帧数':>5} {'丢帧%':>7} {'限位%':>7} {'评分':>5} {'等级':>4}  硬门")
    for r in results:
        m = r["score_detail"]["metrics"]
        gate = "✓" if r["score_detail"]["hard_gate_passed"] else "✗"
        log(
            f"{r['index']:>2} {r['duration_s']:>6.2f} {r['frames']:>5} "
            f"{m['drop_rate'] * 100:>7.2f} {m['limit_occupancy'] * 100:>7.2f} "
            f"{r['score']:>5.1f} {r['grade']:>4}  {gate}"
        )

    good = [r for r in results if r["grade"] != "F"]
    bad = [r for r in results if r["grade"] == "F"]
    ok = len(good) >= 2 and len(bad) >= 1

    pack = service.pack(name="selftest_dataset", task_instruction="selftest: pick and place")
    v = verify_dataset(pack["path"])
    log("")
    log(f"打包: {pack['dataset']}  {pack['episodes']} 集 / {pack['frames']} 帧 / {pack['size_mb']} MB")
    log(f"路径: {pack['path']}")
    log(f"校验: {'通过' if v['ok'] else '失败 ' + str(v['issues'])}")
    log(f"等级分布: {pack['grades']}")

    ok = ok and v["ok"] and pack["episodes"] == len(good)
    service.close()

    log("")
    log("自检结果: " + ("PASS ✅" if ok else "FAIL ❌"))
    return 0 if ok else 1


# ====================================================================== #
# 运动学 / 映射自检（接 reBotArm_control_py，不需要硬件）
# ====================================================================== #
def run_ik_selftest(arm_repo: str | None = None, verbose: bool = True) -> int:
    import time

    from .integrations.rebotarm import RebotArmRepo
    from .teleop.spatial import BX, BZ, RMAX, SpatialVelocityMapper

    log = print if verbose else (lambda *a, **k: None)

    log("")
    log("== 运动学/映射自检（无硬件，依赖 reBotArm_control_py） ==")
    try:
        repo = RebotArmRepo(arm_repo)
        mapper = SpatialVelocityMapper(DeviceProfile.load(), repo=repo)
    except Exception as e:  # noqa: BLE001
        log(f"跳过：{type(e).__name__}: {e}")
        return 1

    log(f"仓库: {repo.path}")
    rate = 100
    dt = 1.0 / rate
    q0 = np.array([0.0, 0.6, 0.9, 0.0, 0.0, 0.0])
    mapper.reset(q0)
    p0 = np.asarray(repo.joint_to_pose(np.clip(q0, mapper._lo, mapper._hi))[0], float)
    log(f"起始末端 XYZ = ({p0[0]:+.3f}, {p0[1]:+.3f}, {p0[2]:+.3f})")

    def drive(dx: float, dy: float, pressure: float, secs: float, shift: bool = False) -> np.ndarray:
        """模拟真实用法：先抬笔 → 落笔锚定 → 把笔从中心挪到目标位移。"""
        q = mapper._q_cmd.copy()
        # 抬笔（清锚点）
        q = mapper.map(PenSample(t=0.0, x=0.5, y=0.5, touching=False, shift=shift), q)
        # 落笔锚定
        warm = max(1, int(0.1 * rate))
        for i in range(warm):
            pen = PenSample(t=i * dt, x=0.5, y=0.5, pressure=pressure, touching=True, shift=shift)
            q = mapper.map(pen, q)
            time.sleep(dt)
        # 斜坡移动到目标位移
        n = max(1, int(secs * rate))
        for i in range(n):
            f = (i + 1) / n
            pen = PenSample(
                t=(warm + i) * dt,
                x=0.5 + dx * f,
                y=0.5 + dy * f,
                pressure=pressure,
                touching=True,
                shift=shift,
            )
            q = mapper.map(pen, q)
            time.sleep(dt)
        return q

    results: list[tuple[str, bool, str]] = []

    # 1) 默认模式：笔上移 → 末端 +X
    q = drive(0.0, -0.15, 0.0, 1.5)
    p1 = np.asarray(repo.joint_to_pose(q[:6])[0], float)
    dx1 = p1[0] - p0[0]
    results.append(("默认模式 笔上移 → +X", dx1 > 0.01, f"ΔX={dx1*100:+.1f}cm"))

    # 2) 升降模式：笔上移 → 末端 +Z
    q = drive(0.0, -0.15, 0.0, 1.5, shift=True)
    p2 = np.asarray(repo.joint_to_pose(q[:6])[0], float)
    dz2 = p2[2] - p1[2]
    results.append(("升降模式 笔上移 → +Z", dz2 > 0.01, f"ΔZ={dz2*100:+.1f}cm"))

    # 3) 安全盒：长时间满速推 → 末端不越界
    q = drive(0.0, -0.5, 0.0, 6.0)
    p3 = np.asarray(repo.joint_to_pose(q[:6])[0], float)
    r3 = float(np.hypot(p3[0], p3[1]))
    in_box = (
        BX[0] - 0.01 <= p3[0] <= BX[1] + 0.01
        and BZ[0] - 0.01 <= p3[2] <= BZ[1] + 0.01
        and r3 <= RMAX + 0.01
    )
    results.append(("安全盒拦截", in_box, f"末端=({p3[0]:.3f},{p3[1]:.3f},{p3[2]:.3f}) r={r3:.3f}"))

    # 4) 关节限位
    q_lo, q_hi = mapper._lo, mapper._hi
    within = bool(np.all(q[:6] >= q_lo - 1e-6) and np.all(q[:6] <= q_hi + 1e-6))
    results.append(("关节软限位", within, f"q={np.round(np.degrees(q[:6]), 1)}°"))

    # 5) 夹爪：笔压 → 目标闭合
    mapper._grip_frac = 1.0
    drive(0.0, 0.0, 0.9, 0.6)
    closed = mapper._grip_frac < 0.9
    results.append(("笔压 → 夹爪闭合", closed, f"grip_frac={mapper._grip_frac:.2f}"))

    ok = all(r[1] for r in results)
    log("")
    log(f"{'检查项':<24} {'结果':<6} 详情")
    for name, passed, detail in results:
        log(f"{name:<24} {'✓' if passed else '✗':<6} {detail}")
    log("")
    log("运动学自检: " + ("PASS ✅" if ok else "FAIL ❌"))
    return 0 if ok else 1
