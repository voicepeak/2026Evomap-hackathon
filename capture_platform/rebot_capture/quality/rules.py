"""质量门（硬门）——任一条不过，episode 不允许入库/上架。

规则来自《采集平台方案.md》§7.1，阈值集中在 QualityConfig。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from ..device.profile import DeviceProfile
from ..recorder.episode import Episode


@dataclass
class QualityConfig:
    min_duration_s: float = 1.0
    max_duration_s: float = 120.0
    max_drop_rate: float = 0.02
    max_limit_occupancy: float = 0.05
    max_gap_ms: float = 50.0
    min_frames: int = 30
    require_success: bool = True
    limit_epsilon: float = 1e-3


@dataclass
class RuleResult:
    name: str
    passed: bool
    detail: str
    value: float | None = None

    def to_dict(self) -> dict[str, Any]:
        return {"name": self.name, "passed": self.passed, "detail": self.detail, "value": self.value}


def compute_metrics(ep: Episode, profile: DeviceProfile, cfg: QualityConfig | None = None) -> dict[str, float]:
    cfg = cfg or QualityConfig()
    n = ep.n_frames
    duration = max(ep.duration_s, 1e-6)
    period = 1.0 / max(1, profile.fps)

    # 丢帧：按"帧间间隔比标称周期多出的帧数"计算（检测卡顿，而不是把机器略慢当丢帧）
    missing = 0.0
    if n > 1:
        gaps = np.diff(ep.t)
        missing = float(np.sum(np.maximum(0.0, np.round(gaps / period) - 1.0)))
    drop_rate = missing / max(1.0, n + missing)
    fps_effective = n / duration

    limits = np.asarray(profile.limits_array(), dtype=float)
    pos = ep.pos[:, : limits.shape[0]]
    at_low = np.abs(pos - limits[:, 0]) < cfg.limit_epsilon
    at_high = np.abs(pos - limits[:, 1]) < cfg.limit_epsilon
    at_limit = at_low | at_high
    limit_occupancy = float(np.mean(np.any(at_limit, axis=1))) if n else 0.0

    gaps_ms = np.diff(ep.t) * 1000.0 if n > 1 else np.array([0.0])
    max_gap_ms = float(np.max(gaps_ms)) if gaps_ms.size else 0.0

    mean_abs_jerk = 0.0
    if n > 4:
        dt = np.clip(np.diff(ep.t), 1e-4, None)
        vel = np.diff(pos, axis=0) / dt[:, None]
        acc = np.diff(vel, axis=0) / dt[1:, None]
        jrk = np.diff(acc, axis=0) / dt[2:, None]
        mean_abs_jerk = float(np.mean(np.abs(jrk)))

    return {
        "duration_s": round(duration, 3),
        "frames": float(n),
        "fps_effective": round(fps_effective, 1),
        "drop_rate": round(drop_rate, 4),
        "limit_occupancy": round(limit_occupancy, 4),
        "max_gap_ms": round(max_gap_ms, 2),
        "mean_abs_jerk": round(mean_abs_jerk, 2),
        "grip_success_ratio": float(np.mean(ep.pen_touching)) if n else 0.0,
    }


def evaluate(
    ep: Episode,
    profile: DeviceProfile,
    cfg: QualityConfig | None = None,
) -> tuple[list[RuleResult], dict[str, float]]:
    cfg = cfg or QualityConfig()
    m = compute_metrics(ep, profile, cfg)
    rules: list[RuleResult] = [
        RuleResult(
            "duration",
            cfg.min_duration_s <= m["duration_s"] <= cfg.max_duration_s,
            f"{m['duration_s']}s（允许 {cfg.min_duration_s}-{cfg.max_duration_s}s）",
            m["duration_s"],
        ),
        RuleResult(
            "frames",
            m["frames"] >= cfg.min_frames,
            f"{int(m['frames'])} 帧（最少 {cfg.min_frames}）",
            m["frames"],
        ),
        RuleResult(
            "recording_rate",
            m["fps_effective"] >= 0.75 * profile.fps,
            f"实际 {m['fps_effective']} fps（目标 {profile.fps}，下限 {0.75 * profile.fps:.0f}）",
            m["fps_effective"],
        ),
        RuleResult(
            "drop_frame",
            m["drop_rate"] <= cfg.max_drop_rate,
            f"丢帧率 {m['drop_rate'] * 100:.2f}%（上限 {cfg.max_drop_rate * 100:.0f}%）",
            m["drop_rate"],
        ),
        RuleResult(
            "limit_occupancy",
            m["limit_occupancy"] <= cfg.max_limit_occupancy,
            f"限位占用 {m['limit_occupancy'] * 100:.2f}%（上限 {cfg.max_limit_occupancy * 100:.0f}%）",
            m["limit_occupancy"],
        ),
        RuleResult(
            "timestamp_gap",
            m["max_gap_ms"] <= cfg.max_gap_ms,
            f"最大时间跳变 {m['max_gap_ms']}ms（上限 {cfg.max_gap_ms:.0f}ms）",
            m["max_gap_ms"],
        ),
        RuleResult(
            "success_flag",
            (ep.success or not cfg.require_success),
            "需标记成功" if cfg.require_success else "不强制成功标记",
            1.0 if ep.success else 0.0,
        ),
    ]
    return rules, m


def hard_gate_passed(rules: list[RuleResult]) -> bool:
    return all(r.passed for r in rules)
