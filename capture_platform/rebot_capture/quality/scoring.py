"""评分卡（软指标）——把质量变成可交易的信号：A / B / C / F。

权重（《采集平台方案.md》§7.2）：
    平滑度 30 ｜ 完整性 25 ｜ 同步 20 ｜ 安全 15 ｜ 隐私 10
"""
from __future__ import annotations

import math
from typing import Any

from ..recorder.episode import Episode
from .rules import QualityConfig, compute_metrics


def _clamp(x: float, lo: float = 0.0, hi: float = 100.0) -> float:
    return max(lo, min(hi, x))


def score_episode(
    ep: Episode,
    profile,
    rules: list[dict[str, Any]] | None = None,
    cfg: QualityConfig | None = None,
) -> dict[str, Any]:
    cfg = cfg or QualityConfig()
    m = compute_metrics(ep, profile, cfg)

    # 平滑度：jerke 越小越好（500 rad/s^3 作为参考尺度，v0 经验值）
    smoothness = 100.0 * math.exp(-m["mean_abs_jerk"] / 500.0)

    # 完整性：录制帧率与时长
    rate_ratio = min(1.0, m["fps_effective"] / max(1.0, profile.fps))
    dur_ratio = min(1.0, m["duration_s"] / max(1.0, cfg.min_duration_s * 2))
    completeness = 100.0 * (0.6 * rate_ratio + 0.4 * dur_ratio)

    # 同步：最大时间跳变
    sync = 100.0 * math.exp(-m["max_gap_ms"] / 100.0)

    # 安全：限位占用 + 峰值力矩占比
    limit_pen = min(1.0, m["limit_occupancy"] / max(1e-6, cfg.max_limit_occupancy))
    tau_peak = float(abs(ep.tau).max()) if ep.n_frames else 0.0
    tau_pen = min(1.0, tau_peak / 36.0)  # RS06 峰值 36 N·m
    safety = 100.0 * (1.0 - 0.7 * limit_pen - 0.3 * tau_pen)

    # 隐私：v0 默认满分（接入画面检测后替换）
    privacy = 100.0

    total = (
        0.30 * _clamp(smoothness)
        + 0.25 * _clamp(completeness)
        + 0.20 * _clamp(sync)
        + 0.15 * _clamp(safety)
        + 0.10 * _clamp(privacy)
    )

    hard_ok = all(r.get("passed", True) for r in (rules or []))
    if not hard_ok:
        grade = "F"
    elif total >= 85.0:
        grade = "A"
    elif total >= 70.0:
        grade = "B"
    else:
        grade = "C"

    return {
        "total": round(total, 1),
        "grade": grade,
        "hard_gate_passed": hard_ok,
        "breakdown": {
            "smoothness": round(_clamp(smoothness), 1),
            "completeness": round(_clamp(completeness), 1),
            "sync": round(_clamp(sync), 1),
            "safety": round(_clamp(safety), 1),
            "privacy": round(_clamp(privacy), 1),
        },
        "metrics": m,
    }
