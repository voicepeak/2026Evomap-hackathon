"""落盘：episode → Parquet（主格式）+ JSONL（无 pyarrow 时的兜底）。"""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np

from .episode import Episode

try:  # pragma: no cover - 环境相关
    import pyarrow as pa
    import pyarrow.parquet as pq

    HAS_PYARROW = True
except Exception:  # pragma: no cover
    HAS_PYARROW = False


JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll"]


def episode_rows(ep: Episode) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for i in range(ep.n_frames):
        row: dict[str, Any] = {
            "frame_index": i,
            "episode_index": ep.index,
            "timestamp": float(ep.t[i] - ep.t[0]),
            "success": bool(ep.success),
            "gripper.pos": float(ep.grip[i]),
            "action.gripper": float(ep.action[i, ep.pos.shape[1]]),
            "pen.pressure": float(ep.pen_pressure[i]),
            "pen.touching": bool(ep.pen_touching[i]),
        }
        for k in range(ep.pos.shape[1]):
            name = JOINT_NAMES[k] if k < len(JOINT_NAMES) else f"joint_{k}"
            row[f"observation.{name}"] = float(ep.pos[i, k])
            row[f"action.{name}"] = float(ep.action[i, k])
            if ep.vel.shape[1] > k:
                row[f"velocity.{name}"] = float(ep.vel[i, k])
            if ep.tau.shape[1] > k:
                row[f"torque.{name}"] = float(ep.tau[i, k])
        rows.append(row)
    return rows


def save_episode_parquet(path: Path, ep: Episode) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = episode_rows(ep)
    if HAS_PYARROW:
        table = pa.Table.from_pylist(rows)
        pq.write_table(table, path)
    else:
        path = path.with_suffix(".jsonl")
        with path.open("w", encoding="utf-8") as f:
            for r in rows:
                f.write(json.dumps(r, ensure_ascii=False) + "\n")
    return path


def save_json(path: Path, obj: Any) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(obj, ensure_ascii=False, indent=2), encoding="utf-8")
    return path


def load_episode_rows(path: Path) -> list[dict[str, Any]]:
    if path.suffix == ".parquet" and HAS_PYARROW:
        return pq.read_table(path).to_pylist()
    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.strip():
            rows.append(json.loads(line))
    return rows
