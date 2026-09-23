"""原始数据清单（MANIFEST）：把 episode 与数据集登记成一份可校验的索引。

用途：
- 防丢：每条 episode 落盘后登记（路径 + sha256 + 统计）
- 交接/售卖：给买家或合作方一份可核对的数据清单
- 训练前自检：一眼看清有多少条、多长、有没有视频
"""
from __future__ import annotations

import hashlib
import json
import time
from pathlib import Path
from typing import Any

from .paths import data_home, datasets_dir, episodes_dir


def _sha256(path: Path, chunk: int = 1 << 20) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        while True:
            b = f.read(chunk)
            if not b:
                break
            h.update(b)
    return h.hexdigest()


def _parquet_stats(path: Path) -> dict[str, Any]:
    try:
        from .recorder.writers import load_episode_rows

        rows = load_episode_rows(path)
        if not rows:
            return {"frames": 0}
        t = [r.get("timestamp", 0.0) for r in rows]
        return {
            "frames": len(rows),
            "duration_s": round(float(t[-1] - t[0]), 2),
            "fps": round(len(rows) / max(1e-6, float(t[-1] - t[0])), 1),
            "has_velocity": any("velocity.shoulder_pan" in r for r in rows[:1]),
            "has_torque": any("torque.shoulder_pan" in r for r in rows[:1]),
            "success": bool(rows[0].get("success", False)),
        }
    except Exception as e:  # noqa: BLE001
        return {"error": f"{type(e).__name__}: {e}"}


def build_index(out: Path | None = None) -> dict[str, Any]:
    raw_files = sorted(episodes_dir().glob("*.parquet"))
    episodes = []
    for p in raw_files:
        episodes.append({
            "file": p.name,
            "path": str(p),
            "size_bytes": p.stat().st_size,
            "sha256": _sha256(p),
            **_parquet_stats(p),
        })

    datasets = []
    for d in sorted(datasets_dir().iterdir()):
        if not d.is_dir():
            continue
        info_p = d / "meta" / "info.json"
        if not info_p.exists():
            continue
        info = json.loads(info_p.read_text(encoding="utf-8"))
        videos = list((d / "videos").rglob("*.mp4")) if (d / "videos").exists() else []
        size = sum(f.stat().st_size for f in d.rglob("*") if f.is_file())
        datasets.append({
            "name": d.name,
            "path": str(d),
            "episodes": info.get("total_episodes"),
            "frames": info.get("total_frames"),
            "features": list((info.get("features") or {}).keys()),
            "videos": len(videos),
            "size_mb": round(size / 1e6, 2),
        })

    total_frames = sum(int(e.get("frames") or 0) for e in episodes)
    total_seconds = sum(float(e.get("duration_s") or 0.0) for e in episodes)
    manifest = {
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "home": str(data_home()),
        "raw_episodes": episodes,
        "datasets": datasets,
        "totals": {
            "raw_episodes": len(episodes),
            "raw_frames": total_frames,
            "raw_minutes": round(total_seconds / 60.0, 2),
            "datasets": len(datasets),
            "with_video": sum(1 for d in datasets if d["videos"] > 0),
        },
    }
    target = Path(out) if out else data_home() / "MANIFEST.json"
    target.write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    manifest["written_to"] = str(target)
    return manifest


def print_index(m: dict[str, Any]) -> None:
    t = m["totals"]
    print(f"原始 episode: {t['raw_episodes']} 条 / {t['raw_frames']} 帧 / {t['raw_minutes']} 分钟")
    for e in m["raw_episodes"]:
        print(f"  - {e['file']:<44s} {e.get('frames', '?'):>6} 帧  {e.get('duration_s', '?'):>7}s  "
              f"{'成功' if e.get('success') else '未标记'}  vel={e.get('has_velocity')} tau={e.get('has_torque')}")
    print(f"数据集: {t['datasets']} 个（带视频 {t['with_video']} 个）")
    for d in m["datasets"]:
        print(f"  - {d['name']:<22s} {d['episodes']} 集 / {d['frames']} 帧 / "
              f"{d['size_mb']}MB / 视频 {d['videos']}")
    print(f"清单已写入: {m['written_to']}")
