"""打包器：episode 集合 → LeRobot 风格数据集目录。

目录结构（对齐 LeRobot v3 约定，便于后续接 `lerobot` 直接训练）：

    <name>/
    ├── meta/
    │   ├── info.json          # 数据集总览（fps/features/total_frames ...）
    │   ├── tasks.jsonl        # 任务描述（instruction）
    │   ├── episodes.jsonl     # 每个 episode 的索引与文件
    │   └── rebot_meta.json    # 平台扩展：设备档案/质量/授权/溯源
    ├── data/chunk-000/episode_000000.parquet
    └── videos/                # （相机接入后写入，当前预留）
"""
from __future__ import annotations

import json
import shutil
import time
from pathlib import Path
from typing import Any

from ..device.profile import DeviceProfile
from ..paths import datasets_dir
from ..recorder.episode import Episode
from ..recorder.writers import HAS_PYARROW, save_episode_parquet, save_json

SCHEMA_VERSION = "rebot-dataset/0.1"


def build_dataset(
    name: str,
    episodes: list[Episode],
    profile: DeviceProfile,
    root: Path | str | None = None,
    fps: int | None = None,
    task_instruction: str | None = None,
) -> dict[str, Any]:
    if not episodes:
        raise ValueError("数据集至少需要一个 episode")

    root_path = Path(root) if root else datasets_dir()
    root_path = root_path.resolve()
    ds_dir = (root_path / name).resolve()
    if ds_dir == root_path or root_path not in ds_dir.parents:
        raise RuntimeError("数据集名称必须指向 datasets 目录内的子目录")
    # 独占创建，避免同名打包覆盖旧文件或混入旧 episode。
    try:
        ds_dir.mkdir(parents=True, exist_ok=False)
    except FileExistsError as exc:
        raise RuntimeError(f"数据集已存在，请使用新名称：{name}") from exc
    (ds_dir / "meta").mkdir()
    (ds_dir / "data" / "chunk-000").mkdir(parents=True, exist_ok=True)

    fps = fps or profile.fps
    total_frames = 0
    total_videos = 0
    video_cams: set[str] = set()
    episode_rows: list[dict[str, Any]] = []
    grades: dict[str, int] = {}

    for ep in sorted(episodes, key=lambda e: e.index):
        rel = Path("data") / "chunk-000" / f"episode_{ep.index:06d}.parquet"
        saved = save_episode_parquet(ds_dir / rel, ep)
        rel_saved = saved.relative_to(ds_dir).as_posix()
        ep.files["data"] = rel_saved
        total_frames += ep.n_frames

        # 相机视频（如有）
        video_rel = None
        vpath = ep.files.get("video")
        if vpath and Path(vpath).exists():
            cam = ep.files.get("video_cam", "cam0")
            video_cams.add(cam)
            dest = ds_dir / "videos" / "chunk-000" / f"observation.images.{cam}" / f"episode_{ep.index:06d}.mp4"
            dest.parent.mkdir(parents=True, exist_ok=True)
            try:
                shutil.copy2(vpath, dest)
                video_rel = dest.relative_to(ds_dir).as_posix()
                total_videos += 1
            except Exception:  # noqa: BLE001
                video_rel = None

        grades[ep.grade] = grades.get(ep.grade, 0) + 1
        episode_rows.append(
            {
                "episode_index": ep.index,
                "task_id": ep.task_id,
                "length": ep.n_frames,
                "duration_s": round(ep.duration_s, 3),
                "success": ep.success,
                "grade": ep.grade,
                "score": round(ep.score_value, 1),
                "data_file": rel_saved,
                "video_file": video_rel,
            }
        )

    instruction = task_instruction or (episodes[0].task_id or "grasp-and-place")
    (ds_dir / "meta" / "tasks.jsonl").write_text(
        json.dumps({"task_index": 0, "task": instruction}, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    with (ds_dir / "meta" / "episodes.jsonl").open("w", encoding="utf-8") as f:
        for row in episode_rows:
            f.write(json.dumps(row, ensure_ascii=False) + "\n")

    info = {
        "codebase_version": SCHEMA_VERSION,
        "robot_type": profile.arm_model,
        "fps": fps,
        "total_episodes": len(episodes),
        "total_frames": total_frames,
        "total_tasks": 1,
        "total_videos": total_videos,
        "features": {
            "observation.state": {"dtype": "float32", "shape": [profile.n_joints + 1]},
            "action": {"dtype": "float32", "shape": [profile.n_motors]},
            "timestamp": {"dtype": "float32", "shape": [1]},
            **{
                f"observation.images.{cam}": {"dtype": "video", "shape": [480, 640, 3]}
                for cam in sorted(video_cams)
            },
        },
        "created_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
    }
    save_json(ds_dir / "meta" / "info.json", info)

    rebot_meta = {
        "schema_version": SCHEMA_VERSION,
        "profile": profile.summary(),
        "auto_config": profile.to_auto_config(),
        "stats": {
            "episodes": len(episodes),
            "frames": total_frames,
            "grades": grades,
        },
        "provenance": {
            "device_serial": None,  # TODO: 接入设备身份芯片/密钥后填充
            "calib_hash": None,
            "signature": None,
            "signed_by": None,
        },
        "license": {
            "default_scope": "train",
            "expiry": None,
            "region": "CN",
        },
    }
    save_json(ds_dir / "meta" / "rebot_meta.json", rebot_meta)

    size_bytes = sum(p.stat().st_size for p in ds_dir.rglob("*") if p.is_file())
    return {
        "dataset": name,
        "path": str(ds_dir),
        "episodes": len(episodes),
        "frames": total_frames,
        "grades": grades,
        "size_mb": round(size_bytes / 1e6, 2),
        "parquet": HAS_PYARROW,
    }


def verify_dataset(path: Path | str) -> dict[str, Any]:
    ds_dir = Path(path)
    meta_dir = ds_dir / "meta"
    result: dict[str, Any] = {"path": str(ds_dir), "ok": False, "issues": []}

    required = ["info.json", "tasks.jsonl", "episodes.jsonl", "rebot_meta.json"]
    for name in required:
        if not (meta_dir / name).exists():
            result["issues"].append(f"缺少 meta/{name}")

    info_path = meta_dir / "info.json"
    if info_path.exists():
        info = json.loads(info_path.read_text(encoding="utf-8"))
        result["total_episodes"] = info.get("total_episodes")
        result["total_frames"] = info.get("total_frames")
        data_files = list((ds_dir / "data").rglob("*.parquet")) + list((ds_dir / "data").rglob("*.jsonl"))
        if len(data_files) != info.get("total_episodes"):
            result["issues"].append(
                f"data 文件数 {len(data_files)} != info.total_episodes {info.get('total_episodes')}"
            )

    result["ok"] = not result["issues"]
    return result
