"""历史数据清单（/api/library）：重启后仍能看到磁盘上的数据集与原始留档。"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from rebot_capture import index as idx
from rebot_capture.recorder.episode import Episode
from rebot_capture.recorder.writers import save_episode_parquet


def _fake_episode(index: int = 1, frames: int = 40, success: bool = True) -> Episode:
    t = np.arange(frames, dtype=float) / 100.0
    pos = np.tile(np.array([0.1, 1.0, 0.8, 0.0, 0.0, 0.0]), (frames, 1))
    return Episode(
        index=index, task_id="grasp-and-place", started_at=0.0, ended_at=float(t[-1]),
        success=success, t=t, pos=pos, vel=np.zeros((frames, 6)), tau=np.zeros((frames, 6)),
        grip=np.zeros(frames), action=np.concatenate([pos, np.zeros((frames, 1))], axis=1),
        pen_pressure=np.zeros(frames), pen_touching=np.zeros(frames, dtype=bool),
    )


@pytest.fixture()
def temp_home(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    monkeypatch.setenv("REBOT_CAPTURE_HOME", str(tmp_path))
    return tmp_path


def test_library_lists_raw_episode_and_video(temp_home: Path):
    from rebot_capture.paths import episodes_dir

    save_episode_parquet(episodes_dir() / "run1_ep000001.parquet", _fake_episode())
    (episodes_dir() / "run1_ep000001.mp4").write_bytes(b"fake")

    lib = idx.library()
    raw = lib["raw_episodes"]
    assert len(raw) == 1
    assert raw[0]["file"] == "run1_ep000001.parquet"
    assert raw[0]["frames"] == 40
    assert raw[0]["success"] is True
    assert raw[0]["has_video"] is True
    assert lib["totals"]["raw_episodes"] == 1


def test_library_lists_dataset_with_episode_grades(temp_home: Path):
    from rebot_capture.paths import datasets_dir

    ds = datasets_dir() / "session_x"
    (ds / "meta").mkdir(parents=True)
    (ds / "data" / "chunk-000").mkdir(parents=True)
    save_episode_parquet(ds / "data" / "chunk-000" / "episode_000002.parquet", _fake_episode(index=2))
    (ds / "meta" / "info.json").write_text(json.dumps({
        "total_episodes": 1, "total_frames": 40, "features": {"observation.state": {}, "action": {}},
    }), encoding="utf-8")
    (ds / "meta" / "tasks.jsonl").write_text(
        json.dumps({"task_index": 0, "task": "抓取-放置"}, ensure_ascii=False) + "\n", encoding="utf-8")
    (ds / "meta" / "episodes.jsonl").write_text(json.dumps({
        "episode_index": 2, "length": 40, "duration_s": 0.39, "success": True,
        "grade": "A", "score": 88.1, "video_file": None,
    }) + "\n", encoding="utf-8")

    lib = idx.library()
    assert lib["totals"]["datasets"] == 1
    d = lib["datasets"][0]
    assert d["name"] == "session_x"
    assert d["tasks"] == ["抓取-放置"]
    assert d["episodes_detail"][0]["grade"] == "A"
    assert d["episodes_detail"][0]["score"] == 88.1


def test_library_does_not_write_manifest(temp_home: Path):
    idx.library()
    assert not (temp_home / "MANIFEST.json").exists()

    idx.build_index()
    assert (temp_home / "MANIFEST.json").exists()
