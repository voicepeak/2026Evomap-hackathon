import json
from unittest.mock import Mock

import numpy as np
import pytest

from rebot_capture.device.backends import JointState
from rebot_capture.device.mock_arm import MockArm
from rebot_capture.device.profile import DeviceProfile
from rebot_capture.recorder.episode import EpisodeRecorder
from rebot_capture.recorder.session import CaptureService
from rebot_capture.recorder.writers import episode_rows, save_episode_parquet


@pytest.fixture
def service():
    profile = DeviceProfile.load()
    return CaptureService(profile, MockArm(profile), auto_save=False)


@pytest.fixture
def episode():
    recorder = EpisodeRecorder(1, "test", 100)
    for i in range(2):
        state = JointState(i, np.zeros(6), np.zeros(6), np.zeros(6), grip=0.2)
        recorder.add(state, None, np.array([0.1] * 6 + [0.8]))
    ep = recorder.stop(True)
    ep.t = np.array([10., 10.01])
    return ep


def test_recorder_snapshots_reused_buffers():
    recorder = EpisodeRecorder(1, None, 100)
    state = JointState(0, np.zeros(6), np.ones(6), np.full(6, 2.))
    action = np.full(7, 3.)
    recorder.add(state, None, action)
    state.pos[:] = state.vel[:] = state.tau[:] = 99
    action[:] = 99
    recorder.add(state, None, action)
    ep = recorder.stop(True)
    np.testing.assert_array_equal(ep.pos[0], np.zeros(6))
    np.testing.assert_array_equal(ep.vel[0], np.ones(6))
    np.testing.assert_array_equal(ep.tau[0], np.full(6, 2.))
    np.testing.assert_array_equal(ep.action[0], np.full(7, 3.))
    assert ep.pos[1, 0] == 99


def test_gripper_target_survives_parquet_roundtrip(tmp_path, service, episode):
    save_episode_parquet(tmp_path / "data/episode_000001.parquet", episode)
    loaded = service._load_replay_from_dataset(str(tmp_path), 1)
    np.testing.assert_allclose(loaded["action"], episode.action)


def test_legacy_jsonl_uses_observed_gripper(tmp_path, service, episode):
    rows = episode_rows(episode)
    for row in rows:
        row.pop("action.gripper")
    data = tmp_path / "data"
    data.mkdir()
    (data / "episode_000001.jsonl").write_text("\n".join(json.dumps(r) for r in rows))
    loaded = service._load_replay_from_dataset(str(tmp_path), 1)
    np.testing.assert_allclose(loaded["action"][:, -1], episode.grip)


def test_missing_episode_does_not_replay_another(tmp_path, service, episode):
    save_episode_parquet(tmp_path / "data/episode_000001.parquet", episode)
    service.goto_smooth = Mock()
    with pytest.raises(RuntimeError, match="没有 episode 999"):
        service.replay_episode(index=999, dataset_path=str(tmp_path))
    service.goto_smooth.assert_not_called()


def test_aborted_approach_cancels_replay(service, episode):
    service._episodes.append(episode)
    service.goto_smooth = Mock(return_value={"ok": False, "aborted": True})
    service.run_steps = Mock()
    result = service.replay_episode(tau_abort=7.0)
    assert result["aborted"] and result["played"] == 0
    assert service.goto_smooth.call_args.kwargs["tau_abort"] == 7.0
    service.run_steps.assert_not_called()
    assert service._override is None


@pytest.mark.parametrize("corruption", ["nan", "time", "shape", "empty"])
def test_invalid_trajectory_rejected_before_movement(service, episode, corruption):
    if corruption == "nan":
        episode.action[0, 0] = np.nan
    elif corruption == "time":
        episode.t[:] = 1
    elif corruption == "shape":
        episode.action = episode.action[:, :6]
    else:
        episode.t = np.array([])
        episode.action = np.empty((0, 7))
    service._load_replay_from_dataset = Mock(return_value={
        "episode": 1, "frames": episode.n_frames,
        "action": episode.action, "t": episode.t,
    })
    service.goto_smooth = Mock()
    with pytest.raises(RuntimeError, match="回放数据无效"):
        service.replay_episode(dataset_path="test")
    service.goto_smooth.assert_not_called()


def test_core_records_sent_gripper_target(service):
    from types import SimpleNamespace
    state = JointState(0, np.zeros(6), np.zeros(6), np.zeros(6), grip=0.2)
    service.backend = Mock()
    service.backend.read.return_value = state
    service.backend.grip_target.return_value = 0.8
    service.core_mapper = Mock()
    service.core_mapper.step.return_value = SimpleNamespace(
        q=np.zeros(6), kp=np.ones(6), kd=np.ones(6), tau=np.zeros(6), grip_send=1.0)
    service.start_session()
    service.start_episode()
    service.tick()
    ep = service._recorder.stop(True)
    assert ep.grip[0] == 0.2
    assert ep.action[0, -1] == 0.8


def test_dataset_repack_preserves_existing_files(tmp_path, service, episode):
    from rebot_capture.packer.lerobot import build_dataset
    build_dataset("session/example", [episode], service.profile, root=tmp_path)
    before = {p.relative_to(tmp_path): p.read_bytes() for p in tmp_path.rglob("*") if p.is_file()}
    with pytest.raises(RuntimeError, match="已存在"):
        build_dataset("session/example", [episode], service.profile, root=tmp_path)
    after = {p.relative_to(tmp_path): p.read_bytes() for p in tmp_path.rglob("*") if p.is_file()}
    assert before == after


@pytest.mark.parametrize("name", ["../outside", ".", "/tmp/outside"])
def test_dataset_cannot_escape_output_root(tmp_path, service, episode, name):
    from rebot_capture.packer.lerobot import build_dataset
    with pytest.raises(RuntimeError, match="子目录"):
        build_dataset(name, [episode], service.profile, root=tmp_path)


def test_goto_checks_torque_before_first_target(service):
    service._last_state = JointState(0, np.zeros(6), np.zeros(6), np.full(6, 30.), grip=0.2)
    service.run_steps = Mock()
    result = service.goto_smooth(np.array([1.] * 6 + [0.2]), duration=0.01)
    assert result["aborted"]
    np.testing.assert_array_equal(service._override[:6], np.zeros(6))


def test_goto_releases_control_when_pen_takes_over(service, monkeypatch):
    from rebot_capture.teleop.samples import PenSample
    service._last_state = JointState(0, np.zeros(6), np.zeros(6), np.zeros(6), grip=0.2)
    service.run_steps = Mock()
    monkeypatch.setattr("rebot_capture.recorder.session.time.sleep",
                        lambda _: service.ingest_pen_sample(PenSample(t=0, x=0.5, y=0.5, touching=True)))
    result = service.goto_smooth(np.array([1.] * 6 + [0.2]), duration=0.1)
    assert result["aborted"]
    assert service._override is None
    service.run_steps.assert_not_called()
