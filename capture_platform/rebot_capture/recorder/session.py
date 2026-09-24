"""采集服务（采集端的大脑）：设备 + 遥操映射 + 录制 + 质检 + 打包。

线程模型：
- 控制/录制循环（后台线程，默认 100Hz）：读设备 → 映射 → 下发 → 记录 → 刷新实时指标
- 服务端（FastAPI）线程：REST/WS 调用这里的公开方法（全部加锁）
"""
from __future__ import annotations

import math
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any, Callable

import numpy as np

from ..device.backends import ArmBackend, JointState
from ..device.profile import DeviceProfile
from ..packer.lerobot import build_dataset
from ..paths import episodes_dir
from ..quality.rules import QualityConfig, evaluate
from ..quality.scoring import score_episode
from ..teleop.mapping import Mapper, MockPenMapper
from ..teleop.samples import PenSample
from .episode import EpisodeRecorder


def _smoothstep(a: float) -> float:
    """最小 jerk 曲线（与 reBotArm_control_py 的归零/回放一致）：10a³-15a⁴+6a⁵。"""
    a = min(max(a, 0.0), 1.0)
    return 10.0 * a**3 - 15.0 * a**4 + 6.0 * a**5


class CaptureService:
    def __init__(
        self,
        profile: DeviceProfile,
        backend: ArmBackend,
        mapper: Mapper | None = None,
        fps: int | None = None,
        quality_cfg: QualityConfig | None = None,
        camera: Any | None = None,
        auto_save: bool = True,
        gesture: Any | None = None,
    ):
        self.profile = profile
        self.backend = backend
        self.mapper = mapper or MockPenMapper(profile)
        self.core_mapper = mapper if getattr(mapper, "is_core", False) else None
        self.camera = camera
        self.gesture = gesture          # 电脑摄像头手势（夹爪行程），只发布状态
        self.auto_save = bool(auto_save)
        self.fps = fps or profile.fps
        self.quality_cfg = quality_cfg or QualityConfig()
        self._run_id = time.strftime("%Y%m%d_%H%M%S")

        self._lock = threading.RLock()
        self._pen: PenSample | None = None
        self._pen_rx: float = 0.0
        self._pen_count = 0
        self._pen_window: deque[float] = deque(maxlen=200)
        self.pen_timeout: float = 0.4  # 秒；与上游 dynamic PEN_TIMEOUT 一致

        self._session: dict[str, Any] | None = None
        self._recorder: EpisodeRecorder | None = None
        self._episodes: list = []
        self._episode_counter = 0
        self._last_dataset: str | None = None
        self._override: np.ndarray | None = None  # warmup/手动目标（优先于遥操映射）

        self._loop_thread: threading.Thread | None = None
        self._loop_stop = threading.Event()
        self._last_state: JointState | None = None
        self._last_action: np.ndarray | None = None
        self._tick_times: deque[float] = deque(maxlen=200)
        self._gesture_updates = 0

    # ================================================================== #
    # 生命周期
    # ================================================================== #
    def connect(self) -> None:
        self.backend.connect()
        if self.core_mapper is not None:
            # 使能后立即同步核心指令到实测姿态（等价上游 q_cmd = S.q）
            st = self.backend.read()
            grip_rad = getattr(self.backend, "grip_rad", lambda: None)()
            self.core_mapper.prime(st.pos, grip_rad)
            self._last_state = st

    def close(self) -> None:
        """退出：与上游 `spatial_teleop.py` 一致 —— 先平滑归零、**不失能**，再停止。"""
        if self.gesture is not None:
            try:
                self.gesture.stop()
            except Exception:  # noqa: BLE001
                pass
        try:
            if self._last_state is not None:
                dev = float(np.max(np.abs(self._last_state.pos)))
                if dev > 0.05:
                    self.park()
        except Exception:  # noqa: BLE001
            pass
        self.stop_loop()
        try:
            self.backend.close()
        except Exception:  # noqa: BLE001
            pass

    # ================================================================== #
    # 会话 / episode
    # ================================================================== #
    def start_session(self, task_id: str | None = None, operator: str | None = None) -> dict:
        with self._lock:
            if self._session is not None:
                raise RuntimeError("已有会话进行中")
            self._session = {
                "task_id": task_id,
                "operator": operator,
                "started_at": time.time(),
                "episodes": 0,
            }
            return dict(self._session)

    def stop_session(self) -> dict:
        with self._lock:
            if self._recorder is not None:
                raise RuntimeError("还有 episode 在录制中，请先 stop/discard")
            session = self._session or {}
            self._session = None
            return {"session": session, "episodes": len(self._episodes)}

    def start_episode(self, task_id: str | None = None) -> dict:
        with self._lock:
            if self._session is None:
                raise RuntimeError("请先 start_session")
            if self._recorder is not None:
                raise RuntimeError("已有 episode 在录制中")
            index = self._episode_counter + 1
            self._recorder = EpisodeRecorder(index=index, task_id=task_id or self._session.get("task_id"), fps=self.fps)
            video = None
            if self.camera is not None and getattr(self.camera, "current", None) is not None:
                path = episodes_dir() / f"{self._run_id}_ep{index:06d}.mp4"
                if self.camera.start_recording(path):
                    video = str(path)
            return {"episode_index": index, "recording": True, "video": video}

    def stop_episode(self, success: bool = True, note: str = "") -> dict:
        with self._lock:
            if self._recorder is None:
                raise RuntimeError("当前没有在录制的 episode")
            ep = self._recorder.stop(success=success, note=note)
            self._recorder = None

            vinfo = self.camera.stop_recording() if self.camera is not None else None
            if vinfo and Path(vinfo["path"]).exists():
                ep.files["video"] = vinfo["path"]
                ep.files["video_frames"] = str(vinfo["frames"])
                ep.files["video_fps"] = str(vinfo["fps"])
                ep.files["video_cam"] = self.camera.current_alias()

            rules, _metrics = evaluate(ep, self.profile, self.quality_cfg)
            ep.qc = [r.to_dict() for r in rules]
            ep.score = score_episode(ep, self.profile, ep.qc, self.quality_cfg)

            self._episode_counter += 1
            self._episodes.append(ep)
            if self._session is not None:
                self._session["episodes"] = len(self._episodes)

            # 自动落盘：无论等级，先保住原始数据
            if self.auto_save:
                try:
                    from ..recorder.writers import save_episode_parquet

                    path = episodes_dir() / f"{self._run_id}_ep{ep.index:06d}.parquet"
                    saved = save_episode_parquet(path, ep)
                    ep.files["data_parquet"] = str(saved)
                except Exception:  # noqa: BLE001
                    pass

            return ep.to_dict(with_details=True)

    def discard_episode(self) -> dict:
        with self._lock:
            if self._recorder is None:
                raise RuntimeError("当前没有在录制的 episode")
            self._recorder = None
            vinfo = self.camera.stop_recording() if self.camera is not None else None
            if vinfo:
                try:
                    Path(vinfo["path"]).unlink(missing_ok=True)
                except Exception:  # noqa: BLE001
                    pass
            return {"discarded": True}

    # ================================================================== #
    # 控制 / 录制循环
    # ================================================================== #
    def tick(self) -> None:
        with self._lock:
            self._apply_gesture_locked()
            state = self.backend.read()
            pen = self._pen
            # 输入超时：笔样本超过 pen_timeout 没更新 → 视为抬笔（松手即停，与上游一致）
            if pen is not None and (time.monotonic() - self._pen_rx) > self.pen_timeout:
                pen = None

            # 手动目标（warmup/park）优先；一旦操作者落笔，自动交还给遥操映射
            if self._override is not None:
                if pen is not None and pen.touching:
                    self._override = None
                    if self.core_mapper is not None:
                        # 交还前把核心同步到实测（等价上游 q_cmd = S.q）
                        grip_rad = getattr(self.backend, "grip_rad", lambda: None)()
                        self.core_mapper.prime(state.pos, grip_rad)
                else:
                    action = self._override
                    self.backend.command(action)
                    self._last_state = state
                    self._last_action = action
                    self._tick_times.append(time.monotonic())
                    if self._recorder is not None:
                        self._recorder.add(state, pen, action)
                    return

            # teleop_core 路径：核心自己算 q/kp/kd/tau 与夹爪
            if self.core_mapper is not None:
                grip_rad = getattr(self.backend, "grip_rad", lambda: None)()
                grip_tau = getattr(self.backend, "grip_tau", lambda: None)()
                # 实测关节速度给核心的直控速度环用（PID 控速；无反馈时核心自行差分）
                # 夹爪力矩给核心的力控/堵转判定用（夹住东西/到限位时会停手）
                cmd = self.core_mapper.step(pen, state.pos, state.tau, grip_rad,
                                            vel_meas=state.vel, grip_tau=grip_tau)
                self.backend.send_mit(cmd.q, cmd.kp, cmd.kd, cmd.tau, grip_rad=cmd.grip_send)
                grip_target = getattr(self.backend, "grip_target", lambda: state.grip)()
                action = np.concatenate([cmd.q, [float(grip_target)]])
                self._last_state = state
                self._last_action = action
                self._tick_times.append(time.monotonic())
                if self._recorder is not None:
                    self._recorder.add(state, pen, action)
                return

            action = self.mapper.map(pen, state.pos)
            self.backend.command(action)

            self._last_state = state
            self._last_action = action
            self._tick_times.append(time.monotonic())

            if self._recorder is not None:
                self._recorder.add(state, pen, action)

    # ================================================================== #
    # 手势（电脑摄像头）→ 夹爪行程
    # ================================================================== #
    def _apply_gesture_locked(self) -> None:
        """把"手势变化"的行程目标交给控制核心（限速/力矩保护由核心负责）。

        只在 updates 变化时下发：同一手势保持不会反复覆盖，手动（笔/Q/A）调整
        夹爪后不会被持续抢回；换手势或手离开后再做同一手势会重新生效。
        """
        if self.gesture is None or self.core_mapper is None:
            return
        st = self.gesture.state()
        target = st.get("target")
        if target is None or int(st.get("updates") or 0) == self._gesture_updates:
            return
        core = self.core_mapper.core
        if core.freeze or not core.grip_ready:
            return
        if abs(core.j_req) > 1e-6 and int(core.j_sel) == 6:
            return          # 手动夹爪直控进行中：等松手后的下一次手势变化再接管
        core.set_gripper_target(float(target))
        self._gesture_updates = int(st.get("updates") or 0)

    def gesture_state(self) -> dict | None:
        return self.gesture.state() if self.gesture is not None else None

    def gesture_control(self, on: bool | None = None) -> dict | None:
        if self.gesture is None:
            return None
        if on is True:
            self.gesture.start()
        elif on is False:
            self.gesture.stop()
        return self.gesture.state()

    def run(self, stop_event: threading.Event | None = None) -> None:
        stop_event = stop_event or self._loop_stop
        period = 1.0 / max(1, self.fps)
        while not stop_event.is_set():
            t0 = time.monotonic()
            self.tick()
            dt = period - (time.monotonic() - t0)
            if dt > 0:
                time.sleep(dt)

    def run_steps(self, seconds: float, pen_feed: Callable[[int], PenSample | None] | None = None) -> None:
        """同步驱动（自检/脚本用）：按 fps 跑指定时长。"""
        period = 1.0 / max(1, self.fps)
        total = max(1, int(seconds * self.fps))
        for i in range(total):
            if pen_feed is not None:
                self.ingest_pen_sample(pen_feed(i))
            t0 = time.monotonic()
            self.tick()
            dt = period - (time.monotonic() - t0)
            if dt > 0:
                time.sleep(dt)

    def warmup(self, target: np.ndarray, seconds: float = 1.0) -> None:
        """把机械臂开到指定姿态（例如 ready 位），不计入数据。

        期间用"手动目标"覆盖遥操映射；操作者一旦落笔，自动交还给遥操。
        说明：target 为 7 维（6 关节 + 夹爪 0-1）；若第 7 位为 0，则保持当前夹爪开度。
        """
        tgt = np.asarray(target, dtype=float).reshape(-1).copy()
        with self._lock:
            if tgt.size > self.profile.n_joints and tgt[self.profile.n_joints] == 0.0 and self._last_state is not None:
                tgt[self.profile.n_joints] = float(self._last_state.grip)
            self._pen = None  # 清除陈旧笔输入，避免 goto 期间被误接管
            self._override = tgt
            if hasattr(self.mapper, "reset"):
                self.mapper.reset(tgt[: self.profile.n_joints])
        self.run_steps(seconds)
        self._sync_mapper_to_measured()

    def _sync_mapper_to_measured(self) -> None:
        """把映射器内部累加器同步到当前实测姿态。

        否则 goto/warmup 结束后，操作者一落笔，映射器会从旧目标开始算 → 机械臂跳变。
        """
        with self._lock:
            if self._last_state is None:
                return
            grip_rad = getattr(self.backend, "grip_rad", lambda: None)()
            if self.core_mapper is not None:
                self.core_mapper.prime(self._last_state.pos.copy(), grip_rad)
            elif hasattr(self.mapper, "reset"):
                self.mapper.reset(self._last_state.pos.copy())

    # ================================================================== #
    # teleop_core 控制接口（等价上游键盘/屏幕按钮）
    # ================================================================== #
    def _require_core(self):
        if self.core_mapper is None:
            raise RuntimeError("当前不是 teleop_core 模式（请用 --backend rebot 启动）")
        return self.core_mapper

    def teleop_state(self) -> dict | None:
        return self.core_mapper.state() if self.core_mapper is not None else None

    def teleop_freeze(self, on: bool | None = None) -> dict:
        m = self._require_core()
        with self._lock:
            if on is None:
                m.toggle_freeze()
            else:
                m.set_freeze(on)
        return m.state()

    def teleop_speed(self, index: int | None = None) -> dict:
        m = self._require_core()
        with self._lock:
            if index is None:
                m.cycle_speed()
            else:
                m.core.speed_i = int(index) % 3
        return m.state()

    def teleop_mode(self, mode: str) -> dict:
        m = self._require_core()
        with self._lock:
            m.set_mode(mode)
        return m.state()

    def teleop_motor(self, index: int | None = None, step: int = 1) -> dict:
        """选中电机并进入笔控直控模式（等价点 J 按钮 / 笔侧键左键）。

        `index=None` 时按 `step` 从**服务端当前**的选择循环（+1 = 下一个）：
        客户端的"下一个是谁"如果因为状态没刷新而算错，就会一直发同一个电机、
        看起来"切换不了"；交给服务端算就永远不会卡。
        """
        m = self._require_core()
        with self._lock:
            if index is None:
                index = (int(m.core.j_sel) + int(step)) % 7
            m.select_motor(int(index))
        return m.state()

    def teleop_float(self, on: bool) -> dict:
        m = self._require_core()
        with self._lock:
            m.set_float(bool(on))
        return m.state()

    def teleop_joint(self, index: int | None = None, hold: float | None = None,
                     step: int = 0) -> dict:
        """选关节（不改模式）/ 关节直控速度。`index=None, step=±1` = 服务端算上/下一个。"""
        m = self._require_core()
        with self._lock:
            if index is None and step:
                index = (int(m.core.j_sel) + int(step)) % 7
            if index is not None:
                m.select_joint(int(index))
            if hold is not None:
                m.set_joint_hold(float(hold))
        return m.state()

    def teleop_align(self) -> dict:
        m = self._require_core()
        with self._lock:
            m.align()
        return m.state()

    def teleop_preset(self, action: str, index: int) -> dict:
        m = self._require_core()
        with self._lock:
            if action == "record":
                m.record_preset(int(index))
            elif action == "goto":
                m.goto_preset(int(index))
            else:
                raise ValueError("action 只能是 record / goto")
            return {"ok": True, **m.state()}

    def teleop_tau_limit(self, value: float) -> dict:
        m = self._require_core()
        with self._lock:
            m.set_tau_limit(float(value))
        return m.state()

    # ================================================================== #
    # 回放 / 落盘
    # ================================================================== #
    def _find_episode(self, index: int | None):
        eps = [e for e in self._episodes if index is None or e.index == int(index)]
        if not eps:
            raise RuntimeError(f"没有找到 episode {index}")
        return eps[-1]

    def save_episode(self, index: int | None = None) -> dict:
        """把一条 episode 落盘为 Parquet（防止只存内存丢失）。"""
        from ..recorder.writers import save_episode_parquet

        with self._lock:
            ep = self._find_episode(index)
            path = episodes_dir() / f"{self._run_id}_ep{ep.index:06d}.parquet"
        saved = save_episode_parquet(path, ep)
        with self._lock:
            ep.files["data_parquet"] = str(saved)
        return {"ok": True, "episode": ep.index, "path": str(saved), "frames": ep.n_frames}

    def replay_episode(self, index: int | None = None, speed: float = 1.0, tau_abort: float = 25.0,
                       dataset_path: str | None = None, max_joint_speed_deg: float = 220.0) -> dict:
        """回放一条 episode：最小 jerk 到起点 → 按原时间轴回放（插值到控制环速率）。

        - `speed`：回放倍速（>1 更快）；`max_joint_speed_deg`：指令关节速度上限（保护，默认 220°/s）
        - `dataset_path`：从已打包数据集回放；否则用内存中的 episode（都没有则用最近打包的数据集）
        - 回放中按 `Space`（冻结）可随时中止；力矩 > tau_abort 也会中止
        - 期间操作者一落笔，自动交还给遥操（与上游 replay 行为一致）
        """
        if dataset_path is None and not self._episodes and self._last_dataset:
            dataset_path = self._last_dataset
        if dataset_path:
            src = self._load_replay_from_dataset(dataset_path, index)
        else:
            with self._lock:
                ep = self._find_episode(index)
                src = {
                    "episode": ep.index, "frames": ep.n_frames,
                    "action": ep.action.copy(), "t": ep.t - ep.t[0] if ep.n_frames else ep.t.copy(),
                }
        action = np.asarray(src["action"], float)
        times = np.asarray(src["t"], float)
        ep_index = int(src["episode"])
        n_frames = int(src["frames"])

        with self._lock:
            if self._recorder is not None:
                raise RuntimeError("正在录制 episode，请先结束再回放")

        if (action.ndim != 2 or action.shape != (n_frames, self.profile.n_motors)
                or times.shape != (n_frames,) or n_frames == 0
                or not np.isfinite(action).all() or not np.isfinite(times).all()
                or times[0] < 0 or np.any(np.diff(times) <= 0)):
            raise RuntimeError("回放数据无效：需要有限的 7 维动作和严格递增的时间戳")
        if not all(math.isfinite(v) and v > 0 for v in (speed, tau_abort, max_joint_speed_deg)):
            raise RuntimeError("回放速度、力矩阈值和关节速度上限必须为有限正数")

        q_max = float(np.max(np.abs(np.degrees(action[:, :6])))) if action.size else 0.0
        v = np.diff(action[:, :6], axis=0) / np.clip(np.diff(times)[:, None], 1e-4, None)
        v_max = float(np.max(np.abs(np.degrees(v)))) if v.size else 0.0
        grip_range = (float(action[:, -1].min()), float(action[:, -1].max())) if action.size else (0.0, 0.0)
        stats = {
            "episode": ep_index, "frames": n_frames, "duration_s": round(float(times[-1]), 2),
            "joint_max_deg": round(q_max, 1), "joint_speed_max_deg_s": round(v_max, 1),
            "grip_range": [round(grip_range[0], 3), round(grip_range[1], 3)],
            "source": dataset_path or "memory", "speed": speed,
            "max_joint_speed_deg_s": max_joint_speed_deg,
        }

        # 1) 平滑到轨迹起点（最小 jerk）
        approach = self.goto_smooth(action[0].copy(), duration=None, tau_abort=tau_abort)
        if not approach.get("ok", False):
            return {"ok": False, "aborted": True, "played": 0,
                    "reason": "到起点运动中止，已取消回放", **stats}

        # 2) 按时间轴回放（带指令速度上限）
        speed = max(0.05, float(speed))
        dt = 1.0 / max(1, self.fps)
        step = math.radians(float(max_joint_speed_deg)) * dt  # 单周期最大变化
        t0 = time.monotonic()
        aborted = False
        n_played = 0
        last = action[0].copy()
        while True:
            tt = (time.monotonic() - t0) * speed
            done = tt >= float(times[-1])
            tt_clamped = min(tt, float(times[-1]))
            raw = np.array([np.interp(tt_clamped, times, action[:, k]) for k in range(action.shape[1])])
            target = last + np.clip(raw - last, -step, step)  # 速度上限保护
            last = target
            with self._lock:
                if self._pen is not None and self._pen.touching and (
                    time.monotonic() - self._pen_rx
                ) < self.pen_timeout:
                    self._override = None
                    if self.core_mapper is not None and self._last_state is not None:
                        grip_rad = getattr(self.backend, "grip_rad", lambda: None)()
                        self.core_mapper.prime(self._last_state.pos, grip_rad)
                    return {"ok": False, "aborted": True, "reason": "操作者落笔，已交还遥操", **stats}
                frozen = bool(self.core_mapper is not None and self.core_mapper.core.freeze)
                tau = self._last_state.tau.copy() if self._last_state is not None else None
                if frozen or (tau is not None and float(np.max(np.abs(tau))) > tau_abort):
                    aborted = True
                    break
                self._override = target
            n_played += 1
            if done and float(np.max(np.abs(last - action[-1]))) < 1e-3:
                break
            time.sleep(dt)

        # 3) 收尾：保持（或中止时原地保持）
        with self._lock:
            if aborted and self._last_state is not None:
                self._override = np.concatenate([self._last_state.pos.copy(), [self._last_state.grip]])
        self.run_steps(0.4)
        self._sync_mapper_to_measured()
        return {"ok": not aborted, "aborted": aborted, "played": n_played, **stats}

    def _load_replay_from_dataset(self, dataset_path: str, index: int | None) -> dict:
        """从打包好的数据集 parquet 回放（服务重启后仍可用）。"""
        from ..recorder.writers import JOINT_NAMES, load_episode_rows

        ds = Path(dataset_path)
        cand = sorted((ds / "data").rglob("episode_*.parquet")) + sorted((ds / "data").rglob("episode_*.jsonl"))
        if not cand:
            raise RuntimeError(f"数据集里没有 episode 数据：{ds}")
        if index is not None:
            want = f"episode_{int(index):06d}."
            cand = [p for p in cand if p.name.startswith(want)]
            if not cand:
                raise RuntimeError(f"数据集里没有 episode {index}：{ds}")
        path = cand[-1]
        rows = load_episode_rows(path)
        if not rows:
            raise RuntimeError(f"空数据：{path}")
        t = np.asarray([r["timestamp"] for r in rows], float)
        keys = [n for n in JOINT_NAMES if f"action.{n}" in rows[0]]
        act = np.asarray([[r[f"action.{n}"] for n in keys] for r in rows], float)
        grip = np.asarray([r.get("action.gripper", r.get("gripper.pos", 0.0)) for r in rows], float)
        action = np.concatenate([act, grip.reshape(-1, 1)], axis=1)
        ep_index = int(rows[0].get("episode_index", 0))
        return {"episode": ep_index, "frames": len(rows), "action": action, "t": t}

    def teleop_twist(self, value: float) -> dict:
        m = self._require_core()
        with self._lock:
            m.set_twist(float(value))
        return m.state()

    def park(self, duration: float = 3.0, hold: float = 0.5, tau_abort: float = 25.0) -> dict:
        """回零：与 reBotArm_control_py 的 Esc 退出逻辑一致。

        - 最小 jerk 曲线（10a³-15a⁴+6a⁵）从当前姿态平滑回折叠零位
        - 全程重力前馈、**不失能**（与上游一致）
        - 力矩监测：任一路 > tau_abort 立即停止归零并原地保持
        """
        with self._lock:
            st = self._last_state
            if st is None:
                return {"ok": False, "reason": "还没有设备状态"}
            q_from = st.pos.copy()
            grip = float(st.grip)

        fps = max(1, self.fps)
        dt = 1.0 / fps
        steps = max(1, int(duration * fps))
        aborted = False

        for i in range(1, steps + 1):
            s = _smoothstep(i / steps)
            q_t = q_from * (1.0 - s)
            with self._lock:
                self._override = np.concatenate([q_t, [grip]])
            time.sleep(dt)
            if i % max(1, int(0.2 * fps)) == 0:
                with self._lock:
                    tau = self._last_state.tau.copy() if self._last_state is not None else None
                if tau is not None and float(np.max(np.abs(tau))) > tau_abort:
                    aborted = True
                    break

        if aborted:
            with self._lock:
                q_hold = self._last_state.pos.copy() if self._last_state is not None else q_from
                self._override = np.concatenate([q_hold, [grip]])
        else:
            with self._lock:
                self._override = np.concatenate([np.zeros(self.profile.n_joints), [grip]])
            self.run_steps(hold)

        self._sync_mapper_to_measured()
        return {"ok": not aborted, "aborted": aborted, "steps": steps}

    def goto_smooth(self, target: np.ndarray, duration: float | None = None, tau_abort: float = 25.0) -> dict:
        """限时平滑 goto：与上游"姿势预设/回放"同样的最小 jerk 曲线。

        - 时长按最大关节位移自动取（0.3 rad/s 参考），限制在 1.5~6 s
        - 第 7 位（夹爪）为 0 时保持当前开度
        - 全程通过 `_override` 下发，操作者落笔会自动交还给遥操
        """
        tgt = np.asarray(target, dtype=float).reshape(-1).copy()
        with self._lock:
            if self._last_state is None:
                raise RuntimeError("还没有设备状态，无法 goto")
            n = self.profile.n_joints
            if tgt.size > n and tgt[n] == 0.0:
                tgt[n] = float(self._last_state.grip)
            q_from = self._last_state.pos.copy()
            grip = float(tgt[n]) if tgt.size > n else float(self._last_state.grip)
            self._pen = None  # 清除陈旧笔输入，避免 goto 期间被误接管
            self._override = np.concatenate([q_from, [grip]])
            if hasattr(self.mapper, "reset"):
                self.mapper.reset(q_from)

        delta = float(np.max(np.abs(tgt[:n] - q_from)))
        if duration is None:
            duration = float(np.clip(delta / 0.3, 1.5, 6.0))

        fps = max(1, self.fps)
        dt = 1.0 / fps
        steps = max(1, int(duration * fps))
        aborted = False

        for i in range(1, steps + 1):
            s = _smoothstep(i / steps)
            q_t = q_from + (tgt[:n] - q_from) * s
            with self._lock:
                if (self._pen is not None and self._pen.touching
                        and time.monotonic() - self._pen_rx < self.pen_timeout):
                    self._override = None
                    self._sync_mapper_to_measured()
                    return {"ok": False, "aborted": True, "reason": "操作者落笔，已交还遥操"}
                frozen = bool(self.core_mapper is not None and self.core_mapper.core.freeze)
                tau = self._last_state.tau if self._last_state is not None else None
                if frozen or (tau is not None and float(np.max(np.abs(tau))) > tau_abort):
                    aborted = True
                    break
                self._override = np.concatenate([q_t, [grip]])
            time.sleep(dt)

        with self._lock:
            if aborted:
                q_hold = self._last_state.pos.copy() if self._last_state is not None else q_t
                self._override = np.concatenate([q_hold, [grip]])
            else:
                self._override = np.concatenate([tgt[:n], [grip]])
        self.run_steps(0.5)
        self._sync_mapper_to_measured()

        return {
            "ok": not aborted,
            "aborted": aborted,
            "duration": round(duration, 2),
            "target": np.round(tgt, 3).tolist(),
        }

    def start_loop(self) -> None:
        if self._loop_thread and self._loop_thread.is_alive():
            return
        self._loop_stop.clear()
        self._loop_thread = threading.Thread(target=self.run, name="capture-loop", daemon=True)
        self._loop_thread.start()

    def stop_loop(self) -> None:
        self._loop_stop.set()
        if self._loop_thread:
            self._loop_thread.join(timeout=2.0)
            self._loop_thread = None

    # ================================================================== #
    # 输入
    # ================================================================== #
    def ingest_pen_sample(self, sample: PenSample) -> None:
        with self._lock:
            self._pen = sample
            self._pen_rx = time.monotonic()
            self._pen_count += 1
            self._pen_window.append(time.monotonic())

    def ingest_pen(self, data: dict) -> None:
        self.ingest_pen_sample(PenSample.from_dict(data))

    # ================================================================== #
    # 状态 / 指标
    # ================================================================== #
    def status(self) -> dict:
        with self._lock:
            st = self.backend.status()
            return {
                "profile": {"name": self.profile.name, "arm_model": self.profile.arm_model},
                "device": st.to_dict(),
                "backend": getattr(self.backend, "name", "unknown"),
                "session": dict(self._session) if self._session else None,
                "recording": self._recorder is not None,
                "episodes": len(self._episodes),
                "fps_target": self.fps,
                "camera": self.camera.status() if self.camera is not None else None,
            }

    def live(self) -> dict:
        with self._lock:
            now = time.monotonic()
            recent = [t for t in self._tick_times if now - t < 1.0]
            fps_actual = float(len(recent))
            pen_recent = [t for t in self._pen_window if now - t < 1.0]

            state = self._last_state
            action = self._last_action
            lights = {"smooth": True, "limit": False, "drop": fps_actual >= self.fps * 0.8, "occlusion": True}

            joints: list[dict[str, Any]] = []
            if state is not None and action is not None:
                for i, jp in enumerate(self.profile.joints):
                    lo, hi = jp.limits_rad
                    margin = float(min(abs(state.pos[i] - lo), abs(hi - state.pos[i])))
                    joints.append(
                        {
                            "name": jp.name,
                            "pos": round(float(state.pos[i]), 3),
                            "target": round(float(action[i]), 3),
                            "margin": round(margin, 3),
                            "at_limit": bool(margin < 1e-3),
                        }
                    )
                lights["limit"] = any(j["at_limit"] for j in joints)

            rec = None
            if self._recorder is not None:
                rec = {"episode_index": self._recorder.index, "frames": len(self._recorder._t)}

            return {
                "t": now,
                "fps_actual": round(fps_actual, 1),
                "pen_hz": round(float(len(pen_recent)), 1),
                "lights": lights,
                "joints": joints,
                "gripper": round(float(state.grip), 3) if state else 0.0,
                "recording": rec,
                "episodes": len(self._episodes),
                "session_active": self._session is not None,
                "teleop": self.teleop_state(),
                "camera": self.camera.status() if self.camera is not None else None,
                "gesture": self.gesture_state(),
            }

    def episodes_list(self, with_details: bool = False) -> list[dict]:
        with self._lock:
            return [e.to_dict(with_details=with_details) for e in self._episodes]

    # ================================================================== #
    # 打包
    # ================================================================== #
    def pack(
        self,
        name: str | None = None,
        task_instruction: str | None = None,
        include_failed: bool = False,
    ) -> dict:
        with self._lock:
            eps = list(self._episodes)
            if not include_failed:
                eps = [e for e in eps if e.grade != "F"]
            if not eps:
                raise RuntimeError("没有可打包的合格 episode（硬门未通过或还没有数据）")
            ds_name = name or time.strftime("rebot_%Y%m%d_%H%M%S")
            result = build_dataset(
                name=ds_name,
                episodes=eps,
                profile=self.profile,
                fps=self.fps,
                task_instruction=task_instruction,
            )
            self._last_dataset = result.get("path")
            return result
