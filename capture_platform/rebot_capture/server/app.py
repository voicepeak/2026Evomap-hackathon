"""采集端本地服务：REST + WebSocket + 静态 Web UI。

启动：
    rebot-capture serve --backend mock
    浏览器打开 http://127.0.0.1:8787
"""
from __future__ import annotations

import asyncio
import dataclasses
import json
import time
from pathlib import Path

from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles

from .. import __version__
from ..camera import CameraHub
from ..device.mock_arm import MockArm
from ..device.profile import DeviceProfile
from ..paths import web_dir
from ..quality.rules import QualityConfig
from ..recorder.session import CaptureService
from .schemas import (
    CameraAliasIn,
    CameraCloseIn,
    CameraOpenIn,
    EpisodeStopIn,
    FloatIn,
    FreezeIn,
    GotoIn,
    JointIn,
    ModeIn,
    PackIn,
    PenSampleIn,
    PresetIn,
    ReplayIn,
    SessionStartIn,
    SpeedIn,
    TauLimitIn,
    TwistIn,
)


def create_app(
    profile_path: str | Path | None = None,
    backend: str = "mock",
    fps: int | None = None,
    arm_repo: str | None = None,
    limit_margin_deg: float = 1.5,
    allow_nonzero_start: bool = False,
) -> FastAPI:
    profile = DeviceProfile.load(profile_path)
    mapper = None
    camera = CameraHub()
    if backend == "mock":
        arm = MockArm(profile)
    elif backend == "rebot":
        from ..device.real_arm import RealArm
        from ..integrations.rebotarm import RebotArmRepo
        from ..teleop.rebot_core import RebotCoreMapper

        repo = RebotArmRepo(arm_repo)
        mapper = RebotCoreMapper(profile, repo=repo)  # 控制律 = teleop_core（与 spatial_teleop 同逻辑）
        arm = RealArm(
            profile,
            repo=repo,
            limit_margin_deg=limit_margin_deg,
            allow_nonzero_start=allow_nonzero_start,
            fps=fps or 100,
        )
        fps = fps or 100
    else:
        raise ValueError(f"未知 backend: {backend}（可选 mock / rebot）")

    service = CaptureService(profile, arm, mapper=mapper, fps=fps, camera=camera)
    service.connect()
    service.start_loop()

    app = FastAPI(title="rebot-capture", version=__version__)
    app.state.service = service

    @app.on_event("shutdown")
    def _on_shutdown() -> None:  # 与上游 Esc 一致：先平滑归零（不失能）
        try:
            service.close()
        except Exception:  # noqa: BLE001
            pass

    # ---------------------------------------------------------------- #
    @app.get("/api/health")
    def health() -> dict:
        return {
            "ok": True,
            "version": __version__,
            "backend": getattr(service.backend, "name", "unknown"),
            "time": time.time(),
        }

    @app.get("/api/profile")
    def get_profile() -> dict:
        return profile.summary()

    @app.get("/api/device/status")
    def device_status() -> dict:
        return service.status()

    @app.get("/api/device/diagnostics")
    def device_diagnostics() -> dict:
        out: dict = {}
        diag = getattr(service.backend, "diagnostics", None)
        if callable(diag):
            out["device"] = diag()
        mdiag = getattr(service.mapper, "diagnostics", None)
        if callable(mdiag):
            out["mapper"] = mdiag()
        return out

    @app.get("/api/live")
    def live() -> dict:
        return service.live()

    @app.get("/api/quality/rules")
    def quality_rules() -> dict:
        cfg = service.quality_cfg
        return {"config": dataclasses.asdict(cfg)}

    # ---------------------------------------------------------------- #
    # 相机：状态 / 探测 / 打开 / 关闭 / MJPEG 预览
    # ---------------------------------------------------------------- #
    @app.get("/api/camera/status")
    def camera_status() -> dict:
        return camera.status()

    @app.post("/api/camera/probe")
    def camera_probe() -> dict:
        found = camera.probe()
        return {"found": found, **camera.status()}

    @app.post("/api/camera/open")
    def camera_open(body: CameraOpenIn) -> dict:
        if body.url:
            return camera.open_url(body.url)
        if body.index is None:
            raise HTTPException(status_code=422, detail="需要 index 或 url")
        return camera.open(body.index)

    @app.post("/api/camera/close")
    def camera_close(body: CameraCloseIn) -> dict:
        return camera.close(body.index)

    @app.post("/api/camera/alias")
    def camera_alias(body: CameraAliasIn) -> dict:
        key = body.url if body.url else body.index
        return camera.set_alias(key, body.name)

    @app.get("/api/camera/stream")
    async def camera_stream(request: Request):
        async def gen():
            boundary = b"--frame\r\n"
            while True:
                if await request.is_disconnected():
                    break
                jpg = camera.snapshot()
                if jpg:
                    yield boundary + b"Content-Type: image/jpeg\r\nContent-Length: " + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n"
                await asyncio.sleep(1.0 / 20.0)
        return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=frame")

    # ---------------------------------------------------------------- #
    @app.post("/api/device/goto")
    def device_goto(body: GotoIn) -> dict:
        return service.goto_smooth(body.target, duration=body.duration)

    @app.post("/api/device/park")
    def device_park() -> dict:
        service.park()
        return {"ok": True, "park": True}

    @app.post("/api/replay")
    def replay(body: ReplayIn) -> dict:
        try:
            return service.replay_episode(
                index=body.index,
                speed=body.speed,
                tau_abort=body.tau_abort,
                dataset_path=body.dataset_path,
                max_joint_speed_deg=body.max_joint_speed_deg,
            )
        except RuntimeError as e:
            raise HTTPException(status_code=409, detail=str(e))

    @app.post("/api/episodes/save")
    def episode_save(body: EpisodeStopIn) -> dict:  # 复用 {success, note} 只为带空 body
        try:
            return service.save_episode()
        except RuntimeError as e:
            raise HTTPException(status_code=409, detail=str(e))

    # ---------------------------------------------------------------- #
    # teleop_core 控制（等价上游键盘/屏幕按钮）
    # ---------------------------------------------------------------- #
    def _core_guard(fn, *args, **kwargs):
        try:
            return fn(*args, **kwargs)
        except RuntimeError as e:
            raise HTTPException(status_code=409, detail=str(e))

    @app.get("/api/teleop/state")
    def teleop_state() -> dict:
        st = service.teleop_state()
        if st is None:
            raise HTTPException(status_code=409, detail="当前不是 teleop_core 模式")
        return st

    @app.post("/api/teleop/freeze")
    def teleop_freeze(body: FreezeIn) -> dict:
        return _core_guard(service.teleop_freeze, body.on)

    @app.post("/api/teleop/speed")
    def teleop_speed(body: SpeedIn) -> dict:
        return _core_guard(service.teleop_speed, body.index)

    @app.post("/api/teleop/mode")
    def teleop_mode(body: ModeIn) -> dict:
        return _core_guard(service.teleop_mode, body.mode)

    @app.post("/api/teleop/float")
    def teleop_float(body: FloatIn) -> dict:
        return _core_guard(service.teleop_float, body.on)

    @app.post("/api/teleop/joint")
    def teleop_joint(body: JointIn) -> dict:
        return _core_guard(service.teleop_joint, body.index, body.hold)

    @app.post("/api/teleop/align")
    def teleop_align() -> dict:
        return _core_guard(service.teleop_align)

    @app.post("/api/teleop/preset")
    def teleop_preset(body: PresetIn) -> dict:
        return _core_guard(service.teleop_preset, body.action, body.index)

    @app.post("/api/teleop/tau_limit")
    def teleop_tau_limit(body: TauLimitIn) -> dict:
        return _core_guard(service.teleop_tau_limit, body.value)

    @app.post("/api/teleop/twist")
    def teleop_twist(body: TwistIn) -> dict:
        return _core_guard(service.teleop_twist, body.value)

    # ---------------------------------------------------------------- #
    @app.post("/api/session/start")
    def session_start(body: SessionStartIn) -> dict:
        try:
            return service.start_session(task_id=body.task_id, operator=body.operator)
        except RuntimeError as e:
            raise HTTPException(status_code=409, detail=str(e))

    @app.post("/api/session/stop")
    def session_stop() -> dict:
        try:
            return service.stop_session()
        except RuntimeError as e:
            raise HTTPException(status_code=409, detail=str(e))

    # ---------------------------------------------------------------- #
    @app.post("/api/episode/start")
    def episode_start() -> dict:
        try:
            return service.start_episode()
        except RuntimeError as e:
            raise HTTPException(status_code=409, detail=str(e))

    @app.post("/api/episode/stop")
    def episode_stop(body: EpisodeStopIn) -> dict:
        try:
            return service.stop_episode(success=body.success, note=body.note)
        except RuntimeError as e:
            raise HTTPException(status_code=409, detail=str(e))

    @app.post("/api/episode/discard")
    def episode_discard() -> dict:
        try:
            return service.discard_episode()
        except RuntimeError as e:
            raise HTTPException(status_code=409, detail=str(e))

    @app.get("/api/episodes")
    def episodes(details: bool = False) -> dict:
        return {"episodes": service.episodes_list(with_details=details)}

    @app.post("/api/pack")
    def pack(body: PackIn) -> dict:
        try:
            return service.pack(name=body.name, task_instruction=body.task_instruction, include_failed=body.include_failed)
        except RuntimeError as e:
            raise HTTPException(status_code=409, detail=str(e))

    # ---------------------------------------------------------------- #
    @app.websocket("/ws/teleop")
    async def ws_teleop(ws: WebSocket) -> None:
        await ws.accept()
        try:
            while True:
                raw = await ws.receive_text()
                try:
                    data = json.loads(raw)
                except json.JSONDecodeError:
                    continue
                if isinstance(data, dict) and data.get("type") == "pen":
                    data = data.get("data", {})
                if isinstance(data, dict):
                    service.ingest_pen(data)
        except WebSocketDisconnect:
            return

    @app.websocket("/ws/state")
    async def ws_state(ws: WebSocket) -> None:
        await ws.accept()
        try:
            while True:
                payload = {"live": service.live(), "status": service.status()}
                await ws.send_text(json.dumps(payload, ensure_ascii=False))
                await asyncio.sleep(0.1)
        except WebSocketDisconnect:
            return

    # Web UI（放最后，避免遮挡 /api 与 /ws）
    app.mount("/", StaticFiles(directory=web_dir(), html=True), name="web")
    return app


# uvicorn 入口：uvicorn rebot_capture.server.app:app
app = create_app()
