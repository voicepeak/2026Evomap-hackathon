"""命令行入口：serve / selftest / doctor。"""
from __future__ import annotations

import argparse
import json
import logging
import os
import platform
import sys

from . import __version__


def _doctor() -> int:
    from .device.profile import DeviceProfile
    from .paths import data_home

    print(f"rebot-capture {__version__}")
    print(f"python  : {sys.version.split()[0]} ({platform.platform()})")
    print(f"data    : {data_home()}")

    try:
        import fastapi  # noqa: F401
        import uvicorn

        print(f"fastapi : {fastapi.__version__}  uvicorn {uvicorn.__version__}")
    except Exception as e:  # pragma: no cover
        print(f"fastapi : 缺失（{e}）")

    try:
        import numpy

        print(f"numpy   : {numpy.__version__}")
    except Exception:  # pragma: no cover
        print("numpy   : 缺失")

    try:
        import pyarrow

        print(f"pyarrow : {pyarrow.__version__}  （Parquet 主格式）")
    except Exception:  # pragma: no cover
        print("pyarrow : 缺失 → 将退化为 JSONL")

    try:
        import cv2  # noqa: F401

        print("opencv  : 已安装（相机录制可用）")
    except Exception:
        print("opencv  : 未安装（可选：pip install 'rebot-capture[camera]'）")

    try:
        profile = DeviceProfile.load()
        print("")
        print(f"设备档案: {profile.name}（{profile.arm_model}，{profile.n_motors} 电机，{profile.control_mode}）")
        for j in profile.joints:
            print(f"  - J{j.motor_id} {j.name:<14s} {j.model}  限位 {j.limits_rad}  kp/kd {j.kp}/{j.kd}")
        print(f"  - G{profile.gripper.motor_id} {profile.gripper.name:<14s} {profile.gripper.model}  力比 {profile.gripper.force_ratio}")
    except Exception as e:
        print(f"设备档案: 加载失败（{e}）")
        return 1
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="rebot-capture", description="RDP 采集端（本地优先）")
    parser.add_argument("--version", action="version", version=__version__)
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_serve = sub.add_parser("serve", help="启动采集端本地服务（REST + WS + Web UI）")
    p_serve.add_argument("--host", default="127.0.0.1")
    p_serve.add_argument("--port", type=int, default=8787)
    p_serve.add_argument("--backend", default="mock", choices=["mock", "rebot"])
    p_serve.add_argument("--profile", default=None, help="设备档案 JSON 路径")
    p_serve.add_argument("--fps", type=int, default=None)
    p_serve.add_argument("--arm-repo", default=None, help="reBotArm_control_py 仓库路径（默认自动探测/环境变量 REBOT_ARM_REPO）")
    p_serve.add_argument("--limit-margin", type=float, default=1.5, help="关节软限位余量（度）")
    p_serve.add_argument("--allow-nonzero-start", action="store_true", help="允许非零位启动使能（默认拒绝）")
    p_serve.add_argument("--gesture-camera", type=int, default=0,
                         help="电脑摄像头索引（手势→夹爪行程；默认 0）")
    p_serve.add_argument("--no-gesture", action="store_true", help="关闭手势夹爪")

    p_gui = sub.add_parser("gui", help="启动桌面 GUI（默认自动拉起本机服务）")
    p_gui.add_argument("--url", default="http://127.0.0.1:8787", help="采集服务地址")
    p_gui.add_argument("--no-autostart", action="store_true", help="不自动拉起服务（只连已有服务）")
    p_gui.add_argument("--windowed", action="store_true", help="窗口模式启动（默认全屏）")
    p_gui.add_argument("--backend", default="rebot", choices=["mock", "rebot"],
                       help="自动拉起服务时用的后端（默认真机）")
    p_gui.add_argument("--arm-repo", default=None, help="自动拉起服务时的 arm_control 路径")
    p_gui.add_argument("--gesture-camera", type=int, default=2, help="自动拉起服务时的手势相机索引")
    p_gui.add_argument("--arm-camera", type=int, default=0,
                       help="机械臂上的相机索引（默认 0；面板/小窗固定显示这一路）")
    p_gui.add_argument("--service-log", default="/tmp/rebot_gui_serve.log", help="自动拉起服务的日志路径")
    p_gui.add_argument("--panel", action="store_true", help="启动即显示控制面板（用于先配相机）")
    p_gui.add_argument("--cover", action="store_true",
                       help="铺满屏幕模式（无边框 + 置顶，不占独立全屏 Space；录屏/截图用）")
    p_gui.add_argument("--debug-events", action="store_true", help="打印键鼠事件（排查幽灵输入用）")

    p_self = sub.add_parser("selftest", help="无硬件自检：遥操→录制→质检→打包")
    p_self.add_argument("--quiet", action="store_true")
    p_self.add_argument("--ik", action="store_true", help="额外做运动学/映射自检（需要 reBotArm_control_py）")
    p_self.add_argument("--arm-repo", default=None, help="reBotArm_control_py 仓库路径")

    sub.add_parser("doctor", help="环境与设备档案检查")

    p_index = sub.add_parser("index", help="生成原始数据清单（含 sha256）")
    p_index.add_argument("--out", default=None, help="输出路径（默认 data/MANIFEST.json）")

    args = parser.parse_args(argv)

    if args.cmd == "serve":
        from .server.app import create_app

        app = create_app(
            profile_path=args.profile,
            backend=args.backend,
            fps=args.fps,
            arm_repo=args.arm_repo,
            limit_margin_deg=args.limit_margin,
            allow_nonzero_start=args.allow_nonzero_start,
            gesture_camera=args.gesture_camera,
            gesture=not args.no_gesture,
        )
        import uvicorn

        print(f"rebot-capture {__version__} → http://{args.host}:{args.port}  (backend={args.backend})")
        uvicorn.run(app, host=args.host, port=args.port, log_level="info")

        # 收尾：uvicorn 已经把该做的都做完了（lifespan 里平滑归零 + 释放设备 + 停线程），
        # 但 MediaPipe 的 GL/推理线程会让解释器卡在 finalization（现象：端口已释放、
        # "Application shutdown complete" 之后进程一直不退）。这里直接收尾。
        logging.shutdown()
        os._exit(0)

    if args.cmd == "gui":
        from .gui.app import run_gui

        return run_gui(
            args.url,
            autostart=not args.no_autostart,
            windowed=args.windowed,
            backend=args.backend,
            arm_repo=args.arm_repo,
            gesture_camera=args.gesture_camera,
            service_log=args.service_log,
            start_panel=args.panel,
            debug_events=args.debug_events,
            cover=args.cover,
            arm_camera=args.arm_camera,
        )

    if args.cmd == "selftest":
        from .selftest import run_ik_selftest, run_selftest

        rc = run_selftest(verbose=not args.quiet)
        if args.ik:
            rc = max(rc, run_ik_selftest(arm_repo=args.arm_repo, verbose=not args.quiet))
        return rc

    if args.cmd == "doctor":
        return _doctor()

    if args.cmd == "index":
        from .index import build_index, print_index

        print_index(build_index(args.out))
        return 0

    return 1


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
