"""桌面 GUI（PySide6）：采集服务客户端。

    from rebot_capture.gui.app import run_gui
    run_gui("http://127.0.0.1:8787")
"""
from __future__ import annotations

__all__ = ["run_gui"]


def run_gui(*args, **kwargs):  # pragma: no cover - 延迟导入，避免无 Qt 环境导入失败
    from .app import run_gui as _run

    return _run(*args, **kwargs)
