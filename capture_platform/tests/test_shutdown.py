"""一键关闭（界面上的「关闭」按钮）：回零 → 失能 → 退出服务。

`POST /api/shutdown` 的链路：
    1) `service.shutdown_sequence()`：先平滑归零（最小 jerk，保持悬停），
       再调 `backend.disable()` —— 真机有"必须在零位附近"的门，不到位会照实报错；
    2) 服务层再让进程退出（后台线程：停控制环 → 关后端 → SIGINT → 3s 兜底硬退）。

这里只测第 1 步（进程退出在路由里，不该在单测里触发）。
"""
import numpy as np


def test_shutdown_sequence_parks_and_disables(core_service):
    service = core_service
    # 前置：先把机械臂挪开零位（用 warmup 的手动目标，不走遥操）
    service.warmup(np.array([0.0, 0.3, 0.4, 0.0, 0.0, 0.0, 0.0]), seconds=0.5)
    st = service.backend.read()
    assert float(np.max(np.abs(st.pos))) > 0.05, "前置条件：机械臂应已离开零位"

    res = service.shutdown_sequence(disable=True)
    assert res["park"]["ok"] is True, res
    assert res["disable"]["ok"] is True, res

    st = service.backend.read()
    assert float(np.max(np.abs(st.pos))) < 0.05, \
        f"没回到零位：{np.round(np.degrees(st.pos), 1)}°"
    assert getattr(service.backend, "_enabled", True) is False, "没有失能"


def test_shutdown_sequence_can_skip_disable(core_service):
    res = core_service.shutdown_sequence(disable=False)
    assert res["park"]["ok"] is True, res
    assert res["disable"] is None
    assert getattr(core_service.backend, "_enabled", False) is True, "不该失能"


def test_shutdown_schema_defaults():
    """请求模型默认：回零后失能 + 退出进程（用 importlib 直接读文件，避免起服务）。"""
    import importlib.util
    from pathlib import Path

    path = Path(__file__).resolve().parents[1] / "rebot_capture" / "server" / "schemas.py"
    spec = importlib.util.spec_from_file_location("_rebot_schemas_shutdown", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    assert mod.ShutdownIn().disable is True
    assert mod.ShutdownIn().exit_process is True
    assert mod.ShutdownIn(disable=False).disable is False
