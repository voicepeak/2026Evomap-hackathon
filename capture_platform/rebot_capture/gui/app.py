"""桌面 GUI 主窗口 + 服务引导 + 全部动作接线。

启动方式（推荐）：
    scripts/run_gui.command          # 双击：继承 Terminal 相机权限，服务 + GUI 一起起
    rebot-capture gui                # 服务已运行时只开界面

设计：GUI 只是采集服务的客户端 —— 遥操/录制/相机都留在服务进程里，
所以后端（已上真机验证的那套）完全没动，网页界面也仍可兜底使用。
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path
from typing import Any

from PySide6.QtCore import QEvent, QObject, Qt, QTimer
from PySide6.QtGui import QColor, QFont, QKeySequence, QPalette, QShortcut
from PySide6.QtWidgets import (QApplication, QFrame, QHBoxLayout, QLabel,
                               QMainWindow, QMessageBox, QStackedWidget,
                               QVBoxLayout, QWidget)

from . import theme
from .client import ApiError, ServiceClient
from .hud import TeleopView
from .panel import PanelView
from .widgets import Chip, button, row

DEFAULT_URL = "http://127.0.0.1:8787"


# ──────────────────────────────────────────────────────────────────────── #
# 服务引导
# ──────────────────────────────────────────────────────────────────────── #
def _health(url: str, timeout: float = 2.0) -> dict | None:
    try:
        with urllib.request.urlopen(url.rstrip("/") + "/api/health", timeout=timeout) as r:
            return json.load(r)
    except Exception:  # noqa: BLE001
        return None


def ensure_service(url: str, *, autostart: bool, backend: str = "rebot",
                   arm_repo: str | None = None, gesture_camera: int = 2,
                   log_path: str = "/tmp/rebot_gui_serve.log") -> tuple[bool, str]:
    """确保本地服务在跑；不在则拉起（继承 Terminal 的相机权限）。"""
    if _health(url) is not None:
        return True, ""
    if not autostart:
        return False, "服务未运行（用 --autostart 或先跑 run_serve.command）"

    port = url.rstrip("/").rsplit(":", 1)[-1]
    cmd = [sys.executable, "-m", "rebot_capture", "serve", "--backend", backend,
           "--port", str(port), "--gesture-camera", str(gesture_camera)]
    if backend == "rebot":
        repo = arm_repo or str(Path(__file__).resolve().parents[3] / "arm_control")
        cmd += ["--arm-repo", repo]
    log = open(log_path, "ab", buffering=0)
    log.write(f"\n=== rebot GUI 拉起服务 {time.strftime('%Y-%m-%d %H:%M:%S')} ===\n".encode())
    subprocess.Popen(cmd, stdout=log, stderr=log, stdin=subprocess.DEVNULL,
                     start_new_session=True)
    for _ in range(120):            # 最多等 30s（真机要连 CAN + 使能）
        time.sleep(0.25)
        if _health(url) is not None:
            return True, ""
    tail = ""
    try:
        tail = Path(log_path).read_text(encoding="utf-8", errors="replace")[-1500:]
    except Exception:  # noqa: BLE001
        pass
    return False, f"服务启动失败，日志尾部：\n\n{tail}"


# ──────────────────────────────────────────────────────────────────────── #
# 主窗口
# ──────────────────────────────────────────────────────────────────────── #
class MainWindow(QMainWindow):
    def __init__(self, url: str = DEFAULT_URL, *, start_panel: bool = False):
        super().__init__()
        self.setWindowTitle("reBot 采集端")
        self.resize(1440, 900)

        self.client = ServiceClient(url)
        self.client.state.connect(self._on_state)
        self.client.ws_ok.connect(self._on_ws)
        self.client.frame.connect(self._on_frame)

        self._last_state_at = 0.0
        self._ws_pen_ok = False
        self._live: dict = {}
        self._status: dict = {}
        self._streams: dict[Any, int] = {}      # 相机 key → 需要的最大宽度
        self._arm_key: Any = None
        self._pc_key: Any = None
        self._selected_key: Any = None
        self._tile_sub: dict[Any, str] = {}
        self._cam_sig: tuple | None = None
        self._auto_fs = True
        self._last_cartesian = "pos"
        self._fs_on = False
        self._fake_fs = False

        root = QWidget()
        self.setCentralWidget(root)
        outer = QVBoxLayout(root)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        # ── 顶栏 ────────────────────────────────────────────────
        top = QFrame()
        top.setObjectName("card")
        top.setStyleSheet(f"#card{{background:{theme.BG};border:none;border-bottom:1px solid {theme.LINE};border-radius:0;}}")
        top_lay = QHBoxLayout(top)
        top_lay.setContentsMargins(18, 10, 18, 10)
        top_lay.setSpacing(14)
        self.dot = QLabel("●")
        self.dot.setFont(theme.ui(15))
        self.dot.setStyleSheet(f"color:{theme.TEXT_FAINT}")
        brand = QLabel("reBot 采集端")
        brand.setFont(theme.ui(15, QFont.Weight.DemiBold))
        self.version = QLabel("v-")
        self.version.setObjectName("faint")
        top_lay.addWidget(self.dot)
        top_lay.addWidget(brand)
        top_lay.addWidget(self.version)
        top_lay.addSpacing(6)
        self.chips = [Chip("后端"), Chip("型号"), Chip("电机"), Chip("电压"), Chip("温度")]
        for c in self.chips:
            top_lay.addWidget(c)
        top_lay.addStretch(1)
        self.b_panel = button("面板")
        self.b_full = button("全屏")
        self.b_panel.clicked.connect(self.toggle_panel)
        self.b_full.clicked.connect(self.toggle_fullscreen)
        top_lay.addWidget(self.b_panel)
        top_lay.addWidget(self.b_full)
        outer.addWidget(top)

        # ── 视图 ────────────────────────────────────────────────
        self.stack = QStackedWidget()
        self.teleop = TeleopView(self)
        self.panel = PanelView(self)
        self.stack.addWidget(self.teleop)
        self.stack.addWidget(self.panel)
        outer.addWidget(self.stack, 1)

        # ── 状态条 ──────────────────────────────────────────────
        self.status_line = QLabel("就绪")
        self.status_line.setObjectName("muted")
        self.status_line.setContentsMargins(18, 5, 18, 6)
        self.status_line.setStyleSheet(
            f"color:{theme.TEXT_DIM};border-top:1px solid {theme.LINE};font-size:12px;"
        )
        outer.addWidget(self.status_line)

        # ── 快捷键（与网页一致）────────────────────────────────
        self._bind_keys()
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(1000)
        self._refresh_episodes()
        self.refresh_library()
        self._load_basics()
        if start_panel:
            self.stack.setCurrentIndex(1)
            self.b_panel.setText("遥操作")
            self.b_panel.setObjectName("on")

    # ================================================================== #
    # 快捷键
    # ================================================================== #
    def _bind_keys(self) -> None:
        pairs = [
            ("Space", lambda: self.post_teleop("freeze")),
            ("F", lambda: self.post_teleop("speed")),
            ("O", self.toggle_mode),
            ("H", self.toggle_float),
            ("R", lambda: self.post_teleop("align")),
            ("Esc", lambda: self.post_teleop("freeze", {"on": True})),
            ("[", lambda: self.select_motor(((self._live.get("teleop") or {}).get("joint_index", 3) - 1) % 7)),
            ("]", lambda: self.select_motor(((self._live.get("teleop") or {}).get("joint_index", 3) + 1) % 7)),
            (",", lambda: self.post_teleop("twist", {"value": -0.611})),
            (".", lambda: self.post_teleop("twist", {"value": 0.611})),
            ("Tab", self.toggle_panel),
            ("F11", self.toggle_fullscreen),
        ]
        for key, fn in pairs:
            sc = QShortcut(QKeySequence(key), self)
            sc.setContext(Qt.ShortcutContext.ApplicationShortcut)
            sc.activated.connect(fn)
    def keyPressEvent(self, ev) -> None:  # noqa: N802
        # 按住才动的键：Q / A（关节直控），松开归零
        key = ev.text().upper()
        if not ev.isAutoRepeat() and key in ("Q", "A"):
            self.post_teleop("joint", {"hold": 1.047 if key == "Q" else -1.047})
            return
        super().keyPressEvent(ev)

    def keyReleaseEvent(self, ev) -> None:  # noqa: N802
        key = ev.text().upper()
        if not ev.isAutoRepeat() and key in ("Q", "A"):
            self.post_teleop("joint", {"hold": 0})
            return
        super().keyReleaseEvent(ev)

    # ================================================================== #
    # 基础信息 / 状态刷新
    # ================================================================== #
    def _load_basics(self) -> None:
        def health(res: Any) -> None:
            if isinstance(res, ApiError):
                self.set_status(f"服务未连接：{res.detail}", error=True)
                return
            self.version.setText("v" + str(res.get("version", "-")))
            self.set_status(f"已连接服务 · backend={res.get('backend')}")
        self.client.get_async("/api/health", health)

    def _refresh_episodes(self) -> None:
        def done(res: Any) -> None:
            if isinstance(res, ApiError):
                return
            self.panel.render_episodes(res.get("episodes") or [])
        self.client.get_async("/api/episodes?details=true", done)

    def refresh_library(self) -> None:
        """磁盘上的历史数据（重启后仍在）。"""
        def done(res: Any) -> None:
            if isinstance(res, ApiError):
                self.panel.hist_summary.setText("读取历史数据失败：" + res.detail)
                return
            self.panel.render_library(res)
            t = res.get("totals") or {}
            self.set_status(f"历史数据：{t.get('datasets', 0)} 个数据集 · "
                            f"{t.get('raw_episodes', 0)} 条原始留档")
        self.client.get_async("/api/library", done)

    def _tick(self) -> None:
        """每秒检查一次状态新鲜度（WS 掉线时顶栏变红）。"""
        fresh = (time.monotonic() - self._last_state_at) < 2.5
        color = theme.OK if fresh and self._ws_pen_ok else (theme.WARN if fresh else theme.BAD)
        self.dot.setStyleSheet(f"color:{color}")

    def pen_channel_ok(self) -> bool:
        return self._ws_pen_ok

    def _on_ws(self, ok: bool) -> None:
        self._ws_pen_ok = bool(ok)
        if not ok:
            self.set_status("笔通道断开，正在重连…", error=True)

    def _on_state(self, payload: dict) -> None:
        self._last_state_at = time.monotonic()
        live = payload.get("live") or {}
        status = payload.get("status") or {}
        self._live, self._status = live, status
        self.teleop.update_state(live, status)
        self.panel.update_state(live, status)
        self._update_chips(status)
        self._sync_streams(live, status)

    def _update_chips(self, status: dict) -> None:
        dev = status.get("device") or {}
        temps = dev.get("temps_c") or []
        profile = status.get("profile") or {}
        vals = [
            str(status.get("backend", "-")),
            str(profile.get("arm_model", "-")),
            f"{dev.get('motor_count', 0)}/{self._n_motors()}",
            f"{dev.get('voltage', 0)} V" if dev.get("voltage") else "—",
            f"{max(temps):.0f} °C" if temps else "—",
        ]
        for chip, val in zip(self.chips, vals):
            chip.set_value(val)

    def _n_motors(self) -> int:
        return 7

    # ================================================================== #
    # 相机流
    # ================================================================== #
    @staticmethod
    def _cam_key(c: dict) -> Any:
        """相机稳定标识：USB 相机用索引，桥接/URL 相机用 URL。"""
        return c.get("index") if c.get("index") is not None else c.get("url")

    def _sync_streams(self, live: dict, status: dict) -> None:
        cam = live.get("camera") or status.get("camera") or {}
        opened = [c for c in (cam.get("cameras") or []) if c.get("opened")]
        sig = tuple(sorted((self._cam_key(c), c.get("opened"), c.get("alias"), c.get("recording"))
                           for c in (cam.get("cameras") or [])))
        if sig != self._cam_sig:
            self._cam_sig = sig
            self.panel._update_camera(cam)

        arm = next((c for c in opened if str(c.get("alias", "")).startswith("wrist")), None)
        if arm is None:
            arm = next((c for c in opened if c.get("index") == 1), None)
        self._arm_key = self._cam_key(arm) if arm else None
        if arm is None:
            self.teleop.tile_arm.clear("未连接")

        g = live.get("gesture") or {}
        pc_index = g.get("camera")
        pc = None
        if pc_index is not None:
            pc = next((c for c in opened if c.get("index") == pc_index), None)
        if pc is None:
            pc = next((c for c in opened if self._cam_key(c) != self._arm_key), None)
        self._pc_key = self._cam_key(pc) if pc else None
        if pc is None:
            self.teleop.tile_pc.clear("未连接")

        # 小窗右下角显示「别名 · fps」
        self._tile_sub = {}
        for c in (arm, pc):
            if c:
                self._tile_sub[self._cam_key(c)] = (
                    f"{c.get('alias') or c.get('name') or 'cam'} · {c.get('fps_actual')}fps"
                )

        selected = cam.get("selected")
        sel = next((c for c in opened if self._cam_key(c) == selected), None)
        self._selected_key = self._cam_key(sel) if sel else None

        # 组装每路需要的流：key → (path, width)
        # 说明：URL 相机无法按索引寻址，只能通过"当前选中"通道取流。
        want: dict[Any, tuple[str, int]] = {}

        def add(c: dict | None, width: int) -> None:
            if not c:
                return
            key = self._cam_key(c)
            if c.get("index") is not None:
                path = f"/api/camera/stream?index={c['index']}"
            elif key:
                path = "/api/camera/stream?url=" + urllib.parse.quote(str(key), safe="")
            else:
                return
            old = want.get(key)
            if old is None or old[1] < width:
                want[key] = (path, width)

        add(arm, 320)
        add(pc, 320)
        add(sel, 640)

        for key, (path, width) in want.items():
            if key not in self._streams or self._streams[key] < width:
                self.client.close_stream(key)
                self.client.open_stream(key, path, width=width)
                self._streams[key] = width
        for key in list(self._streams):
            if key not in want:
                self.client.close_stream(key)
                self._streams.pop(key, None)

    def _on_frame(self, key: Any, image) -> None:
        sub = (self._tile_sub or {}).get(key, "")
        if key is not None and key == self._arm_key:
            self.teleop.tile_arm.set_frame(image, sub)
        if key is not None and key == self._pc_key:
            self.teleop.tile_pc.set_frame(image, sub)
        if key is not None and key == self._selected_key:
            self.panel.cam_preview.set_frame(image)

    # ================================================================== #
    # 动作
    # ================================================================== #
    def set_status(self, text: str, error: bool = False) -> None:
        self.status_line.setText(text)
        self.status_line.setStyleSheet(
            f"color:{theme.BAD if error else theme.TEXT_DIM};"
            f"border-top:1px solid {theme.LINE};font-size:12px;"
        )

    def on_pen_sample(self, sample: dict) -> None:
        self.client.send_pen(sample)
        self.teleop.set_pen_readout(sample)
        if sample.get("touching"):
            if self._auto_fs and not self.isFullScreen():
                self.toggle_fullscreen()
            self._auto_fs = False

    def _ok(self, msg: str):
        def cb(res: Any) -> None:
            if isinstance(res, ApiError):
                self.set_status(msg + " 失败：" + res.detail, error=True)
                return
            self.set_status(msg + " ✓")
        return cb

    def _call(self, path: str, body: dict | None, ok_msg: str, done=None) -> None:
        def cb(res: Any) -> None:
            if isinstance(res, ApiError):
                self.set_status(f"{ok_msg} 失败：{res.detail}", error=True)
                if done:
                    done(res)
                return
            if ok_msg:
                self.set_status(f"{ok_msg} ✓")
            if done:
                done(res)
        self.client.post_async(path, body, cb)

    # ── teleop 控制 ──────────────────────────────────────────────
    def post_teleop(self, cmd: str, payload: dict | None = None) -> None:
        if cmd == "freeze" and payload is None:
            self._call("/api/teleop/freeze", {}, "冻结切换")
        elif cmd == "speed":
            self._call("/api/teleop/speed", {}, "速度切换")
        elif cmd == "float":
            self._call("/api/teleop/float", payload or {}, "悬停切换")
        elif cmd == "joint":
            self._call("/api/teleop/joint", payload or {}, "关节直控")
        elif cmd == "twist":
            self._call("/api/teleop/twist", payload or {}, "自转指令")
        else:
            self._call(f"/api/teleop/{cmd}", payload or {}, "已发送")

    def toggle_mode(self) -> None:
        t = self._live.get("teleop") or {}
        mode = t.get("mode", "pos")
        nxt = "ori" if mode == "pos" else ("pos" if mode == "ori" else self._last_cartesian)
        self._last_cartesian = nxt if nxt in ("pos", "ori") else "pos"
        label = "姿态模式" if nxt == "ori" else "位置模式"
        self.teleop.pad.toast(label)
        self._call("/api/teleop/mode", {"mode": nxt}, label)

    def toggle_float(self) -> None:
        t = self._live.get("teleop") or {}
        self.post_teleop("float", {"on": not t.get("float")})

    def cycle_motor(self) -> None:
        t = self._live.get("teleop") or {}
        cur = int(t.get("joint_index", 3))
        self.select_motor((cur + 1) % 7)

    def select_motor(self, index: int) -> None:
        from .hud import MOTOR_LABELS
        label = "电机 " + (MOTOR_LABELS[index] if 0 <= index < len(MOTOR_LABELS) else str(index))
        self.teleop.pad.toast(label)
        self._call("/api/teleop/motor", {"index": index}, label)

    def toggle_gesture(self) -> None:
        g = self._live.get("gesture")
        if g is None:
            self.set_status("手势模块未启用（服务启动时用 --gesture-camera 指定）", error=True)
            return
        on = not bool(g.get("enabled"))

        def done(res: Any) -> None:
            if isinstance(res, ApiError):
                return
            self.set_status("手势夹爪已" + ("开启" if on else "关闭"))

        self.client.post_async("/api/gesture", {"on": on}, done)

    # ── 会话 / episode ───────────────────────────────────────────
    def toggle_session(self) -> None:
        if self._live.get("session_active"):
            self._call("/api/session/stop", {}, "结束会话")
        else:
            self._call("/api/session/start",
                       {"task_id": "grasp-and-place", "operator": "gui"}, "开始会话")

    def start_episode(self) -> None:
        self._call("/api/episode/start", {}, "开始 Episode")

    def stop_episode(self, success: bool) -> None:
        def done(res: Any) -> None:
            if isinstance(res, ApiError):
                return
            failed = [r.get("name") for r in (res.get("qc") or []) if not r.get("passed")]
            text = (f"#{res.get('index')} 等级 {res.get('grade')}（{res.get('score')} 分）"
                    + (" · 未过：" + ", ".join(failed) if failed else " · 硬门全过"))
            self.panel.pack_result.setText(text)
            self.set_status(text, error=bool(failed))
            self._refresh_episodes()
            self.refresh_library()

        self._call("/api/episode/stop", {"success": success},
                   "结束 Episode（成功）" if success else "结束 Episode（失败）", done=done)

    def do_discard(self) -> None:
        self._call("/api/episode/discard", {}, "已丢弃当前 Episode")

    def do_park(self) -> None:
        if QMessageBox.question(
            self, "回零", "确认回零？机械臂会以限速缓慢回到折叠零位（保持悬停）。",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        ) != QMessageBox.StandardButton.Yes:
            return
        self.set_status("回零中…（请勿触碰机械臂）")
        self._call("/api/device/park", {}, "已回零并保持悬停")

    def do_replay(self, speed: float) -> None:
        self._replay(speed, dataset_path=None, index=None, label=None)

    def do_replay_dataset(self, dataset_path: str | None, index: int | None, speed: float = 1.0) -> None:
        """回放磁盘上某个数据集里的指定 episode（重启后也能用）。"""
        name = Path(dataset_path or "").name
        self._replay(speed, dataset_path=dataset_path, index=index,
                     label=f"{name} #{index}")

    def _replay(self, speed: float, *, dataset_path: str | None, index: int | None,
                label: str | None) -> None:
        what = f"{label} · " if label else ""
        self.panel.replay_status.setText(f"{what}{speed}× 回放中…（Space 可中止）")
        self.set_status(f"回放 {what}{speed}×")
        body: dict[str, Any] = {"speed": speed, "max_joint_speed_deg": min(720.0, 240.0 * speed)}
        if dataset_path:
            body["dataset_path"] = dataset_path
        if index is not None:
            body["index"] = index
        self._call("/api/replay", body, "", done=lambda res: self._replay_done(res, speed))

    def _replay_done(self, res: Any, speed: float) -> None:
        if isinstance(res, ApiError):
            self.panel.replay_status.setText("回放失败：" + res.detail)
            return
        if res.get("ok"):
            self.panel.replay_status.setText(
                f"完成：episode {res.get('episode')} · 原长 {res.get('duration_s')}s · {speed}×")
            self.set_status("回放完成 ✓")
        else:
            self.panel.replay_status.setText(f"已中止（{res.get('reason') or '冻结/力矩保护'}）")
            self.set_status("回放已中止", error=True)

    def do_pack(self) -> None:
        self._pack_with(False)

    def _pack_with(self, include_failed: bool) -> None:
        body: dict[str, Any] = {"task_instruction": "grasp and place"}
        if include_failed:
            body["include_failed"] = True

        def done(res: Any) -> None:
            if isinstance(res, ApiError):
                if res.status == 409 and not include_failed:
                    ans = QMessageBox.question(
                        self, "打包被拒",
                        f"没有合格 episode：\n{res.detail}\n\n"
                        "是否包含未过质检的 episode 一起打包？\n"
                        "（等级会照实写进数据集元数据，便于现场演示）",
                        QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                        QMessageBox.StandardButton.Yes,
                    )
                    if ans == QMessageBox.StandardButton.Yes:
                        self._pack_with(True)
                    else:
                        self.panel.pack_result.setText("打包取消：" + res.detail)
                        self.set_status("打包取消", error=True)
                    return
                self.panel.pack_result.setText("打包失败：" + res.detail)
                self.set_status("打包失败：" + res.detail, error=True)
                return
            text = (f"已打包 {res.get('dataset')} · {res.get('episodes')} 集 / "
                    f"{res.get('frames')} 帧 / {res.get('size_mb')} MB\n{res.get('path')}")
            self.panel.pack_result.setText(text)
            self.set_status("打包完成 ✓  " + str(res.get("path")))
            self._refresh_episodes()
            self.refresh_library()

        self.client.post_async("/api/pack", body, done)

    # ── 相机 ─────────────────────────────────────────────────────
    def do_probe(self) -> None:
        self.panel.cam_preview.set_hint("探测中…")
        self._call("/api/camera/probe", {}, "相机探测")

    def do_open_camera(self, key: Any) -> None:
        body = {"index": key} if isinstance(key, int) else {"url": key}
        label = f"cam{key}" if isinstance(key, int) else "桥接相机"
        self._call("/api/camera/open", body, f"打开 {label}")

    def do_close_camera(self) -> None:
        self._call("/api/camera/close", {}, "关闭相机")

    def do_alias(self, name: str) -> None:
        if not name.strip():
            self.set_status("请先填写名字（wrist / scene）", error=True)
            return
        key = self._selected_key
        if key is None:
            self.set_status("还没有选中的相机", error=True)
            return
        body = {"index": key} if isinstance(key, int) else {"url": key}
        body["name"] = name.strip()
        self._call("/api/camera/alias", body, f"命名 {name.strip()}")

    # ── 视图 ─────────────────────────────────────────────────────
    def toggle_panel(self) -> None:
        show_panel = self.stack.currentIndex() == 0
        if show_panel:
            self.teleop.pad.release()
        self.stack.setCurrentIndex(1 if show_panel else 0)
        self.b_panel.setObjectName("on" if show_panel else "")
        self.b_panel.style().unpolish(self.b_panel)
        self.b_panel.style().polish(self.b_panel)
        self.b_panel.setText("遥操作" if show_panel else "面板")

    # 全屏：先试原生全屏；macOS 后台启动时原生全屏常常"假成功"
    # （isFullScreen()=True 但窗口没铺满）→ 自动回退成无边框铺满 + 置顶。
    def toggle_fullscreen(self) -> None:
        if self._fs_on:
            self._leave_fullscreen()
        else:
            self._enter_fullscreen()

    def _enter_fullscreen(self) -> None:
        self._fs_on = True
        self.b_full.setText("退出全屏")
        self.showFullScreen()
        self.raise_()
        self.activateWindow()
        QTimer.singleShot(500, self._verify_fullscreen)

    def _verify_fullscreen(self) -> None:
        if not self._fs_on:
            return
        scr = (self.screen() or QApplication.primaryScreen()).geometry()
        g = self.geometry()
        covered = g.width() >= scr.width() - 40 and g.height() >= scr.height() - 40
        if self.isFullScreen() and covered:
            return
        # 回退：无边框 + 置顶 + 铺满（不依赖 macOS 全屏 Space）
        self._fake_fs = True
        self.setWindowFlags(Qt.WindowType.FramelessWindowHint | Qt.WindowType.WindowStaysOnTopHint)
        self.setGeometry(scr)
        self.show()
        self.raise_()
        self.activateWindow()
        self.set_status("已用无边框全屏（原生全屏未生效，属 macOS 正常表现）")

    def _leave_fullscreen(self) -> None:
        self._fs_on = False
        self.b_full.setText("全屏")
        if self._fake_fs:
            self._fake_fs = False
            self.setWindowFlags(Qt.WindowType.Window)
            self.show()
        else:
            self.showNormal()
        self.raise_()
        self.activateWindow()
        self.teleop.pad.setFocus()

    def closeEvent(self, ev) -> None:  # noqa: N802
        try:
            self.client.stop()
        except Exception:  # noqa: BLE001
            pass
        super().closeEvent(ev)


# ──────────────────────────────────────────────────────────────────────── #
# 事件探针（排查"幽灵输入"：正常演示不用开）
# ──────────────────────────────────────────────────────────────────────── #
class _EventSpy(QObject):
    def __init__(self, app) -> None:
        super().__init__()
        self.app = app

    def eventFilter(self, obj, ev):  # noqa: N802
        t = ev.type()
        if t in (QEvent.Type.KeyPress, QEvent.Type.KeyRelease):
            focus = self.app.focusWidget()
            print(f"[EV] {t.name} key={ev.key()} text={ev.text()!r} auto={ev.isAutoRepeat()} "
                  f"focus={type(focus).__name__ if focus else None}", flush=True)
        elif t in (QEvent.Type.Shortcut, QEvent.Type.ShortcutOverride):
            # Qt 的快捷键先发 ShortcutOverride，命中后发 Shortcut（不再发 KeyPress）
            print(f"[EV] {t.name} key={getattr(ev, 'key', lambda: '?')()}", flush=True)
        elif t in (QEvent.Type.MouseButtonPress, QEvent.Type.MouseButtonRelease,
                   QEvent.Type.MouseButtonDblClick):
            w = self.app.widgetAt(ev.globalPosition().toPoint())
            print(f"[EV] {t.name} btn={ev.button()} pos={ev.globalPosition().toPoint().x()},"
                  f"{ev.globalPosition().toPoint().y()} under={type(w).__name__ if w else None} "
                  f"text={getattr(w, 'text', lambda: '')() if w else ''}", flush=True)
        return False


# ──────────────────────────────────────────────────────────────────────── #
# 入口
# ──────────────────────────────────────────────────────────────────────── #
def run_gui(url: str = DEFAULT_URL, *, autostart: bool = True, windowed: bool = False,
            backend: str = "rebot", arm_repo: str | None = None,
            gesture_camera: int = 2, service_log: str = "/tmp/rebot_gui_serve.log",
            start_panel: bool = False, debug_events: bool = False) -> int:
    app = QApplication.instance() or QApplication(sys.argv[:1])
    app.setApplicationName("reBot 采集端")
    app.setStyleSheet(theme.QSS)
    spy = None
    if debug_events or os.environ.get("REBOT_GUI_DEBUG_EVENTS"):
        spy = _EventSpy(app)
        app.installEventFilter(spy)
        print("[EV] 事件探针已开启", flush=True)
    pal = app.palette()
    pal.setColor(QPalette.ColorRole.Window, QColor(theme.BG))
    pal.setColor(QPalette.ColorRole.WindowText, QColor(theme.TEXT))
    pal.setColor(QPalette.ColorRole.Base, QColor(theme.CARD_2))
    pal.setColor(QPalette.ColorRole.Text, QColor(theme.TEXT))
    pal.setColor(QPalette.ColorRole.Button, QColor(theme.CARD_2))
    pal.setColor(QPalette.ColorRole.ButtonText, QColor(theme.TEXT))
    app.setPalette(pal)

    ok, detail = ensure_service(url, autostart=autostart, backend=backend,
                                arm_repo=arm_repo, gesture_camera=gesture_camera,
                                log_path=service_log)
    win = MainWindow(url, start_panel=start_panel)
    if not ok:
        win.set_status(detail.splitlines()[0] if detail else "服务未连接", error=True)
        QTimer.singleShot(300, lambda: QMessageBox.warning(
            win, "服务未启动",
            detail + "\n\n界面仍会打开：修好后它会自动重连（右上角圆点变绿）。",
        ))
    win.client.start()
    if debug_events:
        print(f"[EV] 窗口状态 fullscreen={win.isFullScreen()} screen={win.screen().geometry().getRect()}", flush=True)
    win.show()
    win.raise_()
    win.activateWindow()
    if not windowed:
        win._enter_fullscreen()
    return app.exec()


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(run_gui())
