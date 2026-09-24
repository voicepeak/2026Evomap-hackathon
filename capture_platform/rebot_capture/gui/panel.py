"""控制面板视图：相机 / 关节 / 质量 / 录制与数据集 / Episode 列表。

三列布局，卡片式；所有耗时操作都经 mw 异步调用，界面不冻结。
"""
from __future__ import annotations

from PySide6.QtCore import QRectF, Qt
from PySide6.QtGui import QColor, QFont, QImage, QPainter, QPainterPath, QPen
from PySide6.QtWidgets import (QFrame, QGridLayout, QHBoxLayout, QLabel,
                               QLineEdit, QProgressBar, QScrollArea,
                               QSizePolicy, QVBoxLayout, QWidget)

from . import theme
from .widgets import Card, JointBar, Light, button, row

GRADE_COLORS = {"A": theme.OK, "B": theme.ACCENT, "C": theme.WARN, "F": theme.BAD, "?": theme.TEXT_FAINT}


class PreviewView(QWidget):
    """大画面预览（等比适配 + 圆角）。"""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self._image: QImage | None = None
        self._hint = "点「探测相机」→ 选设备 → 预览"
        self.setMinimumHeight(230)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

    def set_frame(self, image: QImage) -> None:
        self._image = image
        self._hint = ""
        self.update()

    def set_hint(self, text: str) -> None:
        self._image = None
        self._hint = text
        self.update()

    def paintEvent(self, _event) -> None:  # noqa: N802
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform)
        r = QRectF(0, 0, self.width(), self.height())
        path = QPainterPath()
        path.addRoundedRect(r, 10, 10)
        p.fillPath(path, QColor("#0d1119"))
        if self._image is None:
            p.setPen(QPen(QColor(theme.TEXT_FAINT)))
            p.setFont(theme.ui(12))
            p.drawText(r, int(Qt.AlignmentFlag.AlignCenter), self._hint)
            return
        p.save()
        p.setClipPath(path)
        img = self._image
        scale = min(r.width() / img.width(), r.height() / img.height())
        w, h = img.width() * scale, img.height() * scale
        p.drawImage(QRectF(r.center().x() - w / 2, r.center().y() - h / 2, w, h), img)
        p.restore()


class PanelView(QWidget):
    def __init__(self, mw, parent: QWidget | None = None):
        super().__init__(parent)
        self.mw = mw
        root = QHBoxLayout(self)
        root.setContentsMargins(18, 14, 18, 16)
        root.setSpacing(14)

        col1, col2, col3 = QVBoxLayout(), QVBoxLayout(), QVBoxLayout()
        for c in (col1, col2, col3):
            c.setSpacing(14)
        root.addLayout(col1, 4)
        root.addLayout(col2, 3)
        root.addLayout(col3, 4)

        # ── 列 1：相机 + 任务卡 ─────────────────────────────────
        cam_status = QLabel("未连接")
        cam_status.setObjectName("muted")
        cam = Card("相机", cam_status)
        self.cam_preview = PreviewView()
        cam.add(self.cam_preview)
        b_probe = button("探测相机")
        b_close = button("关闭相机", "ghost")
        b_probe.clicked.connect(mw.do_probe)
        b_close.clicked.connect(mw.do_close_camera)
        self.cam_buttons = QWidget()
        self.cam_buttons_lay = QHBoxLayout(self.cam_buttons)
        self.cam_buttons_lay.setContentsMargins(0, 0, 0, 0)
        self.cam_buttons_lay.setSpacing(6)
        cam.add(row(b_probe, self.cam_buttons, b_close, stretch=False))
        self.alias_input = QLineEdit()
        self.alias_input.setPlaceholderText("数据集里的名字（wrist / scene …）")
        self.alias_input.setFixedWidth(240)
        b_alias = button("命名当前相机", "ghost")
        b_alias.clicked.connect(lambda: mw.do_alias(self.alias_input.text()))
        cam.add(row(self.alias_input, b_alias))
        self.cam_status = cam_status
        col1.addWidget(cam)

        task = Card("任务卡")
        title = QLabel("抓取 - 放置（桌面）")
        title.setFont(theme.ui(15, QFont.Weight.DemiBold))
        self.task_meta = QLabel("目标 30 条 · 质量 ≥ B")
        self.task_meta.setObjectName("muted")
        self.task_bar = QProgressBar()
        self.task_bar.setRange(0, 100)
        self.task_bar.setValue(0)
        self.task_bar.setTextVisible(False)
        self.task_bar.setFixedHeight(8)
        self.task_bar.setStyleSheet(
            f"QProgressBar{{background:#1a2130;border:none;border-radius:4px;}}"
            f"QProgressBar::chunk{{background:{theme.ACCENT};border-radius:4px;}}"
        )
        self.task_info = QLabel("已录 0 条")
        self.task_info.setObjectName("faint")
        for w in (title, self.task_meta, self.task_bar, self.task_info):
            task.add(w)
        col1.addWidget(task)

        # 历史数据（磁盘）：重启后仍在，可点着回放
        b_refresh = button("刷新", "ghost")
        b_refresh.clicked.connect(mw.refresh_library)
        hist = Card("历史数据（磁盘）", b_refresh)
        self.hist_box = QWidget()
        self.hist_box_lay = QVBoxLayout(self.hist_box)
        self.hist_box_lay.setContentsMargins(0, 0, 0, 0)
        self.hist_box_lay.setSpacing(10)
        hist_scroll = QScrollArea()
        hist_scroll.setWidgetResizable(True)
        hist_scroll.setWidget(self.hist_box)
        hist_scroll.setMinimumHeight(200)
        hist.add(hist_scroll)
        self.hist_summary = QLabel("加载中…")
        self.hist_summary.setObjectName("muted")
        hist.add(self.hist_summary)
        col1.addWidget(hist, 1)

        # ── 列 2：关节 + 质量 + 控制 ────────────────────────────
        self.fps_label = QLabel("0 fps")
        self.fps_label.setObjectName("muted")
        joints = Card("关节状态", self.fps_label)
        self.bars: list[JointBar] = []
        for name in ("J1 肩部水平", "J2 肩部俯仰", "J3 肘部俯仰",
                     "J4 腕部俯仰", "J5 腕部偏航", "J6 腕部自转", "夹爪"):
            bar = JointBar(name)
            self.bars.append(bar)
            joints.add(bar)
        col2.addWidget(joints)

        quality = Card("质量")
        lights = QWidget()
        ll = QHBoxLayout(lights)
        ll.setContentsMargins(0, 0, 0, 0)
        ll.setSpacing(4)
        self.lights = {
            "drop": Light("丢帧"), "limit": Light("限位"),
            "smooth": Light("平滑"), "occlusion": Light("遮挡"),
        }
        for light in self.lights.values():
            ll.addWidget(light)
        ll.addStretch(1)
        quality.add(lights)
        col2.addWidget(quality)

        ctrl = Card("控制")
        self.b_freeze = button("冻结 (Space)")
        self.b_speed = button("速度：中 (F)")
        self.b_mode = button("模式：位置 (O)")
        self.b_float = button("悬停 (H)")
        self.b_align = button("对齐 (R)", "ghost")
        self.b_gesture = button("手势：—")
        self.b_freeze.clicked.connect(lambda: mw.post_teleop("freeze"))
        self.b_speed.clicked.connect(lambda: mw.post_teleop("speed"))
        self.b_mode.clicked.connect(mw.toggle_mode)
        self.b_float.clicked.connect(mw.toggle_float)
        self.b_align.clicked.connect(lambda: mw.post_teleop("align"))
        self.b_gesture.clicked.connect(mw.toggle_gesture)
        ctrl.add(row(self.b_freeze, self.b_speed, self.b_mode))
        ctrl.add(row(self.b_float, self.b_align, self.b_gesture))
        self.ctrl_status = QLabel("—")
        self.ctrl_status.setObjectName("muted")
        ctrl.add(self.ctrl_status)
        col2.addWidget(ctrl)
        col2.addStretch(1)

        # ── 列 3：录制 / 数据集 + Episode 列表 ──────────────────
        rec = Card("录制 / 数据集")
        self.session_info = QLabel("未开始会话 · 已录 0 条")
        self.session_info.setObjectName("muted")
        self.b_discard = button("丢弃当前 Episode", "ghost")
        self.b_discard.clicked.connect(mw.do_discard)
        rec.add(self.session_info)
        rec.add(row(self.b_discard))
        b1 = button("1×")
        b2 = button("2×")
        b4 = button("4×")
        b1.clicked.connect(lambda: mw.do_replay(1.0))
        b2.clicked.connect(lambda: mw.do_replay(2.0))
        b4.clicked.connect(lambda: mw.do_replay(4.0))
        replay_label = QLabel("回放")
        replay_label.setObjectName("muted")
        rec.add(row(replay_label, b1, b2, b4))
        self.replay_status = QLabel("")
        self.replay_status.setObjectName("muted")
        self.replay_status.setWordWrap(True)
        rec.add(self.replay_status)
        col3.addWidget(rec)

        b_pack = button("打包数据集", "primary")
        b_pack.clicked.connect(mw.do_pack)
        eps = Card("Episode 列表", b_pack)
        self.ep_list = QWidget()
        self.ep_list_lay = QVBoxLayout(self.ep_list)
        self.ep_list_lay.setContentsMargins(0, 0, 0, 0)
        self.ep_list_lay.setSpacing(6)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(self.ep_list)
        scroll.setMinimumHeight(180)
        eps.add(scroll)
        self.pack_result = QLabel("")
        self.pack_result.setObjectName("muted")
        self.pack_result.setWordWrap(True)
        eps.add(self.pack_result)
        col3.addWidget(eps, 1)

        self._ep_rows: list[QWidget] = []

    # ------------------------------------------------------------------ #
    def update_state(self, live: dict, status: dict) -> None:
        live, status = live or {}, status or {}
        self.fps_label.setText(f"{live.get('fps_actual', 0)} fps")

        joints = live.get("joints") or []
        for i, bar in enumerate(self.bars):
            if i < len(joints):
                j = joints[i]
                bar.set_values(j.get("pos", 0.0), j.get("target"), j.get("at_limit", False))
            else:
                bar.set_values(live.get("gripper", 0.0) if i == 6 else 0.0)
        if len(self.bars) == 7:
            self.bars[6].set_values(live.get("gripper", 0.0))

        lights = live.get("lights") or {}
        self.lights["drop"].set_state(bool(lights.get("drop")))
        self.lights["limit"].set_state(not bool(lights.get("limit")))
        self.lights["smooth"].set_state(bool(lights.get("smooth")))
        self.lights["occlusion"].set_state(bool(lights.get("occlusion")))

        t = live.get("teleop") or {}
        if t:
            label = "位置"
            if t.get("mode") == "ori":
                label = "姿态"
            elif t.get("mode") == "joint":
                i = int(t.get("joint_index", 0))
                label = f"电机 {i + 1}" if i < 6 else "电机 夹爪"
            extras = []
            if t.get("freeze"):
                extras.append("冻结")
            if t.get("float"):
                extras.append("悬停")
            self.ctrl_status.setText(f"{label} · {t.get('rate_hz')}Hz"
                                     + (f" · {' · '.join(extras)}" if extras else "")
                                     + (f" · {t.get('msg')}" if t.get("msg") else ""))
            speed_names = ["慢", "中", "快"]
            si = int(t.get("speed_index", 1))
            self.b_speed.setText(f"速度：{speed_names[si] if 0 <= si < 3 else '?'} (F)")
            self.b_freeze.setObjectName("warn" if t.get("freeze") else "")
            self.b_float.setObjectName("on" if t.get("float") else "")
            self._repolish(self.b_freeze, self.b_float)

        g = live.get("gesture")
        if g is not None:
            on = bool(g.get("enabled"))
            self.b_gesture.setText("手势：开" if on else "手势：关")
            self.b_gesture.setObjectName("on" if on else "")
            self._repolish(self.b_gesture)

        n = live.get("episodes", 0)
        ses = bool(live.get("session_active"))
        self.session_info.setText(("会话进行中" if ses else "未开始会话") + f" · 已录 {n} 条")
        self.task_bar.setValue(min(100, int(n / 30 * 100)))
        self.task_info.setText(f"已录 {n} 条")
        rec = live.get("recording")
        self.b_discard.setEnabled(bool(rec))

        cam = live.get("camera") or status.get("camera")
        if cam:
            self._update_camera(cam)

    def _update_camera(self, cam: dict) -> None:
        opened = [c for c in (cam.get("cameras") or []) if c.get("opened")]
        selected = cam.get("selected")

        def key_of(c: dict):
            return c.get("index") if c.get("index") is not None else c.get("url")

        # 看的是"面板大画面那一路"（默认 = 机械臂上的相机），不是服务端最后打开的那路
        want_key = getattr(self.mw, "_preview_key", None) or selected
        cur = next((c for c in opened if key_of(c) == want_key), None)
        if cur:
            name = cur.get("alias") or cur.get("name") or "cam"
            self.cam_status.setText(
                f"{name} · {cur.get('fps_actual')} fps"
                + (" · 录制中" if cur.get("recording") else "")
            )
        else:
            self.cam_status.setText("未连接" if cam.get("has_opencv") else "未装 opencv")
            self.cam_preview.set_hint(cam.get("error") or "点「探测相机」→ 选设备 → 预览")
        # 相机按钮
        while self.cam_buttons_lay.count():
            item = self.cam_buttons_lay.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        for c in cam.get("cameras") or []:
            label = c.get("alias") if c.get("alias") and not str(c.get("alias")).startswith("cam") \
                else (f"cam{c.get('index')}" if c.get("index") is not None else "桥接")
            b = button(label + (" ✓" if c.get("opened") else ""), "ghost")
            b.clicked.connect(lambda _=False, k=key_of(c): self.mw.do_open_camera(k))
            self.cam_buttons_lay.addWidget(b)

    # ------------------------------------------------------------------ #
    def render_library(self, lib: dict) -> None:
        """历史数据：已打包数据集（可回放）+ 原始 episode 留档。"""
        while self.hist_box_lay.count():
            item = self.hist_box_lay.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

        datasets = lib.get("datasets") or []
        raw = lib.get("raw_episodes") or []

        for d in datasets:
            block = QWidget()
            lay = QVBoxLayout(block)
            lay.setContentsMargins(0, 0, 0, 0)
            lay.setSpacing(6)

            grades = {}
            for e in d.get("episodes_detail") or []:
                g = e.get("grade") or "?"
                grades[g] = grades.get(g, 0) + 1
            grade_txt = " ".join(f"{g}×{n}" for g, n in sorted(grades.items()))
            name = QLabel(f"{d.get('name')}　<span style='color:{theme.TEXT_FAINT}'>"
                          f"{d.get('episodes')} 集 · {d.get('frames')} 帧 · {d.get('size_mb')}MB · "
                          f"{d.get('videos')} 视频 · {grade_txt}</span>")
            name.setFont(theme.ui(13, QFont.Weight.DemiBold))
            lay.addWidget(name)

            chips = QWidget()
            chips_lay = QHBoxLayout(chips)
            chips_lay.setContentsMargins(0, 0, 0, 0)
            chips_lay.setSpacing(6)
            for e in d.get("episodes_detail") or []:
                idx = e.get("index")
                g = e.get("grade") or "?"
                b = button(f"#{idx} {g} {e.get('score')} ▸", "ghost")
                b.setToolTip(f"回放 {d.get('name')} 第 {idx} 条（{e.get('length')} 帧 / "
                             f"{e.get('duration_s')}s，等级 {g}）")
                b.setStyleSheet(f"color:{GRADE_COLORS.get(g, theme.TEXT)};")
                b.clicked.connect(lambda _=False, p=d.get("path"), i=idx:
                                  self.mw.do_replay_dataset(p, i))
                chips_lay.addWidget(b)
            if (d.get("episodes_detail") or []) == []:
                none = QLabel("（没有 episode 元数据）")
                none.setObjectName("faint")
                chips_lay.addWidget(none)
            chips_lay.addStretch(1)
            lay.addWidget(chips)
            self.hist_box_lay.addWidget(block)

        if raw:
            head = QLabel(f"原始留档　<span style='color:{theme.TEXT_FAINT}'>"
                          f"{len(raw)} 条（未打包的也留着）</span>")
            head.setFont(theme.ui(12, QFont.Weight.DemiBold))
            self.hist_box_lay.addWidget(head)
            for r in raw:
                ok = "✅" if r.get("success") else "✗"
                vid = " · 有视频" if r.get("has_video") else ""
                row_w = QLabel(f"<span style='color:{theme.TEXT_DIM}'>{r.get('file')}</span>"
                               f"　{r.get('frames')} 帧 · {r.get('duration_s')}s · "
                               f"{r.get('fps')}fps · {ok}{vid}")
                row_w.setFont(theme.mono(11))
                self.hist_box_lay.addWidget(row_w)

        if not datasets and not raw:
            empty = QLabel("磁盘上还没有历史数据")
            empty.setObjectName("muted")
            self.hist_box_lay.addWidget(empty)
        self.hist_box_lay.addStretch(1)

        t = lib.get("totals") or {}
        self.hist_summary.setText(
            f"数据集 {t.get('datasets', 0)} 个 · 原始 {t.get('raw_episodes', 0)} 条 / "
            f"{t.get('raw_minutes', 0)} 分钟　（位置：{lib.get('home', '-')}）"
        )

    # ------------------------------------------------------------------ #
    def render_episodes(self, episodes: list[dict]) -> None:
        for w in self._ep_rows:
            w.deleteLater()
        self._ep_rows.clear()
        if not episodes:
            empty = QLabel("暂无数据")
            empty.setObjectName("muted")
            self.ep_list_lay.addWidget(empty)
            self._ep_rows.append(empty)
            return
        for ep in reversed(episodes):
            self._ep_rows.append(self._episode_row(ep))

    def _episode_row(self, ep: dict) -> QWidget:
        w = QFrame()
        w.setObjectName("card")
        w.setStyleSheet(f"#card{{background:{theme.CARD_2};border:1px solid {theme.LINE};border-radius:9px;}}")
        lay = QHBoxLayout(w)
        lay.setContentsMargins(11, 7, 11, 7)
        lay.setSpacing(10)
        idx = QLabel(f"#{ep.get('index')}")
        idx.setFont(theme.mono(13, QFont.Weight.DemiBold))
        idx.setFixedWidth(34)
        info = QLabel(f"{ep.get('frames')} 帧 · {ep.get('duration_s')}s"
                      + ("" if ep.get("success") else " · 失败"))
        info.setObjectName("muted")
        grade = ep.get("grade", "?")
        badge = QLabel(grade)
        badge.setFont(theme.mono(14, QFont.Weight.Bold))
        badge.setStyleSheet(f"color:{GRADE_COLORS.get(grade, theme.TEXT_FAINT)}")
        badge.setFixedWidth(18)
        score = QLabel(f"{ep.get('score')}")
        score.setFont(theme.mono(13))
        score.setStyleSheet(f"color:{theme.TEXT_DIM}")
        lay.addWidget(idx)
        lay.addWidget(info)
        lay.addStretch(1)
        lay.addWidget(badge)
        lay.addWidget(score)
        self.ep_list_lay.addWidget(w)
        return w

    @staticmethod
    def _repolish(*widgets: QWidget) -> None:
        for w in widgets:
            if w is not None:
                w.style().unpolish(w)
                w.style().polish(w)
