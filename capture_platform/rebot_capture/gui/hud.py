"""遥操作视图：全屏画板 + 四角 HUD（与网页版功能一致，但更干净）。

布局（HUD 悬浮在画板之上，画板铺满整个区域）：
    左上：模式/状态 + 控制按钮        右上：腕部 / 电脑 两路小画面
    左下：笔与手势读数                中下：电机选择 + 回零
                                      右下：录制状态 + 会话 / Episode 按钮
"""
from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtGui import QFont
from PySide6.QtWidgets import QFrame, QGridLayout, QHBoxLayout, QLabel, QVBoxLayout, QWidget

from . import theme
from .pad import PenPad
from .widgets import CameraTile, button, row

MOTOR_LABELS = ["J1 肩部水平", "J2 肩部俯仰", "J3 肘部俯仰",
                "J4 腕部俯仰", "J5 腕部偏航", "J6 腕部自转", "夹爪"]


class TeleopView(QWidget):
    def __init__(self, mw, parent: QWidget | None = None):
        super().__init__(parent)
        self.mw = mw
        self.pad = PenPad(self)
        self.pad.sample.connect(mw.on_pen_sample)
        self.pad.right_click.connect(mw.cycle_motor)
        self.pad.right_long_press.connect(mw.toggle_mode)
        self.pad.touched.connect(self._on_touch)

        # ── 左上：状态 + 控制 ────────────────────────────────────
        self.tl = QFrame(self)
        self.tl.setObjectName("card")
        tl_lay = QVBoxLayout(self.tl)
        tl_lay.setContentsMargins(16, 12, 16, 13)
        tl_lay.setSpacing(7)
        self.mode_label = QLabel("等待连接…")
        self.mode_label.setFont(theme.ui(19, QFont.Weight.DemiBold))
        self.msg_label = QLabel("")
        self.msg_label.setObjectName("muted")
        tl_lay.addWidget(self.mode_label)
        tl_lay.addWidget(self.msg_label)
        self.b_freeze = button("冻结 (Space)", tip="冻结 / 解冻（Space）")
        self.b_speed = button("速度：中 (F)", tip="循环 慢/中/快（F）")
        self.b_mode = button("模式：位置 (O)", tip="位置 / 姿态 / 电机直控（O）")
        self.b_float = button("悬停 (H)", tip="重力漂浮（H）")
        self.b_align = button("对齐 (R)", kind="ghost", tip="重设笔锚点（R）")
        self.b_gesture = button("手势：—", tip="手势夹爪开关")
        self.b_freeze.clicked.connect(lambda: mw.post_teleop("freeze"))
        self.b_speed.clicked.connect(lambda: mw.post_teleop("speed"))
        self.b_mode.clicked.connect(mw.toggle_mode)
        self.b_float.clicked.connect(mw.toggle_float)
        self.b_align.clicked.connect(lambda: mw.post_teleop("align"))
        self.b_gesture.clicked.connect(mw.toggle_gesture)
        tl_lay.addWidget(row(self.b_freeze, self.b_speed, self.b_mode,
                             self.b_float, self.b_align, self.b_gesture))

        # ── 右上：两路小画面 ────────────────────────────────────
        self.tr = QWidget(self)
        tr_lay = QHBoxLayout(self.tr)
        tr_lay.setContentsMargins(0, 0, 0, 0)
        tr_lay.setSpacing(10)
        self.tile_arm = CameraTile("腕部")
        self.tile_pc = CameraTile("电脑")
        tr_lay.addWidget(self.tile_arm)
        tr_lay.addWidget(self.tile_pc)

        # ── 左下：笔 / 手势读数 ─────────────────────────────────
        self.bl = QFrame(self)
        self.bl.setObjectName("card")
        bl_lay = QHBoxLayout(self.bl)
        bl_lay.setContentsMargins(14, 9, 14, 10)
        bl_lay.setSpacing(16)
        self.pen_hz = self._metric(bl_lay, "笔")
        self.pen_pressure = self._metric(bl_lay, "压力")
        self.pen_state = self._metric(bl_lay, "状态")
        self.gesture_state = self._metric(bl_lay, "手势", wide=True)

        # ── 中下：电机 + 回零 ───────────────────────────────────
        self.bc = QFrame(self)
        self.bc.setObjectName("card")
        bc_lay = QHBoxLayout(self.bc)
        bc_lay.setContentsMargins(12, 9, 12, 10)
        bc_lay.setSpacing(6)
        self.joint_buttons: list = []
        for i in range(7):
            b = button("夹爪" if i == 6 else f"J{i + 1}")
            b.clicked.connect(lambda _=False, idx=i: mw.select_motor(idx))
            b.setToolTip("选中该电机（随后用笔上下/左右驱动）")
            self.joint_buttons.append(b)
            bc_lay.addWidget(b)
        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.VLine)
        sep.setStyleSheet(f"color:{theme.LINE}")
        bc_lay.addWidget(sep)
        self.b_park = button("回零", kind="ghost", tip="平滑回到折叠零位（保持悬停）")
        self.b_park.clicked.connect(mw.do_park)
        bc_lay.addWidget(self.b_park)

        # ── 右下：录制 ──────────────────────────────────────────
        self.br = QFrame(self)
        self.br.setObjectName("card")
        br_lay = QVBoxLayout(self.br)
        br_lay.setContentsMargins(14, 11, 14, 12)
        br_lay.setSpacing(8)
        self.rec_badge = QLabel("空闲")
        self.rec_badge.setFont(theme.mono(15, QFont.Weight.DemiBold))
        self.rec_badge.setAlignment(Qt.AlignmentFlag.AlignRight)
        br_lay.addWidget(self.rec_badge)
        self.b_session = button("开始会话")
        self.b_start = button("开始 Episode", kind="primary")
        self.b_stop = button("结束（成功）", kind="danger")
        self.b_stop_fail = button("结束（失败）")
        self.b_session.clicked.connect(mw.toggle_session)
        self.b_start.clicked.connect(mw.start_episode)
        self.b_stop.clicked.connect(lambda: mw.stop_episode(True))
        self.b_stop_fail.clicked.connect(lambda: mw.stop_episode(False))
        br_lay.addWidget(row(self.b_session, self.b_start, stretch=False))
        br_lay.addWidget(row(self.b_stop, self.b_stop_fail, stretch=False))

    # ------------------------------------------------------------------ #
    @staticmethod
    def _metric(lay: QHBoxLayout, label: str, wide: bool = False) -> QLabel:
        box = QWidget()
        v = QVBoxLayout(box)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(1)
        value = QLabel("—")
        value.setFont(theme.mono(15, QFont.Weight.DemiBold))
        cap = QLabel(label)
        cap.setObjectName("faint")
        v.addWidget(value)
        v.addWidget(cap)
        lay.addWidget(box)
        if wide:
            value.setMinimumWidth(190)
        return value

    def _on_touch(self, touching: bool) -> None:
        self.pen_state.setText("落笔" if touching else "悬停")
        self.pen_state.setStyleSheet(f"color:{theme.ACCENT if touching else theme.TEXT}")

    def set_pen_readout(self, sample: dict) -> None:
        self.pen_pressure.setText(f"{float(sample.get('pressure', 0.0)):.2f}")

    # ------------------------------------------------------------------ #
    def resizeEvent(self, _ev) -> None:  # noqa: N802
        m = 22
        self.pad.setGeometry(0, 0, self.width(), self.height())
        self.tl.adjustSize()
        self.tl.move(m, m)
        self.tr.adjustSize()
        self.tr.move(max(m, self.width() - self.tr.width() - m), m)
        self.bl.adjustSize()
        self.bl.move(self.tl.x(), max(m, self.height() - self.bl.height() - m))
        self.bc.adjustSize()
        self.bc.move(max(m, (self.width() - self.bc.width()) // 2),
                     max(m, self.height() - self.bc.height() - m))
        self.br.adjustSize()
        self.br.move(max(m, self.width() - self.br.width() - m),
                     max(m, self.height() - self.br.height() - m))

    # ------------------------------------------------------------------ #
    def update_state(self, live: dict, status: dict) -> None:
        t = (live or {}).get("teleop") or {}
        if t:
            label = "位置"
            if t.get("mode") == "ori":
                label = "姿态"
            elif t.get("mode") == "joint":
                i = int(t.get("joint_index", 0))
                label = f"电机 {MOTOR_LABELS[i] if 0 <= i < 7 else i}"
            extra = []
            if t.get("freeze"):
                extra.append("冻结")
            if t.get("float"):
                extra.append("悬停")
            rate = t.get("rate_hz")
            text = label + (f" · {' · '.join(extra)}" if extra else "")
            if rate:
                text += f"   {rate}Hz"
            self.mode_label.setText(text)
            self.msg_label.setText(str(t.get("alarm") or t.get("msg") or ""))
            self.b_freeze.setObjectName("warn" if t.get("freeze") else "")
            self.b_float.setObjectName("on" if t.get("float") else "")
            self._repolish(self.b_freeze, self.b_float)
            speed_names = ["慢", "中", "快"]
            idx = int(t.get("speed_index", 1))
            self.b_speed.setText(f"速度：{speed_names[idx] if 0 <= idx < 3 else '?'} (F)")
            self.b_align.setObjectName("ghost")
            for i, b in enumerate(self.joint_buttons):
                b.setObjectName("sel" if int(t.get("joint_index", -1)) == i else "")
            self._repolish(*self.joint_buttons)
        else:
            backend = (status or {}).get("backend", "?")
            self.mode_label.setText("遥操核心未启用")
            self.msg_label.setText(f"当前后端 {backend}（无 teleop_core）：关节按钮/相机/录制仍可用")
        for b in (self.b_freeze, self.b_speed, self.b_mode, self.b_float, self.b_align):
            b.setEnabled(bool(t))

        live = live or {}
        ws_ok = self.mw.pen_channel_ok()
        self.pen_hz.setText(f"{live.get('pen_hz', 0)} Hz" if ws_ok else "断开")
        self.pen_hz.setStyleSheet(f"color:{theme.TEXT if ws_ok else theme.BAD}")

        g = live.get("gesture")
        gb = self.b_gesture
        if not g:
            gb.setText("手势：未启用")
            gb.setObjectName("")
            self.gesture_state.setText("—")
        else:
            on = bool(g.get("enabled"))
            gb.setText("手势：开" if on else "手势：关")
            gb.setObjectName("on" if on else "")
            if g.get("error"):
                self.gesture_state.setText(str(g["error"])[:22])
                self.gesture_state.setStyleSheet(f"color:{theme.WARN}")
            elif g.get("gesture") == "open":
                self.gesture_state.setText("张开手 → 张开行程")
                self.gesture_state.setStyleSheet(f"color:{theme.ACCENT}")
            elif g.get("gesture") == "fist":
                self.gesture_state.setText("握拳 → 闭合行程")
                self.gesture_state.setStyleSheet(f"color:{theme.ACCENT}")
            elif g.get("raw"):
                self.gesture_state.setText(f"看到 {g['raw']}")
                self.gesture_state.setStyleSheet(f"color:{theme.TEXT_DIM}")
            else:
                self.gesture_state.setText("未检测到手" if on else "已关闭")
                self.gesture_state.setStyleSheet(f"color:{theme.TEXT_DIM}")
        self._repolish(gb)

        rec = live.get("recording")
        if rec:
            self.rec_badge.setText(f"录制中 #{rec.get('episode_index')} · {rec.get('frames')} 帧")
            self.rec_badge.setStyleSheet(f"color:{theme.REC}")
        elif live.get("session_active"):
            self.rec_badge.setText("待录")
            self.rec_badge.setStyleSheet(f"color:{theme.OK}")
        else:
            self.rec_badge.setText("空闲")
            self.rec_badge.setStyleSheet(f"color:{theme.TEXT_DIM}")

        ses = bool(live.get("session_active"))
        self.b_session.setText("结束会话" if ses else "开始会话")
        self.b_session.setObjectName("ghost" if ses else "")
        self.b_start.setEnabled(not rec)
        self.b_stop.setEnabled(bool(rec))
        self.b_stop_fail.setEnabled(bool(rec))
        self._repolish(self.b_session)

    @staticmethod
    def _repolish(*widgets: QWidget) -> None:
        for w in widgets:
            if w is not None:
                w.style().unpolish(w)
                w.style().polish(w)
