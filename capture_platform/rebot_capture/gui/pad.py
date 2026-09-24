"""全屏笔控画板：网格 + 光标 + 中央提示 + 笔/右键事件。

与网页版行为一致：
- 落笔（左键按下 / 笔尖接触）→ 发送 touching=True 的样本
- 移动 → 持续发送；抬笔 → 发送 touching=False（保持悬停目标）
- 落笔期间 50Hz 重发最新样本（服务端 0.4s 无输入视为抬笔）
- 右键：单击 = 下一个电机（服务端循环，客户端状态过期也不会卡），长按 ≈0.5s = 切换 位置/姿态
- 键盘 G = 直接选中夹爪（不用循环）；[ / ] = 上一个 / 下一个电机
"""
from __future__ import annotations

import time

from PySide6.QtCore import QPointF, QRectF, Qt, QTimer, Signal
from PySide6.QtGui import QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import QWidget

from . import theme

RIGHT_HOLD_MS = 500


class PenPad(QWidget):
    sample = Signal(dict)          # 一个笔样本（已归一化 0-1）
    right_click = Signal()         # 笔右键单击：切换电机
    right_long_press = Signal()    # 笔右键长按：切换 位置/姿态
    touched = Signal(bool)         # 落笔 / 抬笔（给 HUD 显示用）

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.setAttribute(Qt.WidgetAttribute.WA_OpaquePaintEvent, True)

        self._touching = False
        self._cursor = QPointF(-100, -100)
        self._last: dict | None = None
        self._toast = ""
        self._toast_until = 0.0

        self._right_timer = QTimer(self)
        self._right_timer.setSingleShot(True)
        self._right_timer.setInterval(RIGHT_HOLD_MS)
        self._right_timer.timeout.connect(self._on_right_hold)
        self._right_long_fired = False

        self._keepalive = QTimer(self)
        self._keepalive.setInterval(20)
        self._keepalive.timeout.connect(self._resend)
        self._keepalive.start()

    # ------------------------------------------------------------------ #
    def toast(self, text: str, ms: int = 900) -> None:
        self._toast = text
        self._toast_until = time.monotonic() + ms / 1000.0
        self.update()

    def release(self) -> None:
        """强制抬笔（切视图时用）。"""
        if self._touching:
            self._send(self._cursor, False)

    # ------------------------------------------------------------------ #
    def _sample(self, pos: QPointF, touching: bool) -> dict:
        w = max(1, self.width())
        h = max(1, self.height())
        return {
            "t": time.time(),
            "x": max(0.0, min(1.0, pos.x() / w)),
            "y": max(0.0, min(1.0, pos.y() / h)),
            "pressure": 0.5 if touching else 0.0,
            "tiltX": 0.0,
            "tiltY": 0.0,
            "twist": 0.0,
            "touching": bool(touching),
        }

    def _send(self, pos: QPointF, touching: bool) -> None:
        sample = self._sample(pos, touching)
        self._last = sample
        self._touching = touching
        self.sample.emit(sample)
        self.touched.emit(touching)
        self.update()

    def _resend(self) -> None:
        if not self._touching or self._last is None:
            return
        self._last["t"] = time.time()
        self.sample.emit(dict(self._last))

    # ------------------------------------------------------------------ #
    def mousePressEvent(self, ev) -> None:  # noqa: N802
        btn = ev.button()
        if btn in (Qt.MouseButton.RightButton, Qt.MouseButton.MiddleButton):
            self._right_long_fired = False
            self._right_timer.start()
            return
        if btn == Qt.MouseButton.LeftButton:
            self._cursor = ev.position()
            self._send(self._cursor, True)

    def mouseMoveEvent(self, ev) -> None:  # noqa: N802
        self._cursor = ev.position()
        pressed = bool(ev.buttons() & Qt.MouseButton.LeftButton)
        self._send(self._cursor, pressed)

    def mouseReleaseEvent(self, ev) -> None:  # noqa: N802
        btn = ev.button()
        if btn in (Qt.MouseButton.RightButton, Qt.MouseButton.MiddleButton):
            self._right_timer.stop()
            if not self._right_long_fired:
                self.right_click.emit()
            self._right_long_fired = False
            return
        if btn == Qt.MouseButton.LeftButton:
            self._cursor = ev.position()
            self._send(self._cursor, False)

    def _on_right_hold(self) -> None:
        self._right_long_fired = True
        self.right_long_press.emit()

    def leaveEvent(self, _ev) -> None:  # noqa: N802
        self.update()

    # ------------------------------------------------------------------ #
    def paintEvent(self, _event) -> None:  # noqa: N802
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        p.fillRect(0, 0, w, h, QColor(theme.PAD_BG))

        # 网格
        p.setPen(QPen(QColor(theme.GRID), 1))
        step = 64
        for x in range(step, w, step):
            p.drawLine(x, 0, x, h)
        for y in range(step, h, step):
            p.drawLine(0, y, w, y)

        # 中心十字
        cx, cy = w / 2, h / 2
        p.setPen(QPen(QColor("#1b2534"), 1))
        p.drawLine(int(cx), int(cy - 26), int(cx), int(cy + 26))
        p.drawLine(int(cx - 26), int(cy), int(cx + 26), int(cy))
        p.setPen(QPen(QColor("#172030"), 1))
        p.drawEllipse(QPointF(cx, cy), 3.5, 3.5)

        # 底部操作提示（放在电机按钮上方，避免与 HUD 卡片重叠）
        p.setPen(QPen(QColor(theme.TEXT_FAINT)))
        p.setFont(theme.ui(12))
        p.drawText(QRectF(0, h - 120, w, 20), int(Qt.AlignmentFlag.AlignHCenter),
                   "笔尖落笔 = 运动控制　·　抬笔 = 悬停")
        p.setFont(theme.ui(11))
        p.drawText(QRectF(0, h - 100, w, 18), int(Qt.AlignmentFlag.AlignHCenter),
                   "笔右键：单击 = 下一个电机（J1…J6 / 夹爪）　长按 = 切换 位置 / 姿态　按 G = 直接选夹爪")

        # 中央提示（切换时短暂显示）
        if self._toast and time.monotonic() < self._toast_until:
            f = theme.ui(26, QFont.Weight.DemiBold)
            p.setFont(f)
            p.setPen(QPen(QColor(theme.ACCENT)))
            p.drawText(QRectF(0, cy - 140, w, 44), int(Qt.AlignmentFlag.AlignHCenter), self._toast)

        # 笔光标
        if self.underMouse() or self._touching:
            active = self._touching
            r = 9 if active else 6
            p.setPen(QPen(QColor(theme.ACCENT if active else theme.TEXT_FAINT), 1.5))
            p.setBrush(QColor(58, 192, 255, 70 if active else 0))
            p.drawEllipse(self._cursor, r, r)
