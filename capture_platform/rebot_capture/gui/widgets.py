"""GUI 基础控件：卡片 / 关节条 / 状态灯 / 相机小窗 / 状态胶囊。

全部自绘（QPainter），保证深色主题下线条与数字都干净。
"""
from __future__ import annotations

from PySide6.QtCore import QRectF, Qt, Signal
from PySide6.QtGui import QColor, QFont, QImage, QPainter, QPainterPath, QPen
from PySide6.QtWidgets import (QFrame, QHBoxLayout, QLabel, QPushButton,
                               QSizePolicy, QVBoxLayout, QWidget)

from . import theme


class Card(QFrame):
    """带标题的卡片容器。"""

    def __init__(self, title: str, right: QWidget | None = None, parent: QWidget | None = None):
        super().__init__(parent)
        self.setObjectName("card")
        outer = QVBoxLayout(self)
        outer.setContentsMargins(16, 13, 16, 15)
        outer.setSpacing(11)

        head = QHBoxLayout()
        head.setSpacing(8)
        self.title = QLabel(title)
        self.title.setObjectName("cardTitle")
        self.title.setFont(theme.ui(12, QFont.Weight.DemiBold))
        head.addWidget(self.title)
        head.addStretch(1)
        if right is not None:
            head.addWidget(right)
        self.head = head
        outer.addLayout(head)

        self.body = QVBoxLayout()
        self.body.setSpacing(9)
        outer.addLayout(self.body)
        outer.addStretch(0)

    def add(self, w: QWidget) -> QWidget:
        self.body.addWidget(w)
        return w

    def add_layout(self, lay) -> None:
        self.body.addLayout(lay)


class Chip(QLabel):
    """`标签 值` 胶囊。"""

    def __init__(self, label: str, value: str = "-", parent: QWidget | None = None):
        super().__init__(parent)
        self._label = label
        self.setObjectName("faint")
        self.set_value(value)

    def set_value(self, value: str, color: str | None = None) -> None:
        color = color or theme.TEXT
        self.setText(
            f'<span style="color:{theme.TEXT_FAINT}">{self._label}</span>'
            f'&nbsp;&nbsp;<span style="color:{color}">{value}</span>'
        )


class Light(QWidget):
    """圆点 + 文字的状态灯。"""

    def __init__(self, text: str, parent: QWidget | None = None):
        super().__init__(parent)
        self._text = text
        self._color = QColor(theme.TEXT_FAINT)
        self.setMinimumWidth(74)
        self.setFixedHeight(22)

    def set_state(self, ok: bool | None, bad_text: str | None = None) -> None:
        if ok is None:
            self._color = QColor(theme.TEXT_FAINT)
        else:
            self._color = QColor(theme.OK if ok else theme.BAD)
        if bad_text:
            self._text = bad_text
        self.update()

    def paintEvent(self, _event) -> None:  # noqa: N802
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        d = 8
        p.setBrush(self._color)
        p.setPen(Qt.PenStyle.NoPen)
        p.drawEllipse(QRectF(1, (self.height() - d) / 2, d, d))
        p.setPen(QPen(QColor(theme.TEXT_DIM)))
        p.setFont(theme.ui(12))
        p.drawText(QRectF(16, 0, self.width() - 16, self.height()),
                   int(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter), self._text)


class JointBar(QWidget):
    """一行关节：名称 + 轨道 + 数值（等宽）。"""

    def __init__(self, name: str, parent: QWidget | None = None):
        super().__init__(parent)
        self._name = name
        self._pos = 0.0
        self._target = None
        self._at_limit = False
        self.setFixedHeight(26)
        self.setMinimumWidth(260)

    def set_values(self, pos: float, target: float | None = None, at_limit: bool = False) -> None:
        self._pos, self._target, self._at_limit = float(pos), target, bool(at_limit)
        self.update()

    def paintEvent(self, _event) -> None:  # noqa: N802
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        h, w = self.height(), self.width()
        name_w, val_w = 92, 58
        track_x, track_w = name_w + 4, w - name_w - val_w - 12
        cy, th = h / 2, 6

        p.setPen(QPen(QColor(theme.TEXT_DIM)))
        p.setFont(theme.ui(12))
        p.drawText(QRectF(0, 0, name_w, h), int(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter),
                   self._name)

        track = QRectF(track_x, cy - th / 2, max(4, track_w), th)
        path = QPainterPath()
        path.addRoundedRect(track, th / 2, th / 2)
        p.fillPath(path, QColor("#1a2130"))

        frac = max(0.0, min(1.0, (self._pos + 3.14) / 6.28))
        fill_w = max(0.0, track_w * frac)
        if fill_w > 0:
            fpath = QPainterPath()
            fpath.addRoundedRect(QRectF(track_x, cy - th / 2, fill_w, th), th / 2, th / 2)
            p.fillPath(fpath, QColor(theme.BAD if self._at_limit else theme.ACCENT))

        if self._target is not None:
            tf = max(0.0, min(1.0, (self._target + 3.14) / 6.28))
            tx = track_x + track_w * tf
            p.setPen(QPen(QColor(theme.TEXT_FAINT), 2))
            p.drawLine(int(tx), int(cy - 8), int(tx), int(cy + 8))

        p.setPen(QPen(QColor(theme.BAD if self._at_limit else theme.TEXT)))
        p.setFont(theme.mono(12))
        p.drawText(QRectF(w - val_w, 0, val_w, h),
                   int(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter), f"{self._pos:+.3f}")


class CameraTile(QWidget):
    """圆角相机小窗（画面上覆盖名称/fps）。"""

    def __init__(self, title: str, parent: QWidget | None = None):
        super().__init__(parent)
        self._title = title
        self._sub = "未连接"
        self._image: QImage | None = None
        self.setFixedSize(232, 138)

    def set_frame(self, image: QImage, sub: str) -> None:
        self._image, self._sub = image, sub
        self.update()

    def clear(self, sub: str = "未连接") -> None:
        self._image, self._sub = None, sub
        self.update()

    def paintEvent(self, _event) -> None:  # noqa: N802
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform)
        r = QRectF(0, 0, self.width(), self.height())
        path = QPainterPath()
        path.addRoundedRect(r, 10, 10)
        p.fillPath(path, QColor("#0d1119"))
        if self._image is not None:
            p.save()
            p.setClipPath(path)
            img = self._image
            scale = max(r.width() / img.width(), r.height() / img.height())
            w, h = img.width() * scale, img.height() * scale
            p.drawImage(QRectF(r.center().x() - w / 2, r.center().y() - h / 2, w, h), img)
            p.restore()
        else:
            p.setPen(QPen(QColor(theme.TEXT_FAINT)))
            p.setFont(theme.ui(11))
            p.drawText(r, int(Qt.AlignmentFlag.AlignCenter), self._sub)

        # 底部渐隐条 + 文字
        p.setPen(Qt.PenStyle.NoPen)
        p.fillRect(QRectF(r.left(), r.bottom() - 26, r.width(), 26), QColor(0, 0, 0, 150))
        p.setFont(theme.mono(10))
        p.setPen(QPen(QColor(theme.TEXT)))
        p.drawText(QRectF(r.left() + 8, r.bottom() - 24, r.width() - 16, 20),
                   int(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter), self._title)
        p.setPen(QPen(QColor(theme.TEXT_DIM)))
        p.drawText(QRectF(r.left() + 8, r.bottom() - 24, r.width() - 16, 20),
                   int(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter), self._sub)


class Metric(QWidget):
    """大号数字指标（等宽数值 + 小标题）。"""

    def __init__(self, label: str, value: str = "—", color: str = theme.TEXT, parent: QWidget | None = None):
        super().__init__(parent)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(2)
        self._value = QLabel(value)
        self._value.setFont(theme.mono(20, QFont.Weight.DemiBold))
        self._value.setStyleSheet(f"color:{color}")
        self._label = QLabel(label)
        self._label.setObjectName("faint")
        lay.addWidget(self._value)
        lay.addWidget(self._label)

    def set_value(self, value: str, color: str | None = None) -> None:
        self._value.setText(value)
        if color:
            self._value.setStyleSheet(f"color:{color}")


def button(text: str, kind: str | None = None, tip: str = "") -> QPushButton:
    """统一风格按钮。kind ∈ {None, 'primary', 'danger', 'ghost', 'on', 'warn', 'rec'}"""
    b = QPushButton(text)
    if kind:
        b.setObjectName(kind)
    if tip:
        b.setToolTip(tip)
    b.setCursor(Qt.CursorShape.PointingHandCursor)
    b.setSizePolicy(QSizePolicy.Policy.Maximum, QSizePolicy.Policy.Fixed)
    return b


def row(*widgets: QWidget, spacing: int = 8, stretch: bool = True) -> QWidget:
    w = QWidget()
    lay = QHBoxLayout(w)
    lay.setContentsMargins(0, 0, 0, 0)
    lay.setSpacing(spacing)
    for item in widgets:
        lay.addWidget(item)
    if stretch:
        lay.addStretch(1)
    return w
