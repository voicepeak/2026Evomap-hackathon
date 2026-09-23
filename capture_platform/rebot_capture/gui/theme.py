"""GUI 视觉主题：颜色 / 字体 / 样式表。

设计原则（"干净"）：
- 深色底 + 少量强调色，信息用文字与细线分层，不堆阴影/描边
- 数字统一等宽字体，避免读数跳动
- 卡片圆角 12px、1px 分隔线、留白比网页更大
"""
from __future__ import annotations

from PySide6.QtGui import QFont

# ── 颜色 ────────────────────────────────────────────────────────────────
BG = "#0a0d13"          # 窗口底
CARD = "#121722"        # 卡片
CARD_2 = "#171d2a"      # 卡片内嵌（按钮底）
LINE = "#232b3a"        # 分隔线
TEXT = "#e6ebf4"        # 主文字
TEXT_DIM = "#8b95a8"    # 次要文字
TEXT_FAINT = "#5b6577"  # 更弱
ACCENT = "#3ac0ff"      # 主色（青蓝）
ON = "#103449"          # 选中底
ON_LINE = "#1c5a7a"     # 选中边
OK = "#39d98a"          # 正常
WARN = "#f5c451"        # 注意
BAD = "#ff6b6b"         # 异常
REC = "#ff5d5d"         # 录制中
PAD_BG = "#080b10"      # 画板底
GRID = "#121a26"        # 画板网格

# ── 字体 ────────────────────────────────────────────────────────────────
UI_FAMILY = "PingFang SC"
MONO_FAMILY = "SF Mono"


def mono(size: int, weight: int = QFont.Weight.Normal) -> QFont:
    f = QFont()
    f.setFamilies(["SF Mono", "Menlo", "Monaco", "Courier New"])
    f.setStyleHint(QFont.StyleHint.Monospace)
    f.setPointSize(size)
    f.setWeight(weight)
    return f


def ui(size: int, weight: int = QFont.Weight.Normal) -> QFont:
    f = QFont()
    f.setFamilies([UI_FAMILY, "Helvetica Neue"])
    f.setPointSize(size)
    f.setWeight(weight)
    return f


QSS = f"""
QWidget {{
    background: {BG};
    color: {TEXT};
    font-family: "{UI_FAMILY}";
    font-size: 13px;
}}

/* ── 卡片 ─────────────────────────────────────────────── */
#card {{
    background: {CARD};
    border: 1px solid {LINE};
    border-radius: 12px;
}}
#cardTitle {{ color: {TEXT_DIM}; font-size: 12px; letter-spacing: 1px; }}
#muted {{ color: {TEXT_DIM}; font-size: 12px; }}
#faint {{ color: {TEXT_FAINT}; font-size: 11px; }}

/* ── 按钮 ─────────────────────────────────────────────── */
QPushButton {{
    background: {CARD_2};
    color: {TEXT};
    border: 1px solid {LINE};
    border-radius: 8px;
    padding: 7px 13px;
    font-size: 13px;
}}
QPushButton:hover {{ background: #1d2534; border-color: #2c3546; }}
QPushButton:pressed {{ background: #141a26; }}
QPushButton:disabled {{ color: {TEXT_FAINT}; background: #0f1420; border-color: #1a2130; }}

QPushButton#primary {{ background: {ACCENT}; color: #04121c; border: none; font-weight: 600; }}
QPushButton#primary:hover {{ background: #55cbff; }}
QPushButton#primary:disabled {{ background: #1b3040; color: {TEXT_FAINT}; }}

QPushButton#danger {{ background: #2a1620; color: {BAD}; border-color: #472334; }}
QPushButton#danger:hover {{ background: #381c29; }}
QPushButton#danger:disabled {{ background: #16101a; color: #6b4b57; border-color: #241a24; }}

QPushButton#ghost {{ background: transparent; border-color: {LINE}; color: {TEXT_DIM}; }}
QPushButton#ghost:hover {{ color: {TEXT}; border-color: #33405a; }}

QPushButton#on {{ background: {ON}; color: {ACCENT}; border-color: {ON_LINE}; }}
QPushButton#sel {{ background: {ON}; color: {ACCENT}; border-color: {ACCENT}; font-weight: 600; }}
QPushButton#warn {{ background: #33290f; color: {WARN}; border-color: #57431a; }}
QPushButton#rec {{ background: #3a1620; color: #ffd9d9; border-color: #6b2534; font-weight: 600; }}

/* ── 输入 ─────────────────────────────────────────────── */
QLineEdit {{
    background: {CARD_2}; border: 1px solid {LINE}; border-radius: 8px;
    padding: 6px 10px; color: {TEXT};
}}
QLineEdit:focus {{ border-color: {ACCENT}; }}

/* ── 滚动条 ───────────────────────────────────────────── */
QScrollArea {{ border: none; background: transparent; }}
QScrollBar:vertical {{ background: transparent; width: 8px; margin: 2px; }}
QScrollBar::handle:vertical {{ background: #28303f; border-radius: 4px; min-height: 24px; }}
QScrollBar::add-line, QScrollBar::sub-line {{ height: 0; }}
QScrollBar:horizontal {{ background: transparent; height: 8px; margin: 2px; }}
QScrollBar::handle:horizontal {{ background: #28303f; border-radius: 4px; min-width: 24px; }}
"""
