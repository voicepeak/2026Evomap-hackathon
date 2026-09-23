"""把真实 episode 画成轨迹图（站点/作品图片用）—— PIL 直画，深色主题。

数据源：data/episodes/20260923_135010_ep000002.parquet
（= session_3 #2 的原始文件：5510 帧 / 59.4s / 关节行程 6.70 rad / 夹爪 0.32–0.76）
"""
from pathlib import Path

import numpy as np
import pyarrow.parquet as pq
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parent
SRC = ROOT.parent / "capture_platform/data/episodes/20260923_135010_ep000002.parquet"
OUT = ROOT / "assets/traj.png"

BG = (10, 13, 19)
PANEL = (18, 23, 34)
GRID = (30, 38, 54)
TEXT = (230, 235, 244)
DIM = (139, 149, 168)
FAINT = (91, 101, 119)
ACCENT = (58, 192, 255)
WARN = (245, 196, 81)

NAMES = ["J1 肩部水平", "J2 肩部俯仰", "J3 肘部俯仰",
         "J4 腕部俯仰", "J5 腕部偏航", "J6 腕部自转"]
KEYS = ["shoulder_pan", "shoulder_lift", "elbow_flex",
        "wrist_flex", "wrist_yaw", "wrist_roll"]
FONT_CANDIDATES = ["/System/Library/Fonts/Hiragino Sans GB.ttc",
                   "/System/Library/Fonts/STHeiti Light.ttc",
                   "/System/Library/Fonts/Songti.ttc"]


def font(size: int):
    for p in FONT_CANDIDATES:
        if Path(p).exists():
            try:
                f = ImageFont.truetype(p, size, index=0)
                if f.getmask("中").getbbox():      # 确认能渲染中文
                    return f
            except Exception:
                continue
    return ImageFont.load_default()


def main():
    rows = pq.read_table(SRC).to_pylist()
    t = np.array([r["timestamp"] for r in rows])
    total = float(t[-1] - t[0]) or 1.0
    grip = np.array([r.get("action.gripper", r.get("gripper.pos", 0.0)) for r in rows])

    W, H = 1600, 900
    im = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(im)

    L, R, T = 176, 44, 208          # 绘图区边距
    B = H - 150
    pw, ph = W - L - R, B - T
    row_h = ph / 6

    # 标题
    d.text((52, 40), "一条 episode 里的 7 自由度动作", font=font(38), fill=TEXT)
    d.text((52, 96), "5510 帧 · 59.4s · 落盘 ~93fps · 关节总行程 6.70 rad · 夹爪 0.32→0.76",
           font=font(21), fill=DIM)
    d.text((52, 130), "数据文件：data/episodes/20260923_135010_ep000002.parquet",
           font=font(17), fill=FAINT)

    d.rounded_rectangle([L - 12, T - 14, W - R + 12, B + 14], 12, fill=PANEL)
    for i in range(7):
        y = T + row_h * i
        d.line([(L, y), (W - R, y)], fill=GRID, width=1)
    for i in range(7):
        x = L + pw * i / 6
        d.line([(x, T), (x, B)], fill=GRID, width=1)
        if i:
            d.text((x - 24, B + 30), f"{total * i / 6:.0f}s", font=font(17), fill=FAINT)

    # 每个关节按自身范围自动缩放（否则小幅运动会被压成直线）
    for idx, (name, key) in enumerate(zip(NAMES, KEYS)):
        v = np.array([r[f"action.{key}"] for r in rows], dtype=float)
        lo, hi = float(v.min()), float(v.max())
        pad = max(0.06, (hi - lo) * 0.18)
        lo, hi = lo - pad, hi + pad
        y0, y1 = T + row_h * idx + 10, T + row_h * (idx + 1) - 10
        if lo < 0 < hi:                                  # 零线
            zy = y1 - (y1 - y0) * (0 - lo) / (hi - lo)
            d.line([(L, zy), (W - R, zy)], fill=(44, 54, 74), width=1)
        pts = [(L + pw * float(t[i] - t[0]) / total,
                y1 - (y1 - y0) * (float(v[i]) - lo) / (hi - lo)) for i in range(len(v))]
        d.line(pts, fill=ACCENT, width=2, joint="curve")
        d.text((44, T + row_h * idx + 16), name, font=font(20), fill=DIM)
        d.text((44, T + row_h * idx + 42), f"{lo + pad:+.2f} ~ {hi - pad:+.2f}",
               font=font(16), fill=FAINT)

    # 夹爪行程（单独一条，0-1）
    gy0, gy1 = B + 40, B + 92
    d.rounded_rectangle([L - 12, gy0 - 12, W - R + 12, gy1 + 12], 10, fill=PANEL)
    d.line([(L, gy1), (W - R, gy1)], fill=GRID, width=1)
    gp = [(L + pw * float(t[i] - t[0]) / total,
           gy1 - (gy1 - gy0) * float(np.clip(grip[i], 0, 1))) for i in range(len(grip))]
    d.line(gp, fill=WARN, width=2)
    d.text((44, gy0 + 4), "夹爪行程", font=font(19), fill=DIM)
    d.text((W - R - 44, gy0 - 2), "1.0", font=font(15), fill=FAINT)
    d.text((W - R - 44, gy1 - 14), "0.0", font=font(15), fill=FAINT)

    OUT.parent.mkdir(parents=True, exist_ok=True)
    im.save(OUT, "PNG", optimize=True)
    jpg = OUT.with_suffix(".jpg")
    im.save(jpg, "JPEG", quality=86, optimize=True)
    print(f"✅ {OUT}  {im.size}  {OUT.stat().st_size/1024:.0f}KB")
    print(f"✅ {jpg}  {jpg.stat().st_size/1024:.0f}KB")


if __name__ == "__main__":
    main()
