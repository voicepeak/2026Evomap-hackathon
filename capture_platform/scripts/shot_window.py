"""按窗口抓图：拿到窗口本体的干净截图（不受其它窗口遮挡、不受所在 Space 影响）。

用途：作品图片 / 站点素材 —— 不会拍到桌面、菜单栏、程序坞、聊天窗口。

用法：
    # 列出当前所有可见窗口（标题 / 所属进程 / 尺寸 / 窗口ID）
    .venv/bin/python scripts/shot_window.py --list

    # 按标题关键字抓某个窗口
    .venv/bin/python scripts/shot_window.py --title "reBot" -o /tmp/gui.png

    # 可选裁剪（比例，0-1）：--crop 左,上,右,下
    .venv/bin/python scripts/shot_window.py --title "reBot" --crop 0,0.05,1,0.97 -o /tmp/gui.png

说明：用 CGWindowListCreateImage 抓窗口缓冲区，因此**即使窗口在别的全屏 Space 也能拍到**；
需要终端（或运行本脚本的程序）具备「屏幕录制」权限。
"""
from __future__ import annotations

import argparse
import subprocess
import sys
import tempfile
from pathlib import Path

import Quartz
from PIL import Image


def list_windows(owner_filter: str | None = None) -> list[dict]:
    wins = Quartz.CGWindowListCopyWindowInfo(
        Quartz.kCGWindowListOptionOnScreenOnly | Quartz.kCGWindowListExcludeDesktopElements,
        Quartz.kCGNullWindowID,
    )
    out = []
    for w in wins or []:
        owner = w.get("kCGWindowOwnerName") or ""
        title = w.get("kCGWindowName") or ""
        bounds = w.get("kCGWindowBounds") or {}
        if owner_filter and owner_filter.lower() not in (owner + title).lower():
            continue
        out.append({
            "id": int(w.get("kCGWindowNumber", 0)),
            "owner": owner,
            "title": title,
            "w": int(bounds.get("Width", 0)),
            "h": int(bounds.get("Height", 0)),
            "layer": int(w.get("kCGWindowLayer", 0)),
        })
    return out


def shoot(window_id: int, out: Path, crop: tuple[float, float, float, float] | None = None) -> Path:
    img = Quartz.CGWindowListCreateImage(
        Quartz.CGRectNull,
        Quartz.kCGWindowListOptionIncludingWindow,
        window_id,
        Quartz.kCGWindowImageBoundsIgnoreFraming,
    )
    if img is None:
        raise RuntimeError(f"抓不到窗口 {window_id}（可能没有屏幕录制权限，或窗口已关闭）")
    tmp = Path(tempfile.mkstemp(suffix=".png")[1])
    url = Quartz.CFURLCreateFromFileSystemRepresentation(None, str(tmp).encode(), len(str(tmp).encode()), False)
    dest = Quartz.CGImageDestinationCreateWithURL(url, "public.png", 1, None)
    Quartz.CGImageDestinationAddImage(dest, img, None)
    Quartz.CGImageDestinationFinalize(dest)

    im = Image.open(tmp)
    if crop:
        l, t, r, b = crop
        im = im.crop((int(im.width * l), int(im.height * t), int(im.width * r), int(im.height * b)))
    out.parent.mkdir(parents=True, exist_ok=True)
    im.save(out)
    tmp.unlink(missing_ok=True)
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description="按窗口抓干净截图")
    ap.add_argument("--list", action="store_true", help="列出窗口")
    ap.add_argument("--filter", default=None, help="列表过滤关键字")
    ap.add_argument("--title", default=None, help="标题/进程关键字（抓图用）")
    ap.add_argument("--id", type=int, default=None, help="直接指定窗口 ID")
    ap.add_argument("-o", "--out", default="/tmp/window.png", help="输出 PNG")
    ap.add_argument("--crop", default=None, help="裁剪比例：左,上,右,下（0-1）")
    args = ap.parse_args()

    if args.list or (args.title is None and args.id is None):
        wins = list_windows(args.filter)
        print(f"{'窗口ID':>8}  {'尺寸':>11}  {'层':>3}  进程 / 标题")
        for w in wins:
            print(f"{w['id']:>8}  {w['w']:>5}x{w['h']:<5}  {w['layer']:>3}  {w['owner']} / {w['title']}")
        return 0

    wid = args.id
    if wid is None:
        cands = [w for w in list_windows(args.filter) or list_windows()]
        matched = [w for w in cands if args.title.lower() in (w["owner"] + " " + w["title"]).lower()
                   and w["layer"] == 0 and w["w"] > 200]
        if not matched:
            print(f"没有匹配 '{args.title}' 的窗口；用 --list 看看", file=sys.stderr)
            return 1
        wid = matched[0]["id"]
        print(f"匹配到窗口 {wid}: {matched[0]['owner']} / {matched[0]['title']} "
              f"{matched[0]['w']}x{matched[0]['h']}")

    crop = None
    if args.crop:
        crop = tuple(float(x) for x in args.crop.split(","))
        assert len(crop) == 4, "--crop 需要 4 个比例值"
    path = shoot(wid, Path(args.out), crop)
    print(f"✅ 已保存 {path}  ({Image.open(path).size[0]}x{Image.open(path).size[1]})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
