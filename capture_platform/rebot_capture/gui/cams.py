"""相机挑选（纯函数，不依赖 Qt —— 方便单测）。

背景：同一台机器上插着好几个相机（机械臂上的腕部相机、桌面/场景相机、电脑自带
摄像头），OpenCV 的索引顺序不稳定，而"腕部"相机的别名曾经被写死成 index 1。
现在改成：**优先用配置的索引（默认 0 = 机械臂上的那路）**，别名只当兜底。

- `pick_arm_camera`：挑"机械臂上的相机"（腕部）
- `pick_other_camera`：挑另一路（优先手势相机索引）
- `pick_preview_camera`：面板大画面显示哪一路（用户点过就以点的那路为准，否则腕部）
"""
from __future__ import annotations

from typing import Any


def cam_key(c: dict) -> Any:
    """相机稳定标识：USB 相机用索引，桥接/URL 相机用 URL。"""
    return c.get("index") if c.get("index") is not None else c.get("url")


def _opened(cameras: list[dict] | None) -> list[dict]:
    return [c for c in (cameras or []) if c.get("opened")]


def pick_arm_camera(cameras: list[dict] | None, preferred_index: int = 0) -> dict | None:
    """机械臂上的相机：配置的索引 → 别名 wrist* → 索引 1（老默认）。"""
    opened = _opened(cameras)
    for c in opened:
        if c.get("index") == preferred_index:
            return c
    for c in opened:
        if str(c.get("alias", "")).startswith("wrist"):
            return c
    for c in opened:
        if c.get("index") == 1:
            return c
    return None


def pick_other_camera(cameras: list[dict] | None, exclude_key: Any,
                      gesture_index: int | None = None) -> dict | None:
    """另一路（电脑摄像头/场景）：优先手势相机索引，否则任意一路不是腕部的。"""
    opened = _opened(cameras)
    if gesture_index is not None:
        for c in opened:
            if c.get("index") == gesture_index and cam_key(c) != exclude_key:
                return c
    for c in opened:
        if cam_key(c) != exclude_key:
            return c
    return None


def pick_preview_camera(cameras: list[dict] | None, preview_key: Any,
                        arm_key: Any) -> dict | None:
    """面板大画面：用户点过的那路（还在开着）优先，否则显示机械臂上的相机。"""
    opened = _opened(cameras)
    for c in opened:
        if preview_key is not None and cam_key(c) == preview_key:
            return c
    for c in opened:
        if arm_key is not None and cam_key(c) == arm_key:
            return c
    return None
