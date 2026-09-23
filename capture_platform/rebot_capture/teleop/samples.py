"""输入样本：数位板 / 手柄 / 手势统一成一种样本。"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass
class PenSample:
    """一帧笔样本（与 tablet_teleop/web 里的字段对齐）。

    坐标 x/y 可以是毫米（板面绝对坐标），也可以是归一化 0-1；
    由 Mapper 决定解释方式，这里只承载数据。
    """

    t: float
    x: float
    y: float
    pressure: float = 0.0
    tilt_x: float = 0.0
    tilt_y: float = 0.0
    twist: float = 0.0
    touching: bool = False
    shift: bool = False  # True = 升降模式（上下→Z）

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> "PenSample":
        return cls(
            t=float(d.get("t", 0.0)),
            x=float(d.get("x", 0.0)),
            y=float(d.get("y", 0.0)),
            pressure=float(d.get("pressure", d.get("p", 0.0))),
            tilt_x=float(d.get("tiltX", d.get("tilt_x", 0.0))),
            tilt_y=float(d.get("tiltY", d.get("tilt_y", 0.0))),
            twist=float(d.get("twist", 0.0)),
            touching=bool(d.get("touching", d.get("touch", False))),
            shift=bool(d.get("shift", False)),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "t": self.t,
            "x": self.x,
            "y": self.y,
            "pressure": self.pressure,
            "tiltX": self.tilt_x,
            "tiltY": self.tilt_y,
            "twist": self.twist,
            "touching": self.touching,
            "shift": self.shift,
        }
