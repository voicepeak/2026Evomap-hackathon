"""设备档案：B601-RS 的关节/电机/限位/增益等出厂参数。

档案是采集端的"单一事实来源"：配置生成、轨道裁剪、质量门、数据集元数据都从这里取。
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from ..paths import DEFAULT_PROFILE


@dataclass
class JointProfile:
    name: str
    motor_id: int
    model: str
    direction: int
    limits_rad: tuple[float, float]
    kp: float
    kd: float


@dataclass
class GripperProfile:
    name: str = "gripper"
    motor_id: int = 7
    model: str = "rs-00"
    force_ratio: float = 0.07
    travel_m: float = 0.0715
    lo_deg: float = 3.0      # 程序可用下界（度）；机械下端 −11.9°，留余量
    hi_deg: float = 328.0    # 程序可用上界（度）；机械上端 +335.4°，留余量
    tau_limit: float = 2.0   # 电机力矩上限（N·m）：到它就判定"到限位/有阻力"并停手
    rate: float = 1.2        # 行程速度（rad/s，约 69°/s）
    direction: int = 1       # +1 = 软件正方向与电机一致（角度增大 = 张开）；
                             # −1 = 反过来（角度增大 = 夹紧，现场实测到的那台）


@dataclass
class DeviceProfile:
    name: str
    arm_model: str
    dof: int
    control_mode: str
    fps: int
    can: dict[str, Any]
    joints: list[JointProfile]
    gripper: GripperProfile
    notes: list[str] = field(default_factory=list)

    # ------------------------------------------------------------------ #
    @classmethod
    def load(cls, path: str | Path | None = None) -> "DeviceProfile":
        p = Path(path) if path else DEFAULT_PROFILE
        raw = json.loads(Path(p).read_text(encoding="utf-8"))
        return cls.from_dict(raw)

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> "DeviceProfile":
        joints = [
            JointProfile(
                name=j["name"],
                motor_id=int(j["motor_id"]),
                model=j["model"],
                direction=int(j.get("direction", 1)),
                limits_rad=(float(j["limits_rad"][0]), float(j["limits_rad"][1])),
                kp=float(j.get("kp", 0.0)),
                kd=float(j.get("kd", 0.0)),
            )
            for j in raw.get("joints", [])
        ]
        g = raw.get("gripper", {})
        return cls(
            name=raw.get("name", "unknown"),
            arm_model=raw.get("arm_model", "unknown"),
            dof=int(raw.get("dof", len(joints))),
            control_mode=raw.get("control_mode", "mit"),
            fps=int(raw.get("fps", 100)),
            can=raw.get("can", {}),
            joints=joints,
            gripper=GripperProfile(
                name=g.get("name", "gripper"),
                motor_id=int(g.get("motor_id", 7)),
                model=g.get("model", "rs-00"),
                force_ratio=float(g.get("force_ratio", 0.07)),
                travel_m=float(g.get("travel_m", 0.0715)),
                lo_deg=float(g.get("lo_deg", 3.0)),
                hi_deg=float(g.get("hi_deg", 328.0)),
                tau_limit=float(g.get("tau_limit", 2.0)),
                rate=float(g.get("rate", 1.2)),
                direction=int(g.get("direction", 1)),
            ),
            notes=list(raw.get("notes", [])),
        )

    # ------------------------------------------------------------------ #
    @property
    def n_joints(self) -> int:
        return len(self.joints)

    @property
    def n_motors(self) -> int:
        """6 个手臂关节 + 1 个夹爪 = 7 个电机。"""
        return self.n_joints + 1

    def limits_array(self) -> list[tuple[float, float]]:
        return [j.limits_rad for j in self.joints]

    def kp_array(self) -> list[float]:
        return [j.kp for j in self.joints]

    def kd_array(self) -> list[float]:
        return [j.kd for j in self.joints]

    # ------------------------------------------------------------------ #
    def to_auto_config(self) -> dict[str, Any]:
        """自动配置生成器 v0 —— 用户不需要手写电机 ID / 方向 / 限位 / 增益。

        输出可直接喂给 LeRobot 风格的机型配置（rebot_b601_follower）。
        """
        return {
            "robot_type": "rebot_b601_rs_follower",
            "motor_can_ids": {j.name: j.motor_id for j in self.joints},
            "motor_models": {j.name: j.model for j in self.joints},
            "joint_directions": {j.name: j.direction for j in self.joints},
            "joint_limits_rad": {j.name: list(j.limits_rad) for j in self.joints},
            "mit_kp": self.kp_array(),
            "mit_kd": self.kd_array(),
            "gripper": {
                "can_id": self.gripper.motor_id,
                "model": self.gripper.model,
                "force_ratio": self.gripper.force_ratio,
                "lo_deg": self.gripper.lo_deg,
                "hi_deg": self.gripper.hi_deg,
                "tau_limit": self.gripper.tau_limit,
                "rate": self.gripper.rate,
                "direction": self.gripper.direction,
            },
            "can": dict(self.can),
            "control_mode": self.control_mode,
        }

    def summary(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "arm_model": self.arm_model,
            "dof": self.dof,
            "motors": self.n_motors,
            "control_mode": self.control_mode,
            "fps": self.fps,
            "can": self.can,
            "joints": [
                {
                    "name": j.name,
                    "motor_id": j.motor_id,
                    "model": j.model,
                    "limits_rad": list(j.limits_rad),
                    "kp": j.kp,
                    "kd": j.kd,
                }
                for j in self.joints
            ],
            "gripper": {
                "motor_id": self.gripper.motor_id,
                "model": self.gripper.model,
                "force_ratio": self.gripper.force_ratio,
                "lo_deg": self.gripper.lo_deg,
                "hi_deg": self.gripper.hi_deg,
                "tau_limit": self.gripper.tau_limit,
                "rate": self.gripper.rate,
                "direction": self.gripper.direction,
            },
            "notes": self.notes,
        }
