"""reBotArm_control_py 仓库加载器。

采集端不复制上游代码，只做"按路径导入"：
- 解析仓库路径（`--arm-repo` / 环境变量 `REBOT_ARM_REPO` / 默认位置）
- 把仓库加入 sys.path 并导入：RebotArm、运动学、动力学
- 汇总常用符号，供 RealArm / SpatialVelocityMapper 使用

上游仓库自带 config/ 解析（基于模块自身路径），因此在任意 CWD 下都可工作。
"""
from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import Any

DEFAULT_CANDIDATES = [
    Path.home() / "Desktop" / "reBotArm_control_py",
    Path.home() / "Documents" / "Default Project" / "reBotArm_control_py",
]


class RebotArmRepo:
    def __init__(self, path: str | Path | None = None):
        self.path = self.resolve(path)
        self._loaded = False
        self._mods: dict[str, Any] = {}

    # ------------------------------------------------------------------ #
    @staticmethod
    def resolve(path: str | Path | None = None) -> Path:
        candidate = path or os.environ.get("REBOT_ARM_REPO")
        if candidate:
            p = Path(candidate).expanduser()
            if not (p / "reBotArm_control_py").is_dir():
                raise FileNotFoundError(f"{p} 下没有 reBotArm_control_py 包")
            return p
        for c in DEFAULT_CANDIDATES:
            if (c / "reBotArm_control_py").is_dir():
                return c
        raise FileNotFoundError(
            "找不到 reBotArm_control_py 仓库。请用 --arm-repo 指定，或设置环境变量 REBOT_ARM_REPO。"
        )

    def __repr__(self) -> str:
        return f"RebotArmRepo({self.path})"

    # ------------------------------------------------------------------ #
    def load(self) -> "RebotArmRepo":
        if self._loaded:
            return self
        sp = str(self.path)
        if sp not in sys.path:
            sys.path.insert(0, sp)
        try:
            from reBotArm_control_py.actuator import RebotArm  # noqa: PLC0415
            from reBotArm_control_py.dynamics import (  # noqa: PLC0415
                compute_generalized_gravity,
                load_dynamics_model,
            )
            from reBotArm_control_py.kinematics import (  # noqa: PLC0415
                get_end_effector_frame_id,
                joint_to_pose,
                load_robot_model,
                pad_q_for_model,
            )
            import pinocchio as pin  # noqa: PLC0415
        except Exception as e:  # noqa: BLE001
            raise RuntimeError(
                f"导入 reBotArm_control_py 失败：{type(e).__name__}: {e}\n"
                "请先安装上游依赖（pin / motorbridge / pyyaml），"
                "例如：pip install pin motorbridge pyyaml"
            ) from e

        self._mods = {
            "RebotArm": RebotArm,
            "compute_generalized_gravity": compute_generalized_gravity,
            "load_dynamics_model": load_dynamics_model,
            "load_robot_model": load_robot_model,
            "pad_q_for_model": pad_q_for_model,
            "joint_to_pose": joint_to_pose,
            "get_end_effector_frame_id": get_end_effector_frame_id,
            "pin": pin,
        }
        self._loaded = True
        return self

    # ------------------------------------------------------------------ #
    # 便捷属性
    # ------------------------------------------------------------------ #
    @property
    def RebotArm(self):
        return self.load()._mods["RebotArm"]

    @property
    def pin(self):
        return self.load()._mods["pin"]

    def load_robot_model(self):
        return self.load()._mods["load_robot_model"]()

    def load_dynamics_model(self):
        return self.load()._mods["load_dynamics_model"]()

    def compute_gravity(self, model, q_padded, data):
        return self.load()._mods["compute_generalized_gravity"](model, q_padded, data)

    def pad_q_for_model(self, model, q, controlled_joints: int | None = None):
        return self.load()._mods["pad_q_for_model"](model, q, controlled_joints)

    def joint_to_pose(self, q):
        return self.load()._mods["joint_to_pose"](q)

    def get_end_effector_frame_id(self, model):
        return self.load()._mods["get_end_effector_frame_id"](model)

    def joint_limits(self, model, controlled_joints: int = 6):
        import numpy as np

        lo = np.asarray(model.lowerPositionLimit[:controlled_joints], dtype=float)
        hi = np.asarray(model.upperPositionLimit[:controlled_joints], dtype=float)
        return lo, hi

    # ------------------------------------------------------------------ #
    # 控制核心（teleop_core.py，无 GUI 版 spatial_teleop）
    # ------------------------------------------------------------------ #
    def load_teleop_core(self):
        """导入仓库里的 TeleopCore（不存在时给出清晰提示）。"""
        self.load()
        try:
            from teleop_core import CoreArgs, CoreCommand, TeleopCore  # noqa: PLC0415
        except Exception as e:  # noqa: BLE001
            raise RuntimeError(
                f"导入 teleop_core 失败：{type(e).__name__}: {e}\n"
                f"请确认 {self.path}/teleop_core.py 存在（由平台随附或从 spatial_teleop.py 抽出）。"
            ) from e
        return TeleopCore, CoreArgs, CoreCommand
