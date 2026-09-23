"""共享测试夹具：工作区 arm_control 的真实运动学模型 + TeleopCore（不连接电机）。"""
from pathlib import Path

import pytest

from rebot_capture.integrations.rebotarm import RebotArmRepo

WORKSPACE_ARM = Path(__file__).resolve().parents[2] / "arm_control"


@pytest.fixture(scope="module")
def core_env():
    if not (WORKSPACE_ARM / "reBotArm_control_py").is_dir():
        pytest.skip("工作区没有 arm_control 仓库")
    repo = RebotArmRepo(WORKSPACE_ARM)
    try:
        TeleopCore, CoreArgs, _ = repo.load_teleop_core()
    except RuntimeError as exc:  # pin / pinocchio 缺失等
        pytest.skip(f"无法加载 teleop_core：{exc}")
    model = repo.load_robot_model()
    fid = repo.get_end_effector_frame_id(model)
    return repo, TeleopCore, CoreArgs, model, fid
