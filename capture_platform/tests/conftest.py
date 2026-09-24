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


@pytest.fixture()
def core_service(core_env):
    """真 teleop_core + MockArm 的采集服务（不接硬件、不开控制线程）。"""
    from rebot_capture.device.mock_arm import MockArm
    from rebot_capture.device.profile import DeviceProfile
    from rebot_capture.recorder.session import CaptureService
    from rebot_capture.teleop.rebot_core import RebotCoreMapper

    repo = core_env[0]
    profile = DeviceProfile.load()
    arm = MockArm(profile)
    mapper = RebotCoreMapper(profile, repo=repo)
    service = CaptureService(profile, arm, mapper=mapper)
    service.connect()
    yield service
    try:
        service.stop_loop()
    except Exception:  # noqa: BLE001
        pass
