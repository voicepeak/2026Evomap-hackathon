"""设备层：档案、后端协议、Mock 实现（真机后端后续接入）。"""
from .backends import ArmBackend, ArmStatus, BackendError, JointState
from .mock_arm import MockArm
from .profile import DeviceProfile, GripperProfile, JointProfile

__all__ = [
    "ArmBackend",
    "ArmStatus",
    "BackendError",
    "JointState",
    "MockArm",
    "DeviceProfile",
    "GripperProfile",
    "JointProfile",
]
