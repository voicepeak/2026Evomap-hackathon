"""HTTP / WebSocket 请求模型。"""
from __future__ import annotations

from typing import Any

from pydantic import BaseModel, Field


class SessionStartIn(BaseModel):
    task_id: str | None = None
    operator: str | None = None


class EpisodeStopIn(BaseModel):
    success: bool = True
    note: str = ""


class PackIn(BaseModel):
    name: str | None = None
    task_instruction: str | None = None
    include_failed: bool = False


class PenSampleIn(BaseModel):
    t: float = 0.0
    x: float = 0.0
    y: float = 0.0
    pressure: float = 0.0
    tiltX: float = 0.0
    tiltY: float = 0.0
    twist: float = 0.0
    touching: bool = False
    extra: dict[str, Any] = Field(default_factory=dict)


class GotoIn(BaseModel):
    target: list[float] = Field(..., min_length=7, max_length=7, description="7 维：6 关节(rad) + 夹爪(0-1)")
    duration: float | None = Field(None, description="最小 jerk 曲线时长（秒）；None=按位移自动 1.5~6s")


class FreezeIn(BaseModel):
    on: bool | None = None  # None = 切换


class SpeedIn(BaseModel):
    index: int | None = None  # None = 循环 慢→中→快


class ModeIn(BaseModel):
    mode: str = Field(..., pattern="^(pos|ori)$")


class FloatIn(BaseModel):
    on: bool


class JointIn(BaseModel):
    index: int | None = Field(None, ge=0, le=6)
    step: int = Field(0, ge=-1, le=1,
                      description="相对当前选择 ±1（服务端算；客户端状态过期也不会卡住）")
    hold: float | None = Field(None, description="关节直控速度 rad/s（0=松开）")


class MotorIn(BaseModel):
    index: int | None = Field(None, ge=0, le=6,
                              description="0-5=J1-J6，6=夹爪；省略 = 按 step 从当前电机循环")
    step: int = Field(1, ge=-1, le=1,
                      description="index 省略时用：+1 下一个 / −1 上一个（服务端按自己的状态算）")


class GestureIn(BaseModel):
    on: bool | None = Field(None, description="True=启用手势夹爪，False=停用，省略=查询当前状态")


class PresetIn(BaseModel):
    action: str = Field(..., pattern="^(record|goto)$")
    index: int = Field(..., ge=1, le=4)


class TauLimitIn(BaseModel):
    value: float = Field(..., ge=0.0)


class TwistIn(BaseModel):
    value: float = Field(..., description="J6 自转速度 rad/s（0=松开）")


class CameraOpenIn(BaseModel):
    index: int | None = Field(None, ge=0, le=8)
    url: str | None = Field(None, description="MJPEG 流地址（相机桥）")


class CameraCloseIn(BaseModel):
    index: Any | None = Field(None, description="索引或 URL；省略=当前选中")


class CameraAliasIn(BaseModel):
    name: str = Field(..., description="数据集里的名字（wrist / scene / camN）")
    index: Any | None = None
    url: str | None = None


class ReplayIn(BaseModel):
    index: int | None = Field(None, description="episode 序号；省略=最后一条")
    speed: float = Field(1.0, gt=0.0, le=4.0)
    tau_abort: float = Field(25.0, gt=0.0)
    dataset_path: str | None = Field(None, description="从已打包数据集回放（服务重启后可用）")
    max_joint_speed_deg: float = Field(220.0, gt=0.0, le=720.0, description="指令关节速度上限（°/s）")

