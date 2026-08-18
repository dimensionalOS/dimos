from collections.abc import Iterable as _Iterable, Mapping as _Mapping
from typing import ClassVar as _ClassVar, Union as _Union

from google.protobuf import descriptor as _descriptor, message as _message
from google.protobuf.internal import containers as _containers

DESCRIPTOR: _descriptor.FileDescriptor

class Empty(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class Health(_message.Message):
    __slots__ = ("detail", "ready")
    READY_FIELD_NUMBER: _ClassVar[int]
    DETAIL_FIELD_NUMBER: _ClassVar[int]
    ready: bool
    detail: str
    def __init__(self, ready: bool | None = ..., detail: str | None = ...) -> None: ...

class WatchRequest(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class CameraFrame(_message.Message):
    __slots__ = (
        "camera",
        "camera_to_robot_base",
        "depth_meters",
        "height",
        "intrinsic",
        "rgb",
        "width",
    )
    CAMERA_FIELD_NUMBER: _ClassVar[int]
    WIDTH_FIELD_NUMBER: _ClassVar[int]
    HEIGHT_FIELD_NUMBER: _ClassVar[int]
    RGB_FIELD_NUMBER: _ClassVar[int]
    DEPTH_METERS_FIELD_NUMBER: _ClassVar[int]
    INTRINSIC_FIELD_NUMBER: _ClassVar[int]
    CAMERA_TO_ROBOT_BASE_FIELD_NUMBER: _ClassVar[int]
    camera: str
    width: int
    height: int
    rgb: bytes
    depth_meters: bytes
    intrinsic: _containers.RepeatedScalarFieldContainer[float]
    camera_to_robot_base: _containers.RepeatedScalarFieldContainer[float]
    def __init__(
        self,
        camera: str | None = ...,
        width: int | None = ...,
        height: int | None = ...,
        rgb: bytes | None = ...,
        depth_meters: bytes | None = ...,
        intrinsic: _Iterable[float] | None = ...,
        camera_to_robot_base: _Iterable[float] | None = ...,
    ) -> None: ...

class RobotSnapshot(_message.Message):
    __slots__ = (
        "cameras",
        "gripper_position",
        "joint_position",
        "joint_velocity",
        "tick",
        "timestamp_s",
    )
    TICK_FIELD_NUMBER: _ClassVar[int]
    TIMESTAMP_S_FIELD_NUMBER: _ClassVar[int]
    JOINT_POSITION_FIELD_NUMBER: _ClassVar[int]
    JOINT_VELOCITY_FIELD_NUMBER: _ClassVar[int]
    GRIPPER_POSITION_FIELD_NUMBER: _ClassVar[int]
    CAMERAS_FIELD_NUMBER: _ClassVar[int]
    tick: int
    timestamp_s: float
    joint_position: _containers.RepeatedScalarFieldContainer[float]
    joint_velocity: _containers.RepeatedScalarFieldContainer[float]
    gripper_position: float
    cameras: _containers.RepeatedCompositeFieldContainer[CameraFrame]
    def __init__(
        self,
        tick: int | None = ...,
        timestamp_s: float | None = ...,
        joint_position: _Iterable[float] | None = ...,
        joint_velocity: _Iterable[float] | None = ...,
        gripper_position: float | None = ...,
        cameras: _Iterable[_Union[CameraFrame, _Mapping]] | None = ...,
    ) -> None: ...

class JointTargets(_message.Message):
    __slots__ = ("gripper_position", "joint_position", "sequence")
    JOINT_POSITION_FIELD_NUMBER: _ClassVar[int]
    GRIPPER_POSITION_FIELD_NUMBER: _ClassVar[int]
    SEQUENCE_FIELD_NUMBER: _ClassVar[int]
    joint_position: _containers.RepeatedScalarFieldContainer[float]
    gripper_position: float
    sequence: int
    def __init__(
        self,
        joint_position: _Iterable[float] | None = ...,
        gripper_position: float | None = ...,
        sequence: int | None = ...,
    ) -> None: ...

class Ack(_message.Message):
    __slots__ = ("sequence",)
    SEQUENCE_FIELD_NUMBER: _ClassVar[int]
    sequence: int
    def __init__(self, sequence: int | None = ...) -> None: ...

class InitializeTrialRequest(_message.Message):
    __slots__ = (
        "control_frequency_hz",
        "horizon_ticks",
        "init_state_index",
        "settling_ticks",
        "suite",
        "task_index",
        "task_order_index",
    )
    SUITE_FIELD_NUMBER: _ClassVar[int]
    TASK_ORDER_INDEX_FIELD_NUMBER: _ClassVar[int]
    TASK_INDEX_FIELD_NUMBER: _ClassVar[int]
    INIT_STATE_INDEX_FIELD_NUMBER: _ClassVar[int]
    HORIZON_TICKS_FIELD_NUMBER: _ClassVar[int]
    CONTROL_FREQUENCY_HZ_FIELD_NUMBER: _ClassVar[int]
    SETTLING_TICKS_FIELD_NUMBER: _ClassVar[int]
    suite: str
    task_order_index: int
    task_index: int
    init_state_index: int
    horizon_ticks: int
    control_frequency_hz: int
    settling_ticks: int
    def __init__(
        self,
        suite: str | None = ...,
        task_order_index: int | None = ...,
        task_index: int | None = ...,
        init_state_index: int | None = ...,
        horizon_ticks: int | None = ...,
        control_frequency_hz: int | None = ...,
        settling_ticks: int | None = ...,
    ) -> None: ...

class TrialReady(_message.Message):
    __slots__ = ("instruction", "task_name")
    TASK_NAME_FIELD_NUMBER: _ClassVar[int]
    INSTRUCTION_FIELD_NUMBER: _ClassVar[int]
    task_name: str
    instruction: str
    def __init__(self, task_name: str | None = ..., instruction: str | None = ...) -> None: ...

class TerminalResult(_message.Message):
    __slots__ = (
        "backend_ticks",
        "error",
        "policy_ticks",
        "reward",
        "score",
        "success",
        "terminal_reason",
    )
    SUCCESS_FIELD_NUMBER: _ClassVar[int]
    SCORE_FIELD_NUMBER: _ClassVar[int]
    REWARD_FIELD_NUMBER: _ClassVar[int]
    TERMINAL_REASON_FIELD_NUMBER: _ClassVar[int]
    POLICY_TICKS_FIELD_NUMBER: _ClassVar[int]
    BACKEND_TICKS_FIELD_NUMBER: _ClassVar[int]
    ERROR_FIELD_NUMBER: _ClassVar[int]
    success: bool
    score: float
    reward: float
    terminal_reason: str
    policy_ticks: int
    backend_ticks: int
    error: str
    def __init__(
        self,
        success: bool | None = ...,
        score: float | None = ...,
        reward: float | None = ...,
        terminal_reason: str | None = ...,
        policy_ticks: int | None = ...,
        backend_ticks: int | None = ...,
        error: str | None = ...,
    ) -> None: ...
