from google.protobuf.internal import containers as _containers
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from collections.abc import Iterable as _Iterable, Mapping as _Mapping
from typing import ClassVar as _ClassVar, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class Empty(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class Health(_message.Message):
    __slots__ = ("ready", "detail")
    READY_FIELD_NUMBER: _ClassVar[int]
    DETAIL_FIELD_NUMBER: _ClassVar[int]
    ready: bool
    detail: str
    def __init__(self, ready: _Optional[bool] = ..., detail: _Optional[str] = ...) -> None: ...

class WatchRequest(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class CameraFrame(_message.Message):
    __slots__ = ("camera", "width", "height", "rgb", "depth_meters", "intrinsic", "camera_to_robot_base")
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
    def __init__(self, camera: _Optional[str] = ..., width: _Optional[int] = ..., height: _Optional[int] = ..., rgb: _Optional[bytes] = ..., depth_meters: _Optional[bytes] = ..., intrinsic: _Optional[_Iterable[float]] = ..., camera_to_robot_base: _Optional[_Iterable[float]] = ...) -> None: ...

class RobotSnapshot(_message.Message):
    __slots__ = ("tick", "timestamp_s", "joint_position", "joint_velocity", "gripper_position", "cameras")
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
    def __init__(self, tick: _Optional[int] = ..., timestamp_s: _Optional[float] = ..., joint_position: _Optional[_Iterable[float]] = ..., joint_velocity: _Optional[_Iterable[float]] = ..., gripper_position: _Optional[float] = ..., cameras: _Optional[_Iterable[_Union[CameraFrame, _Mapping]]] = ...) -> None: ...

class JointTargets(_message.Message):
    __slots__ = ("joint_position", "gripper_position", "sequence")
    JOINT_POSITION_FIELD_NUMBER: _ClassVar[int]
    GRIPPER_POSITION_FIELD_NUMBER: _ClassVar[int]
    SEQUENCE_FIELD_NUMBER: _ClassVar[int]
    joint_position: _containers.RepeatedScalarFieldContainer[float]
    gripper_position: float
    sequence: int
    def __init__(self, joint_position: _Optional[_Iterable[float]] = ..., gripper_position: _Optional[float] = ..., sequence: _Optional[int] = ...) -> None: ...

class Ack(_message.Message):
    __slots__ = ("sequence",)
    SEQUENCE_FIELD_NUMBER: _ClassVar[int]
    sequence: int
    def __init__(self, sequence: _Optional[int] = ...) -> None: ...

class InitializeTrialRequest(_message.Message):
    __slots__ = ("suite", "task_order_index", "task_index", "init_state_index", "horizon_ticks", "control_frequency_hz", "settling_ticks")
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
    def __init__(self, suite: _Optional[str] = ..., task_order_index: _Optional[int] = ..., task_index: _Optional[int] = ..., init_state_index: _Optional[int] = ..., horizon_ticks: _Optional[int] = ..., control_frequency_hz: _Optional[int] = ..., settling_ticks: _Optional[int] = ...) -> None: ...

class TrialReady(_message.Message):
    __slots__ = ("task_name", "instruction")
    TASK_NAME_FIELD_NUMBER: _ClassVar[int]
    INSTRUCTION_FIELD_NUMBER: _ClassVar[int]
    task_name: str
    instruction: str
    def __init__(self, task_name: _Optional[str] = ..., instruction: _Optional[str] = ...) -> None: ...

class TerminalResult(_message.Message):
    __slots__ = ("success", "score", "reward", "terminal_reason", "policy_ticks", "backend_ticks", "error")
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
    def __init__(self, success: _Optional[bool] = ..., score: _Optional[float] = ..., reward: _Optional[float] = ..., terminal_reason: _Optional[str] = ..., policy_ticks: _Optional[int] = ..., backend_ticks: _Optional[int] = ..., error: _Optional[str] = ...) -> None: ...
