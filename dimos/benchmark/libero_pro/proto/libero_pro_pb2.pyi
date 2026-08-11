from typing import Any

class _Message:
    def __init__(self, **kwargs: Any) -> None: ...
    def SerializeToString(self) -> bytes: ...
    @classmethod
    def FromString(cls, value: bytes) -> Any: ...

class Empty(_Message): ...
class WatchRequest(_Message): ...

class Health(_Message):
    ready: bool
    detail: str

class ImageFrame(_Message):
    camera: str
    width: int
    height: int
    rgb: bytes

class RobotSnapshot(_Message):
    tick: int
    timestamp_s: float
    joint_position: Any
    joint_velocity: Any
    gripper_position: float
    images: Any

class JointTargets(_Message):
    joint_position: Any
    gripper_position: float
    sequence: int

class Ack(_Message):
    sequence: int

class InitializeTrialRequest(_Message):
    suite: str
    task_order_index: int
    task_index: int
    init_state_index: int
    horizon_ticks: int
    control_frequency_hz: int
    settling_ticks: int

class TrialReady(_Message):
    task_name: str
    instruction: str

class TerminalResult(_Message):
    success: bool
    score: float
    reward: float
    terminal_reason: str
    policy_ticks: int
    backend_ticks: int
    error: str
