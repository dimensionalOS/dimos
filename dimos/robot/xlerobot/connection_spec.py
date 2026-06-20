"""XLeRobot connection spec for dependency injection."""

from typing import Protocol

from dimos.spec.utils import Spec


class XLeRobotSpec(Spec, Protocol):
    """Protocol defining the full XLeRobot connection interface."""

    def move_base(
        self,
        x: float = 0.0,
        y: float = 0.0,
        theta: float = 0.0,
        duration: float = 0.0,
    ) -> str: ...

    def stop_base(self) -> str: ...

    def move_arm(self, arm: str, joint_positions: str, duration: float = 1.0) -> str: ...

    def move_head(self, pan: float, tilt: float) -> str: ...

    def open_gripper(self, arm: str) -> str: ...

    def close_gripper(self, arm: str) -> str: ...

    def get_joint_positions(self) -> str: ...

    def home_arms(self) -> str: ...

    def observe(self) -> object: ...
