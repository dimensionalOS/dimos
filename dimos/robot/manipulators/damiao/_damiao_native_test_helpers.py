# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Native Robot test boundary, a small robot's data and synchronous local ports."""

from __future__ import annotations

from collections.abc import Callable
from types import SimpleNamespace
from typing import Any

import numpy as np
from numpy.typing import NDArray

from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators.damiao.robot import (
    DamiaoBus,
    DamiaoGripper,
    DamiaoMotor,
    DamiaoRobot,
)


def _test_model() -> RobotModel:
    """Three turning joints in a chain, then two sliding fingers on the last link.

    left_finger runs 0 to 0.04 m and right_finger -0.02 to 0 m.
    """
    limit = 'lower="-3.141592653589793" upper="3.141592653589793" velocity="2.0" effort="50.0"'
    chain = "".join(
        f'<joint name="joint{i}" type="revolute"><parent link="link{i - 1}"/>'
        f'<child link="link{i}"/><limit {limit}/></joint>'
        for i in range(1, 4)
    )
    fingers = "".join(
        f'<joint name="{name}" type="prismatic"><parent link="link3"/><child link="{name}_link"/>'
        f'<axis xyz="0 1 0"/><limit lower="{lower}" upper="{upper}" velocity="0.1" effort="10"/>'
        "</joint>"
        for name, lower, upper in (("left_finger", 0.0, 0.04), ("right_finger", -0.02, 0.0))
    )
    links = "".join(f'<link name="{name}"/>' for name in ("link0", "link1", "link2", "link3"))
    finger_links = '<link name="left_finger_link"/><link name="right_finger_link"/>'
    return RobotModel.from_urdf_string(
        f'<robot name="test">{links}{finger_links}{chain}{fingers}</robot>'
    )


def make_test_robot() -> DamiaoRobot:
    """Three arm joints on two buses, plus one gripper on each bus.

    The left jaw is open at its finger's upper end, the right at its lower end.
    """
    model = _test_model()
    robot = DamiaoRobot(
        buses=[
            DamiaoBus(
                name="left_arm",
                arm=[
                    DamiaoMotor("joint1", "DM4340", 1, 17),
                    DamiaoMotor("joint2", "DM4310", 2, 18),
                ],
                kp=[80.0, 10.0],
                kd=[5.0, 1.5],
                gripper=DamiaoGripper(
                    name="left_gripper",
                    joint="left_finger",
                    open_at="upper",
                    motor=DamiaoMotor("left_gripper", "DM4310", 8, 24),
                    opening="decreasing_position",
                    current=0.15,
                ),
            ),
            DamiaoBus(
                name="right_arm",
                arm=[DamiaoMotor("joint3", "DM4310", 1, 17)],
                kp=[20.0],
                kd=[0.6],
                gripper=DamiaoGripper(
                    name="right_gripper",
                    joint="right_finger",
                    open_at="lower",
                    motor=DamiaoMotor("right_gripper", "DM4310", 8, 24),
                    opening="increasing_position",
                    current=0.1,
                ),
            ),
        ],
        model=model,
        gravity_model=model,
        base_link="link0",
        joint_names=["joint1", "joint2", "joint3", "left_finger", "right_finger"],
        gravity=False,
    )
    return robot


#: The shared robot the Damiao fixtures deploy; tests clone it with overrides.
TEST_ROBOT = make_test_robot()


class LocalTransport:
    """Synchronous command delivery with a publication history for assertions."""

    def __init__(self) -> None:
        self.sent: list[Any] = []
        self.subscribers: list[Callable[[Any], Any]] = []

    def broadcast(self, stream: Any, value: Any) -> None:
        self.sent.append(value)
        for callback in tuple(self.subscribers):
            callback(value)

    def publish(self, value: Any) -> None:
        self.broadcast(None, value)

    def subscribe(self, callback: Callable[[Any], Any], stream: Any = None) -> Callable[[], None]:
        self.subscribers.append(callback)
        return lambda: self.subscribers.remove(callback)

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass


class FakeArm:
    def __init__(self, count: int, initial: float = 0.2) -> None:
        self.q = np.full(count, initial)
        self.dq = np.full(count, 0.3)
        self.tau = np.full(count, 0.4)
        self.commands: list[NDArray[np.float64]] = []
        self.modes: list[str] = []
        self.error: Exception | None = None

    def positions(self) -> NDArray[np.float64]:
        return self.q.copy()

    def velocities(self) -> NDArray[np.float64]:
        return self.dq.copy()

    def torques(self) -> NDArray[np.float64]:
        return self.tau.copy()

    def mit_control(self, rows: NDArray[np.float64]) -> None:
        if self.error:
            raise self.error
        self.commands.append(rows.copy())

    def set_mode(self, mode: str) -> None:
        self.modes.append(mode)


class FakeGripper:
    opening = 0.4

    def __init__(self) -> None:
        self.commands: list[float] = []
        self.motor = SimpleNamespace(position=0.4)
        self.modes: list[str] = []
        self.mit_commands: list[tuple[float, float, float, float, float]] = []
        self.enable_count = 0
        self.disable_count = 0

    def set_opening(self, opening: float) -> None:
        self.commands.append(opening)

    def set_mode(self, mode: str) -> None:
        self.modes.append(mode)

    def enable(self) -> None:
        self.enable_count += 1

    def disable(self) -> None:
        self.disable_count += 1

    def mit_control(self, kp: float, kd: float, q: float, dq: float, tau: float) -> None:
        self.mit_commands.append((kp, kd, q, dq, tau))


class FakeRobot:
    def __init__(self) -> None:
        self.default_position = 0.2
        self.groups: dict[str, FakeArm | FakeGripper] = {}
        self.events: list[str] = []
        self.connected = False
        self.enable_error: Exception | None = None
        self.disable_error: Exception | None = None
        self.refresh_error: Exception | None = None
        self.tick_error: Exception | None = None
        self.bus_specs: list[tuple[Any, ...]] = []
        self.arm_specs: list[tuple[Any, ...]] = []
        self.gripper_specs: list[tuple[Any, ...]] = []

    def __getitem__(self, name: str) -> FakeArm | FakeGripper:
        return self.groups[name]

    def connect(self) -> None:
        self.events.append("connect")
        self.connected = True

    def enable(self) -> None:
        self.events.append("enable")
        if self.enable_error:
            raise self.enable_error

    def disable(self) -> None:
        self.events.append("disable")
        if self.disable_error:
            raise self.disable_error

    def refresh(self) -> None:
        self.events.append("refresh")
        if self.refresh_error:
            raise self.refresh_error

    def tick(self, deadline: int) -> None:
        self.events.append("tick")
        assert deadline == 1000
        if self.tick_error:
            raise self.tick_error


class Builder:
    def __init__(self, robot: FakeRobot) -> None:
        self.robot = robot

    def add_bus(self, *args: Any) -> Builder:
        self.robot.bus_specs.append(args)
        return self

    def add_arm(self, name: str, *, bus: str, motors: list[Any]) -> Builder:
        self.robot.arm_specs.append((name, bus, motors))
        self.robot.groups[name] = FakeArm(len(motors), self.robot.default_position)
        return self

    def add_gripper(self, name: str, **kwargs: Any) -> Builder:
        self.robot.gripper_specs.append((name, kwargs))
        self.robot.groups[name] = FakeGripper()
        return self

    def build(self) -> FakeRobot:
        return self.robot
