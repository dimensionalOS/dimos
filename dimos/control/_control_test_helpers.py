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

"""Shared control-task stubs for tests.

Not named ``test_*``/``*_test`` so pytest does not collect it; both
``test_control.py`` and ``test_coordinator_routing.py`` import ``RecordingTask``
from here rather than reaching into each other's test modules.

``turning_joints`` writes out a test robot's ``ControlDescription`` by hand,
for tests that have no robot model.

``FakeArm`` is a robot driver with no hardware behind it, for tests of
``ConnectionModule`` and of anything that talks to one.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Mapping, Sequence
import functools
from pathlib import Path
import tempfile
from typing import Any

from pydantic import Field

from dimos.control.connection.connection_module import ConnectionConfig, ConnectionModule
from dimos.control.contract.description import (
    ControlDescription,
    Limits,
    Resource,
    ResourceKind,
)
from dimos.control.contract.keys import Interface, Unit
from dimos.control.contract.ports import JointPort
from dimos.control.task import (
    BaseControlTask,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import RobotModel


class RecordingTask(BaseControlTask):
    """Stub task that records every stream handler invocation."""

    def __init__(self, name: str, joints: frozenset[str] = frozenset()) -> None:
        self._name = name
        self._joints = frozenset(joints)
        self.cartesian_calls: list[tuple[Any, float]] = []
        self.left_cartesian_calls: list[tuple[Pose | PoseStamped, float]] = []
        self.right_cartesian_calls: list[tuple[Pose | PoseStamped, float]] = []
        self.ee_twist_calls: list[tuple[Any, float]] = []
        self.joint_command_calls: list[tuple[Any, float]] = []
        self.buttons_calls: list[Any] = []
        self.gripper_calls: list[tuple[Any, float]] = []

    def claim(self) -> ResourceClaim:
        return ResourceClaim(joints=self._joints)

    def is_active(self) -> bool:
        return False

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        return None

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        pass

    def on_cartesian_command(self, pose: Any, t_now: float) -> bool:
        self.cartesian_calls.append((pose, t_now))
        return True

    def on_left_cartesian_command(self, pose: Pose | PoseStamped, t_now: float) -> bool:
        self.left_cartesian_calls.append((pose, t_now))
        return True

    def on_right_cartesian_command(self, pose: Pose | PoseStamped, t_now: float) -> bool:
        self.right_cartesian_calls.append((pose, t_now))
        return True

    def on_ee_twist_command(self, twist: Any, t_now: float) -> bool:
        self.ee_twist_calls.append((twist, t_now))
        return True

    def on_joint_command(self, msg: Any, t_now: float) -> bool:
        self.joint_command_calls.append((msg, t_now))
        return True

    def on_gripper_command(self, msg: Any, t_now: float) -> bool:
        self.gripper_calls.append((msg, t_now))
        return True

    def on_buttons(self, msg: Any) -> bool:
        self.buttons_calls.append(msg)
        return True

    def on_teleop_buttons(self, msg: Any, t_now: float) -> bool:
        # Mirrors TeleopIKTask: the uniform handler delegates to on_buttons.
        return self.on_buttons(msg)


#: The unit of each joint interface, for a joint that turns.
_TURNING: Mapping[str, Unit] = {
    Interface.POSITION: Unit.RAD,
    Interface.VELOCITY: Unit.RAD_PER_S,
    Interface.EFFORT: Unit.NM,
    Interface.KP: Unit.UNITLESS,
    Interface.KD: Unit.UNITLESS,
}


def turning_joints(
    source: str,
    joints: Sequence[str],
    *,
    state: Sequence[Interface],
    command: Sequence[Interface],
    limits: Mapping[str, Limits] | None = None,
    others: Sequence[Resource] = (),
    state_rate_hz: float = 100.0,
    deadman_timeout_s: float = 0.1,
) -> ControlDescription:
    """Describe a robot of turning joints by hand, without a robot model.

    Args:
        source: The robot's name, the first part of every key, e.g. "mock".
        joints: The joint names, in order.
        state: What each joint reports.
        command: What each joint can be told.
        limits: Limits by full name, such as "mock/joint1/position". A
            command with no limit is unlimited.
        others: Parts to add after the joints, such as an IMU.
        state_rate_hz: How many times a second it reports.
        deadman_timeout_s: How long it waits for a command, in seconds,
            before halting.
    """
    resources = tuple(
        Resource(
            name=joint,
            kind=ResourceKind.JOINT,
            state_interfaces=tuple(state),
            command_interfaces=tuple(command),
            units={interface: _TURNING[interface] for interface in (*state, *command)},
        )
        for joint in joints
    )
    return ControlDescription(
        source=source,
        resources=(*resources, *others),
        limits=dict(limits or {}),
        state_rate_hz=state_rate_hz,
        deadman_timeout_s=deadman_timeout_s,
    )


#: The fake arm's joints, in order.
FAKE_ARM_JOINTS: tuple[str, ...] = tuple(f"joint{i}" for i in range(1, 8))


@functools.cache
def fake_arm_model() -> RobotModel:
    """A model of seven turning joints, joint1 to joint7, one after another.

    Each may be told to go anywhere within one turn either side of 0 (radians),
    at up to 2 rad/s and 50 Nm.
    """
    limit = 'lower="-3.141592653589793" upper="3.141592653589793" velocity="2.0" effort="50.0"'
    links = "".join(f'<link name="link{i}"/>' for i in range(len(FAKE_ARM_JOINTS) + 1))
    joints = "".join(
        f'<joint name="{joint}" type="revolute"><parent link="link{i}"/>'
        f'<child link="link{i + 1}"/><limit {limit}/></joint>'
        for i, joint in enumerate(FAKE_ARM_JOINTS)
    )
    path = Path(tempfile.mkdtemp(prefix="fake_arm_")) / "fake_arm.urdf"
    path.write_text(f'<robot name="fake_arm">{links}{joints}</robot>')
    return RobotModel.from_file(path)


class FakeArmConfig(ConnectionConfig):
    """Settings for ``FakeArm``: named "mock", driven by joint position."""

    source: str = "mock"
    model: RobotModel | None = Field(default_factory=fake_arm_model)
    position_command: JointPort = JointPort()


class FakeArm(ConnectionModule):
    """A pretend arm, driven by joint position, with no hardware behind it.

    Its joints are wherever they were last told to be, at once, and it reports
    every other reading as the last value it was told, or zero. ``writes``
    keeps the latest commands passed to ``write`` and ``halts`` counts calls
    to ``halt``, so a test can see what reached the "hardware".
    """

    config: FakeArmConfig

    position_command: In[JointState]
    joint_state: Out[JointState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        #: The last 1000 commands passed to ``write``, oldest first.
        self.writes: deque[dict[str, float]] = deque(maxlen=1000)
        #: How many times ``halt`` has been called.
        self.halts = 0
        self._readings: dict[str, float] = {}

    def connect(self) -> None:
        self._readings = dict.fromkeys(self.description.state_keys(), 0.0)

    def read_state(self) -> dict[str, float]:
        return dict(self._readings)

    def write(self, values: dict[str, float]) -> None:
        self.writes.append(dict(values))
        # Replaced whole, so the reading thread never sees it half-changed.
        # Only keys it reports are kept, so its readings stay valid.
        self._readings = {key: values.get(key, value) for key, value in self._readings.items()}

    def halt(self) -> None:
        self.halts += 1
