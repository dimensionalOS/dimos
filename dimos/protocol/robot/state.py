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

"""A robot at a moment (RobotState), and a model read from a source over time (Robot).

SKETCH: signatures only.
"""

from __future__ import annotations

from collections.abc import Iterator, Mapping
from dataclasses import dataclass, field
from typing import Literal

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.robot.model import (
    Joint,
    JointSelector,
    JointType,
    JointValues,
    Limits,
    LimitViolation,
    RobotModel,
)
from dimos.protocol.robot.source import RobotSource


class StaleError(LookupError):
    """The newest data is older than `max_age` allows."""


@dataclass(frozen=True)
class JointView:
    """A joint's structure and its values at one moment, read in one place."""

    joint: Joint
    position: float | None = None  # None: never received; not 0
    velocity: float | None = None
    effort: float | None = None

    @property
    def limits(self) -> Limits:
        return self.joint.limits

    @property
    def type(self) -> JointType:
        return self.joint.type

    @property
    def axis(self) -> tuple[float, float, float]:
        return self.joint.axis

    @property
    def within_limits(self) -> bool | None:
        """None when the position is unknown."""
        raise NotImplementedError


class JointViews(Mapping[str, JointView]):
    def __getitem__(self, name: str) -> JointView:
        raise NotImplementedError

    def __iter__(self) -> Iterator[str]:
        raise NotImplementedError

    def __len__(self) -> int:
        raise NotImplementedError

    def select(self, joints: JointSelector) -> JointViews:
        """By name or glob, like Joints.select."""
        raise NotImplementedError


@dataclass(frozen=True)
class RobotState:
    """Immutable. From `model.state(...)` (detached) or `robot.state(t)` (attached: world
    frames resolve through the source at `ts`)."""

    model: RobotModel
    positions: JointValues = field(default_factory=dict)  # missing joints: unknown
    velocities: JointValues = field(default_factory=dict)
    efforts: JointValues = field(default_factory=dict)
    ts: float | None = None  # the moment asked for
    joints_ts: float | None = None  # when the joint values were measured
    # Set on attached states; compare=False keeps equality about the robot, not the bus.
    source: RobotSource | None = field(default=None, compare=False, repr=False)
    name: str = ""  # the robot's name on the bus, for world lookups through `source`

    @property
    def joints(self) -> JointViews:
        raise NotImplementedError

    # Kinematics

    def get(self, parent: str, child: str) -> Transform:
        """parent <- child at this state. Unknown joints count as 0 clamped into limits.

        A frame outside the model (map, other robots) comes from the source: the robot is
        entered through whichever of its frames has the freshest tf edge at `ts` (the
        root on a tie), so odom -> mid360_link from Point-LIO works as well as
        odom -> base_link. A detached state raises LookupError for those.
        """
        raise NotImplementedError

    def tf(self, joints: Literal["fixed", "movable", "all"] = "all") -> TFMessage:
        """The tree as tf edges at this state, for consumers that only read tf."""
        raise NotImplementedError

    # Checks: unknown joints are skipped

    def violations(self) -> list[LimitViolation]:
        raise NotImplementedError

    def clamped(self) -> RobotState:
        raise NotImplementedError

    def with_state(self, values: JointState | JointValues) -> RobotState:
        """Joint values on top of this state; mimic joints are derived, not taken."""
        raise NotImplementedError


class Robot:
    """A model read from a source. Same API live or over a recording.

    Which model: the attached one is authoritative. Parts received for this robot only
    add what it doesn't have (an arm, a sensor); among received parts the newest
    distinct content wins, so a resend never undoes a calibration. Attach to the
    received model itself (`RobotModel.root().merge(*parts)`) to follow the bus instead.

    Every read goes to the source: `state(t)` is the core, `joints`/`get` are shorthand
    for `state().<read>`, so two shorthand reads on a live robot can see different
    moments. In a control step take `s = robot.state()` once and read from `s`.
    """

    def __init__(self, model: RobotModel, source: RobotSource, name: str = "") -> None:
        """Names in the model, plus its odom frame, are read off the source as
        `<name>/<local>` (bare when `name` is ""); everything else as written. Nothing is
        checked up front: a rig with no odometry is a valid Robot. A query that needs a
        frame the source doesn't have raises, naming robots that do have it:
        `no frame "base_link"; did you mean name="go2a" or name="go2b"?`
        """
        raise NotImplementedError

    @property
    def name(self) -> str:
        raise NotImplementedError

    @property
    def model(self) -> RobotModel:
        """The attached model plus received additions; cached on the parts' digests."""
        raise NotImplementedError

    def state(self, t: float | None = None, max_age: float | None = None) -> RobotState:
        """The robot at `t` (default: the source's latest). With `max_age`, raises
        StaleError when the joint values are older than that."""
        raise NotImplementedError

    def states(self, t0: float, t1: float, hz: float) -> Iterator[RobotState]:
        """States at a fixed rate over a window, in order; replays and analysis loops."""
        raise NotImplementedError

    # Shorthand for state().<read>

    @property
    def joints(self) -> JointViews:
        raise NotImplementedError

    def get(self, parent: str, child: str, t: float | None = None) -> Transform:
        raise NotImplementedError
