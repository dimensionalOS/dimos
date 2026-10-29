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

"""Where a Robot reads from: tf, joint states and model parts, live or recorded.

A Robot never knows which. `LiveSource` is what a RobotAware module builds over its
ports; `StoreSource` is a recording; a test can implement the protocol over dicts.

SKETCH: signatures only.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field
from typing import Any, Protocol

from dimos.core.stream import In
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.protocol.robot.model import RobotModel
from dimos.protocol.tf.tf import TFLookup


@dataclass(frozen=True)
class JointSample:
    """Joint values at `ts`, interpolated: linear in position (wrapped for continuous
    joints), velocity and effort from the nearer sample, never differenced."""

    ts: float
    positions: Mapping[str, float] = field(default_factory=dict)
    velocities: Mapping[str, float] = field(default_factory=dict)
    efforts: Mapping[str, float] = field(default_factory=dict)


@dataclass(frozen=True)
class ModelPart:
    """One received piece of a robot: its owner's subtree, a mount, or a whole robot."""

    robot: str  # which robot on the bus, "" for a single one
    part: str  # the publisher's own key for it (by default its module name)
    model: RobotModel
    first_seen: float  # when this content was first received; a resend doesn't bump it


class RobotSource(Protocol):
    def transform(self, parent: str, child: str, t: float | None = None) -> Transform | None:
        """A tf lookup (nearest sample within tolerance), as TFLookup.get."""
        ...

    def joints(self, names: Iterable[str], t: float | None = None) -> JointSample | None:
        """The named joints at `t` (default: latest); None outside the sample window."""
        ...

    def parts(self, robot: str = "") -> Iterable[ModelPart]:
        """Model parts received for `robot`, oldest content first."""
        ...

    def latest(self) -> float | None:
        """The newest timestamp across tf and joint states."""
        ...


class LiveSource:
    """Over a module's ports: its tf buffer plus a joint buffer and the parts received."""

    def __init__(
        self,
        tf: TFLookup,
        joint_state: In[JointState] | None = None,
        robot_model: In[RobotModel] | None = None,
        window: float = 10.0,
    ) -> None:
        """`window` seconds of joint samples are kept for interpolation."""
        raise NotImplementedError

    def transform(self, parent: str, child: str, t: float | None = None) -> Transform | None:
        raise NotImplementedError

    def joints(self, names: Iterable[str], t: float | None = None) -> JointSample | None:
        raise NotImplementedError

    def parts(self, robot: str = "") -> Iterable[ModelPart]:
        raise NotImplementedError

    def latest(self) -> float | None:
        raise NotImplementedError


class StoreSource:
    """A recording's tf, joint_state and robot_model streams, loaded together per window."""

    @classmethod
    def from_store(cls, store: Any, cache_span: float = 300.0) -> StoreSource:
        raise NotImplementedError

    def transform(self, parent: str, child: str, t: float | None = None) -> Transform | None:
        raise NotImplementedError

    def joints(self, names: Iterable[str], t: float | None = None) -> JointSample | None:
        raise NotImplementedError

    def parts(self, robot: str = "") -> Iterable[ModelPart]:
        """The recorded robot_model stream is small (on change), so it's loaded whole."""
        raise NotImplementedError

    def latest(self) -> float | None:
        raise NotImplementedError
