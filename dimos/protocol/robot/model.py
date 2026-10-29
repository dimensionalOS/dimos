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

"""What a robot is: its links, joints, limits and base, edited in Python and saved as URDF.

Pure structure: no time, no state, no bus name. Where a robot is at a moment is a
RobotState; a model read live or from a recording is a Robot (`model.attach(source)`).

SKETCH: types and signatures only; every body raises NotImplementedError.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable, Iterator, Mapping, Sequence
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import TYPE_CHECKING

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.tf2_msgs.TFMessage import TFMessage

if TYPE_CHECKING:
    from dimos.protocol.robot.source import RobotSource
    from dimos.protocol.robot.state import Robot, RobotState
    from dimos.protocol.tf.tf import TFLookup

# Joint name -> position (rad or m), velocity, or effort.
JointValues = Mapping[str, float]

# A joint name, a glob over joint names ("*_calf_joint"), or several of either.
JointSelector = str | Sequence[str]

Vec3 = tuple[float, float, float]


class JointType(str, Enum):
    FIXED = "fixed"
    REVOLUTE = "revolute"
    CONTINUOUS = "continuous"
    PRISMATIC = "prismatic"
    # Floating and planar carry more than one value, so they're not posed from joint
    # values: their pose is a tf edge, read from the source like odometry.
    FLOATING = "floating"
    PLANAR = "planar"


def _own(obj: object, **values: object) -> None:
    """Store private copies of Pose fields; they're mutable until Pose is."""
    for name, value in values.items():
        object.__setattr__(obj, name, value)


# Joints


@dataclass(frozen=True, kw_only=True)
class Limits:
    """SI units: rad or m, rad/s or m/s, N*m or N. None is unbounded."""

    lower: float | None = None
    upper: float | None = None
    velocity: float | None = None
    effort: float | None = None
    # Not in URDF; saved in the joint's <dimos> block.
    acceleration: float | None = None
    jerk: float | None = None

    @property
    def span(self) -> float | None:
        """upper - lower, None if either side is unbounded."""
        raise NotImplementedError

    def contains(self, position: float) -> bool:
        raise NotImplementedError

    def clamp(self, position: float) -> float:
        raise NotImplementedError


@dataclass(frozen=True, kw_only=True)
class Dynamics:
    damping: float = 0.0
    friction: float = 0.0


@dataclass(frozen=True, kw_only=True)
class SafetyController:
    soft_lower: float | None = None
    soft_upper: float | None = None
    k_position: float | None = None
    k_velocity: float = 0.0


@dataclass(frozen=True, kw_only=True)
class Mimic:
    """position = multiplier * joint's position + offset.

    A mimic joint isn't actuated: it's not in `Joints.movable`, its value is always derived
    from `joint` (a published value for it is ignored), and `fix(joint)` fixes it too.
    """

    joint: str
    multiplier: float = 1.0
    offset: float = 0.0


@dataclass(frozen=True, kw_only=True)
class Joint:
    name: str
    type: JointType
    parent: str  # link
    child: str  # link
    origin: Pose = field(default_factory=Pose)  # parent -> child at position 0
    axis: Vec3 = (1.0, 0.0, 0.0)
    limits: Limits = field(default_factory=Limits)  # all None on fixed joints
    dynamics: Dynamics = field(default_factory=Dynamics)
    safety: SafetyController | None = None
    mimic: Mimic | None = None
    # Free-form key/values, saved in the joint's <dimos> block.
    meta: Mapping[str, str] = field(default_factory=dict)

    def __post_init__(self) -> None:
        _own(self, origin=Pose(self.origin))

    @property
    def is_movable(self) -> bool:
        """Revolute, continuous or prismatic, and not a mimic."""
        raise NotImplementedError

    def transform(self, position: float = 0.0) -> Transform:
        """parent -> child with the joint at `position`; raises for floating and planar."""
        raise NotImplementedError


# Links


@dataclass(frozen=True, kw_only=True)
class Box:
    size: Vec3


@dataclass(frozen=True, kw_only=True)
class Cylinder:
    radius: float
    length: float


@dataclass(frozen=True, kw_only=True)
class Sphere:
    radius: float


@dataclass(frozen=True, kw_only=True)
class Mesh:
    uri: str  # as written in the URDF (package://, file://, relative); kept for to_urdf
    scale: Vec3 = (1.0, 1.0, 1.0)

    def resolve(
        self, packages: Mapping[str, Path] | None = None, base_dir: Path | None = None
    ) -> Path:
        """The file on this machine; each machine resolves with its own `packages`."""
        raise NotImplementedError


Geometry = Box | Cylinder | Sphere | Mesh


@dataclass(frozen=True, kw_only=True)
class Material:
    name: str
    rgba: tuple[float, float, float, float] | None = None
    texture: str | None = None  # uri, like Mesh


@dataclass(frozen=True, kw_only=True)
class Visual:
    geometry: Geometry
    origin: Pose = field(default_factory=Pose)
    material: Material | None = None
    name: str | None = None

    def __post_init__(self) -> None:
        _own(self, origin=Pose(self.origin))


@dataclass(frozen=True, kw_only=True)
class Collision:
    geometry: Geometry
    origin: Pose = field(default_factory=Pose)
    name: str | None = None

    def __post_init__(self) -> None:
        _own(self, origin=Pose(self.origin))


@dataclass(frozen=True, kw_only=True)
class Inertial:
    mass: float
    origin: Pose = field(default_factory=Pose)
    # ixx, ixy, ixz, iyy, iyz, izz
    inertia: tuple[float, float, float, float, float, float] = (0.0,) * 6

    def __post_init__(self) -> None:
        _own(self, origin=Pose(self.origin))


@dataclass(frozen=True, kw_only=True)
class Link:
    name: str
    inertial: Inertial | None = None
    visuals: tuple[Visual, ...] = ()
    collisions: tuple[Collision, ...] = ()
    meta: Mapping[str, str] = field(default_factory=dict)


# Embodiment: what planners and policies condition on


@dataclass(frozen=True, kw_only=True)
class TwistLimits:
    """Per-axis magnitude limits in the base frame: m/s (or m/s^2), rad/s (or rad/s^2).

    None is unconstrained; 0 is an axis the base can't move along (vz, wx, wy on a Go2).
    Saved with explicit units (`<twist unit="m/s,rad/s">`) so wz can't be read as degrees.
    """

    vx: float | None = None
    vy: float | None = None
    vz: float | None = None
    wx: float | None = None
    wy: float | None = None
    wz: float | None = None

    def clamp(self, twist: Twist) -> Twist:
        raise NotImplementedError

    def contains(self, twist: Twist) -> bool:
        raise NotImplementedError


@dataclass(frozen=True, kw_only=True)
class Base:
    """What the base can physically do, independent of any one controller."""

    frame: str = "base_link"
    twist: TwistLimits = field(default_factory=TwistLimits)
    accel: TwistLimits = field(default_factory=TwistLimits)
    # Planner geometry, in `frame`.
    footprint: Box | Cylinder | None = None
    height_clearance: float | None = None  # free space needed above the ground
    width_clearance: float | None = None


# Collections


class Joints(Mapping[str, Joint]):
    """Read-only, in URDF order. Equal when the same joints in the same order."""

    def __init__(self, joints: Iterable[Joint] = ()) -> None:
        raise NotImplementedError

    def __getitem__(self, name: str) -> Joint:
        raise NotImplementedError

    def __iter__(self) -> Iterator[str]:
        raise NotImplementedError

    def __len__(self) -> int:
        raise NotImplementedError

    def select(self, joints: JointSelector) -> Joints:
        """By name or glob; an unmatched name or glob raises."""
        raise NotImplementedError

    @property
    def movable(self) -> Joints:
        """Actuated joints: no fixed, floating, planar or mimic joints."""
        raise NotImplementedError

    @property
    def names(self) -> tuple[str, ...]:
        raise NotImplementedError


class Links(Mapping[str, Link]):
    """Read-only, in URDF order."""

    def __init__(self, links: Iterable[Link] = ()) -> None:
        raise NotImplementedError

    def __getitem__(self, name: str) -> Link:
        raise NotImplementedError

    def __iter__(self) -> Iterator[str]:
        raise NotImplementedError

    def __len__(self) -> int:
        raise NotImplementedError


# Errors


class MergeConflictError(ValueError):
    """Two merged parts both define the same link or joint.

    The message names every clash and the fix, e.g.:

        merge: both parts define link "base_link" (inertial, visuals on each side).
        Prefix one part to keep them apart: arm.prefixed("arm/"), then mount it with
        Transform.from_xyz_rpy("base_link", "arm/base_link").
    """

    def __init__(self, links: tuple[str, ...], joints: tuple[str, ...]) -> None:
        raise NotImplementedError


# Checks


@dataclass(frozen=True)
class LimitViolation:
    joint: str
    quantity: str  # "position", "velocity" or "effort"
    value: float
    limit: float


# The robot


@dataclass(frozen=True, eq=False)
class RobotModel:
    """What the robot is, not where it is (RobotState) or which one (Robot's name).

    Every edit returns a new one. Two models are equal when their canonical URDF is, and
    hash as its sha256 (`digest`), which is also the wire hash.
    """

    links: Links
    joints: Joints
    # The frame this robot's odometry is in. Per robot, like its links: a robot attached
    # as "go2b" reads and publishes it as go2b/odom. Frames outside the model (map, other
    # robots) stay global.
    odom_frame: str = "odom"
    # The URDF's <robot name>, kept for to_urdf. Not an identity and not a namespace.
    urdf_name: str = ""
    # Raw XML of elements we don't model (<gazebo>, <transmission>, ...), kept for to_urdf.
    extras: tuple[str, ...] = ()
    # None for things that don't drive (an arm, a sensor rig).
    base: Base | None = None

    def __eq__(self, other: object) -> bool:
        raise NotImplementedError

    def __hash__(self) -> int:
        raise NotImplementedError

    @property
    def digest(self) -> str:
        """sha256 of the canonical URDF."""
        raise NotImplementedError

    # Construction

    @classmethod
    def root(cls, frame: str = "base_link") -> RobotModel:
        """A single link, for robots and rigs without a URDF."""
        raise NotImplementedError

    @classmethod
    def from_urdf(cls, source: str | Path) -> RobotModel:
        """A path, or XML when the string starts with "<". Mesh uris are kept as written."""
        raise NotImplementedError

    def to_urdf(self, path: str | Path | None = None) -> str:
        """The URDF XML, also written to `path` if given. Lossless: from_urdf gives it back."""
        raise NotImplementedError

    # Tree

    @property
    def root_link(self) -> str:
        raise NotImplementedError

    def chain(self, from_link: str, to_link: str) -> tuple[Joint, ...]:
        """The joints between two links, in order; raises LookupError if unconnected."""
        raise NotImplementedError

    # Kinematics, at the zero pose. A posed robot is a RobotState.

    def get(self, parent: str, child: str) -> Transform:
        """parent <- child, movable joints at 0 clamped into their limits; mounts, mostly.

        Raises LookupError for a frame not in the model.
        """
        raise NotImplementedError

    def tf(self, root: str | None = None) -> TFMessage:
        """The fixed joints as tf edges (the mounts), re-rooted at `root`.

        For bridging to consumers that only read tf (rerun, foxglove); mounts don't go on
        the bus as tf otherwise, readers get them from the model.
        """
        raise NotImplementedError

    def state(
        self, values: JointState | JointValues | None = None, ts: float | None = None
    ) -> RobotState:
        """This robot at a moment, from a JointState or {joint: position}. Joints not given
        are unknown (None), not 0. Detached: no world frames."""
        raise NotImplementedError

    def attach(self, source: RobotSource | TFLookup, name: str = "") -> Robot:
        """This model read from a source: live (a RobotAware module's) or a recording's.

        `name` is which robot on the bus: "go2b" reads go2b/<link>, go2b/<joint> and
        go2b/odom, while the model keeps local names. "" is a single robot, bare names.
        A bare TFLookup is a source with no joint states: fine for a rig.
        """
        raise NotImplementedError

    # Edits

    def with_frames(self, *edges: Transform) -> RobotModel:
        """Place frames from tf edges (frame_id is the parent), overriding what's there.

        A new child becomes an empty link on a fixed joint named `<child>_joint`. An
        existing child is moved: its joint gets the new parent and origin and keeps its
        type, axis and limits. `merge` is the strict version, for joining.
        """
        raise NotImplementedError

    def with_joint(self, joint: Joint) -> RobotModel:
        """Add, or replace by name."""
        raise NotImplementedError

    def map_joints(self, joints: JointSelector, fn: Callable[[Joint], Joint]) -> RobotModel:
        """Replace the selected joints with `fn(joint)`: the typed way to edit many at once.

        go2.map_joints("*", lambda j: replace(j, limits=replace(j.limits, velocity=...)))
        """
        raise NotImplementedError

    def edit(self, path: str, value: object) -> RobotModel:
        """Set any field by path; the value is the new value, or a function of the old one.

            robot.edit("joints.FR_hip_joint.limits.lower", -0.8)
            robot.edit("joints.*_calf_joint.limits.velocity", lambda v: v * 0.5)
            robot.edit("base.twist.vx", 1.0)

        The name may be a glob. An unknown name or field, or a glob matching nothing, raises.
        """
        raise NotImplementedError

    def fix(self, joints: JointSelector, at: JointValues | float = 0.0) -> RobotModel:
        """Turn joints fixed, their position baked into the origin; their mimics follow."""
        raise NotImplementedError

    def without(self, link: str) -> RobotModel:
        """Drop a link and everything below it."""
        raise NotImplementedError

    def rename_frames(self, names: Mapping[str, str]) -> RobotModel:
        """Rename links; joints follow."""
        raise NotImplementedError

    def prefixed(self, prefix: str) -> RobotModel:
        """Every link and joint name with `prefix`: a sub-assembly inside one robot, like
        an arm (`arm/link6`), so its names can't collide with the body's."""
        raise NotImplementedError

    def merge(self, *parts: RobotModel | Transform) -> RobotModel:
        """Join trees on shared frame names; a Transform is a one-edge tree.

            go2.merge(arm.prefixed("arm/"), Transform.from_xyz_rpy("base_link", "arm/link_base"))

        The result must be one tree. A frame that's an empty link on one side joins the
        other's. A link or joint defined with data on both sides raises MergeConflictError,
        suggesting `prefixed()`; a frame given two parents raises too (use `with_frames`
        to override on purpose). `base`, `odom_frame` and `urdf_name` come from the
        leftmost part that has them.
        """
        raise NotImplementedError

    # On the wire, as a topic message: the URDF text in an envelope (robot, part, digest).

    msg_name = "dimos.RobotModel"

    def lcm_encode(self) -> bytes:
        raise NotImplementedError

    @classmethod
    def lcm_decode(cls, data: bytes) -> RobotModel:
        raise NotImplementedError
