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

"""The arm config contract: what a calibration tool reads and changes on a running arm.

A connection says what it can change in :class:`ConfigKeys`, reports how each
arm is driven now in :class:`ArmConfigState`, and takes changes as a
:class:`ConfigPatch`: only what a patch gives changes. A patch applies for the
config session (``"session"``: undone when the session ends) or is saved to
the arm's calibration file and applied from it (``"persist"``). What took
effect comes back as :class:`ConfigApplied`.

Plain pydantic models, so they cross processes over RPC and import without
any motor SDK or pinocchio. Joints are named as in the robot's model, without
the connection's source prefix, e.g. "left_joint2"; gains are kp in N m/rad
and kd in N m s/rad, torques in N m.
"""

from __future__ import annotations

from typing import Annotated, Any, Literal

from pydantic import Field

from dimos.protocol.service.spec import BaseConfig

__all__ = [
    "DEFAULT_CONFIG_LEASE_S",
    "MAX_CONFIG_LEASE_S",
    "AppliedFile",
    "ArmConfigState",
    "CommandResult",
    "ConfigApplied",
    "ConfigKeys",
    "ConfigPatch",
    "ConfigScope",
    "FiniteFloat",
    "JointGains",
    "LinkGravity",
    "NonNegativeFloat",
    "PositiveFloat",
]

PositiveFloat = Annotated[float, Field(gt=0.0, allow_inf_nan=False)]
NonNegativeFloat = Annotated[float, Field(ge=0.0, allow_inf_nan=False)]
FiniteFloat = Annotated[float, Field(allow_inf_nan=False)]

#: The longest lease a config session may ask for, seconds.
MAX_CONFIG_LEASE_S = 30.0
#: The lease a config session gets unless it asks for another, seconds.
DEFAULT_CONFIG_LEASE_S = 2.0

#: ``"session"``: live until the config session ends. ``"persist"``: saved to
#: the arm's calibration file, then applied from it.
ConfigScope = Literal["session", "persist"]


class JointGains(BaseConfig):
    """How stiffly one joint is held: kp in N m/rad, kd in N m s/rad."""

    kp: NonNegativeFloat
    kd: NonNegativeFloat


class LinkGravity(BaseConfig):
    """Fitted mass and centre of mass of one link's body, centre of mass in the link frame.

    Attributes:
        mass_kg: The body's mass, including ``merged_links``.
        com_m: The body's centre of mass, link frame, metres.
        merged_links: Links whose URDF inertia the body includes.
    """

    mass_kg: PositiveFloat
    com_m: tuple[FiniteFloat, FiniteFloat, FiniteFloat]
    merged_links: list[str] = Field(default_factory=list)


class ConfigPatch(BaseConfig):
    """A change to how one arm is driven; only what is given changes.

    Attributes:
        arm_id: The arm, e.g. "dual_openyam/left".
        gains: By joint, merged per joint. kp 0 floats the joint with kd
            damping, gravity still held (session only).
        torque_offset_nm: By joint, added to its gravity torque; 0 removes it
            (session only).
        links: Fitted link masses; replace the stored set as a whole (persist
            only).
        fit: The fit's statistics, stored with the links (persist only).
        friction_nm: Coulomb friction by joint; stored, not applied (persist
            only).
        checks: Validation results by name, merged by name (persist only).
        notes: Free text for the operator (persist only).
    """

    arm_id: str
    gains: dict[str, JointGains] | None = None
    torque_offset_nm: dict[str, FiniteFloat] | None = None
    links: dict[str, LinkGravity] | None = None
    fit: dict[str, Any] | None = None
    friction_nm: dict[str, NonNegativeFloat] | None = None
    checks: dict[str, Any] | None = None
    notes: str | None = None

    def keys(self) -> list[str]:
        """Every value the patch gives, dotted, e.g. "gains.left_joint2.kp", "links.link3".

        A field given as an empty mapping is named alone, e.g. "links".
        """
        out: list[str] = []
        if self.gains is not None:
            out += [f"gains.{joint}.{gain}" for joint in self.gains for gain in ("kp", "kd")]
            out += [] if self.gains else ["gains"]
        for name in ("torque_offset_nm", "links", "friction_nm", "checks"):
            value = getattr(self, name)
            if value is not None:
                out += [f"{name}.{key}" for key in value] or [name]
        if self.fit is not None:
            out.append("fit")
        if self.notes is not None:
            out.append("notes")
        return out


class ConfigApplied(BaseConfig):
    """What one ``config_set`` did.

    Attributes:
        arm_id: The arm.
        scope: As asked.
        applied: The values as they took effect, after clamping.
        clamped: Keys whose value was clamped.
        unsupported: Keys the connection cannot change in this scope; they
            changed nothing.
        why: By key in ``unsupported``, why not.
        session_id: The config session the change belongs to, if one is live.
        lease_s: That session's lease, seconds.
        path: The calibration file written (persist).
        applied_now: Whether the saved file was applied to the arm at once
            (persist).
        refused: Why nothing was written: applying it would step a joint's
            gravity torque too far at this pose (persist).
        gravity_step_nm: By joint, how much its gravity torque at the
            measured pose changed, or would have (persist).
    """

    arm_id: str
    scope: ConfigScope
    applied: ConfigPatch
    clamped: list[str] = Field(default_factory=list)
    unsupported: list[str] = Field(default_factory=list)
    why: dict[str, str] = Field(default_factory=dict)
    session_id: str | None = None
    lease_s: float | None = None
    path: str | None = None
    applied_now: bool = True
    refused: str | None = None
    gravity_step_nm: dict[str, float] = Field(default_factory=dict)


class AppliedFile(BaseConfig):
    """The calibration file an arm runs, as it was read.

    Attributes:
        path: The file.
        created: When it was written, ISO 8601, UTC. A string, since the RPC
            codec carries no datetime.
        sha256: The file's bytes as read, hex; a file rewritten since reads
            differently.
    """

    path: str
    created: str
    sha256: str


class ArmConfigState(BaseConfig):
    """How one arm is driven now.

    Attributes:
        arm_id: The arm.
        gains: Every arm joint's holding gains: a session override, else the
            file's, else the robot's defaults.
        floating: Joints at kp 0, with the damping each floats with.
        torque_offset_nm: Torques added to the gravity torque; non-zero only.
        links: Links whose masses differ from the URDF, as the arm runs them.
        file: The calibration file it runs, or ``None`` for its defaults.
        session_id: The live config session, if any.
    """

    arm_id: str
    gains: dict[str, JointGains]
    floating: dict[str, float] = Field(default_factory=dict)
    torque_offset_nm: dict[str, float] = Field(default_factory=dict)
    links: dict[str, LinkGravity] = Field(default_factory=dict)
    file: AppliedFile | None = None
    session_id: str | None = None


class ConfigKeys(BaseConfig):
    """What a connection lets a calibration tool read and change.

    Attributes:
        source: The connection's robot name, the prefix of its joint state
            names ("{source}/{joint}").
        sessions: Whether it serves config sessions (``config_sessions``);
            without them nothing can be set.
        arms: Arm id to its joints, in motor order.
        readable: What ``config_get`` reports, e.g. ["gains", "links",
            "torque_offset_nm"].
        session: What a session can change, e.g. ["gains", "float",
            "torque_offset_nm"]; "float" is a gain with kp 0.
        persist: What can be saved, e.g. ["gains", "links", "fit",
            "friction_nm", "checks", "notes"].
        persist_paths: Arm id to the calibration file ``persist`` writes.
        gravity_compensation: Whether the connection holds the arm's weight up.
        max_lease_s: The longest lease a session may ask for, seconds.
    """

    source: str
    sessions: bool
    arms: dict[str, list[str]] = Field(default_factory=dict)
    readable: list[str] = Field(default_factory=list)
    session: list[str] = Field(default_factory=list)
    persist: list[str] = Field(default_factory=list)
    persist_paths: dict[str, str] = Field(default_factory=dict)
    gravity_compensation: bool = False
    max_lease_s: float = MAX_CONFIG_LEASE_S


class CommandResult(BaseConfig):
    """Whether position targets reached the motors.

    Attributes:
        ok: The targets were sent.
        error: Why they were not, else ``None``.
    """

    ok: bool
    error: str | None = None
