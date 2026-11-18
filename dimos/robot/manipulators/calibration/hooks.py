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

"""What a calibration procedure needs from a running arm connection.

A connection that implements :class:`ArmTuningHooks` lets a calibration tool
change motor gains live, float a joint, add a feed-forward torque, read the
measured state and step the arm through poses. Every method is prefixed
``tuning_`` because it changes how the motors are driven while they are
energized: these are for calibration tools only, never for normal operation,
and a connection refuses them unless it was deployed for calibration.

The tool owns the arm inside a tuning session (:meth:`ArmTuningHooks.
tuning_begin`): while it lasts, the connection ignores every other position
command, and every tuning call renews its lease. When it ends, for any
reason, every live change is undone: floating joints are held where they
are, feed-forward torques are dropped and gains go back to the applied
calibration, else the robot's defaults.

Gains are those of a Damiao MIT frame: kp in N m/rad and kd in N m s/rad,
torques in N m. The frame carries kp 0 to 500 and kd 0 to 5; those ceilings
are the range of its fields, not a choice, and a larger gain is clamped to
them.

Arguments and results are plain numbers, strings, lists and dicts (the
``TypedDict`` shapes below), so the hooks work the same over RPC. Joints are
named as in the robot's model, without the connection's source prefix, e.g.
"left_joint2".
"""

from __future__ import annotations

from typing import Protocol, TypedDict, runtime_checkable


class JointGainsState(TypedDict):
    """How one arm joint is driven now.

    Attributes:
        kp: The position gain it is held with when not floating, N m/rad.
        kd: The velocity gain it is held with when not floating, N m s/rad.
        floating: Whether it floats: zero stiffness, gravity still compensated.
        float_kd: The damping it floats with, N m s/rad, or ``None`` when not
            floating.
        feedforward_nm: The torque added to its gravity torque, N m.
    """

    kp: float
    kd: float
    floating: bool
    float_kd: float | None
    feedforward_nm: float


class JointSample(TypedDict):
    """One joint in the connection's latest reading.

    Attributes:
        position: Measured position, rad (m for a sliding finger).
        velocity: Measured velocity.
        effort: The torque the motor reports it produces, N m.
        ts: When the reading was taken, Unix seconds.
    """

    position: float
    velocity: float
    effort: float
    ts: float


class AppliedCalibration(TypedDict):
    """The calibration file one arm runs.

    Attributes:
        path: The file.
        created: When it was written, ISO 8601, UTC.
        gains: The joints it gave gains to.
        links: The links it gave masses to.
    """

    path: str
    created: str
    gains: list[str]
    links: list[str]


class LinkMass(TypedDict):
    """One link's mass and centre of mass as an arm runs it.

    Attributes:
        mass_kg: Its mass.
        com_m: Its centre of mass in the link frame, m.
    """

    mass_kg: float
    com_m: list[float]


class AppliedInertials(TypedDict):
    """The calibration file one arm runs and the link masses it gave the arm.

    Attributes:
        path: The file, as it was read.
        created: When it was written, ISO 8601, UTC.
        sha256: The file's bytes as read, hex; a file rewritten since reads
            differently.
        links: The masses the arm runs, by link; only the links the file set.
    """

    path: str
    created: str
    sha256: str
    links: dict[str, LinkMass]


class TuningSession(TypedDict):
    """A tuning session that has just begun.

    Attributes:
        session_id: Names the session in the connection's log.
        lease_s: How long it lasts without a tuning call, seconds.
    """

    session_id: str
    lease_s: float


class CommandResult(TypedDict):
    """Whether position targets reached the motors.

    Attributes:
        ok: The targets were sent.
        error: Why they were not, else ``None``.
    """

    ok: bool
    error: str | None


class CalibrationApplied(TypedDict):
    """A calibration that was just applied.

    Attributes:
        arms: What each arm runs now, by arm id; ``None`` for its defaults.
        gravity_step_nm: By arm joint, how much its gravity torque at the
            measured pose changed, N m.
    """

    arms: dict[str, AppliedCalibration | None]
    gravity_step_nm: dict[str, float]


@runtime_checkable
class ArmTuningHooks(Protocol):
    """The live tuning a calibration procedure drives, for calibration tools only.

    Each call changes motor gains or torques of an energized arm at once.
    Gains are clamped into the MIT frame's range and torques as each method
    says; non-finite values and unknown joints are refused with
    ``ValueError``, a value of the wrong type with ``TypeError``. Any tuning
    call that raises ends the tuning session.
    """

    def tuning_begin(self, lease_s: float = 2.0) -> TuningSession:
        """Own the arm until ``tuning_end``, or until ``lease_s`` passes without a tuning call.

        While the session lasts, every other position command is ignored. It
        also ends when the connection halts (the command deadman included),
        faults or stops. Raises when a session is already active.
        """
        ...

    def tuning_end(self) -> None:
        """End the session, undoing every live change; nothing happens without one."""
        ...

    def tuning_arm_ids(self) -> list[str]:
        """The calibration ids of the connection's arms, e.g. "dual_openyam/left"."""
        ...

    def tuning_joint_names(self) -> list[str]:
        """The arm joints whose gains can be tuned, in motor order."""
        ...

    def tuning_get_gains(self) -> dict[str, JointGainsState]:
        """How each arm joint is driven now."""
        ...

    def tuning_set_gains(self, joint: str, kp: float, kd: float) -> JointGainsState:
        """Hold ``joint`` with these gains for the session; returns them as applied.

        Needs a session.
        """
        ...

    def tuning_reset_gains(self) -> None:
        """Drop every gain set with ``tuning_set_gains``: back to the calibration, else the defaults."""
        ...

    def tuning_set_float(self, joint: str, on: bool, kd: float | None = None) -> JointGainsState:
        """Let ``joint`` float with ``kd`` damping, or hold it again where it is.

        A floating joint has no stiffness, so this is refused on a robot that
        does not compensate gravity. Needs a session.
        """
        ...

    def tuning_set_feedforward(self, joint: str, tau_nm: float) -> JointGainsState:
        """Add ``tau_nm`` to ``joint``'s gravity torque; 0 removes it. Needs a session.

        For friction-sized terms only: clamped to a fifth of the motor's rated
        torque. Returns the joint's state with the torque as applied.
        """
        ...

    def tuning_reset_all(self) -> None:
        """Undo every live change, as ending the session does, and keep the session."""
        ...

    def tuning_read(self) -> dict[str, JointSample]:
        """The latest reading of every joint."""
        ...

    def tuning_command(self, targets: dict[str, float]) -> CommandResult:
        """Send position targets by joint, checked and deadman-guarded like any command.

        Needs a session. Targets past a joint limit or of an unknown joint
        raise ``ValueError``; a command the motors did not get is reported as
        not ok.
        """
        ...

    def tuning_apply_calibration(
        self, path: str | None = None, force: bool = False
    ) -> CalibrationApplied:
        """Apply a calibration now: each arm's saved file (``None``), one file, or none ("off").

        One file changes only the arm it names. Apply near home: a change of
        more than 2 N m in any joint's gravity torque is refused unless
        ``force``.
        """
        ...

    def tuning_applied_calibration(self) -> dict[str, AppliedInertials | None]:
        """What each arm runs now, by arm id: its file and link masses, ``None`` for the URDF.

        The masses are those the connection compensates gravity with, read
        when the file was applied; the file on disk may have changed since.
        """
        ...
