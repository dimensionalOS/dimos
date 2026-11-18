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
energized: these are for calibration tools only, never for normal operation.

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
        kp: The position gain it is held with when not floating.
        kd: The velocity gain it is held with when not floating.
        floating: Whether it floats: zero stiffness, gravity still compensated.
        float_kd: The damping it floats with, or ``None`` when not floating.
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


@runtime_checkable
class ArmTuningHooks(Protocol):
    """The live tuning a calibration procedure drives, for calibration tools only.

    Each call changes motor gains or torques of an energized arm at once.
    Gains are clamped into the motor driver's range and torques to the
    motor's peak; non-finite values and unknown joints are refused with
    ``ValueError``.
    """

    def tuning_arm_ids(self) -> list[str]:
        """The calibration ids of the connection's arms, e.g. "dual_openyam/left"."""
        ...

    def tuning_joint_names(self) -> list[str]:
        """The arm joints whose gains can be tuned, in motor order."""
        ...

    def tuning_get_gains(self) -> dict[str, JointGainsState]:
        """How each arm joint is driven now."""
        ...

    def tuning_set_gains(self, joint: str, kp: float, kd: float) -> None:
        """Hold ``joint`` with these gains until reset."""
        ...

    def tuning_reset_gains(self) -> None:
        """Drop every gain set with ``tuning_set_gains``: back to the calibration, else the defaults."""
        ...

    def tuning_set_float(self, joint: str, on: bool, kd: float | None = None) -> None:
        """Let ``joint`` float with ``kd`` damping, or hold it again where it is."""
        ...

    def tuning_set_feedforward(self, joint: str, tau_nm: float) -> None:
        """Add ``tau_nm`` to ``joint``'s gravity torque; 0 removes it."""
        ...

    def tuning_read(self) -> dict[str, JointSample]:
        """The latest reading of every joint."""
        ...

    def tuning_command(self, targets: dict[str, float]) -> None:
        """Send position targets by joint, checked and deadman-guarded like any command.

        Raises ``ValueError`` when the command is refused.
        """
        ...

    def tuning_apply_calibration(
        self, path: str | None = None
    ) -> dict[str, AppliedCalibration | None]:
        """Apply a calibration now: each arm's saved file (``None``), one file, or none ("off").

        Returns what each arm runs afterwards, by arm id.
        """
        ...
