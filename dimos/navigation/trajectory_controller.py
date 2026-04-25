# Copyright 2025-2026 Dimensional Inc.
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

"""Trajectory controller interface (P1-2).

A controller maps one timed **reference** sample and one **measured** sample
to a body-frame ``cmd_vel`` ``Twist`` (see ``trajectory_types`` for frame
notes). Implementations are pluggable: planners or runners hold a
``TrajectoryController`` without depending on a concrete class.

``reset`` is called on discontinuities (new path, pause, re-arm) at the
integration point; it is a separate concern from
``trajectory_command_limits.clamp_holonomic_cmd_vel``, which runs every tick
with the last published command.
"""

from __future__ import annotations

from typing import Protocol, runtime_checkable

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.navigation.trajectory_command_limits import HolonomicCommandLimits
from dimos.navigation.trajectory_types import TrajectoryMeasuredSample, TrajectoryReferenceSample


@runtime_checkable
class TrajectoryController(Protocol):
    """(reference, measurement) -> body-frame command, plus explicit reset."""

    def control(
        self,
        reference: TrajectoryReferenceSample,
        measurement: TrajectoryMeasuredSample,
    ) -> Twist: ...

    def reset(self) -> None: ...


@runtime_checkable
class ConfigurableTrajectoryController(TrajectoryController, Protocol):
    """Optional: wire ``HolonomicCommandLimits`` into a limit-aware or tunable implementation."""

    def configure(self, limits: HolonomicCommandLimits) -> None: ...


__all__ = [
    "ConfigurableTrajectoryController",
    "TrajectoryController",
]
