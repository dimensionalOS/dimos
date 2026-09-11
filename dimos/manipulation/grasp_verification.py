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

"""Gripper feedback polling and grasp verification.

Gripper readback is measured, not an echo of the command (see
``GripperTask.get_normalized``), so jaws stalled on an object report a position
short of the one commanded. That difference is the whole grasp signal: settle
the readback, then read where it stopped.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import time

from pydantic import Field, model_validator

from dimos.protocol.service.spec import BaseConfig


class GraspVerificationConfig(BaseConfig):
    """Per-robot gripper feedback thresholds.

    Positions are normalized gripper travel as ``GripperTask`` reports it:
    0.0 fully closed, 1.0 fully open.

    Calibrating ``empty_epsilon`` for a new gripper: close on empty air and
    record the settled readback (``e``), then close on the smallest object the
    robot must pick and record that settle (``h``). Put ``empty_epsilon``
    between the two, biased toward ``e``; their midpoint is a good start. On
    the xArm7 sim ``e`` is 0.02 to 0.04 and a 6cm block gives ``h`` of 0.62, so
    the 0.10 default clears both by a wide margin. ``open_margin`` only needs
    the same treatment if the jaws rest short of the open command.
    """

    enabled: bool = True
    open_position: float = Field(default=1.0, ge=0.0, le=1.0)
    closed_position: float = Field(default=0.0, ge=0.0, le=1.0)
    # Readback must clear closed by more than this to mean "something is in the way".
    empty_epsilon: float = Field(default=0.10, gt=0.0)
    # Below open by less than this means the jaws never travelled.
    open_margin: float = Field(default=0.05, gt=0.0)
    # How far short of open_position a settled open may stop and still count as
    # open. A healthy reopen lands within 0.06 on the xArm7 sim; jaws still
    # holding something stop far below this.
    open_tolerance: float = Field(default=0.15, gt=0.0)
    timeout: float = Field(default=3.0, gt=0.0)
    # Each poll is a blocking RPC and the measurement only refreshes on the
    # control tick, so polling faster buys resolution the gripper cannot supply.
    poll_interval: float = Field(default=0.05, gt=0.0)
    settle_tolerance: float = Field(default=0.005, gt=0.0)
    settle_samples: int = Field(default=3, ge=2)

    @model_validator(mode="after")
    def _validate_band(self) -> GraspVerificationConfig:
        if self.closed_position >= self.open_position:
            raise ValueError("closed_position must be below open_position")
        if self.held_low >= self.held_high:
            raise ValueError(
                "empty_epsilon and open_margin leave no held band between "
                f"closed_position={self.closed_position} and "
                f"open_position={self.open_position}"
            )
        if self.poll_interval > self.timeout:
            raise ValueError("poll_interval must not exceed timeout")
        return self

    @property
    def held_low(self) -> float:
        """Readback above this cleared the empty-close region."""
        return self.closed_position + self.empty_epsilon

    @property
    def held_high(self) -> float:
        """Readback below this travelled off the open stop."""
        return self.open_position - self.open_margin


@dataclass(frozen=True)
class GripperSettle:
    """Outcome of polling a gripper until its readback stops changing."""

    settled: bool
    position: float | None
    moved: bool
    elapsed: float


def await_gripper_settle(
    read: Callable[[], float | None],
    target: float,
    config: GraspVerificationConfig,
    *,
    arrival_tolerance: float | None = None,
    sleep: Callable[[float], None] = time.sleep,
    clock: Callable[[], float] = time.monotonic,
) -> GripperSettle:
    """Poll ``read`` until the gripper stops moving or the deadline expires.

    Settling requires ``settle_samples`` consecutive stationary readings *and*
    either observed travel or arrival at ``target``. Without that second
    condition a poll started before the jaws react would call the pre-command
    position settled. ``sleep`` and ``clock`` are injectable for tests.

    ``arrival_tolerance`` answers a different question from ``settle_tolerance``:
    stillness is how little the jaws moved between polls, arrival is how near the
    target they stopped. It defaults to ``settle_tolerance`` because a close is
    meant to stop short, on the object, and only an exact arrival at
    ``closed_position`` means empty jaws. An open is the opposite: a gripper
    driven to its mechanical stop always halts short of the commanded extreme --
    the xArm rests at 0.988 of a nominal 850 count -- so the open path must pass
    the band that decides whether where the jaws stopped counts as open.
    """
    reached = config.settle_tolerance if arrival_tolerance is None else arrival_tolerance
    start = clock()
    deadline = start + config.timeout
    first: float | None = None
    last: float | None = None
    stable = 0
    moved = False

    while True:
        position = read()
        if position is not None:
            if first is None:
                first = position
            if last is not None and abs(position - last) <= config.settle_tolerance:
                stable += 1
            else:
                stable = 1
            if abs(position - first) > config.settle_tolerance:
                moved = True
            last = position
            arrived = abs(position - target) <= reached
            if stable >= config.settle_samples and (moved or arrived):
                return GripperSettle(True, position, moved, clock() - start)

        now = clock()
        if now >= deadline:
            return GripperSettle(False, last, moved, now - start)
        sleep(min(config.poll_interval, deadline - now))


def grasp_failure(settle: GripperSettle, config: GraspVerificationConfig) -> str | None:
    """Why a settled close is not a grasp, or None when an object is held."""
    if settle.position is None:
        return "gripper readback unavailable"
    reading = f"readback {settle.position:.3f}"
    if not settle.settled:
        return f"gripper did not settle within {config.timeout:.2f}s ({reading})"
    if settle.position <= config.held_low:
        return f"nothing in the jaws ({reading}, at or below {config.held_low:.3f})"
    if settle.position >= config.held_high:
        return f"jaws never closed ({reading}, at or above {config.held_high:.3f})"
    return None


def open_failure(settle: GripperSettle, config: GraspVerificationConfig) -> str | None:
    """Why a settled open did not reach the open position, or None when open."""
    if settle.position is None:
        return "gripper readback unavailable"
    reading = f"readback {settle.position:.3f}"
    if not settle.settled:
        return f"gripper did not settle within {config.timeout:.2f}s ({reading})"
    shortfall = config.open_position - settle.position
    if shortfall > config.open_tolerance:
        return f"jaws settled {shortfall:.3f} short of open ({reading})"
    return None
