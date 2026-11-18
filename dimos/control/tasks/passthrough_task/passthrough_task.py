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

"""Pass-through: a sender's joint position targets, sent to the robot as they came.

A sender streams ``JointState`` samples of position targets, named as the
coordinator names joints ("arm/left_joint2"). Each tick the task outputs the
latest sample unchanged: no rate limit, interpolation or filter, so a step
stays a step and reaches the robot on the next tick. A sample naming a joint
past its position limit, a non-finite position, or names and positions that
do not pair up is refused whole.

When no sample has arrived for ``stale_after_s`` the targets are stale: the
task latches each joint's measured position once and outputs that until the
next sample. ``hold`` (over ``task_invoke``) and an e-stop latch at once.
``stale`` says whether the last output was a latch or nothing, so a
coordinator can send the robot nothing then.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import math
from typing import TYPE_CHECKING, Any

from pydantic import Field

from dimos.control.contract.limits import Limits
from dimos.control.task import (
    BaseControlTask,
    ControlMode,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.protocol.service.spec import BaseConfig
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.control.connection_source import ConnectionSource
    from dimos.control.coordinator import TaskConfig

logger = setup_logger()

PASSTHROUGH_PRIORITY = 50
DEFAULT_STALE_AFTER_S = 0.1
REFUSAL_LOG_PERIOD_S = 1.0


@dataclass(frozen=True)
class PassthroughStatus:
    """What the pass-through does now.

    Attributes:
        stale: No fresh sample: it outputs the latched positions, or nothing.
        joints: The joints the latest sample named.
        refused: Samples refused since it was made.
        estopped: The coordinator's e-stop is latched.
    """

    stale: bool
    joints: tuple[str, ...] = ()
    refused: int = 0
    estopped: bool = False


class PassthroughTask(BaseControlTask):
    """Outputs the latest sample's targets each tick, unchanged; the measured positions once stale.

    Args:
        name: The task's name, which ``task_invoke`` addresses.
        joint_names: The joints it may drive; a sample's other joints are skipped.
        limits: Each joint's position limit.
        priority: Arbitration priority.
        stale_after_s: How long after the last sample its targets go stale.
    """

    def __init__(
        self,
        name: str,
        joint_names: Sequence[str],
        limits: Mapping[str, Limits],
        *,
        priority: int = PASSTHROUGH_PRIORITY,
        stale_after_s: float = DEFAULT_STALE_AFTER_S,
    ) -> None:
        if not joint_names:
            raise ValueError(f"pass-through task {name!r} needs joints")
        if not (math.isfinite(stale_after_s) and stale_after_s > 0.0):
            raise ValueError(f"stale_after_s must be positive seconds, got {stale_after_s}")
        missing = sorted(set(joint_names) - set(limits))
        if missing:
            raise ValueError(f"pass-through task {name!r} has no position limit for {missing}")
        self._name = name
        self._order = list(dict.fromkeys(joint_names))
        self._claim = ResourceClaim(
            joints=frozenset(self._order), priority=priority, mode=ControlMode.POSITION
        )
        self._limits = {joint: limits[joint] for joint in self._order}
        self._stale_after = stale_after_s
        self._targets: dict[str, float] = {}
        self._received_at: float | None = None
        self._latch: dict[str, float] | None = None
        self._measured: dict[str, float] = {}
        self._stale = True
        self._estopped = False
        self._refused = 0
        self._last_logged = float("-inf")

    @property
    def stale(self) -> bool:
        """Whether the last output was a latch or nothing, or ``hold`` came since."""
        return self._stale

    def claim(self) -> ResourceClaim:
        return self._claim

    def is_active(self) -> bool:
        return True

    def on_joint_command(self, msg: JointState, t_now: float) -> bool:
        """Take one sample of position targets; False when refused or naming none of its joints."""
        names, positions = list(msg.name), list(msg.position)
        if self._estopped:
            return self._refuse("the e-stop is latched", t_now)
        if not positions or len(names) != len(positions):
            return self._refuse(f"{len(names)} names for {len(positions)} positions", t_now)
        if not all(math.isfinite(position) for position in positions):
            return self._refuse(f"a position is not finite: {positions}", t_now)
        targets = {
            name: float(position)
            for name, position in zip(names, positions, strict=True)
            if name in self._limits
        }
        if not targets:
            return False
        past = [
            f"{name} {value:.4f} outside {self._limits[name].lo}..{self._limits[name].hi}"
            for name, value in targets.items()
            if not _within(value, self._limits[name])
        ]
        if past:
            return self._refuse(f"past a position limit: {'; '.join(past)}", t_now)
        self._targets = targets
        self._received_at = t_now
        self._latch = None
        return True

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        """The latest sample while fresh, else each joint as measured when it went stale."""
        positions = state.joints.joint_positions
        self._measured = {
            joint: positions[joint]
            for joint in self._order
            if joint in positions and math.isfinite(positions[joint])
        }
        if self._received_at is None:
            self._stale = True
            return None
        if self._latch is None and state.t_now - self._received_at > self._stale_after:
            self._latch = self._measured_targets()
        self._stale = self._latch is not None
        output = self._targets if self._latch is None else self._latch
        names = [joint for joint in self._order if joint in output]
        if not names:
            return None
        return JointCommandOutput(
            joint_names=names,
            positions=[output[joint] for joint in names],
            mode=ControlMode.POSITION,
        )

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        logger.error(
            "Pass-through lost joints to a higher-priority task",
            task_name=self._name,
            by_task=by_task,
            joints=sorted(joints),
        )

    def hold(self) -> PassthroughStatus:
        """Latch the latest sample's joints where they are measured now, until the next sample."""
        self._latch = self._measured_targets()
        self._stale = True
        return self.get_status()

    def set_estop(self, estopped: bool) -> None:
        """Latch on a stop and refuse samples until it is cleared; the next sample after goes on."""
        if estopped:
            self.hold()
        self._estopped = estopped

    def get_status(self) -> PassthroughStatus:
        return PassthroughStatus(
            stale=self._stale,
            joints=tuple(joint for joint in self._order if joint in self._targets),
            refused=self._refused,
            estopped=self._estopped,
        )

    def _measured_targets(self) -> dict[str, float]:
        return {joint: self._measured[joint] for joint in self._targets if joint in self._measured}

    def _refuse(self, reason: str, t_now: float) -> bool:
        self._refused += 1
        if t_now - self._last_logged >= REFUSAL_LOG_PERIOD_S:
            self._last_logged = t_now
            logger.warning(f"Pass-through {self._name} refused a sample: {reason}")
        return False


def _within(value: float, limit: Limits) -> bool:
    return (limit.lo is None or value >= limit.lo) and (limit.hi is None or value <= limit.hi)


class PassthroughTaskParams(BaseConfig):
    """Task-owned parameters from ``TaskConfig.params``."""

    stale_after_s: float = Field(default=DEFAULT_STALE_AFTER_S, gt=0.0, allow_inf_nan=False)


def create_task(
    cfg: TaskConfig,
    hardware: Any,
    connections: Mapping[str, ConnectionSource] | None = None,
) -> PassthroughTask:
    """A pass-through over joints of robots driven through connection modules, with their limits."""
    params = PassthroughTaskParams.model_validate(cfg.params)
    limits: dict[str, Limits] = {}
    for joint in cfg.joint_names:
        source = next(
            (source for source in (connections or {}).values() if joint in source.joint_names),
            None,
        )
        if source is None:
            raise ValueError(
                f"pass-through joint {joint!r} is on no robot driven through a connection module"
            )
        limits[joint] = source.position_limit(joint)
    return PassthroughTask(
        cfg.name,
        cfg.joint_names,
        limits,
        priority=cfg.priority,
        stale_after_s=params.stale_after_s,
    )
