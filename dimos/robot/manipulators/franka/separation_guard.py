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

"""Keep the two FR3s of a tabletop-clamped pair from touching each other.

The planner already keeps plans apart, but commands need not come from it: a
policy or teleop can drive the coordinator directly. This sits between the
coordinator and the two FrankaConnections and passes on a joint target only if,
with the other arm at its own latest target, two gaps stay open:

- the grippers (each Franka Hand and its fingers) at least ``gripper_separation_m``
  (5 mm) from each other, so the hands can meet to hand an object over, at the
  risk of a light bump;
- everything else (the arm links, wrist included, and the wrist cameras) at least
  ``arm_separation_m`` (5 cm) from anything on the other arm.

A target that breaks either is dropped, so that arm stops at the last target it
was given (the connection holds it). A target that widens a too-small gap still
passes, so a stopped arm can be backed out.
"""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import TYPE_CHECKING, Any

from pydantic import Field

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.franka.tabletop_bimanual import (
    SIDES,
    arm_joint_names,
    link,
    make_tabletop_bimanual_model_config,
    wrist_camera_link,
)
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.manipulation.planning.world.drake_world import DrakeWorld

logger = setup_logger()

# An arm's gripper, and the rest of it that has collision geometry.
_GRIPPER = ("hand", "leftfinger", "rightfinger")
_REST = tuple(f"link{i}" for i in range(8))
# Pairs further apart than this are not worth measuring, metres.
_MEASURE_WITHIN = 0.3


class SeparationGuardConfig(ModuleConfig):
    # Least clearance from the arm links and wrist cameras to anything on the other
    # arm, metres. The arm-to-arm calibration is good to about 1 cm in the shared
    # middle of the table.
    arm_separation_m: float = Field(default=0.05, gt=0.0)
    # Least clearance between the two grippers, metres: small enough for a handover.
    gripper_separation_m: float = Field(default=0.005, ge=0.0)
    # Seconds between repeats of the same refusal in the log.
    log_interval_s: float = Field(default=1.0, gt=0.0)


@dataclass(frozen=True)
class Gaps:
    """Clearances between the two arms, metres; negative where they overlap."""

    grippers: float
    arms: float

    @property
    def least(self) -> float:
        return min(self.grippers, self.arms)

    def slack(self, config: SeparationGuardConfig) -> float:
        """How far inside both allowances, metres; negative if either is broken."""
        return min(self.grippers - config.gripper_separation_m, self.arms - config.arm_separation_m)


class SeparationGuard(Module):
    """Pass each arm's joint targets on only while the arms stay apart.

    Wire ``<side>_request`` from the coordinator's ``<side>_position_command``,
    ``<side>_command`` into that arm's FrankaConnection ``position_command``, and
    ``<side>_joint_state`` from that connection's ``joint_state``.
    """

    config: SeparationGuardConfig

    right_request: In[JointState]
    left_request: In[JointState]
    right_joint_state: In[JointState]
    left_joint_state: In[JointState]
    right_command: Out[JointState]
    left_command: Out[JointState]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._joints = {side: arm_joint_names(side) for side in SIDES}
        # Each arm's latest target that was passed on, else its latest reading.
        self._target: dict[str, list[float] | None] = {side: None for side in SIDES}
        self._measured: dict[str, list[float] | None] = {side: None for side in SIDES}
        self._refused = {side: 0 for side in SIDES}
        self._last_log = {side: 0.0 for side in SIDES}
        self._last_gaps: Gaps | None = None
        self._world: DrakeWorld | None = None
        self._ctx: Any = None
        self._config = make_tabletop_bimanual_model_config()

    @rpc
    def start(self) -> None:
        self._load_world()
        super().start()
        logger.info(
            f"SeparationGuard: keeping the arms {self.config.arm_separation_m * 100:.0f} cm "
            f"and the grippers {self.config.gripper_separation_m * 1000:.0f} mm apart"
        )

    def _load_world(self) -> None:
        from dimos.manipulation.planning.spec.validation import prepare_robot_model
        from dimos.manipulation.planning.world.drake_world import DrakeWorld

        world = DrakeWorld(enable_viz=False)
        world.load_model(prepare_robot_model(self._config))
        world.finalize()
        self._gripper = {side: [link(side, name) for name in _GRIPPER] for side in SIDES}
        self._rest = {
            side: [*(link(side, name) for name in _REST), wrist_camera_link(side)] for side in SIDES
        }
        with self._lock:
            self._world, self._ctx = world, world.get_live_context()

    async def handle_right_joint_state(self, state: JointState) -> None:
        self._on_state("right", state)

    async def handle_left_joint_state(self, state: JointState) -> None:
        self._on_state("left", state)

    async def handle_right_request(self, command: JointState) -> None:
        self._on_request("right", command)

    async def handle_left_request(self, command: JointState) -> None:
        self._on_request("left", command)

    @rpc
    def status(self) -> dict[str, object]:
        """The latest clearances checked (m) and how many targets each arm had refused."""
        with self._lock:
            gaps = self._last_gaps
            return {
                "gripper_clearance_m": gaps.grippers if gaps else float("nan"),
                "arm_clearance_m": gaps.arms if gaps else float("nan"),
                "refused": dict(self._refused),
            }

    def _on_state(self, side: str, state: JointState) -> None:
        q = _pick(state, self._joints[side])
        if q is not None:
            with self._lock:
                self._measured[side] = q

    def _on_request(self, side: str, command: JointState) -> None:
        other = "left" if side == "right" else "right"
        q_new = _pick(command, self._joints[side])
        with self._lock:
            if q_new is None:  # Only the Hand's finger: no arm motion to check.
                self._send(side, command)
                return
            q_other = self._target[other] or self._measured[other]
            q_now = self._target[side] or self._measured[side]
            if self._world is None or q_other is None:
                self._refuse(side, "the other arm's position is not known yet")
                return
            after = self._gaps(side, q_new, q_other)
            self._last_gaps = after
            if after.slack(self.config) >= 0.0:
                self._pass(side, q_new, command)
                return
            before = self._gaps(side, q_now, q_other) if q_now is not None else None
            # Inside an allowance but apart: a target that widens the gap passes, so the
            # arm can back out. Overlapping in the model (only if the arms were pushed or
            # the calibration is off): overlap depth says nothing of direction, so only a
            # target clear of the other arm passes.
            if before is not None and (
                (
                    before.least >= 0.0
                    and after.least >= 0.0
                    and after.slack(self.config) > before.slack(self.config)
                )
                or (before.least < 0.0 <= after.least)
            ):
                self._pass(side, q_new, command)
                return
            self._refuse(
                side,
                f"the grippers would be {after.grippers * 1000:.0f} mm apart "
                f"(at least {self.config.gripper_separation_m * 1000:.0f} mm) and the rest of "
                f"the arms {after.arms * 100:.1f} cm (at least "
                f"{self.config.arm_separation_m * 100:.0f} cm)",
            )

    def _pass(self, side: str, q: list[float], command: JointState) -> None:
        self._target[side] = q
        self._send(side, command)

    def _send(self, side: str, command: JointState) -> None:
        (self.right_command if side == "right" else self.left_command).publish(command)

    def _refuse(self, side: str, why: str) -> None:
        self._refused[side] += 1
        now = time.monotonic()
        if now - self._last_log[side] >= self.config.log_interval_s:
            self._last_log[side] = now
            logger.warning(f"SeparationGuard: holding the {side} arm: {why}")

    def _gaps(self, side: str, q_side: list[float], q_other: list[float]) -> Gaps:
        other = "left" if side == "right" else "right"
        positions = {side: q_side, other: q_other}
        state = JointState(
            name=[name for s in SIDES for name in self._joints[s]],
            position=[v for s in SIDES for v in positions[s]],
        )
        world = self._world
        assert world is not None
        world.set_joint_state(self._ctx, state)

        def gap(a: list[str], b: list[str]) -> float:
            return world.get_min_distance_between(self._ctx, a, b, _MEASURE_WITHIN)

        g, r = self._gripper, self._rest
        return Gaps(
            grippers=gap(g["right"], g["left"]),
            arms=min(gap(r["right"], [*r["left"], *g["left"]]), gap(g["right"], r["left"])),
        )


def _pick(state: JointState, names: list[str]) -> list[float] | None:
    """The positions of ``names`` from ``state``, or None if any is missing."""
    index = {name: i for i, name in enumerate(state.name)}
    if not all(name in index for name in names) or len(state.position) < len(state.name):
        return None
    return [float(state.position[index[name]]) for name in names]
