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

"""Development-only demonstration-derived grasp/hold/press/replace flow.

Targets and held-object collision admission are mandatory caller inputs. This
does not detect a handle/button, infer task success, or expose evaluator truth.
One client commands the shared manipulation runtime. Checked press phases use
its existing synchronized Cartesian planner for both TCPs, with explicit torso
assistance. No competing planners command the same robot.
"""

from collections.abc import Callable, Mapping, Sequence
from contextvars import ContextVar
from dataclasses import dataclass, replace
import math
import threading
import time
from typing import Any

from dimos.core.core import rpc
from dimos.manipulation.manipulation_spec import (
    CommandResult,
    CommandStatus,
    ExecutionResult,
    ExecutionStatus,
    ManipulationSnapshot,
    PlanningGroupInfo,
    PlanResult,
    PlanStatus,
)
from dimos.manipulation.planning.groups.models import PlanningGroupSelection
from dimos.manipulation.planning.spec.models import PlanningGroupID
from dimos.manipulation.sdk import Arm
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.simulation.behavior.radio_baselines import PressIntent
from dimos.simulation.behavior.radio_motion import RadioManipulationModule
from dimos.simulation.behavior.radio_policy import vector

GRIPPER_TASKS = {"left_arm": "r1pro_left_gripper", "right_arm": "r1pro_right_gripper"}


class BimanualRadioManipulationModule(RadioManipulationModule):
    """Task-local routing: each SDK arm addresses its own configured gripper."""

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._radio_checkpoint: Any = None
        self._radio_start_targets: ContextVar[dict[str, tuple[PoseStamped, PoseStamped]] | None] = (
            ContextVar("radio_start_targets", default=None)
        )

    def _resolve_group_plan_start(
        self, group_ids: tuple[PlanningGroupID, ...], planning_epoch: int
    ) -> tuple[PlanningGroupSelection, JointState] | None:
        resolved = super()._resolve_group_plan_start(group_ids, planning_epoch)
        targets = self._radio_start_targets.get()
        if resolved is not None and targets is not None:
            assert self._world_monitor is not None
            world = self._world_monitor.world
            with world.scratch_context() as ctx:
                # Same selected state and frozen scene as the Cartesian planner.
                full = world.get_joint_state(ctx)
                positions = dict(zip(full.name, full.position, strict=True))
                positions.update(zip(resolved[1].name, resolved[1].position, strict=True))
                world.set_joint_state(
                    ctx, JointState(name=full.name, position=[positions[n] for n in full.name])
                )
                for group, (_, goal) in tuple(targets.items()):
                    targets[group] = (world.get_group_ee_pose(ctx, group), goal)
        return resolved

    @rpc
    def configure_radio_checkpoint(self, description: Mapping[str, Any]) -> str:
        """Owner-only development geometry, distinct from stationary-box demos."""
        from dimos.simulation.behavior.radio_checkpoint import RadioCheckpointScene

        with self._lock:
            if self._world_monitor is None or self._state.name == "EXECUTING":
                raise RuntimeError("Checkpoint requires an initialized idle SDK world")
            if self._radio_checkpoint is not None:
                if dict(description) != self._radio_checkpoint.description:
                    raise ValueError("Checkpoint collision geometry ownership changed")
            else:
                self._radio_checkpoint = RadioCheckpointScene(
                    self._world_monitor.world, description
                )
            return str(self._radio_checkpoint.signature)

    @rpc
    def plan_radio_checkpoint(
        self,
        target: PoseStamped,
        request: Mapping[str, Any],
        expected: JointState,
        auxiliary_torso: bool = False,
        left_target: PoseStamped | None = None,
    ) -> PlanResult:
        """Store an ordinary SDK plan using the same checked grasp/lift scene."""
        import numpy as np

        from dimos.manipulation.planning.planners.config import CartesianPathConfig
        from dimos.utils.transform_utils import pose_to_matrix

        checkpoint = self._radio_checkpoint
        if checkpoint is None or self._world_monitor is None:
            raise RuntimeError("Configure the task-local checkpoint geometry first")
        if target.frame_id != "world" or request["phase"] not in (
            "pregrasp",
            "grasp",
            "departure",
            "lift",
            "reposition",
            "press_approach",
            "press_contact",
            "reorient",
            "lower",
            "place",
            "retract",
            "table_approach",
            "table_press",
        ):
            raise ValueError("Use a world-frame checkpoint target")
        dual = request["phase"] in ("reposition", "press_approach", "press_contact")
        if dual != (left_target is not None) or (
            left_target is not None and left_target.frame_id != "world"
        ):
            raise ValueError("Coordinated press requires both world-frame TCP targets")
        measured = self._world_monitor.current_model_joint_state()
        actual = dict(zip(measured.name, measured.position, strict=True))
        desired = dict(zip(expected.name, expected.position, strict=True))
        if set(actual) != set(desired) or any(abs(actual[n] - desired[n]) > 0.002 for n in actual):
            raise RuntimeError("Checkpoint SDK start differs from fresh client measurement")
        groups = ["right_arm", "left_arm"] if dual else ["right_arm"]
        selection = self._world_monitor.planning_groups.select(
            [*groups, "torso"] if auxiliary_torso else groups
        )
        with checkpoint.context(request):
            if request["phase"] in ("pregrasp", "reposition", "reorient", "table_approach"):
                pose_targets = {"right_arm": target}
                if request["phase"] == "reposition":
                    assert left_target is not None
                    if not np.allclose(
                        pose_to_matrix(target), request["right_target_pose"], atol=1e-8, rtol=0
                    ) or not np.allclose(
                        pose_to_matrix(left_target), request["left_target_pose"], atol=1e-8, rtol=0
                    ):
                        raise ValueError("Reposition targets differ from checked endpoint contract")
                    pose_targets["left_arm"] = left_target
                result = super().plan_to_poses(
                    pose_targets,
                    speed_scale=0.05
                    if request["phase"] in ("reposition", "reorient", "table_approach")
                    else 0.15,
                    auxiliary_groups=["torso"] if auxiliary_torso else [],
                )
            else:
                current = self._world_monitor.get_group_ee_pose("right_arm", measured)
                targets = {"right_arm": (current, target)}
                if dual:
                    if not np.allclose(
                        pose_to_matrix(target), request["holding_pose"], atol=1e-8, rtol=0
                    ):
                        raise ValueError("Holding target differs from checked press contract")
                    checkpoint.holding_sample(measured, request)
                    assert left_target is not None
                    targets["left_arm"] = (
                        self._world_monitor.get_group_ee_pose("left_arm", measured),
                        left_target,
                    )
                if (
                    np.linalg.norm(pose_to_matrix(current)[:3, :3] - pose_to_matrix(target)[:3, :3])
                    > 0.03
                ):
                    raise ValueError(
                        "Checkpoint linear stages must preserve measured wrist orientation"
                    )
                token = self._radio_start_targets.set(targets)
                try:
                    plan = self.generate_cartesian_plan(
                        targets,
                        CartesianPathConfig(
                            speed_mode="bounded",
                            max_linear_speed=0.04 if request["phase"] == "press_approach" else 0.01,
                            max_position_error=0.002,
                            max_orientation_error=0.005,
                        ),
                        auxiliary_groups=["torso"] if auxiliary_torso else [],
                        speed_scale=1.0,
                        check_collision=True,
                    )
                finally:
                    self._radio_start_targets.reset(token)
                result = (
                    PlanResult(PlanStatus.SUCCEEDED, plan.message, plan)
                    if plan
                    else PlanResult(PlanStatus.FAILED, self._error_message)
                )
        if result.succeeded and result.plan is not None:
            try:
                checkpoint.validate(
                    measured, result.plan.trajectory, selection.joint_names, request
                )
            except BaseException:
                self._clear_pending_plan()
                raise
        return result

    def _get_group_gripper_position(self) -> None:
        # The base implementation has a single device binding. Populate group
        # feedback below instead of reading that scalar for every arm/torso.
        return None

    @rpc
    def list_planning_groups(self) -> tuple[PlanningGroupInfo, ...]:
        return tuple(
            replace(group, has_gripper=group.id in GRIPPER_TASKS)
            for group in super().list_planning_groups()
        )

    @rpc
    def get_state(self) -> ManipulationSnapshot:
        snapshot = super().get_state()
        groups = dict(snapshot.groups)
        for group, task in GRIPPER_TASKS.items():
            if group in groups:
                values = self._control_coordinator.task_invoke(task, "get_normalized", {})
                groups[group] = replace(
                    groups[group], gripper_position=float(values[0]) if values else None
                )
        return replace(snapshot, groups=groups)

    @rpc
    def set_gripper_position(
        self, position: float, planning_group: str | None = None
    ) -> CommandResult:
        if not math.isfinite(position) or not 0 <= position <= 1:
            return CommandResult(CommandStatus.REJECTED, "Use finite normalized travel in [0, 1]")
        if planning_group not in GRIPPER_TASKS:
            return CommandResult(CommandStatus.REJECTED, "Choose left_arm or right_arm explicitly")
        if self._world_monitor is None:
            return CommandResult(CommandStatus.FAILED, "Planning is not initialized")
        accepted = self._control_coordinator.task_invoke(
            GRIPPER_TASKS[planning_group], "set_normalized", {"values": [float(position)]}
        )
        return CommandResult(
            CommandStatus.SUCCEEDED if accepted else CommandStatus.FAILED,
            "Gripper command accepted; measured feedback still required",
        )


@dataclass(frozen=True)
class RadioPoseIntent:
    position: Sequence[float]
    orientation: Sequence[float]
    provenance: str

    def __post_init__(self) -> None:
        vector(self.position, 3)
        q = vector(self.orientation, 4)
        if abs(math.hypot(*q) - 1) > 1e-6 or not self.provenance:
            raise ValueError("Declare provenance and use a unit quaternion")


@dataclass(frozen=True)
class BimanualRadioIntent:
    pregrasp: RadioPoseIntent
    grasp: RadioPoseIntent
    hold: RadioPoseIntent
    replace: RadioPoseIntent


class BimanualRadioFlow:
    """Bounded synchronous SDK stages; tick performs at most one stage.

    checked_move must collision-check the effective dispatched trajectory,
    include both arms and the moving/held radio, freeze base (and torso while
    holding), permit
    only declared finger-object contact, and check fresh measured completion.
    It must reject unsupported attachment/scene updates, never remove the radio
    to make a lift pass. hold_verified must establish physical retention, not
    merely a closed-gripper command. press_intent observes the radio AFTER lift.
    support_verified must establish placement support before release. Before
    grasp, explicitly declared torso assistance may establish a shared posture.

    Cancel can be called from another thread while a blocking stage runs. It
    halts admission, calls the shared SDK RPC cancel, and never opens a holding
    gripper automatically. UNCERTAIN/FAULT remain unconfirmed stops. Completion
    is only completion of SDK stages; an independent evaluator checks BDDL.
    """

    def __init__(
        self,
        right: Arm,
        left: Arm,
        intent: BimanualRadioIntent,
        checked_move: Callable[[Arm, RadioPoseIntent, bool, bool, float], None],
        hold_verified: Callable[[], bool],
        press_intent: Callable[[], PressIntent],
        simulation_step: Callable[[], int],
        support_verified: Callable[[], bool],
        *,
        timeout: float = 120,
    ) -> None:
        if right.info.id != "right_arm" or left.info.id != "left_arm":
            raise ValueError("Use explicit right holding arm and left pressing arm")
        if right.rpc is not left.rpc:
            raise ValueError("Use one shared manipulation runtime and command owner")
        if not math.isfinite(timeout) or not 0 < timeout <= 120:
            raise ValueError("Development flow deadline must be within 120 seconds")
        self.right, self.left, self.intent = right, left, intent
        self.checked_move, self.hold_verified = checked_move, hold_verified
        self.press_intent, self.simulation_step = press_intent, simulation_step
        self.support_verified = support_verified
        self.deadline = time.monotonic() + timeout
        self.index = 0
        self.holding = False
        self.halted: str | None = None
        self.evidence: list[dict[str, Any]] = []
        self._cancelled = threading.Event()
        self._press: PressIntent | None = None
        self._contact_start: int | None = None

    @property
    def completed(self) -> bool:
        return self.index == 11 and self.halted is None

    def _remaining(self) -> float:
        remaining = self.deadline - time.monotonic()
        if self._cancelled.is_set():
            raise RuntimeError("Cancellation requested; no further stages admitted")
        if remaining <= 0:
            raise TimeoutError("Bimanual radio flow deadline elapsed")
        return min(20, remaining)

    def _move(self, arm: Arm, pose: RadioPoseIntent, contact: bool = False) -> None:
        if self.holding and not self.hold_verified():
            raise RuntimeError("Physical radio retention not verified")
        record = self.evidence[-1]
        record["provenance"] = pose.provenance
        self.checked_move(arm, pose, contact, self.holding, self._remaining())

    def _gripper(self, arm: Arm, position: float) -> bool:
        record = self.evidence[-1]
        if not record.get("command_sent"):
            self._remaining()
            arm.set_gripper_position(position)
            record["command_sent"] = True
        measured = arm.state().gripper_position
        record["measured_gripper"] = measured
        return measured is not None and math.isfinite(measured) and abs(measured - position) <= 0.05

    def tick(self) -> str:
        if self.halted is not None:
            return self.halted
        if self.completed:
            return "completed"
        labels = (
            "open_right",
            "pregrasp",
            "grasp",
            "close_right",
            "lift_and_hold",
            "close_left",
            "observe_held_precontact",
            "press",
            "hold_contact",
            "retract_left",
            "replace_and_release",
        )
        label = labels[self.index]
        if not self.evidence or self.evidence[-1]["stage"] != label:
            self.evidence.append({"stage": label, "state": "running"})
        stage_record = self.evidence[-1]
        try:
            self._remaining()
            done = True
            if self.index == 0:
                done = self._gripper(self.right, 1)
            elif self.index == 1:
                self._move(self.right, self.intent.pregrasp)
            elif self.index == 2:
                self._move(self.right, self.intent.grasp, contact=True)
            elif self.index == 3:
                fully_closed = self._gripper(self.right, 0)
                measured = stage_record["measured_gripper"]
                # A real handle prevents the fingers reaching the empty-hand
                # zero target. Position feedback alone cannot prove retention.
                measurable_closure = (
                    measured is not None and math.isfinite(measured) and 0 <= measured < 0.95
                )
                retained = measurable_closure and self.hold_verified()
                stage_record["physical_retention_verified"] = retained
                done = retained
                if fully_closed and not retained:
                    raise RuntimeError("Gripper closed without verified physical radio retention")
                if retained:
                    stage_record["object_blocked_closure"] = not fully_closed
                    self.holding = True
            elif self.index == 4:
                self._move(self.right, self.intent.hold)
            elif self.index == 5:
                done = self._gripper(self.left, 0)
            elif self.index == 6:
                self._press = self.press_intent()
                self._move(self.left, self._press_pose(-0.012))
            elif self.index == 7:
                self._move(self.left, self._press_pose(0), contact=True)
                self._contact_start = self.simulation_step()
            elif self.index == 8:
                if not self.hold_verified():
                    raise RuntimeError("Radio slipped during contact")
                assert self._contact_start is not None
                done = self.simulation_step() - self._contact_start >= 8
            elif self.index == 9:
                self._move(self.left, self._press_pose(-0.012), contact=True)
            elif self.index == 10:
                record = self.evidence[-1]
                if not record.get("placed"):
                    self._move(self.right, self.intent.replace, contact=True)
                    record["placed"] = True
                if not self.support_verified():
                    raise RuntimeError("Placement support not verified; holding gripper preserved")
                done = self._gripper(self.right, 1)
                if done:
                    self.holding = False
            self._remaining()
            if done:
                stage_record["state"] = "completed"
                self.index += 1
            return label
        except Exception as error:
            self.halted = self.halted or str(error)
            stage_record.update(state="failed", error=str(error))
            if not self._cancelled.is_set():
                self.cancel()
            raise

    def _press_pose(self, offset: float) -> RadioPoseIntent:
        assert self._press is not None
        return RadioPoseIntent(
            tuple(
                a + offset * b
                for a, b in zip(self._press.position, self._press.inward, strict=True)
            ),
            self._press.orientation,
            self._press.provenance,
        )

    def cancel(self) -> ExecutionResult:
        self._cancelled.set()
        self.halted = self.halted or "cancelled"
        try:
            result = self.right.rpc.cancel()
        except Exception as error:
            result = ExecutionResult(ExecutionStatus.UNCERTAIN, f"Cancellation RPC failed: {error}")
        self.evidence.append(
            {
                "stage": "cancel",
                "status": result.status.name,
                "stop_confirmed": result.status
                in (
                    ExecutionStatus.ABORTED,
                    ExecutionStatus.NO_EXECUTION,
                    ExecutionStatus.COMPLETED,
                ),
                "holding_gripper_preserved": self.holding,
            }
        )
        return result
