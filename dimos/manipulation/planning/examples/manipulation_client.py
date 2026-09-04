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

"""Typed Python client for a running manipulation blueprint.

Start ``dimos run xarm-perception-sim-agent``, then run this module.
With no arguments it demonstrates motion. Add ``--object cup --place X Y Z``
to scan, pick, and place at a verified planning-frame position.
"""

from __future__ import annotations

import argparse

from dimos.manipulation.manipulation_spec import (
    CommandResult,
    ExecutionResult,
    ExecutionStatus,
    ManipulationSpec,
    MoveResult,
    PlanResult,
)
from dimos.manipulation.pick_and_place_spec import PickAndPlaceSpec, PickPlaceResult
from dimos.manipulation.planning.spec.models import PlanningGroupID
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.porcelain.dimos import Dimos


def _require(
    result: CommandResult | ExecutionResult | MoveResult | PlanResult | PickPlaceResult,
) -> None:
    print(result)
    if not result.succeeded:
        raise RuntimeError(str(result))


def run_motion(motion: ManipulationSpec) -> PlanningGroupID:
    """Demonstrate planning, preview, execution, cancellation, and gripper control."""
    groups = tuple(
        group
        for group in motion.list_planning_groups()
        if group.tip_frame is not None and group.has_gripper
    )
    if len(groups) != 1:
        raise RuntimeError(f"Expected one gripper-capable pose group, got {groups!r}")
    group = groups[0]
    snapshot = motion.get_state()
    print(snapshot)
    state = snapshot.groups[group.id]
    if state.joints is None or state.end_effector_pose is None:
        raise RuntimeError("Wait for joint state and end-effector pose before running the example")
    initial = state.joints
    positions = list(initial.position)
    positions[0] += 0.02
    target = JointState(name=list(initial.name), position=positions)

    planned = motion.plan_to_joints({group.id: target}, speed_scale=0.2)
    _require(planned)
    _require(motion.preview_plan(planned.plan))
    print("Preview:", motion.get_visualization_url())
    _require(motion.execute())

    pose = motion.get_state().groups[group.id].end_effector_pose
    if pose is None:
        raise RuntimeError("End-effector pose is unavailable")
    lifted = PoseStamped(
        frame_id=pose.frame_id,
        position=pose.position + Vector3(0.0, 0.0, 0.01),
        orientation=pose.orientation,
    )
    _require(motion.plan_to_poses({group.id: lifted}, speed_scale=0.2))
    _require(motion.execute(blocking=False))
    completed = motion.wait_for_execution(timeout=60.0)
    _require(completed)
    if completed.status is not ExecutionStatus.COMPLETED:
        raise RuntimeError(f"Expected physical completion, got {completed!r}")

    _require(motion.move_linear(dz=-0.01, planning_group=group.id, check_collision=True))
    _require(motion.plan_to_joints({group.id: initial}, speed_scale=0.1))
    _require(motion.execute(blocking=False))
    cancelled = motion.cancel()
    print(cancelled)
    # A sufficiently short trajectory can complete before cancellation arrives.
    if cancelled.status not in {ExecutionStatus.ABORTED, ExecutionStatus.COMPLETED}:
        raise RuntimeError(f"Cancellation was not confirmed: {cancelled!r}")

    # Restore the original camera/arm pose before the object workflow.
    _require(motion.plan_to_joints({group.id: initial}, speed_scale=0.2))
    _require(motion.execute())
    _require(motion.clear_planned_path())
    _require(motion.set_gripper_position(1.0, group.id))
    return group.id


def run_pick_place(
    pick_place: PickAndPlaceSpec,
    prompt: str,
    destination: tuple[float, float, float],
    planning_group: PlanningGroupID,
) -> None:
    """Pick the uniquely detected object and place it at a verified position."""
    scan = pick_place.scan_objects([prompt])
    _require(scan)
    if len(scan.objects) != 1:
        raise RuntimeError(f"Use a prompt selecting exactly one object; detected {scan.objects!r}")
    target = scan.objects[0]
    print(pick_place.get_object(target.object_id))
    picked = pick_place.pick_object(target.object_id, planning_group)
    _require(picked)
    print(pick_place.get_grasp_candidates())
    _require(pick_place.place_at(*destination, planning_group=planning_group))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--object", help="Unique object description for the current scene")
    parser.add_argument("--place", nargs=3, type=float, metavar=("X", "Y", "Z"))
    args = parser.parse_args()
    if (args.object is None) != (args.place is None):
        parser.error("--object and --place must be supplied together")

    app = Dimos.connect()
    try:
        motion = app.get_module(ManipulationSpec)
        group = run_motion(motion)
        if args.object is not None:
            pick_place = app.get_module(PickAndPlaceSpec)
            run_pick_place(pick_place, args.object, tuple(args.place), group)
    finally:
        # Disconnecting does not stop the remote blueprint or cancel its motion.
        app.stop()


if __name__ == "__main__":
    main()
