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

"""BT tree builders and RetryOnFailure decorator for pick-and-place."""

from __future__ import annotations

from typing import TYPE_CHECKING

import py_trees
from py_trees.common import Status

from dimos.manipulation.bt import actions, conditions
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.manipulation.bt.pick_place_module import PickPlaceModule

logger = setup_logger()

class RetryOnFailure(py_trees.decorators.Decorator):
    """Retry the child subtree on FAILURE, up to *max_attempts*.

    On each FAILURE the child is stopped (reset) so the next tick
    re-runs it from scratch.  SUCCESS and RUNNING pass through.
    """

    def __init__(self, name: str, child: py_trees.behaviour.Behaviour, max_attempts: int) -> None:
        super().__init__(name=name, child=child)
        self.max_attempts = max_attempts
        self._attempt = 0

    def initialise(self) -> None:
        self._attempt = 0

    def update(self) -> Status:
        if self.decorated.status == Status.SUCCESS:
            return Status.SUCCESS

        if self.decorated.status == Status.RUNNING:
            return Status.RUNNING

        self._attempt += 1
        logger.info(
            f"[RetryOnFailure:{self.name}] Attempt {self._attempt}/{self.max_attempts} failed"
        )

        if self._attempt >= self.max_attempts:
            logger.warning(f"[RetryOnFailure:{self.name}] All {self.max_attempts} attempts exhausted")
            return Status.FAILURE

        self.decorated.stop(Status.INVALID)
        return Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        if new_status == Status.INVALID and self.decorated.status == Status.RUNNING:
            self.decorated.stop(Status.INVALID)

def build_pick_tree(
    module: PickPlaceModule,
    object_name: str,
    object_id: str | None = None,
    max_attempts: int = 5,
    home_joints_override: list[float] | None = None,
) -> py_trees.behaviour.Behaviour:
    """Build a complete pick Behavior Tree with two-level retry.

    Inner retry (GraspWithRetry) sweeps grasp candidates.
    Outer retry (PickWithRescan) re-scans and regenerates grasps.

    Tree structure::

        Sequence(memory=True) "Pick"
        ├── SetHasObject(False)
        ├── Selector "EnsureReady"
        │   ├── RobotIsHealthy
        │   └── Sequence [ResetRobot, RobotIsHealthy]
        ├── RetryOnFailure "PickWithRescan"
        │   └── Sequence(memory=True) "ScanAndGrasp"
        │       ├── ClearGraspState
        │       ├── Sequence "ReturnHome"
        │       ├── Selector "EnsureScanned" [HasDetections, ScanObjects]
        │       ├── FindObject
        │       ├── Selector "GenerateGraspCandidates" [MLGrasps, Heuristic]
        │       ├── FilterGraspWorkspace
        │       ├── VisualizeGrasps
        │       └── RetryOnFailure "GraspWithRetry"
        │           └── Selector(memory=True) "AttemptOrRecover"
        │               ├── Sequence(memory=True) "GraspAttempt"
        │               │   [select, pre-grasp, approach, grasp, verify, lift]
        │               └── Sequence(memory=True) "RecoverThenFail"
        │                   [cancel, reset, recovery(local/home), Failure]
        ├── StorePickPosition
        └── SetResultMessage
    """
    cfg = module.config

    init_state = actions.SetHasObject("InitHasObject", module, value=False)

    ensure_ready = py_trees.composites.Selector(
        "EnsureReady",
        memory=False,
        children=[
            conditions.RobotIsHealthy("RobotIsHealthy?", module),
            py_trees.composites.Sequence(
                "ResetAndVerify",
                memory=True,
                children=[
                    actions.ResetRobot("ResetRobot", module),
                    conditions.RobotIsHealthy("RobotIsHealthy?2", module),
                ],
            ),
        ],
    )

    clear_state = actions.ClearGraspState("ClearGraspState", module)

    ensure_scanned = py_trees.composites.Selector(
        "EnsureScanned",
        memory=False,
        children=[
            conditions.HasDetections("HasDetections?", module),
            actions.ScanObjects("ScanObjects", module, min_duration=cfg.scan_duration),
        ],
    )

    find_object = actions.FindObject("FindObject", module)

    # ML grasps need gripper adaptation (base to contact frame); heuristic don't.
    adapt_grasps_nodes: list[py_trees.behaviour.Behaviour] = []
    if cfg.adapt_grasps:
        adapt_grasps_nodes.append(
            actions.AdaptGrasps(
                "AdaptGrasps",
                module,
                source_gripper=cfg.source_gripper,
                target_gripper=cfg.target_gripper,
            )
        )

    if cfg.use_dl_grasps:
        dl_grasps = py_trees.composites.Sequence(
            "DLGrasps",
            memory=True,
            children=[
                actions.GetObjectPointcloud("GetObjectPointcloud", module),
                actions.GetScenePointcloud("GetScenePointcloud", module),
                actions.GenerateGrasps("GenerateGrasps", module),
                *adapt_grasps_nodes,
            ],
        )
        generate_grasp_candidates: py_trees.behaviour.Behaviour = py_trees.composites.Selector(
            "GenerateGraspCandidates",
            memory=True,
            children=[
                dl_grasps,
                actions.GenerateHeuristicGrasps("HeuristicGrasps", module),
            ],
        )
    else:
        generate_grasp_candidates = actions.GenerateHeuristicGrasps("HeuristicGrasps", module)

    grasp_attempt = py_trees.composites.Sequence(
        "GraspAttempt",
        memory=True,
        children=[
            actions.SetHasObject("ResetHasObject", module, value=False),
            actions.SelectNextGrasp("SelectNextGrasp", module),
            actions.ComputePreGrasp("ComputePreGrasp", module),
            actions.SetGripper("OpenGripper", module, position=cfg.gripper_open_position, settle_time=0.5),
            actions.PlanToPose("PlanToPreGrasp", module, pose_key="pre_grasp_pose"),
            actions.ExecuteTrajectory("ExecuteApproach", module),
            conditions.VerifyReachedPose("VerifyPreGrasp", module, pose_key="pre_grasp_pose"),
            actions.PlanToPose("PlanToGrasp", module, pose_key="current_grasp"),
            actions.ExecuteTrajectory("ExecuteGrasp", module),
            conditions.VerifyReachedPose(
                "VerifyGraspPose", module, pose_key="current_grasp",
                pos_tol=cfg.grasp_position_tolerance,
            ),
            actions.SetGripper(
                "CloseGripper", module,
                position=cfg.gripper_close_position,
                settle_time=cfg.gripper_settle_time,
            ),
            conditions.GripperHasObject("VerifyGrasp", module),
            actions.ComputeLiftPose("ComputeLiftPose", module),
            actions.PlanToPose("PlanLift", module, pose_key="lift_pose"),
            actions.ExecuteTrajectory("ExecuteLift", module),
            conditions.VerifyHoldAfterLift("VerifyHoldAfterLift", module),
        ],
    )

    # Recovery: conditional gripper open (skip if holding).
    # Two instances needed — py_trees requires unique node objects.
    maybe_open_gripper = py_trees.composites.Selector(
        "MaybeOpenGripper",
        memory=False,
        children=[
            conditions.HasObject("HasObject?", module),
            actions.SetGripper("OpenGripperRecovery", module, position=cfg.gripper_open_position, settle_time=0.5),
        ],
    )

    maybe_open_gripper2 = py_trees.composites.Selector(
        "MaybeOpenGripper2",
        memory=False,
        children=[
            conditions.HasObject("HasObject?2", module),
            actions.SetGripper("OpenGripperRecovery2", module, position=cfg.gripper_open_position, settle_time=0.5),
        ],
    )

    try_local_retreat = py_trees.composites.Sequence(
        "TryLocalRetreat",
        memory=True,
        children=[
            maybe_open_gripper,
            actions.ComputeLocalRetreatPose("ComputeRetreatPose", module),
            actions.PlanToPose("PlanRetreat", module, pose_key="retreat_pose"),
            actions.ExecuteTrajectory("ExecuteRetreat", module),
        ],
    )

    go_home_retreat = py_trees.composites.Sequence(
        "GoHomeRetreat",
        memory=True,
        children=[
            maybe_open_gripper2,
            actions.ResetRobot("ResetForHome", module),
            actions.PlanToJoints("PlanToHome", module, joints_key="home_joints"),
            actions.ExecuteTrajectory("ExecuteGoHome", module),
        ],
    )

    recover_then_fail = py_trees.composites.Sequence(
        "RecoverThenFail",
        memory=True,
        children=[
            actions.CancelMotion("CancelMotion", module),
            actions.ResetRobot("ResetAfterFailure", module),
            actions.ExhaustRetriesIfHolding("ExhaustRetriesIfHolding", module),
            py_trees.composites.Selector(
                "Recovery", memory=True,
                children=[try_local_retreat, go_home_retreat],
            ),
            py_trees.behaviours.Failure("RecoveryComplete"),
        ],
    )

    # memory=True: recovery must complete before RetryOnFailure starts the next attempt
    attempt_or_recover = py_trees.composites.Selector(
        "AttemptOrRecover",
        memory=True,
        children=[grasp_attempt, recover_then_fail],
    )

    grasp_with_retry = RetryOnFailure(
        "GraspWithRetry",
        child=attempt_or_recover,
        max_attempts=max_attempts,
    )

    # Return home before each rescan so the arm is out of the camera view
    return_home = py_trees.composites.Sequence(
        "ReturnHome",
        memory=True,
        children=[
            actions.CancelMotion("CancelBeforeHome", module, settle_time=0.3),
            actions.ResetRobot("ResetBeforeHome", module),
            actions.SetGripper(
                "OpenGripperBeforeHome", module,
                position=cfg.gripper_open_position, settle_time=0.5,
            ),
            actions.PlanToJoints("PlanToHomeForRescan", module, joints_key="home_joints"),
            actions.ExecuteTrajectory("ExecuteHomeForRescan", module),
        ],
    )

    scan_and_grasp = py_trees.composites.Sequence(
        "ScanAndGrasp",
        memory=True,
        children=[
            clear_state,
            return_home,
            ensure_scanned,
            find_object,
            generate_grasp_candidates,
            actions.FilterGraspWorkspace("FilterGraspWorkspace", module),
            actions.VisualizeGrasps("VisualizeGrasps", module),
            grasp_with_retry,
        ],
    )

    pick_with_rescan = RetryOnFailure(
        "PickWithRescan",
        child=scan_and_grasp,
        max_attempts=cfg.max_rescan_attempts,
    )

    root = py_trees.composites.Sequence(
        "Pick",
        memory=True,
        children=[
            init_state,
            ensure_ready,
            pick_with_rescan,
            actions.StorePickPosition("StorePickPosition", module),
            actions.SetResultMessage(
                "SetResult", module,
                message=f"Pick complete — grasped '{object_name}' successfully",
            ),
        ],
    )

    bb = py_trees.blackboard.Client(name="PickTreeSetup")
    bb.register_key(key="object_name", access=py_trees.common.Access.WRITE)
    bb.register_key(key="object_id", access=py_trees.common.Access.WRITE)
    bb.register_key(key="home_joints", access=py_trees.common.Access.WRITE)
    bb.object_name = object_name
    bb.object_id = object_id
    bb.home_joints = home_joints_override if home_joints_override is not None else cfg.home_joints

    return root

def build_place_tree(
    module: PickPlaceModule,
    x: float,
    y: float,
    z: float,
) -> py_trees.behaviour.Behaviour:
    """Build a place Behavior Tree.

    Tree structure::

        Sequence(memory=True) "Place"
        ├── ComputePlacePose
        ├── PlanToPose("pre_place_pose") → ExecuteTrajectory
        ├── PlanToPose("place_pose") → ExecuteTrajectory
        ├── SetGripper(open)
        ├── PlanToPose("pre_place_pose") → ExecuteTrajectory  ← retract
        └── SetResultMessage
    """
    cfg = module.config

    return py_trees.composites.Sequence(
        "Place",
        memory=True,
        children=[
            actions.ComputePlacePose("ComputePlacePose", module, x=x, y=y, z=z),
            actions.PlanToPose("PlanToPrePlace", module, pose_key="pre_place_pose"),
            actions.ExecuteTrajectory("ExecutePrePlace", module),
            actions.PlanToPose("PlanToPlace", module, pose_key="place_pose"),
            actions.ExecuteTrajectory("ExecutePlace", module),
            actions.SetGripper(
                "OpenGripper", module,
                position=cfg.gripper_open_position, settle_time=0.5,
            ),
            actions.PlanToPose("PlanRetract", module, pose_key="pre_place_pose"),
            actions.ExecuteTrajectory("ExecuteRetract", module),
            actions.SetResultMessage(
                "SetResult", module,
                message=f"Place complete — object released at ({x:.3f}, {y:.3f}, {z:.3f})",
            ),
        ],
    )

def build_go_home_tree(
    module: PickPlaceModule,
    home_joints_override: list[float] | None = None,
) -> py_trees.behaviour.Behaviour:
    """Build a go-home Behavior Tree.

    Tree structure::

        Sequence(memory=True) "GoHome"
        ├── ProbeGripperState
        ├── Selector "MaybeOpenGripper" [HasObject?, SetGripper(open)]
        ├── FailureIsSuccess "BestEffortLift"
        │   └── Selector "MaybeLiftFirst"
        │       ├── Inverter(HasObject?)
        │       └── Sequence [ComputeLiftPose, PlanToPose, Execute]
        ├── PlanToJoints("home_joints") → ExecuteTrajectory
        └── SetResultMessage
    """
    cfg = module.config

    root = py_trees.composites.Sequence(
        "GoHome",
        memory=True,
        children=[
            actions.ProbeGripperState("ProbeGripper", module),
            py_trees.composites.Selector(
                "MaybeOpenGripper",
                memory=False,
                children=[
                    conditions.HasObject("HasObject?", module),
                    actions.SetGripper(
                        "OpenGripper", module,
                        position=cfg.gripper_open_position, settle_time=0.5,
                    ),
                ],
            ),
            # Best-effort lift if holding (avoids dragging); FailureIsSuccess
            # so a lift IK failure doesn't abort the joint-space go-home.
            py_trees.decorators.FailureIsSuccess(
                name="BestEffortLift",
                child=py_trees.composites.Selector(
                    "MaybeLiftFirst",
                    memory=False,
                    children=[
                        py_trees.decorators.Inverter(
                            name="NotHolding?",
                            child=conditions.HasObject("HasObject?Lift", module),
                        ),
                        py_trees.composites.Sequence(
                            "LiftBeforeHome",
                            memory=True,
                            children=[
                                actions.ComputeLiftPose("ComputeSafeLift", module, lift_height=cfg.lift_height),
                                actions.PlanToPose("PlanSafeLift", module, pose_key="lift_pose"),
                                actions.ExecuteTrajectory("ExecuteSafeLift", module),
                            ],
                        ),
                    ],
                ),
            ),
            actions.PlanToJoints("PlanToHome", module, joints_key="home_joints"),
            actions.ExecuteTrajectory("ExecuteGoHome", module),
            actions.SetResultMessage("SetResult", module, message="Moved to home position"),
        ],
    )

    bb = py_trees.blackboard.Client(name="GoHomeSetup")
    bb.register_key(key="home_joints", access=py_trees.common.Access.WRITE)
    bb.home_joints = home_joints_override if home_joints_override is not None else cfg.home_joints

    return root
