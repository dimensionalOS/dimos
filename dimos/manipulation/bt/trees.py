"""Behavior-tree builders for pick-and-place."""

from __future__ import annotations

from typing import TYPE_CHECKING

import py_trees
from py_trees.common import Status

from dimos.manipulation.bt import actions, conditions
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.manipulation.pick_and_place_module import PickAndPlaceModule

logger = setup_logger()


class RetryOnFailure(py_trees.decorators.Decorator):
    """Retry a child subtree on failure up to ``max_attempts``."""

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
        logger.info(f"[{self.name}] attempt {self._attempt}/{self.max_attempts} failed")
        if self._attempt >= self.max_attempts:
            return Status.FAILURE

        self.decorated.stop(Status.INVALID)
        return Status.RUNNING


def build_pick_tree(
    module: PickAndPlaceModule,
    object_name: str,
    object_id: str | None,
    robot_name: str | None,
) -> py_trees.behaviour.Behaviour:
    """Build a robust pick tree with GraspGen-first and heuristic fallback."""

    cfg = module.config

    ensure_ready = py_trees.composites.Selector(
        "EnsureReady",
        memory=False,
        children=[
            conditions.RobotIsHealthy("RobotIsHealthy", module),
            py_trees.composites.Sequence(
                "ResetAndVerify",
                memory=True,
                children=[
                    actions.ResetRobot("ResetRobot", module),
                    conditions.RobotIsHealthy("RobotIsHealthyAfterReset", module),
                ],
            ),
        ],
    )

    ensure_scanned = py_trees.composites.Selector(
        "EnsureScanned",
        memory=False,
        children=[
            conditions.HasDetections("HasDetections", module),
            actions.ScanObjects("ScanObjects", module, min_duration=cfg.bt_scan_duration),
        ],
    )

    prepare_detections = py_trees.composites.Selector(
        "PrepareDetections",
        memory=False,
        children=[
            conditions.HasTargetDetection("UseExistingTargetSnapshot", module),
            py_trees.composites.Sequence(
                "RefreshTargetSnapshot",
                memory=True,
                children=[
                    actions.ClearPerceptionObstacles("ClearPerceptionObstacles", module),
                    ensure_scanned,
                ],
            ),
        ],
    )

    if cfg.bt_enable_graspgen:
        dl_grasps = py_trees.composites.Sequence(
            "GraspGenCandidates",
            memory=True,
            children=[
                actions.GetObjectPointcloud("GetObjectPointcloud", module),
                actions.GetScenePointcloud("GetScenePointcloud", module),
                actions.GenerateGrasps("GenerateGrasps", module),
            ],
        )

        generate_candidates: py_trees.behaviour.Behaviour = py_trees.composites.Selector(
            "GenerateGraspCandidates",
            memory=True,
            children=[
                dl_grasps,
                actions.GenerateHeuristicGrasps("HeuristicGrasps", module),
            ],
        )
    else:
        generate_candidates = actions.GenerateHeuristicGrasps("GenerateGraspCandidates", module)

    grasp_attempt = py_trees.composites.Sequence(
        "GraspAttempt",
        memory=True,
        children=[
            actions.SelectNextGrasp("SelectNextGrasp", module),
            actions.ComputePreGrasp("ComputePreGrasp", module),
            actions.SetGripper(
                "OpenGripper",
                module,
                position=cfg.bt_gripper_open_position,
                settle_time=0.5,
            ),
            actions.PlanToPose("PlanToPreGrasp", module, pose_key="pre_grasp_pose"),
            actions.ExecuteTrajectory("ExecuteApproach", module),
            actions.PlanToPose("PlanToGrasp", module, pose_key="current_grasp"),
            actions.ExecuteTrajectory("ExecuteGrasp", module),
            actions.SetGripper(
                "CloseGripper",
                module,
                position=cfg.bt_gripper_close_position,
                settle_time=cfg.bt_gripper_settle_time,
            ),
            conditions.GripperHasObject("VerifyGrasp", module),
            actions.ComputeLiftPose("ComputeLiftPose", module),
            actions.PlanToPose("PlanLift", module, pose_key="lift_pose"),
            actions.ExecuteTrajectory("ExecuteLift", module),
            conditions.VerifyHoldAfterLift("VerifyHoldAfterLift", module),
        ],
    )

    recover_then_fail = py_trees.composites.Sequence(
        "RecoverThenFail",
        memory=True,
        children=[
            actions.CancelMotion("CancelMotion", module),
            actions.ResetRobot("ResetAfterFailure", module),
            py_trees.behaviours.Failure("RecoveryComplete"),
        ],
    )

    attempt_or_recover = py_trees.composites.Selector(
        "AttemptOrRecover",
        memory=True,
        children=[grasp_attempt, recover_then_fail],
    )

    grasp_with_retry = RetryOnFailure(
        "GraspWithRetry",
        child=attempt_or_recover,
        max_attempts=cfg.bt_max_pick_attempts,
    )

    scan_and_grasp = py_trees.composites.Sequence(
        "ScanAndGrasp",
        memory=True,
        children=[
            actions.ClearGraspState("ClearGraspState", module),
            prepare_detections,
            actions.FindObject("FindObject", module),
            generate_candidates,
            actions.FilterGraspWorkspace("FilterGraspWorkspace", module),
            grasp_with_retry,
        ],
    )

    pick_with_rescan = RetryOnFailure(
        "PickWithRescan",
        child=scan_and_grasp,
        max_attempts=cfg.bt_max_rescan_attempts,
    )

    root = py_trees.composites.Sequence(
        "Pick",
        memory=True,
        children=[
            ensure_ready,
            pick_with_rescan,
            actions.StorePickPosition("StorePickPosition", module),
            actions.SetResultMessage(
                "SetResult",
                module,
                message=f"Pick complete - grasped '{object_name}' successfully{{grasp_source}}",
            ),
        ],
    )

    bb = py_trees.blackboard.Client(name="PickTreeSetup")
    for key in ("object_name", "object_id", "robot_name"):
        bb.register_key(key=key, access=py_trees.common.Access.WRITE)
    bb.object_name = object_name
    bb.object_id = object_id
    bb.robot_name = robot_name
    return root


def build_place_tree(
    module: PickAndPlaceModule,
    x: float,
    y: float,
    z: float,
    robot_name: str | None,
) -> py_trees.behaviour.Behaviour:
    """Build a place tree using current module planning/execution primitives."""

    cfg = module.config
    root = py_trees.composites.Sequence(
        "Place",
        memory=True,
        children=[
            actions.ComputePlacePose("ComputePlacePose", module, x=x, y=y, z=z),
            actions.PlanToPose("PlanToPrePlace", module, pose_key="pre_place_pose"),
            actions.ExecuteTrajectory("ExecutePrePlace", module),
            actions.PlanToPose("PlanToPlace", module, pose_key="place_pose"),
            actions.ExecuteTrajectory("ExecutePlace", module),
            actions.SetGripper(
                "OpenGripper",
                module,
                position=cfg.bt_gripper_open_position,
                settle_time=0.5,
            ),
            actions.PlanToPose("PlanRetract", module, pose_key="pre_place_pose"),
            actions.ExecuteTrajectory("ExecuteRetract", module),
            actions.SetResultMessage(
                "SetResult",
                module,
                message=f"Place complete - object released at ({x:.3f}, {y:.3f}, {z:.3f})",
            ),
        ],
    )

    bb = py_trees.blackboard.Client(name="PlaceTreeSetup")
    bb.register_key(key="robot_name", access=py_trees.common.Access.WRITE)
    bb.robot_name = robot_name
    return root
