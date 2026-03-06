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

"""BT action leaf nodes for pick-and-place orchestration.

Each action wraps one or more PickAndPlaceModule / OSR / GraspGen RPC calls,
following the py_trees lifecycle: initialise() → update() → terminate().

Long-running operations (trajectory execution, gripper settle) return RUNNING
and poll for completion on subsequent ticks.
"""

from __future__ import annotations

import math
import threading
import time
from typing import TYPE_CHECKING, Any

import py_trees
from py_trees.common import Status

from dimos.manipulation.grasping.gripper_adapter import GripperAdapter
from dimos.msgs.geometry_msgs import Pose, Quaternion, Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import offset_distance

if TYPE_CHECKING:
    from dimos.manipulation.bt.pick_place_module import PickPlaceModule

logger = setup_logger()

# --- Base class ---

class ManipulationAction(py_trees.behaviour.Behaviour):
    """Base class for all BT action nodes.

    Holds a reference to the PickPlaceModule so leaf nodes can call RPCs
    and read configuration.
    """

    def __init__(self, name: str, module: PickPlaceModule) -> None:
        super().__init__(name=name)
        self.module = module
        self.bb = self.attach_blackboard_client(name=self.name)
        # Register common keys for error/result messages
        self.bb.register_key(key="error_message", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="result_message", access=py_trees.common.Access.WRITE)

# --- Perception actions ---

class ScanObjects(ManipulationAction):
    """Set detection prompts, refresh obstacles, and populate blackboard detections."""

    def __init__(self, name: str, module: PickPlaceModule, min_duration: float = 1.0) -> None:
        super().__init__(name, module)
        self.min_duration = min_duration
        self.bb.register_key(key="detections", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="object_name", access=py_trees.common.Access.READ)

    def update(self) -> Status:
        try:
            object_name: str | None = getattr(self.bb, "object_name", None)
            if object_name:
                self.module.get_rpc_calls(
                    "ObjectSceneRegistrationModule.set_prompts"
                )(text=[object_name])
                time.sleep(self.module.config.prompt_settle_time)

            self.module.get_rpc_calls(
                "PickAndPlaceModule.refresh_obstacles"
            )(self.min_duration)

            detections = self.module.get_rpc_calls(
                "PickAndPlaceModule.list_cached_detections"
            )() or []

            self.bb.detections = detections
            logger.info(f"[ScanObjects] Found {len(detections)} detection(s)")
            return Status.SUCCESS if detections else Status.FAILURE
        except Exception as e:
            logger.error(f"[ScanObjects] Failed: {e}")
            self.bb.error_message = f"Error: Scan failed — {e}"
            return Status.FAILURE

class FindObject(ManipulationAction):
    """Find target object in detection list by name or object_id."""

    def __init__(self, name: str, module: PickPlaceModule) -> None:
        super().__init__(name, module)
        self.bb.register_key(key="detections", access=py_trees.common.Access.READ)
        self.bb.register_key(key="object_name", access=py_trees.common.Access.READ)
        self.bb.register_key(key="object_id", access=py_trees.common.Access.READ)
        self.bb.register_key(key="target_object", access=py_trees.common.Access.WRITE)

    def update(self) -> Status:
        detections: list[dict[str, Any]] = self.bb.detections
        object_name: str = self.bb.object_name
        object_id: str | None = getattr(self.bb, "object_id", None)

        for det in detections:
            if object_id and det.get("object_id") == object_id:
                self.bb.target_object = det
                logger.info(f"[FindObject] Found by ID: {det.get('name')}")
                return Status.SUCCESS
            det_name = det.get("name", "")
            if object_name.lower() in det_name.lower() or det_name.lower() in object_name.lower():
                self.bb.target_object = det
                logger.info(f"[FindObject] Found '{det_name}' matching '{object_name}'")
                return Status.SUCCESS

        available = [d.get("name", "?") for d in detections]
        msg = f"Error: Object '{object_name}' not found. Available: {available}"
        logger.warning(f"[FindObject] {msg}")
        self.bb.error_message = msg
        return Status.FAILURE

class GetObjectPointcloud(ManipulationAction):
    """Fetch object pointcloud from ObjectSceneRegistrationModule."""

    def __init__(self, name: str, module: PickPlaceModule) -> None:
        super().__init__(name, module)
        self.bb.register_key(key="target_object", access=py_trees.common.Access.READ)
        self.bb.register_key(key="object_pointcloud", access=py_trees.common.Access.WRITE)

    def update(self) -> Status:
        target: dict[str, Any] = self.bb.target_object
        obj_id = target.get("object_id")
        obj_name = target.get("name", "object")

        try:
            if obj_id:
                pc = self.module.get_rpc_calls(
                    "ObjectSceneRegistrationModule.get_object_pointcloud_by_object_id"
                )(obj_id)
            else:
                pc = self.module.get_rpc_calls(
                    "ObjectSceneRegistrationModule.get_object_pointcloud_by_name"
                )(obj_name)

            if pc is None:
                self.bb.error_message = f"Error: No pointcloud for '{obj_name}'"
                return Status.FAILURE

            self.bb.object_pointcloud = pc
            logger.info(f"[GetObjectPointcloud] Got pointcloud for '{obj_name}'")
            return Status.SUCCESS
        except Exception as e:
            logger.error(f"[GetObjectPointcloud] Failed: {e}")
            self.bb.error_message = f"Error: Pointcloud fetch failed — {e}"
            return Status.FAILURE

class GetScenePointcloud(ManipulationAction):
    """Fetch full scene pointcloud for collision filtering."""

    def __init__(self, name: str, module: PickPlaceModule) -> None:
        super().__init__(name, module)
        self.bb.register_key(key="target_object", access=py_trees.common.Access.READ)
        self.bb.register_key(key="scene_pointcloud", access=py_trees.common.Access.WRITE)

    def update(self) -> Status:
        try:
            target: dict[str, Any] = self.bb.target_object
            exclude_id = target.get("object_id")
            pc = self.module.get_rpc_calls(
                "ObjectSceneRegistrationModule.get_full_scene_pointcloud"
            )(exclude_object_id=exclude_id)
            self.bb.scene_pointcloud = pc
            return Status.SUCCESS
        except Exception as e:
            logger.warning(f"[GetScenePointcloud] Could not get scene PC: {e}")
            self.bb.scene_pointcloud = None
            return Status.SUCCESS  # Non-fatal — grasps work without scene PC

# --- Grasp generation actions ---

class GenerateGrasps(ManipulationAction):
    """Generate DL-based grasps via GraspGen Docker module.

    Runs the RPC in a background thread so Docker startup doesn't block the
    BT tick loop.  Returns RUNNING while in-flight, falls back to heuristic on FAILURE.
    """

    def __init__(self, name: str, module: PickPlaceModule, rpc_timeout: float = 1800.0) -> None:
        super().__init__(name, module)
        self.rpc_timeout = rpc_timeout
        self.bb.register_key(key="object_pointcloud", access=py_trees.common.Access.READ)
        self.bb.register_key(key="scene_pointcloud", access=py_trees.common.Access.READ)
        self.bb.register_key(key="grasp_candidates", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="grasp_index", access=py_trees.common.Access.WRITE)
        self._thread: threading.Thread | None = None
        self._result: Any = None
        self._error: Exception | None = None

    def initialise(self) -> None:
        self._result = None
        self._error = None
        obj_pc = self.bb.object_pointcloud
        scene_pc = getattr(self.bb, "scene_pointcloud", None)

        def _run() -> None:
            try:
                self._result = self.module.get_rpc_calls(
                    "PickAndPlaceModule.generate_grasps"
                )(obj_pc, scene_pc, rpc_timeout=self.rpc_timeout)
            except Exception as e:
                self._error = e

        self._thread = threading.Thread(target=_run, daemon=True, name="GenerateGrasps-RPC")
        self._thread.start()
        logger.info("[GenerateGrasps] Started grasp generation (may include Docker startup)")

    def update(self) -> Status:
        # Thread still running — Docker may be initializing
        if self._thread is not None and self._thread.is_alive():
            return Status.RUNNING

        # Thread completed — check results
        if self._error is not None:
            logger.warning(f"[GenerateGrasps] GraspGen RPC failed: {self._error}")
            self.bb.error_message = f"Error: GraspGen failed — {self._error}"
            return Status.FAILURE

        result = self._result
        if result is not None and result.poses:
            self.bb.grasp_candidates = list(result.poses)
            self.bb.grasp_index = 0
            logger.info(f"[GenerateGrasps] Generated {len(result.poses)} DL grasps")
            return Status.SUCCESS

        logger.warning("[GenerateGrasps] GraspGen returned no usable grasps")
        self.bb.error_message = "Error: GraspGen returned no grasps"
        return Status.FAILURE

    def terminate(self, new_status: Status) -> None:
        if new_status == Status.INVALID:
            # BT interrupted — thread may still be running in background.
            # We can't cancel a blocking RPC, but we detach our reference
            # so results are discarded on next initialise().
            self._thread = None
            self._result = None
            self._error = None

class GenerateHeuristicGrasps(ManipulationAction):
    """Fallback: single top-down grasp from the detection center (pitch=pi)."""

    def __init__(self, name: str, module: PickPlaceModule) -> None:
        super().__init__(name, module)
        self.bb.register_key(key="target_object", access=py_trees.common.Access.READ)
        self.bb.register_key(key="grasp_candidates", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="grasp_index", access=py_trees.common.Access.WRITE)

    def update(self) -> Status:
        target: dict[str, Any] = self.bb.target_object
        center: list[float] | None = target.get("center")
        if center is None:
            self.bb.error_message = "Error: No detection center for heuristic grasp"
            return Status.FAILURE

        cx, cy, cz = float(center[0]), float(center[1]), float(center[2])
        grasp_pose = Pose(
            Vector3(cx, cy, cz),
            Quaternion.from_euler(Vector3(0.0, math.pi, 0.0)),
        )
        self.bb.grasp_candidates = [grasp_pose]
        self.bb.grasp_index = 0
        logger.info(
            f"[GenerateHeuristicGrasps] Top-down grasp at "
            f"({cx:.3f}, {cy:.3f}, {cz:.3f})"
        )
        return Status.SUCCESS

class VisualizeGrasps(ManipulationAction):
    """Render grasp candidates in MeshCat (best-effort, always SUCCESS)."""

    def __init__(self, name: str, module: PickPlaceModule) -> None:
        super().__init__(name, module)
        self.bb.register_key(key="grasp_candidates", access=py_trees.common.Access.READ)

    def update(self) -> Status:
        candidates = self.bb.grasp_candidates
        if not candidates:
            return Status.SUCCESS
        try:
            self.module.get_rpc_calls("PickAndPlaceModule.visualize_grasps")(candidates)
        except Exception as e:
            logger.warning(f"[VisualizeGrasps] Visualization failed (non-fatal): {e}")
        return Status.SUCCESS

class AdaptGrasps(ManipulationAction):
    """Adapt grasps from source gripper frame to target gripper frame. Always SUCCESS."""

    def __init__(
        self,
        name: str,
        module: PickPlaceModule,
        source_gripper: str = "robotiq_2f_140",
        target_gripper: str = "ufactory_xarm",
    ) -> None:
        super().__init__(name, module)
        self.bb.register_key(key="grasp_candidates", access=py_trees.common.Access.READ)
        self.bb.register_key(key="grasp_candidates", access=py_trees.common.Access.WRITE)
        self._adapter = GripperAdapter(source=source_gripper, target=target_gripper)

    def update(self) -> Status:
        candidates = self.bb.grasp_candidates
        if not candidates:
            return Status.SUCCESS

        adapted = self._adapter.adapt_grasps(candidates)
        self.bb.grasp_candidates = adapted
        return Status.SUCCESS

class FilterGraspWorkspace(ManipulationAction):
    """Filter grasps by min-Z, max-distance, and approach angle. Sorts by quality."""

    def __init__(self, name: str, module: PickPlaceModule) -> None:
        super().__init__(name, module)
        self.bb.register_key(key="grasp_candidates", access=py_trees.common.Access.READ)
        self.bb.register_key(key="grasp_candidates", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="grasp_index", access=py_trees.common.Access.WRITE)

    def update(self) -> Status:
        cfg = self.module.config
        candidates: list[Pose] = list(self.bb.grasp_candidates)
        total = len(candidates)
        if total == 0:
            return Status.SUCCESS

        cos_threshold = math.cos(cfg.max_approach_angle)
        passed: list[tuple[float, Pose]] = []
        rej_z = rej_dist = rej_angle = 0

        for pose in candidates:
            if pose.position.z < cfg.min_grasp_z:
                rej_z += 1
                continue
            if pose.position.magnitude() > cfg.max_grasp_distance:
                rej_dist += 1
                continue
            # Approach direction Z-component from quaternion: rot[2,2] = 1 - 2(qx² + qy²)
            qx, qy = pose.orientation.x, pose.orientation.y
            approach_z = 1.0 - 2.0 * (qx * qx + qy * qy)
            if approach_z > -cos_threshold:
                rej_angle += 1
                continue
            passed.append((approach_z, pose))

        # Sort by approach quality — most top-down first (lowest approach_z)
        passed.sort(key=lambda t: t[0])
        filtered = [p for _, p in passed]

        logger.info(
            f"[FilterGraspWorkspace] {len(filtered)}/{total} passed "
            f"(rejected: {rej_z} below-Z, {rej_dist} beyond-reach, "
            f"{rej_angle} steep-angle)"
        )

        if not filtered:
            self.bb.error_message = (
                f"Error: No grasps in workspace — {rej_z} below table, "
                f"{rej_dist} beyond reach, {rej_angle} steep approach"
            )
            return Status.FAILURE

        self.bb.grasp_candidates = filtered
        self.bb.grasp_index = 0
        return Status.SUCCESS

class SelectNextGrasp(ManipulationAction):
    """Select the next grasp candidate from the ranked list."""

    def __init__(self, name: str, module: PickPlaceModule) -> None:
        super().__init__(name, module)
        self.bb.register_key(key="grasp_candidates", access=py_trees.common.Access.READ)
        self.bb.register_key(key="grasp_index", access=py_trees.common.Access.READ)
        self.bb.register_key(key="grasp_index", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="current_grasp", access=py_trees.common.Access.WRITE)
        self._logged_exhausted = False

    def update(self) -> Status:
        candidates: list[Pose] = self.bb.grasp_candidates
        idx: int = self.bb.grasp_index

        if idx >= len(candidates):
            self.bb.error_message = "Error: All grasp candidates exhausted"
            if not self._logged_exhausted:
                logger.warning(f"[SelectNextGrasp] Exhausted {len(candidates)} candidates")
                self._logged_exhausted = True
            return Status.FAILURE

        self.bb.current_grasp = candidates[idx]
        self.bb.grasp_index = idx + 1
        logger.info(f"[SelectNextGrasp] Selected candidate {idx + 1}/{len(candidates)}")
        return Status.SUCCESS

class ComputePreGrasp(ManipulationAction):
    """Compute pre-grasp pose offset along approach direction."""

    def __init__(self, name: str, module: PickPlaceModule) -> None:
        super().__init__(name, module)
        self.bb.register_key(key="current_grasp", access=py_trees.common.Access.READ)
        self.bb.register_key(key="pre_grasp_pose", access=py_trees.common.Access.WRITE)

    def update(self) -> Status:
        grasp_pose: Pose = self.bb.current_grasp
        offset = self.module.config.pre_grasp_offset
        self.bb.pre_grasp_pose = offset_distance(grasp_pose, offset)
        return Status.SUCCESS

# --- Motion actions ---

class PlanToPose(ManipulationAction):
    """Plan collision-free path to a pose from the blackboard.

    The target pose is read from ``bb.{pose_key}``.
    """

    def __init__(self, name: str, module: PickPlaceModule, pose_key: str) -> None:
        super().__init__(name, module)
        self.pose_key = pose_key
        self.bb.register_key(key=pose_key, access=py_trees.common.Access.READ)

    def update(self) -> Status:
        try:
            pose = getattr(self.bb, self.pose_key)
            logger.info(
                f"[PlanToPose] Planning to {self.pose_key} "
                f"({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})"
            )
            self.module.get_rpc_calls("PickAndPlaceModule.reset")()
            result = self.module.get_rpc_calls("PickAndPlaceModule.plan_to_pose")(pose)
            if result:
                return Status.SUCCESS
            self.bb.error_message = f"Error: Planning to {self.pose_key} failed"
            return Status.FAILURE
        except Exception as e:
            self.bb.error_message = f"Error: Planning exception — {e}"
            logger.error(f"[PlanToPose] {e}")
            return Status.FAILURE

class PlanToJoints(ManipulationAction):
    """Plan collision-free path to joint configuration from the blackboard."""

    def __init__(self, name: str, module: PickPlaceModule, joints_key: str) -> None:
        super().__init__(name, module)
        self.joints_key = joints_key
        self.bb.register_key(key=joints_key, access=py_trees.common.Access.READ)

    def update(self) -> Status:
        try:
            joints = getattr(self.bb, self.joints_key)
            if isinstance(joints, list):
                joints = JointState(position=joints)
            logger.info(f"[PlanToJoints] Planning to {self.joints_key}")
            self.module.get_rpc_calls("PickAndPlaceModule.reset")()
            result = self.module.get_rpc_calls("PickAndPlaceModule.plan_to_joints")(joints)
            if result:
                return Status.SUCCESS
            self.bb.error_message = f"Error: Joint planning to {self.joints_key} failed"
            return Status.FAILURE
        except Exception as e:
            self.bb.error_message = f"Error: Joint planning exception — {e}"
            logger.error(f"[PlanToJoints] {e}")
            return Status.FAILURE

class ExecuteTrajectory(ManipulationAction):
    """Execute planned trajectory. Returns RUNNING while executing, cancels on interrupt."""

    def __init__(
        self,
        name: str,
        module: PickPlaceModule,
        timeout: float = 60.0,
    ) -> None:
        super().__init__(name, module)
        self.timeout = timeout
        self._start_time: float = 0.0
        self._execute_ok: bool = False
        self._seen_executing: bool = False

    def initialise(self) -> None:
        self._seen_executing = False
        try:
            result = self.module.get_rpc_calls("PickAndPlaceModule.execute")()
            self._execute_ok = bool(result)
        except Exception as e:
            logger.error(f"[ExecuteTrajectory] Execute call failed: {e}")
            self._execute_ok = False
        self._start_time = time.time()

    def update(self) -> Status:
        if not self._execute_ok:
            self.bb.error_message = "Error: Trajectory execution send failed"
            return Status.FAILURE

        try:
            status = self.module.get_rpc_calls("PickAndPlaceModule.get_trajectory_status")()
        except Exception as e:
            logger.warning(f"[ExecuteTrajectory] Status poll failed: {e}")
            status = None

        if status is not None:
            state_val = int(status.get("state", -1)) if isinstance(status, dict) else int(status)

            # Track whether we've seen EXECUTING at least once to handle
            # the race where IDLE is reported before the coordinator
            # transitions to EXECUTING after execute().
            if state_val == TrajectoryState.EXECUTING:
                self._seen_executing = True
                return Status.RUNNING
            if state_val == TrajectoryState.COMPLETED:
                return Status.SUCCESS
            if state_val == TrajectoryState.IDLE:
                if self._seen_executing:
                    return Status.SUCCESS
                return Status.RUNNING
            if state_val in (TrajectoryState.ABORTED, TrajectoryState.FAULT):
                self.bb.error_message = f"Error: Trajectory execution failed (state={TrajectoryState(state_val).name})"
                return Status.FAILURE

        if time.time() - self._start_time > self.timeout:
            self.bb.error_message = "Error: Trajectory execution timed out"
            return Status.FAILURE

        return Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        if new_status == Status.INVALID:
            try:
                self.module.get_rpc_calls("PickAndPlaceModule.cancel")()
                logger.info("[ExecuteTrajectory] Cancelled trajectory on interrupt")
            except Exception as e:
                logger.warning(f"[ExecuteTrajectory] Cancel on interrupt failed (best-effort): {e}")

# --- Gripper actions ---

class SetGripper(ManipulationAction):
    """Set gripper to target position. Returns RUNNING during settle, then SUCCESS."""

    def __init__(
        self,
        name: str,
        module: PickPlaceModule,
        position: float,
        settle_time: float = 0.5,
    ) -> None:
        super().__init__(name, module)
        self.position = position
        self.settle_time = settle_time
        self._start_time: float = 0.0
        self._command_sent: bool = False

    def initialise(self) -> None:
        try:
            self.module.get_rpc_calls("PickAndPlaceModule.set_gripper")(self.position)
            self._command_sent = True
        except Exception as e:
            logger.error(f"[SetGripper] Command failed: {e}")
            self._command_sent = False
        self._start_time = time.time()

    def update(self) -> Status:
        if not self._command_sent:
            self.bb.error_message = "Error: Gripper command failed"
            return Status.FAILURE

        if time.time() - self._start_time >= self.settle_time:
            logger.info(f"[SetGripper] Gripper set to {self.position:.2f}m")
            return Status.SUCCESS
        return Status.RUNNING

# --- Utility actions ---

class StorePickPosition(ManipulationAction):
    """Store the current grasp position for place_back()."""

    def __init__(self, name: str, module: PickPlaceModule) -> None:
        super().__init__(name, module)
        self.bb.register_key(key="current_grasp", access=py_trees.common.Access.READ)

    def update(self) -> Status:
        grasp: Pose = self.bb.current_grasp
        self.module._last_pick_position = grasp.position
        logger.info(
            f"[StorePickPosition] Stored ({grasp.position.x:.3f}, "
            f"{grasp.position.y:.3f}, {grasp.position.z:.3f})"
        )
        return Status.SUCCESS

class ComputePlacePose(ManipulationAction):
    """Compute top-down place pose and pre-place offset."""

    def __init__(
        self, name: str, module: PickPlaceModule, x: float, y: float, z: float
    ) -> None:
        super().__init__(name, module)
        self.x = x
        self.y = y
        self.z = z
        self.bb.register_key(key="place_pose", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="pre_place_pose", access=py_trees.common.Access.WRITE)

    def update(self) -> Status:
        place_pose = Pose(
            Vector3(self.x, self.y, self.z),
            Quaternion.from_euler(Vector3(0.0, math.pi, 0.0)),
        )
        pre_place_pose = offset_distance(place_pose, self.module.config.pre_grasp_offset)
        self.bb.place_pose = place_pose
        self.bb.pre_place_pose = pre_place_pose
        logger.info(
            f"[ComputePlacePose] Place at ({self.x:.3f}, {self.y:.3f}, {self.z:.3f})"
        )
        return Status.SUCCESS

class SetResultMessage(ManipulationAction):
    """Set a result message on the blackboard."""

    def __init__(self, name: str, module: PickPlaceModule, message: str) -> None:
        super().__init__(name, module)
        self.message = message

    def update(self) -> Status:
        self.bb.result_message = self.message
        return Status.SUCCESS

# --- Robot state actions ---

class ResetRobot(ManipulationAction):
    """Call PickAndPlaceModule.reset() to clear fault/abort state."""

    def update(self) -> Status:
        try:
            result = self.module.get_rpc_calls("PickAndPlaceModule.reset")()
            if result:
                logger.info("[ResetRobot] Reset succeeded")
                return Status.SUCCESS
            logger.warning("[ResetRobot] Reset returned False")
            self.bb.error_message = "Error: Robot reset failed"
            return Status.FAILURE
        except Exception as e:
            logger.error(f"[ResetRobot] Reset RPC failed: {e}")
            self.bb.error_message = f"Error: Robot reset exception — {e}"
            return Status.FAILURE

class CancelMotion(ManipulationAction):
    """Cancel active motion and wait for settle. Always SUCCESS."""

    def __init__(
        self, name: str, module: PickPlaceModule, settle_time: float = 0.5
    ) -> None:
        super().__init__(name, module)
        self.settle_time = settle_time
        self._start_time: float = 0.0
        self._cancel_sent: bool = False

    def initialise(self) -> None:
        self._cancel_sent = False
        self._start_time = time.time()

    def update(self) -> Status:
        if not self._cancel_sent:
            try:
                self.module.get_rpc_calls("PickAndPlaceModule.cancel")()
                logger.info("[CancelMotion] Cancel sent, waiting for settle")
            except Exception as e:
                logger.warning(f"[CancelMotion] Cancel failed (best-effort): {e}")
            self._cancel_sent = True
            self._start_time = time.time()

        if time.time() - self._start_time >= self.settle_time:
            return Status.SUCCESS
        return Status.RUNNING

class ClearGraspState(ManipulationAction):
    """Reset all grasp-related blackboard keys for a fresh rescan. Always SUCCESS."""

    def __init__(self, name: str, module: PickPlaceModule) -> None:
        super().__init__(name, module)
        self.bb.register_key(key="detections", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="target_object", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="object_pointcloud", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="scene_pointcloud", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="grasp_candidates", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="grasp_index", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="current_grasp", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="pre_grasp_pose", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="has_object", access=py_trees.common.Access.WRITE)

    def update(self) -> Status:
        self.bb.detections = []
        self.bb.target_object = None
        self.bb.object_pointcloud = None
        self.bb.scene_pointcloud = None
        self.bb.grasp_candidates = []
        self.bb.grasp_index = 0
        self.bb.current_grasp = None
        self.bb.pre_grasp_pose = None
        self.bb.has_object = False
        return Status.SUCCESS

class SetHasObject(ManipulationAction):
    """Write a fixed value to ``bb.has_object``. Always SUCCESS."""

    def __init__(self, name: str, module: PickPlaceModule, value: bool) -> None:
        super().__init__(name, module)
        self.value = value
        self.bb.register_key(key="has_object", access=py_trees.common.Access.WRITE)

    def update(self) -> Status:
        self.bb.has_object = self.value
        return Status.SUCCESS

class ProbeGripperState(ManipulationAction):
    """Query gripper and set ``bb.has_object``. Always SUCCESS."""

    def __init__(self, name: str, module: PickPlaceModule) -> None:
        super().__init__(name, module)
        self.bb.register_key(key="has_object", access=py_trees.common.Access.WRITE)

    def update(self) -> Status:
        try:
            pos = float(self.module.get_rpc_calls("PickAndPlaceModule.get_gripper")())
            threshold = self.module.config.gripper_grasp_threshold
            open_pos = self.module.config.gripper_open_position
            # Upper bound at 90% of open prevents false positive when gripper is wide open
            holding = threshold < pos < open_pos * 0.9
            self.bb.has_object = holding
            logger.info(f"[ProbeGripperState] pos={pos:.4f}m, holding={holding}")
        except Exception as e:
            logger.warning(f"[ProbeGripperState] get_gripper failed: {e}")
            self.bb.has_object = False
        return Status.SUCCESS

class ExhaustRetriesIfHolding(ManipulationAction):
    """If holding an object, clear grasp candidates to stop further retries. Always SUCCESS."""

    def __init__(self, name: str, module: PickPlaceModule) -> None:
        super().__init__(name, module)
        self.bb.register_key(key="has_object", access=py_trees.common.Access.READ)
        self.bb.register_key(key="grasp_candidates", access=py_trees.common.Access.WRITE)

    def update(self) -> Status:
        if self.bb.has_object:
            self.bb.grasp_candidates = []
            self.bb.error_message = "Error: Holding object but lift failed — retries exhausted"
            logger.warning("[ExhaustRetriesIfHolding] Object held — aborting retries")
        return Status.SUCCESS

# --- Lift / retreat actions ---

class ComputeLiftPose(ManipulationAction):
    """Compute lift pose: current EE + lift_height in Z. Falls back to bb.current_grasp."""

    def __init__(
        self, name: str, module: PickPlaceModule, lift_height: float | None = None
    ) -> None:
        super().__init__(name, module)
        self._lift_height = lift_height
        self.bb.register_key(key="current_grasp", access=py_trees.common.Access.READ)
        self.bb.register_key(key="lift_pose", access=py_trees.common.Access.WRITE)

    def update(self) -> Status:
        lift_h = self._lift_height if self._lift_height is not None else self.module.config.lift_height

        try:
            base_pose = self.module.get_rpc_calls("PickAndPlaceModule.get_ee_pose")()
        except Exception as e:
            logger.warning(f"[ComputeLiftPose] get_ee_pose failed, falling back to current_grasp: {e}")
            base_pose = None
        if base_pose is None:
            try:
                base_pose = self.bb.current_grasp
            except (KeyError, AttributeError):
                base_pose = None
        if base_pose is None:
            self.bb.error_message = "Error: No EE pose available for lift"
            return Status.FAILURE

        self.bb.lift_pose = Pose(
            Vector3(base_pose.position.x, base_pose.position.y, base_pose.position.z + lift_h),
            base_pose.orientation,
        )
        return Status.SUCCESS

class ComputeLocalRetreatPose(ManipulationAction):
    """Compute retreat pose: current EE + retreat_height in Z. FAILURE if no EE pose."""

    def __init__(
        self, name: str, module: PickPlaceModule, retreat_height: float | None = None
    ) -> None:
        super().__init__(name, module)
        self._retreat_height = retreat_height
        self.bb.register_key(key="retreat_pose", access=py_trees.common.Access.WRITE)

    def update(self) -> Status:
        height = self._retreat_height if self._retreat_height is not None else self.module.config.lift_height

        try:
            base_pose = self.module.get_rpc_calls("PickAndPlaceModule.get_ee_pose")()
        except Exception as e:
            logger.warning(f"[ComputeLocalRetreatPose] get_ee_pose failed: {e}")
            base_pose = None

        if base_pose is None:
            self.bb.error_message = "Error: Cannot determine EE position for retreat"
            return Status.FAILURE

        self.bb.retreat_pose = Pose(
            Vector3(base_pose.position.x, base_pose.position.y, base_pose.position.z + height),
            base_pose.orientation,
        )
        return Status.SUCCESS
