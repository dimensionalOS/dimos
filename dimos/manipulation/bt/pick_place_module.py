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

"""BT-driven PickPlaceModule for robust pick-and-place orchestration."""

from __future__ import annotations

import atexit
from pydantic import Field
import subprocess
import threading
import time
from typing import TYPE_CHECKING, Any

import py_trees

from dimos.agents.annotation import skill
from dimos.core.module import Module, ModuleConfig
from dimos.core.transport import pLCMTransport
from dimos.manipulation.bt.trees import build_go_home_tree, build_pick_tree, build_place_tree
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs import Vector3

logger = setup_logger()

def _cleanup_graspgen_containers() -> None:
    """Stop all running GraspGen Docker containers at exit."""
    try:
        result = subprocess.run(
            ["docker", "ps", "-q", "--filter", "name=dimos_graspgenmodule"],
            capture_output=True, text=True, timeout=5,
        )
        container_ids = [cid for cid in result.stdout.strip().split("\n") if cid]
        if container_ids:
            subprocess.run(["docker", "stop", *container_ids], capture_output=True, timeout=30)
            logger.info(f"[PickPlaceModule] Cleaned up {len(container_ids)} GraspGen container(s)")
    except Exception:
        pass

atexit.register(_cleanup_graspgen_containers)

class PickPlaceModuleConfig(ModuleConfig):
    """Configuration for BT-based PickPlaceModule."""

    # Grasp strategy — False skips DL grasps (no Docker/OSR), heuristic only
    use_dl_grasps: bool = True
    tick_rate_hz: float = 20.0
    max_pick_attempts: int = 3
    max_rescan_attempts: int = 3  # outer retry: rescan + regenerate grasps

    # Gripper parameters (meters) — override per-robot
    gripper_open_position: float = 0.85
    gripper_close_position: float = 0.0
    gripper_grasp_threshold: float = 0.005  # above = object present
    gripper_settle_time: float = 1.5

    # Timing
    execution_timeout: float | None = None  # optional hard upper bound for full BT run
    scan_duration: float = 1.0
    prompt_settle_time: float = 3.0  # seconds to wait after setting detection prompts

    # Gripper adaptation (GraspGen source → physical robot target)
    adapt_grasps: bool = True
    source_gripper: str = "robotiq_2f_140"
    target_gripper: str = "ufactory_xarm"

    # Approach / lift
    pre_grasp_offset: float = 0.10
    lift_height: float = 0.1

    # Pose verification tolerances
    pose_position_tolerance: float = 0.02
    pose_orientation_tolerance: float = 0.15  # radians (~8.6 deg)
    grasp_position_tolerance: float = 0.05  # more lenient than pre-grasp

    # Geometric workspace filter
    min_grasp_z: float = 0.05
    max_grasp_distance: float = 0.9
    max_approach_angle: float = 1.05  # radians (~60 deg)

    # Home joint configuration — resolved at runtime via get_init_joints RPC if empty
    home_joints: list[float] = Field(default_factory=list)

class PickPlaceModule(Module):
    """BT-orchestrated pick-and-place module.

    Exposes @skill methods to the agent (pick, place, stop) that build and tick
    py_trees Behavior Trees. Each BT calls PickAndPlaceModule / OSR / GraspGen
    RPCs. This module orchestrates PickAndPlaceModule, not replaces it.
    """

    default_config = PickPlaceModuleConfig
    config: PickPlaceModuleConfig

    rpc_calls: list[str] = [
        "PickAndPlaceModule.plan_to_pose",
        "PickAndPlaceModule.plan_to_joints",
        "PickAndPlaceModule.execute",
        "PickAndPlaceModule.get_trajectory_status",
        "PickAndPlaceModule.cancel",
        "PickAndPlaceModule.reset",
        "PickAndPlaceModule.refresh_obstacles",
        "PickAndPlaceModule.list_cached_detections",
        "PickAndPlaceModule.get_ee_pose",
        "PickAndPlaceModule.get_robot_info",
        "PickAndPlaceModule.get_init_joints",
        "PickAndPlaceModule.set_gripper",
        "PickAndPlaceModule.get_gripper",
        "PickAndPlaceModule.generate_grasps",
        "PickAndPlaceModule.visualize_grasps",
        "ObjectSceneRegistrationModule.set_prompts",
        "ObjectSceneRegistrationModule.get_object_pointcloud_by_name",
        "ObjectSceneRegistrationModule.get_object_pointcloud_by_object_id",
        "ObjectSceneRegistrationModule.get_full_scene_pointcloud",
    ]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._last_pick_position: Vector3 | None = None

        # Direct stop channel — CLI publishes here to bypass blocked agent thread
        self._stop_transport: pLCMTransport[bool] = pLCMTransport("/bt_stop")
        self._stop_transport.subscribe(self._on_stop_signal)

        # Result notification — publishes BT completion to agent via /human_input
        self._result_transport: pLCMTransport[str] = pLCMTransport("/human_input")

    def _on_stop_signal(self, _msg: object) -> None:
        """Handle direct stop signal from CLI (bypasses blocked agent)."""
        if self._lock.locked():
            self._stop_event.set()
            logger.warning("[PickPlaceModule] Direct stop signal received from CLI")

    def _start_async(self, fn: Any, *args: object, **kwargs: object) -> str | None:
        """Run *fn* in a background thread; return None (started) or error string (busy).

        When *fn* finishes, the result is published to the agent via /human_input.
        """
        if not self._lock.acquire(blocking=False):
            return "Error: A pick/place operation is already running"
        self._stop_event.clear()

        def _worker() -> None:
            try:
                result = fn(*args, **kwargs)
            except Exception as e:
                logger.error(f"[PickPlaceModule] Async operation failed: {e}")
                result = f"Error: {e}"
            try:
                self._notify_agent(str(result))
            finally:
                self._lock.release()

        threading.Thread(target=_worker, daemon=True, name="PickPlaceBT").start()
        return None

    def _notify_agent(self, result: str) -> None:
        """Publish BT completion result to the agent via /human_input."""
        try:
            self._result_transport.publish(f"[Operation result] {result}")
        except Exception as e:
            logger.error(f"[PickPlaceModule] Failed to notify agent: {e}")

    def _resolve_home_joints(self) -> list[float]:
        """Return home joints: config if non-empty, else robot's init joints via RPC.

        Raises RuntimeError if neither source provides joints (fail loudly
        rather than silently sending wrong joint count to a different robot).
        """
        cfg_joints = self.config.home_joints
        if cfg_joints and any(j != 0.0 for j in cfg_joints):
            return cfg_joints

        try:
            init_js = self.get_rpc_calls("PickAndPlaceModule.get_init_joints")()
            if init_js is not None and init_js.position:
                return list(init_js.position)
        except Exception as e:
            logger.warning(f"[PickPlaceModule] Could not fetch init joints: {e}")

        if not cfg_joints:
            raise RuntimeError(
                "[PickPlaceModule] Cannot resolve home joints — config.home_joints is empty "
                "and get_init_joints RPC failed. Set home_joints in PickPlaceModuleConfig."
            )
        return cfg_joints

    def _abort_tree(self, root: py_trees.behaviour.Behaviour, open_gripper: bool = False) -> None:
        """Halt the BT and cancel any in-flight motion."""
        root.stop(py_trees.common.Status.INVALID)
        try:
            self.get_rpc_calls("PickAndPlaceModule.cancel")()
        except Exception:
            pass
        if open_gripper:
            try:
                self.get_rpc_calls("PickAndPlaceModule.set_gripper")(self.config.gripper_open_position)
            except Exception:
                pass

    @staticmethod
    def _read_blackboard(key: str, fallback: str) -> str:
        """Read a single blackboard key, returning *fallback* on any error."""
        try:
            bb = py_trees.blackboard.Client(name="BBReader")
            bb.register_key(key=key, access=py_trees.common.Access.READ)
            return str(getattr(bb, key))
        except (KeyError, AttributeError):
            return fallback

    def _tick_tree(self, root: py_trees.behaviour.Behaviour) -> str:
        """Run a BT tick loop until SUCCESS, FAILURE, timeout, or stop."""
        # Reset blackboard to prevent stale data from a previous run
        bb = py_trees.blackboard.Client(name="TreeReset")
        for key, default in {
            "detections": [], "target_object": None, "object_pointcloud": None,
            "scene_pointcloud": None, "grasp_candidates": [], "grasp_index": 0,
            "current_grasp": None, "pre_grasp_pose": None, "place_pose": None,
            "pre_place_pose": None, "lift_pose": None, "retreat_pose": None,
            "has_object": False, "error_message": "", "result_message": "",
        }.items():
            bb.register_key(key=key, access=py_trees.common.Access.WRITE)
            setattr(bb, key, default)

        tree = py_trees.trees.BehaviourTree(root=root)
        tree.setup()

        start = time.time()
        period = 1.0 / self.config.tick_rate_hz

        while True:
            if self._stop_event.is_set():
                self._abort_tree(root, open_gripper=True)
                return "Operation stopped by user"

            tree.tick()

            if root.status == py_trees.common.Status.SUCCESS:
                return self._read_blackboard("result_message", "Operation completed successfully")

            if root.status == py_trees.common.Status.FAILURE:
                return self._read_blackboard("error_message", "Error: Operation failed")

            if (
                self.config.execution_timeout is not None
                and time.time() - start > self.config.execution_timeout
            ):
                self._abort_tree(root)
                return "Error: Operation timed out"

            time.sleep(period)

    def _exec_pick(
        self, object_name: str, object_id: str | None = None, max_attempts: int | None = None,
    ) -> str:
        """Synchronous pick implementation (runs in background thread)."""
        attempts = max_attempts or self.config.max_pick_attempts
        strategy = "DL+heuristic" if self.config.use_dl_grasps else "heuristic-only"
        logger.info(
            f"[PickPlaceModule] pick('{object_name}', id={object_id}, "
            f"attempts={attempts}, strategy={strategy})"
        )

        root = build_pick_tree(
            module=self, object_name=object_name, object_id=object_id,
            max_attempts=attempts, home_joints_override=self._resolve_home_joints(),
        )
        return self._tick_tree(root)

    @skill
    def pick(
        self, object_name: str, object_id: str | None = None, max_attempts: int | None = None,
    ) -> str | None:
        """Pick up an object using BT-orchestrated grasp with DL-based grasp generation.

        Runs asynchronously — returns immediately while the BT executes in the
        background. Type ``stop`` to cancel.
        """
        return self._start_async(self._exec_pick, object_name, object_id, max_attempts)

    def _exec_place(self, x: float, y: float, z: float) -> str:
        """Synchronous place implementation (runs in background thread)."""
        logger.info(f"[PickPlaceModule] place({x:.3f}, {y:.3f}, {z:.3f})")
        root = build_place_tree(module=self, x=x, y=y, z=z)
        return self._tick_tree(root)

    @skill
    def place(self, x: float, y: float, z: float) -> str | None:
        """Place the held object at the specified position (meters, world frame).

        Runs asynchronously. Type ``stop`` to cancel.
        """
        return self._start_async(self._exec_place, x, y, z)

    @skill
    def place_back(self) -> str | None:
        """Place the held object back at its original pick position.

        ONLY call when the user explicitly asks to return/put back the object.
        For "go home" or "return home", use go_home instead (keeps the object).
        Runs asynchronously. Type ``stop`` to cancel.
        """
        if self._last_pick_position is None:
            return "Error: No stored pick position — pick an object first"
        pos = self._last_pick_position
        return self._start_async(self._exec_place, pos.x, pos.y, pos.z)

    @skill
    def pick_and_place(
        self, object_name: str, place_x: float, place_y: float, place_z: float,
        object_id: str | None = None, max_attempts: int | None = None,
    ) -> str | None:
        """Pick an object and place it at the target location.

        Sequentially runs pick then place in the background.
        Type ``stop`` to cancel at any time.
        """
        def _run() -> str:
            pick_result = self._exec_pick(object_name, object_id, max_attempts)
            if self._stop_event.is_set() or pick_result.startswith("Error"):
                return pick_result
            return self._exec_place(place_x, place_y, place_z)

        return self._start_async(_run)

    @skill
    def go_home(self) -> str | None:
        """Move to the home/safe position while keeping any held object.

        Does NOT open the gripper if holding — only opens when nothing is held.
        Use for "go back", "come home", "return", etc. Runs asynchronously.
        """
        def _run() -> str:
            logger.info("[PickPlaceModule] go_home()")
            root = build_go_home_tree(module=self, home_joints_override=self._resolve_home_joints())
            return self._tick_tree(root)

        return self._start_async(_run)

    @skill
    def stop(self) -> str:
        """Emergency stop — cancel all motion and open gripper. Safe to call at any time."""
        if self._lock.locked():
            self._stop_event.set()
            logger.warning("[PickPlaceModule] Stop requested — halting BT")
            return "Stop signal sent — operation will halt on next tick"

        try:
            self.get_rpc_calls("PickAndPlaceModule.cancel")()
        except Exception:
            pass
        try:
            self.get_rpc_calls("PickAndPlaceModule.set_gripper")(self.config.gripper_open_position)
        except Exception:
            pass
        return "Robot stopped — no operation was running"
