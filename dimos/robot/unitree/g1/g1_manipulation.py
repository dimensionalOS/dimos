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

"""G1-aware ManipulationModule that keeps the planning world's pelvis on /odom.

The G1 catalog uses ``weld_base=False`` so the planning world (Drake or
MuJoCo backend) treats the pelvis as a floating body.  Before each
Cartesian plan we push the latest ``/odom`` pose into the world via
``set_floating_base_pose`` — that way the planning world frame matches
the simulator's world frame and the parent ``move_to_pose`` / ``pick`` /
``refresh_obstacles`` paths can use world coordinates throughout (no
per-skill frame conversions).
"""

from __future__ import annotations

import threading
from typing import Any, Literal

import numpy as np
from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.experimental.pimsim.entity import EntityStateBatch
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.manipulation.pointing import solve_pointing
from dimos.manipulation.scene_objects import ObjectPoseSource, PrivilegedObjectSource
from dimos.msgs.geometry_msgs.Point import Point
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import (
    compose_transforms,
    invert_transform,
    matrix_to_pose,
    pose_to_matrix,
)

logger = setup_logger()


class G1ManipulationModule(PickAndPlaceModule):
    """PickAndPlaceModule that syncs the planning world's pelvis to /odom.

    All Cartesian skills inherited from PickAndPlaceModule (move_to_pose,
    pick, place, drop_on, refresh_obstacles, look, scan_objects, …) work
    unmodified — they all consume world-frame coordinates and the planning
    world (Drake or MuJoCo backend) has the pelvis at the live /odom pose
    for the duration of each plan.
    """

    odom: In[PoseStamped]
    # Interactive trigger: anything that publishes a PointStamped here
    # (typically the Viser "Set point goal" button) drives a full
    # reset-and-point cycle on the configured arm. Decoupled from MCP so
    # a human can drive pointing without going through the agent loop.
    point_goal: In[PointStamped]
    # Interactive grasp trigger: a world-frame point (e.g. a Babylon click)
    # resolves to the nearest scene object and the arm reaches for it. Same
    # source-blind path as the ``grasp_object`` skill — see ``_on_grasp_goal``.
    grasp_goal: In[PointStamped]
    # Reactive arm IK target for the coordinator's mink_arms QP task. grasp_object
    # publishes a pelvis-frame wrist pose here (frame_id ``mink_arms/<side>_ee``);
    # mink servos the arm to it. Bypasses the jacobian planner — mink is a QP that
    # always returns a best-effort solution, where the local jacobian IK fails.
    cartesian_command: Out[PoseStamped]

    _latest_odom: PoseStamped | None
    _odom_lock: threading.Lock
    _object_source: ObjectPoseSource

    def __init__(self, *, sim_mjcf_path: str | None = None, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._latest_odom = None
        self._odom_lock = threading.Lock()
        # Source-blind object resolver: turns a click / object-id into the
        # object's current world pose, which the (already source-blind) reach
        # consumes. Phase 1 uses the privileged sim ground-truth source; the
        # real-robot path swaps this for a perceived source behind the same
        # ObjectPoseSource interface, leaving the grasp path unchanged.
        self._object_source = PrivilegedObjectSource()
        # Easy-mode "ground truth" object lookup: when set, lets the
        # reach_for_sim_object skill pull body world poses straight from
        # the MJCF instead of going through perception.  Lazy-loaded.
        self._sim_mjcf_path = sim_mjcf_path
        self._sim_model: Any = None
        self._sim_data: Any = None

    @rpc
    def start(self) -> None:
        super().start()
        try:
            unsub = self.odom.subscribe(self._on_odom)
            self.register_disposable(Disposable(unsub))
        except Exception as e:
            logger.warning(f"G1ManipulationModule: odom subscribe failed: {e}")
        try:
            unsub = self.point_goal.subscribe(self._on_point_goal)
            self.register_disposable(Disposable(unsub))
        except Exception as e:
            logger.warning(f"G1ManipulationModule: point_goal subscribe failed: {e}")
        try:
            unsub = self.grasp_goal.subscribe(self._on_grasp_goal)
            self.register_disposable(Disposable(unsub))
        except Exception as e:
            logger.warning(f"G1ManipulationModule: grasp_goal subscribe failed: {e}")

    def _on_entity_states(self, msg: EntityStateBatch) -> None:
        """Forward entity poses to the planning world *and* the object resolver.

        The parent uses the batch only for collision (``sync_entity_poses``);
        we additionally cache it so a click / ``grasp_object`` can resolve an
        object's live world pose from the same authority-agnostic stream.
        """
        super()._on_entity_states(msg)
        try:
            self._object_source.update(msg)
        except Exception as e:
            logger.debug(f"object_source update failed: {e}")

    def _on_grasp_goal(self, msg: PointStamped) -> None:
        """Reach for the scene object nearest a clicked world point.

        Resolves the click to an object via the source-blind resolver, then
        servos the arm to it via mink (``grasp_object`` → ``_servo_grasp``).
        Fire-and-hold: it publishes one ``cartesian_command`` and returns; mink
        holds the last solution. Logged only.
        """
        try:
            result = self.grasp_object(x=msg.x, y=msg.y, z=msg.z)
            logger.info(f"grasp_goal → {result}")
        except Exception as e:
            logger.warning(f"grasp_goal handler failed: {e}")

    def _on_point_goal(self, msg: PointStamped) -> None:
        """Run point_at on the configured arm when a new goal arrives.

        Fired from the Viser viewer's "Set point goal" button (or any
        other producer of /point_goal). Runs in the subscriber thread —
        point_at is synchronous (sends two trajectories + waits) so this
        thread blocks for ~3-4 s per click; the In-stream queue absorbs
        rapid re-clicks. Result string is logged; no return value.
        """
        try:
            result = self.point_at(target=msg)
            logger.info(f"point_goal → {result}")
        except Exception as e:
            logger.warning(f"point_goal handler failed: {e}")

    def _on_odom(self, msg: PoseStamped) -> None:
        with self._odom_lock:
            self._latest_odom = msg
        # Sync pelvis into Drake on every odom tick so get_ee_pose
        # (used by go_init's safe-waypoint and any external query)
        # sees a live pose.  The set_floating_base_pose itself is
        # cheap; what was killing IK was the meshcat publish it
        # used to trigger — that's now gone (visualization will
        # update on the next plan).
        self._sync_floating_base()

    def _get_default_robot_name(self) -> str | None:
        # Base picks "single registered robot" else None — which makes every
        # skill fail with "Multiple robots configured" the moment two arms
        # are registered.  Prefer left_arm when both are present so the LLM
        # can call point_at / scan_objects / go_home etc. without having to
        # know about robot_name.  move_to_pose / point_at do their own
        # smarter target-side picking before this fallback runs.
        if "left_arm" in self._robots:
            return "left_arm"
        return super()._get_default_robot_name()

    def _begin_planning(self, robot_name: Any = None) -> Any:
        self._sync_floating_base()
        return super()._begin_planning(robot_name)

    def _sync_floating_base(self) -> None:
        if self._world_monitor is None:
            return
        with self._odom_lock:
            odom = self._latest_odom
        if odom is None:
            return
        world = self._world_monitor.world
        setter = getattr(world, "set_floating_base_pose", None)
        if setter is None:
            return
        for robot_name, (robot_id, _, _) in self._robots.items():
            try:
                setter(robot_id, odom)
            except Exception as e:
                logger.debug(f"set_floating_base_pose failed for {robot_name}: {e}")

    # ------------------------------------------------------------------
    # Easy mode: bypass perception, use MJCF ground-truth body positions
    # ------------------------------------------------------------------
    def _ensure_sim_model(self) -> bool:
        """Lazy-load the MJCF model used for ground-truth lookups."""
        if self._sim_model is not None:
            return True
        if not self._sim_mjcf_path:
            return False
        try:
            import mujoco
        except ImportError:
            logger.warning("mujoco not installed; reach_for_sim_object disabled")
            return False
        try:
            # The G1 MJCF references mesh STL/OBJs by bare filename
            # (Menagerie convention).  MujocoSimModule injects the
            # bytes via dimos.simulation.mujoco.model.get_assets — do
            # the same here so from_xml_string can find them without
            # depending on the working directory.
            from dimos.simulation.mujoco.model import get_assets

            assets = get_assets()
            with open(self._sim_mjcf_path) as f:
                xml_str = f.read()
            self._sim_model = mujoco.MjModel.from_xml_string(xml_str, assets=assets)
            self._sim_data = mujoco.MjData(self._sim_model)
            mujoco.mj_forward(self._sim_model, self._sim_data)
            logger.info(f"Sim ground-truth model loaded from {self._sim_mjcf_path}")
            return True
        except Exception as e:
            logger.warning(f"Failed to load sim model: {e}")
            return False

    @skill
    def point_at(
        self,
        target: PointStamped | PoseStamped,
        robot_name: str | None = None,
    ) -> str:
        """Aim the closer arm so the fingertip points at a world point.

        Closed-form heuristic — picks left or right arm based on which
        side of the body the target is on, then solves shoulder pitch/
        roll directly. Far faster and more predictable than IK; sub-ms
        compute, deterministic poses, no random restarts.

        Pass any stamped geometry message that carries a world-frame
        position — ``PointStamped`` from ``ObjectFinder3D`` / saved-object
        DBs, or a ``PoseStamped`` whose ``.position`` is the point of
        interest. Only the (x, y, z) is consumed; orientation is ignored
        (pointing is a ray, not a pose).

        For 6-DOF EE pose tracking (grasping etc.), use ``move_to_pose``.

        Args:
            target: World-frame point/pose to aim at. ``PointStamped`` is
                preferred (semantically a 3D point); ``PoseStamped`` is
                accepted for callers that already have one and don't want
                to strip orientation themselves.
            robot_name: Force a specific arm. Default is auto-select.
        """
        with self._odom_lock:
            pelvis = self._latest_odom
        if pelvis is None:
            return "Error: no /odom yet — robot pose unknown"

        # Extract (x, y, z) for the error message; solve_pointing pulls
        # them itself from the stamped target.
        if isinstance(target, PoseStamped):
            tx, ty, tz = target.position.x, target.position.y, target.position.z
        elif isinstance(target, Point):  # PointStamped is a Point subclass
            tx, ty, tz = target.x, target.y, target.z
        else:
            return f"Error: target must be PointStamped or PoseStamped, got {type(target).__name__}"

        side: Literal["left", "right", "auto"] = "auto"
        if robot_name == "left_arm":
            side = "left"
        elif robot_name == "right_arm":
            side = "right"

        sol = solve_pointing(target, pelvis, side=side)
        if sol is None:
            return (
                f"Error: ({tx:.2f}, {ty:.2f}, {tz:.2f}) is outside the arm's "
                f"pointing workspace (behind torso, or too far above the shoulder)."
            )

        chosen_robot = "left_arm" if sol.side == "left" else "right_arm"
        if chosen_robot not in self._robots:
            return f"Error: '{chosen_robot}' not registered"
        chosen_id, chosen_config, _ = self._robots[chosen_robot]

        if self._world_monitor is None:
            return "Error: planning world not initialized"
        world = self._world_monitor.world

        client = self._get_coordinator_client()
        if client is None or not chosen_config.coordinator_task_name:
            return "Error: coordinator client unavailable"

        # --- Phase 1: hard reset *both* arms to the all-zeros baseline ---
        # Symmetric reset (not just the pointing arm): keeps the idle arm
        # from accumulating drift across calls, and avoids the LLM seeing
        # the previous-pointed arm stay raised when point_at chooses the
        # other side on the next call. Trajectories are issued back-to-back
        # then awaited in parallel — the coordinator runs the two task
        # streams independently so we don't pay 2×duration.
        in_flight: list[tuple[str, np.ndarray]] = []  # (robot_name, q_zero)
        for arm_name in ("left_arm", "right_arm"):
            if arm_name not in self._robots:
                continue
            rid, cfg, _ = self._robots[arm_name]
            with world.scratch_context() as ctx:
                seed = world.get_joint_state(ctx, rid)
            q_start = np.array(seed.position, dtype=np.float64)
            q_zero = np.zeros_like(q_start)
            # Fast-path: skip the trajectory if this arm is already at zero.
            if float(np.max(np.abs(q_start - q_zero))) <= 1e-3:
                continue
            reset_traj = self._build_arm_trajectory(
                cfg.joint_names, q_start, q_zero, duration=1.25, via_zero=False
            )
            reset_translated = self._translate_trajectory_to_coordinator(reset_traj, cfg)
            if not client.task_invoke(
                cfg.coordinator_task_name, "execute", {"trajectory": reset_translated}
            ):
                return f"Error: coordinator rejected reset trajectory for {arm_name}"
            in_flight.append((arm_name, q_zero))

        for arm_name, _ in in_flight:
            if not self._wait_for_trajectory_completion(arm_name, timeout=4.0):
                return f"Error: reset-to-zero trajectory for {arm_name} timed out"

        # --- Phase 2: point at the target from the zero baseline ---
        # After the reset, the chosen arm is at q_zero; build the
        # pointing trajectory from that baseline directly.
        q_zero_chosen = np.zeros(len(chosen_config.joint_names), dtype=np.float64)
        q_target = np.array(
            [sol.joints[name] for name in chosen_config.joint_names], dtype=np.float64
        )
        point_traj = self._build_arm_trajectory(
            chosen_config.joint_names,
            q_zero_chosen,
            q_target,
            duration=1.25,
            via_zero=False,
        )
        point_translated = self._translate_trajectory_to_coordinator(point_traj, chosen_config)
        if not client.task_invoke(
            chosen_config.coordinator_task_name, "execute", {"trajectory": point_translated}
        ):
            return "Error: coordinator rejected pointing trajectory"

        if not self._wait_for_trajectory_completion(chosen_robot, timeout=4.0):
            return "Error: pointing trajectory timed out"

        return f"Pointing {sol.side} arm at ({tx:.2f}, {ty:.2f}, {tz:.2f})"

    @staticmethod
    def _build_arm_trajectory(
        joint_names: list[str],
        q_start: np.ndarray,
        q_target: np.ndarray,
        duration: float = 2.5,
        n_waypoints: int = 9,
        via_zero: bool = True,
    ) -> JointTrajectory:
        """Cosine-smoothed JointTrajectory from q_start to q_target.

        With ``via_zero=True`` (default), the trajectory passes through
        the all-zeros pose at the midpoint — a forced reset between
        successive ``point_at`` calls so each pointing starts from the
        same baseline. Without this, repeated point_at calls compound
        residual joint errors.
        """
        if via_zero:
            q_mid = np.zeros_like(q_start)
            half_n = (n_waypoints + 1) // 2
            half_dur = duration / 2.0
            points: list[TrajectoryPoint] = []
            # Phase 1: q_start -> 0 (cosine-smoothed)
            for i in range(half_n):
                s = i / (half_n - 1) if half_n > 1 else 0.0
                alpha = 0.5 - 0.5 * float(np.cos(np.pi * s))
                q = q_start + alpha * (q_mid - q_start)
                points.append(TrajectoryPoint(positions=q.tolist(), time_from_start=s * half_dur))
            # Phase 2: 0 -> q_target (cosine-smoothed); skip first point
            # to avoid duplicating the zero waypoint at the seam.
            for i in range(1, n_waypoints - half_n + 1):
                s = i / (n_waypoints - half_n)
                alpha = 0.5 - 0.5 * float(np.cos(np.pi * s))
                q = q_mid + alpha * (q_target - q_mid)
                points.append(
                    TrajectoryPoint(
                        positions=q.tolist(),
                        time_from_start=half_dur + s * half_dur,
                    )
                )
            return JointTrajectory(joint_names=list(joint_names), points=points)

        points = []
        for i in range(n_waypoints):
            s = i / (n_waypoints - 1)
            alpha = 0.5 - 0.5 * float(np.cos(np.pi * s))
            q = q_start + alpha * (q_target - q_start)
            points.append(TrajectoryPoint(positions=q.tolist(), time_from_start=s * duration))
        return JointTrajectory(joint_names=list(joint_names), points=points)

    @skill
    def point_at_sim_object(
        self,
        body_name: str = "manip_cube",
        robot_name: str | None = None,
    ) -> str:
        """Easy-mode: point the arm at a sim object using its MJCF ground-truth pose.

        Same MJCF-bypass-perception approach as ``reach_for_sim_object``,
        but uses the (out-of-reach-tolerant) ``point_at`` skill instead
        of trying to grasp.  Useful for verifying the arm-aiming pipeline
        when the object is too far for the arm to reach.

        Args:
            body_name: MJCF body to point at (default 'manip_cube').
            robot_name: Robot to use (only needed for multi-arm setups).
        """
        if not self._ensure_sim_model():
            return "Easy mode unavailable: sim_mjcf_path not configured."
        import mujoco

        body_id = mujoco.mj_name2id(self._sim_model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if body_id < 0:
            return f"Body '{body_name}' not found in MJCF."
        pos = self._sim_data.xpos[body_id]
        x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
        logger.info(f"point_at_sim_object('{body_name}') → world ({x:.3f}, {y:.3f}, {z:.3f})")
        target = PointStamped(x=x, y=y, z=z, frame_id="map")
        return self.point_at(target=target, robot_name=robot_name)

    @skill
    def reach_for_sim_object(
        self,
        body_name: str = "manip_cube",
        robot_name: str | None = None,
    ) -> str:
        """Easy-mode: reach for a sim object using its MJCF ground-truth pose.

        Bypasses the perception pipeline (YOLO-E detection, RGBD
        back-projection, frame transforms) and instead reads the
        target body's world pose directly from the MuJoCo model.  Use
        this to isolate manipulation issues from perception issues —
        if this works but ``move_to_pose`` after ``detect`` doesn't,
        the bug is in perception.

        Args:
            body_name: MJCF body to reach for (default 'manip_cube').
            robot_name: Robot to use (only needed for multi-arm setups).
        """
        if not self._ensure_sim_model():
            return "Easy mode unavailable: sim_mjcf_path not configured."
        import mujoco

        body_id = mujoco.mj_name2id(self._sim_model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if body_id < 0:
            return f"Body '{body_name}' not found in MJCF."
        pos = self._sim_data.xpos[body_id]
        x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
        logger.info(f"reach_for_sim_object('{body_name}') → world ({x:.3f}, {y:.3f}, {z:.3f})")
        return self.move_to_pose(x=x, y=y, z=z, robot_name=robot_name)

    @rpc
    def list_scene_objects(self) -> list[dict[str, Any]]:
        """List the scene objects the resolver can currently reach for.

        Mirror of perception's scan/look but for the source-blind object
        resolver: returns every object known to the active ``ObjectPoseSource``
        (privileged sim ground-truth today) with its id, kind, and current
        world position. Pass an ``object_id`` from here to ``grasp_object``.
        """
        objects: list[dict[str, Any]] = []
        for obj in self._object_source.list_objects():
            ox, oy, oz = obj.xyz
            objects.append(
                {"object_id": obj.object_id, "kind": obj.object.kind, "x": ox, "y": oy, "z": oz}
            )
        return objects

    @skill
    def grasp_object(
        self,
        object_id: str | None = None,
        x: float | None = None,
        y: float | None = None,
        z: float | None = None,
        robot_name: str | None = None,
    ) -> str:
        """Reach for a scene object resolved from the live entity stream.

        Source-blind: the object's world pose comes from whatever
        ``ObjectPoseSource`` is wired (privileged sim ground-truth today,
        perception on the real robot) — the reach path is identical. Specify the
        target either by ``object_id`` (exact) or by a world point ``x, y, z``
        (the nearest known object is chosen — the click-to-grasp case).

        Reach-and-hold via the coordinator's ``mink_arms`` QP IK (reactive): the
        object's grasp pose is published as a pelvis-frame wrist target on
        ``cartesian_command`` and mink servos the chosen arm to it. This uses the
        same MuJoCo-native IK that already drives the arms in the WBC, instead of
        the manipulation planner's jacobian IK (which fails to converge on these
        poses). Trade-off: it is reactive (no RRT collision-aware routing) — fine
        for an unobstructed reach, and the path to swap in collision-aware
        planning is a proper QP ``KinematicsSpec`` (cc's ``pink-ik``). The G1 hand
        has no binary gripper, so this is reach-and-hold; finger close is later.

        Args:
            object_id: Exact id of an object in the entity stream.
            x, y, z: World-frame point; the nearest object is selected. Used
                when ``object_id`` is omitted.
            robot_name: Force a specific arm (default: by which side the object
                is on).
        """
        if object_id is not None:
            obj = self._object_source.resolve(object_id)
            if obj is None:
                known = ", ".join(o.object_id for o in self._object_source.list_objects()) or "none"
                return f"Error: no object '{object_id}' in scene (known: {known})"
        elif x is not None and y is not None and z is not None:
            obj = self._object_source.resolve(PointStamped(x=x, y=y, z=z, frame_id="map"))
            if obj is None:
                return f"Error: no scene object near ({x:.2f}, {y:.2f}, {z:.2f})"
        else:
            return "Error: provide object_id, or x/y/z of a world point to grasp near"

        ox, oy, oz = obj.xyz
        logger.info(
            f"grasp_object → '{obj.object_id}' center ({ox:.3f}, {oy:.3f}, {oz:.3f}); "
            f"servoing arm via mink"
        )
        result = self._servo_grasp(Vector3(ox, oy, oz), robot_name)
        return f"Grasping '{obj.object_id}': {result}"

    def _servo_grasp(self, target_world: Vector3, robot_name: str | None = None) -> str:
        """Drive an arm to a world-frame point via the mink_arms QP IK task.

        Transforms the point into the pelvis frame (mink solves with the floating
        base pinned at the pelvis) using the latest ``/odom``, picks the arm by
        which side the target is on, and commands the wrist there **keeping the
        arm's current orientation** — mink only translates. Commanding a forced
        approach orientation (the xArm grasp heuristic mapped into the pelvis
        frame) makes mink contort the wrist, which is the "twisted up" failure;
        an approach orientation + grasp-offset is a later refinement. Published on
        ``cartesian_command`` as ``frame_id="mink_arms/<side>_ee"``; mink holds the
        last solution, so this is fire-and-hold.
        """
        with self._odom_lock:
            pelvis = self._latest_odom
        if pelvis is None:
            return "Error: no /odom yet — robot pose unknown"

        # world → pelvis for the target point.
        inv_pelvis = invert_transform(pose_to_matrix(pelvis))
        tp = inv_pelvis @ np.array([target_world.x, target_world.y, target_world.z, 1.0])
        px, py, pz = float(tp[0]), float(tp[1]), float(tp[2])

        if robot_name in ("left_arm", "right_arm"):
            side = "left" if robot_name == "left_arm" else "right"
        else:
            # +y is the robot's left in the pelvis frame.
            side = "left" if py >= 0.0 else "right"
        arm = f"{side}_arm"

        # Keep the arm's current wrist orientation (world EE → pelvis frame) so
        # mink only translates to the target instead of twisting toward a pose.
        ee = self.get_ee_pose(arm)
        if ee is None:
            return f"Error: no end-effector pose for '{arm}'"
        orientation = matrix_to_pose(compose_transforms(inv_pelvis, pose_to_matrix(ee))).orientation

        command = PoseStamped(
            frame_id=f"mink_arms/{side}_ee",
            position=Vector3(px, py, pz),
            orientation=orientation,
        )
        try:
            self.cartesian_command.publish(command)
        except Exception as e:
            return f"Error: cartesian_command publish failed: {e}"
        return (
            f"{side} arm → pelvis-frame ({px:.3f}, {py:.3f}, {pz:.3f}) via mink (orientation held)"
        )
