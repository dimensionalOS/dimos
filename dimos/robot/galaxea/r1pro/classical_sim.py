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

"""Classical apartment sensing, feasibility and physical outcome checks."""

from dataclasses import asdict
import json
from pathlib import Path
import subprocess
import sys
import threading
import time
from typing import Any, Protocol, cast

import mujoco
import numpy as np
from numpy.typing import NDArray
from pydantic import TypeAdapter

from dimos.core.core import rpc
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.robot.galaxea.r1pro.apartment_navigation import ApartmentSimSpec
from dimos.robot.galaxea.r1pro.apartment_route import (
    apartment_approach,
    apartment_departure,
    refine_apartment_route,
)
from dimos.robot.galaxea.r1pro.apartment_sim import R1ProApartmentSim
from dimos.robot.galaxea.r1pro.classical_perception import segmented_object_cloud
from dimos.robot.galaxea.r1pro.classical_planning import ClassicalGraspPlanner
from dimos.robot.galaxea.r1pro.classical_tray import TRAY_DOCK_OFFSET, tray_footprint, tray_report
from dimos.robot.galaxea.r1pro.grasping_sim import VIRTUAL_BASE_JOINTS
from dimos.robot.galaxea.r1pro.grasping_transport import PlanarTransport
from dimos.robot.galaxea.r1pro.home_surfaces import station_name
from dimos.robot.galaxea.r1pro.learning import R1PRO_PICK_PLACE_JOINTS
from dimos.robot.galaxea.r1pro.navigation_base import PlanarVelocityServo
from dimos.robot.galaxea.r1pro.navigation_sim import carrying_offset
from dimos.robot.galaxea.r1pro.object_packing_scene import ObjectLayout
from dimos.robot.galaxea.r1pro.object_primitive_state import PrimitiveSceneState
from dimos.robot.galaxea.r1pro.object_primitives import ARMS, Arm
from dimos.robot.galaxea.r1pro.primitive_scene import placement_options
from dimos.robot.galaxea.r1pro.tray_motion import TrayMotion
from dimos.robot.galaxea.r1pro.tray_sim import configure_tray_holding
from dimos.simulation.engines.mujoco_engine import MujocoEngine
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class ClassicalSimSpec(ApartmentSimSpec, Protocol):
    def save_classical_state(self) -> str: ...
    def classical_carry_posture(self) -> list[list[float]]: ...
    def classical_init_posture(self) -> dict[str, Any]: ...
    def classical_move_arm(
        self, arm: str, target: list[list[float]], linear: bool
    ) -> list[list[float]]: ...
    def classical_align(
        self, index: int, arm: str, target: list[list[float]]
    ) -> list[list[float]]: ...
    def classical_posture(
        self, index: int, arm: str, positions: list[float], target: list[list[float]] | None = None
    ) -> list[list[float]]: ...
    def classical_object_cloud(self, index: int) -> PointCloud2: ...
    def classical_line(
        self, index: int, arm: str, target: list[list[float]]
    ) -> list[list[float]]: ...
    def classical_place_goals(self, arm: str, region: str) -> list[dict[str, Any]]: ...
    def begin_classical_pick_assessment(
        self, index: int, candidates: GraspCandidateArray, arm: str
    ) -> None: ...
    def classical_pick_assessment(self) -> dict[str, Any]: ...
    def cancel_classical_pick_assessment(self) -> None: ...
    def tray_state(self) -> dict[str, Any]: ...
    def tray_dock_pose(self, region: str | None = None) -> dict[str, Any]: ...
    def prepare_tray_holding(self) -> None: ...
    def plan_tray_motion(
        self, phase: str, target: list[float] | None = None
    ) -> list[dict[str, Any]]: ...
    def prepare_tray_approach(self) -> None: ...
    def prepare_tray_navigation(self, destination: str) -> dict[str, Any]: ...


class R1ProClassicalSim(R1ProApartmentSim):
    """Keep the physical apartment while replacing ACT with measured Cartesian plans."""

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._tray_transport: list[str] | None = None
        self._last_tray_check = float("-inf")
        self._assessment_worker: subprocess.Popen[bytes] | None = None
        self._assessment: Path | None = None
        self._assessment_warmed = False

    def _reachable_base_path(
        self, planner: PlanarTransport, pose: NDArray[Any]
    ) -> list[list[float]]:
        yaw = np.arctan2(np.sin(pose[2] - planner.start[2]), np.cos(pose[2] - planner.start[2]))
        if abs(yaw) > 0.02:
            return planner.plan_stance(pose.tolist(), separate_turns=True)
        return super()._reachable_base_path(planner, pose)

    @rpc
    def save_classical_state(self) -> str:
        """Record exact physical state and runtime model parameters for replay, including failures."""
        if self._engine is None:
            raise RuntimeError("Simulation is not ready")
        output = self.config.output / f"classical-state-{time.time_ns()}.npz"
        with self._engine._lock:
            self._save_model()
            scene = self._state(self._engine)
            np.savez(
                output,
                qpos=scene.data.qpos,
                qvel=scene.data.qvel,
                ctrl=scene.data.ctrl,
                time=scene.data.time,
                selected_left=scene.arms["left"].selected,
                selected_right=scene.arms["right"].selected,
            )
        return str(output)

    def _save_model(self) -> Path:
        """Write the compiled model once; callers hold the engine lock."""
        assert self._engine is not None
        model_path = self.config.output / "classical-model.mjb"
        if not model_path.exists():
            mujoco.mj_saveModel(self._engine.model, str(model_path), None)
        return model_path

    def _assessment_process(self) -> subprocess.Popen[bytes]:
        """One long-lived worker keeps the compiled model and kinematics world cached."""
        if self._assessment_worker is None or self._assessment_worker.poll() is not None:
            log = (self.config.output / "assessment-worker.log").open("ab")
            self._assessment_worker = subprocess.Popen(
                [sys.executable, "-m", "dimos.robot.galaxea.r1pro.classical_assessment", "--serve"],
                stdin=subprocess.PIPE,
                stdout=log,
                stderr=subprocess.STDOUT,
            )
        return self._assessment_worker

    def _kill_assessment_worker(self) -> None:
        worker = self._assessment_worker
        self._assessment_worker = None
        if worker is not None and worker.poll() is None:
            worker.kill()
            try:
                worker.wait(timeout=5)
            except subprocess.TimeoutExpired:
                logger.error("Assessment worker did not exit after kill")

    def _send_assessment(self, command: str) -> None:
        for attempt in range(2):
            worker = self._assessment_process()
            try:
                assert worker.stdin is not None
                worker.stdin.write(f"{command}\n".encode())
                worker.stdin.flush()
                return
            except (BrokenPipeError, OSError):
                self._kill_assessment_worker()
                if attempt:
                    raise

    @rpc
    def warm_classical_assessment(self) -> None:
        """Start the assessment worker and build its kinematics world before the first pick."""
        if self._engine is None:
            return
        with self._engine._lock:
            model_path = self._save_model()
        self._send_assessment(f"warm {model_path}")

    def _warm_assessment_once(self) -> None:
        """Build the assessment world while the demo starts, not during the first pick."""
        if self._assessment_warmed:
            return
        self._assessment_warmed = True
        threading.Thread(
            target=self.warm_classical_assessment, name="classical-assessment-warm", daemon=True
        ).start()

    @rpc
    def begin_classical_pick_assessment(
        self, index: int, candidates: GraspCandidateArray, arm: str
    ) -> None:
        """Rank GraspGenX proposals in the assessment worker; poll classical_pick_assessment."""
        if candidates.header.frame_id != "world":
            raise ValueError("Grasp proposals must be expressed in world")
        if (
            self._assessment is not None
            and not (self._assessment / "result.json").exists()
            and self._assessment_worker is not None
            and self._assessment_worker.poll() is None
        ):
            raise RuntimeError("A grasp assessment is already running")
        scene = self._snapshot()
        poses, scores = [], []
        for candidate in candidates.candidates:
            matrix = np.eye(4)
            matrix[:3, :3] = candidate.pose.orientation.to_rotation_matrix()
            matrix[:3, 3] = candidate.pose.position.to_numpy()
            poses.append(matrix)
            scores.append(candidate.score)
        assert self._engine is not None and self._layout is not None
        with self._engine._lock:
            model_path = self._save_model()
        request = self.config.output / f"classical-assessment-{time.time_ns()}"
        request.mkdir()
        np.savez(
            request / "request.npz",
            qpos=scene.data.qpos,
            qvel=scene.data.qvel,
            ctrl=scene.data.ctrl,
            poses=np.asarray(poses),
            scores=np.asarray(scores, dtype=float),
        )
        (request / "request.json").write_text(
            json.dumps(
                dict(
                    model=str(model_path),
                    layout=TypeAdapter(ObjectLayout).dump_python(self._layout),
                    home=list(self.config.reset_joint_positions or []),
                    index=index,
                    arm=arm,
                )
            )
        )
        self._assessment = request
        self._send_assessment(f"rank {request}")

    @rpc
    def classical_pick_assessment(self) -> dict[str, Any]:
        """Report the assessment as running, done with ranked options, or failed."""
        if self._assessment is None:
            raise RuntimeError("No grasp assessment was started")
        result_path = self._assessment / "result.json"
        if not result_path.exists():
            worker = self._assessment_worker
            if worker is None or worker.poll() is not None:
                log = self.config.output / "assessment-worker.log"
                tail = log.read_text(errors="replace")[-600:] if log.exists() else ""
                return dict(state="failed", error=f"Assessment worker exited: {tail}")
            return dict(state="running")
        result = json.loads(result_path.read_text())
        if "error" in result:
            return dict(state="failed", error=str(result["error"]))
        return dict(state="done", options=result["plans"])

    @rpc
    def cancel_classical_pick_assessment(self) -> None:
        """Kill an unfinished assessment; a stopped search must not keep burning a core."""
        if self._assessment is not None and not (self._assessment / "result.json").exists():
            self._kill_assessment_worker()

    @rpc
    def stop(self) -> None:
        self._kill_assessment_worker()
        super().stop()

    def _publish_shm_and_lcm(self, engine: MujocoEngine) -> None:
        with engine._lock:
            if self._servo is None:
                pose = np.array([engine.data.joint(name).qpos[0] for name in VIRTUAL_BASE_JOINTS])
                self._servo = PlanarVelocityServo(
                    pose, max_speed=0.3, max_accel=0.2, max_yaw_rate=0.4, max_yaw_accel=0.4
                )
        super()._publish_shm_and_lcm(engine)
        self._warm_assessment_once()
        now = time.monotonic()
        if self._tray_transport is not None and now - self._last_tray_check >= 0.05:
            with engine._lock:
                self._last_tray_check = now
                try:
                    self._check_tray_transport(self._state(engine))
                except RuntimeError as exc:
                    self._error = str(exc)

    def _check_tray_transport(self, scene: PrimitiveSceneState) -> dict[str, Any]:
        tray = tray_report(scene, self._regions)
        if not tray["bimanual_grasp"] or tray["tilt_radians"] > 0.25:
            raise RuntimeError("Lost the two-handed tray hold during transport")
        missing = set(self._tray_transport or ()) - set(tray["cargo"])
        if missing:
            raise RuntimeError(f"Cargo left the tray: {sorted(missing)}")
        return tray

    def _tray_planner(
        self, scene: PrimitiveSceneState, *, collision_margin: float
    ) -> PlanarTransport:
        return PlanarTransport(
            scene.model,
            scene.data,
            cargo_bodies=tuple(row["object"] for row in scene.inventory() if row["inside"]),
            carry_tray=True,
            sweep_spacing=0.005,
            collision_margin=collision_margin,
        )

    @rpc
    def tray_state(self) -> dict[str, Any]:
        """Measure tray contacts, cargo and the platform it rests on."""
        engine = self._engine
        if engine is None:
            raise RuntimeError("Simulation is not ready")
        with engine._lock:
            return tray_report(self._state(engine), self._regions)

    @rpc
    def tray_dock_pose(self, region: str | None = None) -> dict[str, Any]:
        """Base pose keeping the tray at its worktable offset; None docks at the resting tray."""
        scene = self._snapshot()
        tray = tray_report(scene, self._regions)
        offset = TRAY_DOCK_OFFSET
        if region is not None and tray["held"]:
            # A torso lift changes where the held tray sits; dock from the measured offset.
            base = scene.data.body("base_link")
            local = base.xmat.reshape(3, 3).T @ (np.asarray(tray["position"]) - base.xpos)
            offset = local[:2]
        if region is None:
            if tray["held"] or tray["finger_contacts"] or tray["station"] is None:
                raise RuntimeError("The tray must rest on a known platform before pickup")
            name = str(tray["station"])
            target = np.asarray(tray["position"], dtype=float)
            yaw = float(self._navigation_heading(name))
        else:
            name = station_name(region)
            name = "worktable" if name in ("worktop", "table") else name
            if name not in self._regions:
                raise ValueError("Use a measured platform name from get_surfaces")
            yaw = float(self._navigation_heading(name))
            target = tray_footprint(scene, self._regions[name], yaw)
            if tray["held"] and float(target[2]) > float(tray["position"][2]) - 0.02:
                raise ValueError(
                    f"{name} is {target[2]:.2f} m high; the carried tray bottom is at "
                    f"{tray['position'][2]:.2f} m. Choose a lower platform."
                )
        c, s = np.cos(yaw), np.sin(yaw)
        base = target[:2] - np.array([[c, -s], [s, c]]) @ offset
        # Platform legs block the nominal offset; stand off along the heading
        # until the robot, with its carried tray, is clear. The arms reach the rest.
        planner = (
            self._tray_planner(scene, collision_margin=0.02)
            if tray["held"]
            else PlanarTransport(
                scene.model, scene.data, cargo_bodies=(), carry_tray=False, sweep_spacing=0.005
            )
        )
        clear = False
        pose = np.array([*base, yaw])
        for back in np.arange(0.0, 0.41, 0.02):
            pose = np.array([base[0] - c * back, base[1] - s * back, yaw])
            if planner.clear_pose_segment(pose, pose):
                clear = True
                break
        return dict(
            region=name,
            target=target.tolist(),
            base_pose=[float(value) for value in pose],
            clear=clear,
            support_geoms=list(self._regions[name].support_geoms),
        )

    @rpc
    def prepare_tray_holding(self) -> None:
        """Steady the loaded torso before two-handed tray motion."""
        engine = self._engine
        if engine is None:
            raise RuntimeError("Simulation is not ready")
        with engine._lock:
            configure_tray_holding(engine.model)

    @rpc
    def plan_tray_motion(
        self, phase: str, target: list[float] | None = None
    ) -> list[dict[str, Any]]:
        """Compute checked bimanual waypoints in a snapshot; execution belongs to the coordinator."""
        scene = self._snapshot()
        cargo = tuple(row["object"] for row in scene.inventory() if row["inside"])
        motion = TrayMotion(scene.model, scene.data, cargo_bodies=cargo)
        if phase == "pickup":
            points = motion.pickup(scene.data)
        elif phase == "place" and target is not None and len(target) == 3:
            points = motion.placement(scene.data, target)
        else:
            raise ValueError("Expected pickup or place with a three-dimensional target")
        return [asdict(point) for point in points]

    @rpc
    def prepare_tray_approach(self) -> None:
        """Guard every object while the empty-handed base docks beside the tray."""
        scene = self._snapshot()
        assert self._engine is not None
        with self._engine._lock:
            self._transport_initial = scene.inventory()
            self._active = None
            self._initial = None

    @rpc
    def prepare_tray_navigation(self, destination: str) -> dict[str, Any]:
        """Plan a clear departure and dock for the carried tray at a platform's free footprint."""
        dock = self.tray_dock_pose(destination)
        scene = self._snapshot()
        tray = tray_report(scene, self._regions)
        if not tray["held"]:
            raise RuntimeError("Both hands must hold the lifted tray before carrying it")
        planner = self._tray_planner(
            scene, collision_margin=self.config.navigation_clearance_m + 0.02
        )
        local = self._tray_planner(scene, collision_margin=0.02)
        departure = apartment_departure(local, planner)
        pose = np.asarray(departure[-1])
        planner.clear_pose_segment(pose, pose)
        aligned = PlanarTransport(
            scene.model,
            planner.probe,
            cargo_bodies=planner.cargo_bodies,
            carry_tray=True,
            sweep_spacing=planner.sweep_spacing,
            collision_margin=planner.collision_margin,
        )
        goal, docking = apartment_approach(aligned, np.asarray(dock["base_pose"], dtype=float))
        offset = carrying_offset(scene.model, scene.data, planner.robot_bodies)
        assert self._engine is not None
        with self._engine._lock:
            self._tray_transport = list(tray["cargo"])
            self._transport_initial = None
            self._active = None
            self._initial = None
        return dict(
            destination=dock["region"],
            goal=goal.tolist(),
            departure=departure,
            arrival=[goal.tolist(), docking.tolist()],
            footprint_offset=offset.tolist(),
            cloud=str(Path(self.config.output) / "navigation-cloud.npy"),
            dock=dock,
        )

    @rpc
    def validate_object_navigation(self, path: list[list[float]]) -> list[list[float]]:
        if self._tray_transport is None:
            return super().validate_object_navigation(path)
        poses = np.asarray(path, dtype=float)
        if poses.ndim != 2 or poses.shape[1] != 3 or len(poses) < 2 or not np.isfinite(poses).all():
            raise ValueError("Expected at least two finite planar poses")
        scene = self._snapshot()
        self._check_tray_transport(scene)
        planner = self._tray_planner(scene, collision_margin=self.config.navigation_clearance_m)
        return refine_apartment_route(planner, path)

    @rpc
    def finish_object_navigation(self) -> None:
        if self._tray_transport is None:
            super().finish_object_navigation()
            return
        assert self._engine is not None
        with self._engine._lock:
            self._check_tray_transport(self._state(self._engine))
            self._tray_transport = None

    @rpc
    def reset(self) -> bool:
        self._tray_transport = None
        return super().reset()

    @rpc
    def primitive_state(self) -> dict[str, Any]:
        state = super().primitive_state()
        assert self._engine is not None
        with self._engine._lock:
            for row in state["objects"]:
                row["orientation_wxyz"] = self._engine.data.body(row["object"]).xquat.tolist()
            state["gripper_commands"] = {
                side: float(self._engine.data.actuator(f"r1pro/{side}_gripper").ctrl[0])
                for side in ARMS
            }
            state["joint_commands"] = {
                name: float(self._engine.data.actuator(name).ctrl[0])
                for name in R1PRO_PICK_PLACE_JOINTS
            }
            state["joint_velocities"] = {
                name: float(self._engine.data.joint(name).qvel[0])
                for name in R1PRO_PICK_PLACE_JOINTS
            }
            state["tcp_poses"] = {}
            for side in ARMS:
                site = self._engine.data.site(f"{side}_tcp")
                pose = np.eye(4)
                pose[:3, :3], pose[:3, 3] = site.xmat.reshape(3, 3), site.xpos
                state["tcp_poses"][side] = pose.tolist()
        return state

    def _snapshot(self) -> PrimitiveSceneState:
        if self._engine is None:
            raise RuntimeError("Simulation is not ready")
        with self._engine._lock:
            if self._error:
                raise RuntimeError(self._error)
            return self._state(self._engine).snapshot()

    @rpc
    def classical_object_cloud(self, index: int) -> PointCloud2:
        """Acquire current raycast depth with simulation instance labels."""
        scene = self._snapshot()
        if not 0 <= index < len(scene.layout.objects):
            raise ValueError("Unknown object index")
        return segmented_object_cloud(
            scene.model, scene.data, scene.model.body(scene.layout.objects[index].name).id
        )

    @rpc
    def classical_carry_posture(self) -> list[list[float]]:
        """Prepare compact hands for navigation and guard every held object during motion."""
        scene = self._snapshot()
        initial = scene.inventory()
        points = ClassicalGraspPlanner(scene).carry_posture()
        self._guard_posture_motion(scene, initial, "carry preparation")
        return points

    @rpc
    def classical_init_posture(self) -> dict[str, Any]:
        """Plan the recorded startup torso/arms without moving the base or opening hands."""
        tray = self.tray_state()
        if tray["held"] or tray["finger_contacts"]:
            raise RuntimeError("Put down the tray before returning the arms and torso to init")
        scene = self._snapshot()
        initial = scene.inventory()
        points = ClassicalGraspPlanner(scene).init_posture()
        self._guard_posture_motion(scene, initial, "init preparation")
        return dict(waypoints=points, target_joints=scene.arms["right"].home[:18].tolist())

    def _guard_posture_motion(
        self, scene: PrimitiveSceneState, initial: list[dict[str, Any]], preparation: str
    ) -> None:
        """Accept a checked path only if its start and cargo still match the live scene."""
        assert self._engine is not None
        with self._engine._lock:
            if self._error:
                raise RuntimeError(self._error)
            self._state(self._engine).validate(initial, arm="right", selected=-1)
            # Planning runs outside the physics lock. A settled grip or base
            # that moved meanwhile cannot safely execute this path's start.
            joint_ids = [scene.model.joint(name).qposadr[0] for name in R1PRO_PICK_PLACE_JOINTS]
            base_ids = [scene.model.joint(name).qposadr[0] for name in VIRTUAL_BASE_JOINTS]
            joint_drift = self._engine.data.qpos[joint_ids] - scene.data.qpos[joint_ids]
            base_drift = self._engine.data.qpos[base_ids] - scene.data.qpos[base_ids]
            if (
                np.max(np.abs(joint_drift)) > 0.005
                or np.linalg.norm(base_drift[:2]) > 0.005
                or abs(np.arctan2(np.sin(base_drift[2]), np.cos(base_drift[2]))) > 0.005
            ):
                raise RuntimeError(f"Robot moved during {preparation}; replan before execution")
            self._transport_initial = initial
            self._active = None
            self._initial = None

    @rpc
    def classical_move_arm(
        self, arm: str, target: list[list[float]], linear: bool
    ) -> list[list[float]]:
        """Check a standalone arm motion against the measured scene and all held cargo."""
        if arm not in ARMS:
            raise ValueError("Choose left or right")
        tray = self.tray_state()
        if tray["held"] or tray["finger_contacts"]:
            raise RuntimeError("Use tray handling while the hands contact the tray")
        scene = self._snapshot()
        initial = scene.inventory()
        points = ClassicalGraspPlanner(scene).move_arm(
            cast("Arm", arm), np.asarray(target, dtype=float), linear=linear
        )
        self._guard_posture_motion(scene, initial, "arm motion preparation")
        return points

    @rpc
    def primitive_recovery(self) -> dict[str, Any]:
        if self._tray_transport is not None and self._active is None:
            assert self._engine is not None
            with self._engine._lock:
                tray = self._check_tray_transport(self._state(self._engine))
            return dict(mode="tray_hold", cargo=tray["cargo"])
        if self._active is not None or self._transport_initial is None:
            return super().primitive_recovery()
        assert self._engine is not None
        with self._engine._lock:
            scene = self._state(self._engine)
            scene.validate(self._transport_initial, arm="right", selected=-1)
            return dict(mode="navigation_hold", held_objects=scene.held_objects())

    @rpc
    def finish_primitive_recovery(self) -> dict[str, Any]:
        if self._tray_transport is not None and self._active is None:
            recovery = self.primitive_recovery()
            assert self._engine is not None
            with self._engine._lock:
                self._tray_transport = None
                self._error = None
            return recovery
        if self._active is not None or self._transport_initial is None:
            return super().finish_primitive_recovery()
        recovery = self.primitive_recovery()
        assert self._engine is not None
        with self._engine._lock:
            self._transport_initial = None
            self._error = None
        return recovery

    @rpc
    def classical_align(self, index: int, arm: str, target: list[list[float]]) -> list[list[float]]:
        if arm not in ARMS:
            raise ValueError("Choose left or right")
        return ClassicalGraspPlanner(self._snapshot()).align_pose(
            index, cast("Arm", arm), np.asarray(target, dtype=float)
        )

    @rpc
    def classical_posture(
        self, index: int, arm: str, positions: list[float], target: list[list[float]] | None = None
    ) -> list[list[float]]:
        if arm not in ARMS:
            raise ValueError("Choose left or right")
        planner = ClassicalGraspPlanner(self._snapshot())
        if target is not None:
            try:
                return planner.transfer_path(index, cast("Arm", arm), np.asarray(target))
            except RuntimeError as transfer_error:
                try:
                    return planner.posture_path(index, cast("Arm", arm), positions)
                except RuntimeError as posture_error:
                    raise RuntimeError(f"{transfer_error}; {posture_error}") from posture_error
        return planner.posture_path(index, cast("Arm", arm), positions)

    @rpc
    def classical_line(self, index: int, arm: str, target: list[list[float]]) -> list[list[float]]:
        """Replan the local Cartesian leg from fresh measured joints and contacts."""
        if arm not in ARMS:
            raise ValueError("Choose left or right")
        scene = self._snapshot()
        planner = ClassicalGraspPlanner(scene)
        try:
            planner.initialize_local_probe(index, cast("Arm", arm))
            return planner.segment(cast("Arm", arm), index, np.asarray(target, dtype=float))
        except RuntimeError:
            np.savez(
                self.config.output / f"classical-line-failure-{time.time_ns()}.npz",
                qpos=scene.data.qpos,
                qvel=scene.data.qvel,
                ctrl=scene.data.ctrl,
                target=target,
                arm=arm,
                index=index,
            )
            raise

    @rpc
    def classical_place_goals(self, arm: str, region: str) -> list[dict[str, Any]]:
        """Intersect empty support footprints with the holding hand's measured transform."""
        if arm not in ARMS:
            raise ValueError("Choose left or right")
        scene = self._snapshot()
        side = cast("Arm", arm)
        index = scene.held_objects()[side]
        if index is None:
            raise ValueError(f"The {arm} hand does not hold an object")
        state = scene.arms[side]
        points, surface = placement_options(
            state, self._regions.get(region, region), None, check_gripper=False
        )
        np.savez(
            self.config.output / f"classical-place-{side}-{time.time_ns()}.npz",
            qpos=scene.data.qpos,
            qvel=scene.data.qvel,
            ctrl=scene.data.ctrl,
        )
        planner = ClassicalGraspPlanner(scene)
        return [
            dict(plan, region=asdict(surface))
            for plan in planner.rank_places(index, side, [np.asarray(point) for point in points])
        ]

    @rpc
    def validate_primitive_base_plan(self, trajectory: JointTrajectory) -> None:
        """Check SDK local motion with held cargo during manipulation or departure."""
        if (
            self._active is None
            and self._transport_initial is None
            and self._tray_transport is None
        ):
            raise RuntimeError("No selected primitive or prepared departure")
        if set(trajectory.joint_names) != set(VIRTUAL_BASE_JOINTS) or not trajectory.points:
            raise ValueError("Prepositioning requires a nonempty base-only trajectory")
        scene = self._snapshot()
        if self._tray_transport is not None:
            self._check_tray_transport(scene)
            planner = self._tray_planner(scene, collision_margin=0.02)
        else:
            if self._transport_initial is not None:
                scene.validate(self._transport_initial, arm="right", selected=-1)
            planner = scene.transport_planner()
        columns = [trajectory.joint_names.index(name) for name in VIRTUAL_BASE_JOINTS]
        start = planner.start
        for point in trajectory.points:
            target = np.asarray(point.positions)[columns]
            if not np.isfinite(target).all() or not planner.clear_pose_segment(start, target):
                raise RuntimeError("DimOS base plan is obstructed with the held objects")
            start = target

    def _navigation_heading(self, region: str) -> float:
        return {"worktable": 0.0, "dining_table": -np.pi / 2, "kitchen": np.pi}[region]

    @rpc
    def prepare_object_navigation(
        self, destination: str, arm: str = "right", stance: list[float] | None = None
    ) -> dict[str, Any]:
        """Plan a clear departure and a dock outside the requested support or object."""
        if arm not in ARMS or self._engine is None:
            raise ValueError("A live scene and valid arm are required")
        scene = self._snapshot()
        name = station_name(destination)
        if name.startswith("object_"):
            index = int(name.removeprefix("object_")) - 1
            rows = scene.inventory()
            if not 0 <= index < len(rows) or rows[index]["held_by"] is not None:
                raise ValueError("Navigation source must be an unheld object")
            target = np.asarray(rows[index]["position"])
            region_name = next(
                (
                    n
                    for n, r in self._regions.items()
                    if set(r.support_geoms) & set(rows[index]["support_geoms"])
                ),
                None,
            )
            if region_name is None:
                raise RuntimeError("Object has no registered apartment support")
        else:
            region_name = "worktable" if name in ("worktop", "table") else name
            if region_name not in self._regions:
                raise ValueError("Use a measured apartment region or an object ID")
            target = np.asarray(self._regions[region_name].center)
        yaw = self._navigation_heading(region_name)
        goal = scene.preposition_pose(cast("Arm", arm), target, yaw=float(yaw))
        if stance is not None:
            goal = np.asarray(stance, dtype=float)
            if goal.shape != (3,) or not np.isfinite(goal).all():
                raise ValueError("Expected a finite assessed base pose")
            yaw = float(goal[2])
        # Leave an extra 2 cm at the arrival stop for measured settling error,
        # so the next checked turn still has the full transit clearance.
        planner = scene.transport_planner(
            collision_margin=self.config.navigation_clearance_m + 0.02
        )
        local = scene.transport_planner()
        departure = apartment_departure(local, planner)
        # The approach turn begins from the departure heading, not the grasp's
        # diagonal heading. Rebase the planning copy including measured cargo.
        pose = np.asarray(departure[-1])
        planner.clear_pose_segment(pose, pose)
        aligned = PlanarTransport(
            scene.model,
            planner.probe,
            cargo_bodies=planner.cargo_bodies,
            carry_tray=False,
            sweep_spacing=planner.sweep_spacing,
            collision_margin=planner.collision_margin,
        )
        goal, docking = apartment_approach(aligned, goal)
        offset = carrying_offset(scene.model, scene.data, planner.robot_bodies)
        with self._engine._lock:
            self._transport_initial = scene.inventory()
            self._active = None
            self._initial = None
        return dict(
            destination=region_name,
            goal=goal.tolist(),
            departure=departure,
            arrival=[goal.tolist(), docking.tolist()],
            footprint_offset=offset.tolist(),
            cloud=str(Path(self.config.output) / "navigation-cloud.npy"),
        )
