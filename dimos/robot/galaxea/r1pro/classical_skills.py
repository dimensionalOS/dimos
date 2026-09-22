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

"""Explicit classical pick/hold/place state machine for the apartment demo."""

from collections.abc import Callable
from concurrent.futures import CancelledError
import json
from pathlib import Path as FilePath
import threading
import time
from typing import Any

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from dimos.agents.annotation import skill
from dimos.control.tasks.trajectory_task.trajectory_task import TrajectoryExecutionStatus
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.manipulation.grasping.grasp_gen_spec import GraspGenSpec
from dimos.manipulation.manipulation_spec import ExecutionStatus, ManipulationSpec
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState
from dimos.robot.galaxea.r1pro.apartment_navigation import (
    APARTMENT_NAV_TASK,
    CLASSICAL_POSITION_TASK,
    ApartmentNavigationSpec,
)
from dimos.robot.galaxea.r1pro.apartment_route import (
    CLASSICAL_TRACKING_LIMIT_M,
    navigation_tracking_error,
)
from dimos.robot.galaxea.r1pro.classical_motion import joint_trajectory
from dimos.robot.galaxea.r1pro.classical_selection import color_name, resolve_classical_object
from dimos.robot.galaxea.r1pro.classical_sim import ClassicalSimSpec
from dimos.robot.galaxea.r1pro.classical_tray import run_tray_motion
from dimos.robot.galaxea.r1pro.config import R1PRO_PLANAR_BASE
from dimos.robot.galaxea.r1pro.home_spec import HomeControlSpec
from dimos.robot.galaxea.r1pro.learning import R1PRO_PICK_PLACE_JOINTS
from dimos.robot.galaxea.r1pro.navigation_sim import pose_message
from dimos.robot.galaxea.r1pro.object_primitives import ARMS
from dimos.utils.logging_config import setup_logger

logger = setup_logger()
# Cross-body reaches with a held object can take minutes to rank; bound the wait.
ASSESSMENT_TIMEOUT_S = 150.0


class R1ProClassicalSkills(Module):
    _sim: ClassicalSimSpec
    _control: HomeControlSpec
    _manipulation: ManipulationSpec
    _grasp_generator: GraspGenSpec
    _navigation: ApartmentNavigationSpec

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._lock = threading.RLock()
        self._cancel = threading.Event()
        self._done = threading.Event()
        self._done.set()
        self._thread: threading.Thread | None = None
        self._closing = False
        self._counter = 0
        self._action: dict[str, Any] = dict(state="idle", recovery_required=False)

    def _status(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._action)

    def _pause(self, seconds: float) -> None:
        if self._cancel.wait(seconds):
            raise CancelledError("Cancelled; hands hold their current positions")

    def _start(
        self, name: str, operation: Callable[[dict[str, Any]], None], *, recovery: bool = False
    ) -> str:
        with self._lock:
            if self._closing or not self._done.is_set():
                return json.dumps(dict(accepted=False, reason="busy_or_stopping"))
            if self._action.get("recovery_required") and not recovery:
                return json.dumps(
                    dict(accepted=False, reason="recovery_required", action=self._action)
                )
            prior_recovery = bool(self._action.get("recovery_required"))
            self._counter += 1
            self._action = dict(
                id=self._counter,
                name=name,
                state="running",
                success=False,
                recovery_required=prior_recovery,
            )
            self._cancel.clear()
            self._done.clear()
            self._thread = threading.Thread(
                target=self._run, args=(operation,), daemon=True, name="r1pro-classical-action"
            )
            self._thread.start()
            return json.dumps(dict(accepted=True, **self._action))

    def _run(self, operation: Callable[[dict[str, Any]], None]) -> None:
        report: dict[str, Any] = {}
        prior_recovery = bool(self._status().get("recovery_required"))
        terminal: dict[str, Any] = dict(state="failed", success=False, recovery_required=False)
        try:
            operation(report)
            terminal.update(state="completed", success=True, recovery_required=False)
        except CancelledError as exc:
            terminal.update(
                state="cancelled",
                error=str(exc),
                recovery_required=prior_recovery or bool(report.get("motion_started")),
            )
        except Exception as exc:
            logger.exception("Classical action failed")
            terminal.update(
                error=str(exc),
                recovery_required=prior_recovery or bool(report.get("motion_started")),
            )
        finally:
            try:
                self._stop_control()
            except Exception as exc:
                terminal.update(
                    state="failed", success=False, recovery_required=True, cleanup_error=str(exc)
                )
            try:
                report["final"] = self._sim.primitive_state()
                report["snapshot"] = self._sim.save_classical_state()
                output = (
                    FilePath(self._sim.prepare_primitive_session()["output"])
                    / f"action-{self._counter:03d}.json"
                )
                output.write_text(
                    json.dumps({**self._status(), **report, **terminal}, indent=2) + "\n"
                )
                terminal["evidence"] = str(output)
            except Exception as exc:
                terminal["report_error"] = str(exc)
            with self._lock:
                self._action.update(terminal)
                self._done.set()

    @skill
    def wait_for_action(self, seconds: float = 10.0) -> str:
        """Wait up to 20 seconds for the current action; repeat while state is running."""
        self._done.wait(min(max(seconds, 0.0), 20.0))
        return json.dumps(self._status())

    @skill
    def stop_action(self) -> str:
        """Cancel current motion and preserve each hand's position; never release or reset."""
        self._cancel.set()
        return json.dumps(self._status())

    @skill
    def recover_action(self) -> str:
        """Preserve a confirmed hold, or open supported contacts and restore only the failed arm.

        Recovery never retries an grasp and never resets scene progress or the other hand.
        """

        def operation(report: dict[str, Any]) -> None:
            self._stop_control()
            recovery = self._sim.primitive_recovery()
            report["recovery"] = recovery
            if recovery["mode"] == "empty_hand":
                arm = recovery["arm"]
                joints = recovery["joints"]
                measured = self._control.get_joint_positions()
                start = [measured[n] for n in joints]
                opened = [*start[:-1], 0.05]
                trajectory = JointTrajectory(
                    joint_names=joints,
                    points=[
                        TrajectoryPoint(positions=start, time_from_start=0.0),
                        TrajectoryPoint(positions=opened, time_from_start=0.4),
                        TrajectoryPoint(positions=opened, time_from_start=0.8),
                    ],
                )
                report["motion_started"] = True
                opening_result = self._control.execute_trajectory(trajectory, f"primitive_{arm}")
                if opening_result.status is not TrajectoryExecutionStatus.ACCEPTED:
                    raise RuntimeError(f"Recovery gripper opening was rejected: {opening_result}")
                self._pause(0.9)
                stopped = self._control.cancel_trajectory(f"primitive_{arm}")
                if not stopped.safe:
                    raise RuntimeError("Cannot confirm recovery gripper stopped")
                plan = self._manipulation.plan_to_joints(
                    {f"{arm}_arm": JointState(name=joints[:-1], position=recovery["home"][:-1])},
                    speed_scale=0.5,
                )
                if not plan.succeeded or plan.plan is None:
                    raise RuntimeError(f"Recovery planning failed: {plan.message}")
                self._sim.validate_primitive_recovery_plan(plan.plan.trajectory)
                self._pause(0)
                result = self._manipulation.execute(blocking=False, plan_id=plan.plan.plan_id)
                if result.status is not ExecutionStatus.ACCEPTED:
                    raise RuntimeError(f"Recovery was rejected: {result}")
                initial = self._sim.primitive_state()
                sim_start = stamp = float(initial["sim_time"])
                fresh = progressed = time.monotonic()
                previous = np.asarray([initial["joint_positions"][n] for n in joints[:-1]])
                while True:
                    self._pause(0.05)
                    result = self._manipulation.wait_for_execution(timeout=0.01)
                    if result.succeeded:
                        break
                    if result.status not in (
                        ExecutionStatus.EXECUTING,
                        ExecutionStatus.ACCEPTED,
                        ExecutionStatus.TIMED_OUT,
                    ):
                        raise RuntimeError(f"Recovery failed: {result}")
                    state = self._sim.primitive_state()
                    now = time.monotonic()
                    sim_time = float(state["sim_time"])
                    if sim_time < stamp:
                        raise RuntimeError("Simulation clock moved backwards during recovery")
                    if sim_time > stamp:
                        fresh, stamp = now, sim_time
                    current = np.asarray([state["joint_positions"][n] for n in joints[:-1]])
                    if np.max(np.abs(current - previous)) > 1e-4:
                        progressed, previous = now, current
                    if now - fresh > 10:
                        raise RuntimeError("Simulation stopped updating during recovery")
                    if now - progressed > 30:
                        raise RuntimeError("Recovery made no measured arm progress for 30 s")
                    if sim_time - sim_start > plan.plan.trajectory.duration + 25:
                        raise RuntimeError("Recovery exceeded its simulation-time settling budget")
                self._pause(0.5)
            report["recovered"] = self._sim.finish_primitive_recovery()

        return self._start("recover", operation, recovery=True)

    @skill
    def return_to_init(self) -> str:
        """Restore the fixed startup arm-and-torso posture, keeping the base here.

        Preserve both grippers and held objects. This is not a scene reset. If the exact
        startup posture would collide or tip held cargo, report why without substituting
        a different posture or automatically placing/releasing the object.
        """
        tray = self._sim.tray_state()
        if tray["held"] or tray["finger_contacts"]:
            return json.dumps(
                dict(accepted=False, reason="Put down the tray before returning to init")
            )

        def operation(report: dict[str, Any]) -> None:
            self._stop_control()
            self._phase("return_to_init", report)
            before = self._sim.primitive_state()
            if before["error"]:
                raise RuntimeError(before["error"])
            held = dict(before["held_objects"])
            base = np.asarray(before["base_pose"], dtype=float)
            plan = self._sim.classical_init_posture()
            target = np.asarray(plan["target_joints"], dtype=float)
            report["init_posture"] = dict(target_joints=target.tolist(), verified=False)
            self._drive(plan["waypoints"], report)
            self._pause(0.5)
            after = self._sim.primitive_state()
            if after["error"]:
                raise RuntimeError(after["error"])
            if after["held_objects"] != held:
                raise RuntimeError("Cargo ownership changed while returning to init")
            measured = np.asarray(
                [after["joint_positions"][name] for name in R1PRO_PICK_PLACE_JOINTS[:18]]
            )
            error = float(np.max(np.abs(measured - target)))
            if not np.isfinite(error) or error > 0.02:
                raise RuntimeError(
                    f"Measured arms/torso did not reach the fixed init posture ({error:.4f} rad)"
                )
            base_error = np.asarray(after["base_pose"], dtype=float) - base
            base_error[2] = np.arctan2(np.sin(base_error[2]), np.cos(base_error[2]))
            if (
                not np.all(np.isfinite(base_error))
                or np.linalg.norm(base_error[:2]) > 0.005
                or abs(base_error[2]) > 0.005
            ):
                raise RuntimeError("Base moved while returning arms and torso to init")
            self._sim.finish_object_navigation()
            report["init_posture"].update(
                verified=True, measured_joints=measured.tolist(), held_objects=held
            )
            self._phase("at_init", report)

        return self._start("return_to_init", operation)

    @skill
    def reset_scene(self) -> str:
        """Reset objects and both hands only on an explicit user reset request; requires idle actions."""

        def operation(report: dict[str, Any]) -> None:
            self._stop_control()
            if not self._sim.reset():
                raise RuntimeError("Simulator rejected reset")
            self._pause(0.8)

        return self._start("reset", operation, recovery=True)

    @skill
    def prepare_carry(self) -> str:
        """Retract to a compact cargo-safe pose without moving the base or opening hands.

        Optional after a pick, depending on the next task; not the exact startup joints.
        """

        def operation(report: dict[str, Any]) -> None:
            self._stop_control()
            tray = self._sim.tray_state()
            if tray["held"] or tray["finger_contacts"]:
                raise RuntimeError("Use tray handling while the hands contact the tray")
            self._prepare_carry(report)
            self._sim.finish_object_navigation()
            self._phase("holding", report)

        return self._start("prepare_carry", operation)

    def _move_arm(
        self,
        name: str,
        arm: str,
        target_from_current: Callable[[NDArray[Any]], NDArray[Any]],
        *,
        linear: bool,
    ) -> str:
        if arm not in ARMS:
            return json.dumps(dict(accepted=False, reason="Choose left or right"))

        def operation(report: dict[str, Any]) -> None:
            self._stop_control()
            self._phase(name, report)
            before = self._sim.primitive_state()
            if before["error"]:
                raise RuntimeError(before["error"])
            held = dict(before["held_objects"])
            base = np.asarray(before["base_pose"], dtype=float)
            target = target_from_current(np.asarray(before["tcp_poses"][arm], dtype=float))
            report["arm_motion"] = dict(arm=arm, target=target.tolist(), linear=linear)
            points = self._sim.classical_move_arm(arm, target.tolist(), linear)
            self._drive(points, report)
            after = self._sim.primitive_state()
            if after["error"]:
                raise RuntimeError(after["error"])
            if after["held_objects"] != held:
                raise RuntimeError("Cargo ownership changed during arm motion")
            actual = np.asarray(after["tcp_poses"][arm], dtype=float)
            position_error = float(np.linalg.norm(actual[:3, 3] - target[:3, 3]))
            angle_error = float(Rotation.from_matrix(target[:3, :3].T @ actual[:3, :3]).magnitude())
            if (
                not np.isfinite([position_error, angle_error]).all()
                or position_error > 0.005
                or angle_error > 0.02
            ):
                raise RuntimeError(
                    f"Measured hand missed target: {position_error:.4f} m, {angle_error:.4f} rad"
                )
            base_error = np.asarray(after["base_pose"], dtype=float) - base
            base_error[2] = np.arctan2(np.sin(base_error[2]), np.cos(base_error[2]))
            if (
                not np.isfinite(base_error).all()
                or np.linalg.norm(base_error[:2]) > 0.005
                or abs(base_error[2]) > 0.005
            ):
                raise RuntimeError("Base moved during arm-only motion")
            self._sim.finish_object_navigation()
            report["arm_motion"].update(
                verified=True, position_error_m=position_error, angle_error_rad=angle_error
            )
            self._phase("holding", report)

        return self._start(name, operation)

    @skill
    def move_linear(self, arm: str, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0) -> str:
        """Translate one hand along a checked straight line in world axes, distances in metres.

        Keep its orientation, both gripper commands, cargo and base position. No release,
        new grasp or placement; use pick_object/place_object for contact with a support.
        """
        delta = np.asarray([dx, dy, dz], dtype=float)
        if not np.isfinite(delta).all():
            return json.dumps(dict(accepted=False, reason="Displacement must be finite"))

        def target(current: NDArray[Any]) -> NDArray[Any]:
            result = current.copy()
            result[:3, 3] += delta
            return result

        return self._move_arm("move_linear", arm, target, linear=True)

    @skill
    def move_to_pose(
        self,
        arm: str,
        x: float,
        y: float,
        z: float,
        roll: float | None = None,
        pitch: float | None = None,
        yaw: float | None = None,
    ) -> str:
        """Move one hand to world XYZ in metres, optionally world roll/pitch/yaw in radians.

        Omitted angles retain their current values. Choose a checked transfer corridor;
        keep base, both grippers and cargo. Not a pick/place or permission to tip cargo.
        """
        values = [x, y, z, *[v for v in (roll, pitch, yaw) if v is not None]]
        if not np.isfinite(values).all():
            return json.dumps(dict(accepted=False, reason="Pose values must be finite"))

        def target(current: NDArray[Any]) -> NDArray[Any]:
            result = current.copy()
            result[:3, 3] = [x, y, z]
            if any(v is not None for v in (roll, pitch, yaw)):
                angles = Rotation.from_matrix(current[:3, :3]).as_euler("xyz")
                for i, value in enumerate((roll, pitch, yaw)):
                    if value is not None:
                        angles[i] = value
                result[:3, :3] = Rotation.from_euler("xyz", angles).as_matrix()
            return result

        return self._move_arm("move_to_pose", arm, target, linear=False)

    @rpc
    def stop(self) -> None:
        with self._lock:
            self._closing = True
        self._cancel.set()
        if self._thread is not None:
            self._thread.join(timeout=10)
        super().stop()

    def _follow(self, path: list[list[float]], report: dict[str, Any]) -> None:
        if len(path) < 2:
            return
        path = self._sim.validate_object_navigation(path)
        self._execute_base_path(path, report, CLASSICAL_TRACKING_LIMIT_M, arrival_tolerance=0.025)

    def _execute_base_path(
        self,
        path: list[list[float]],
        report: dict[str, Any],
        tracking_limit: float,
        task_name: str = APARTMENT_NAV_TASK,
        arrival_tolerance: float | None = None,
    ) -> None:
        """Follow an already checked route by measured progress, including turns."""
        report.setdefault("commanded_paths", []).append(path)
        before = self._sim.primitive_state()
        self._pause(0)
        self._control.task_invoke(task_name, "reset", {})
        accepted = self._control.task_invoke(
            task_name,
            "start_path",
            {
                "path": Path(poses=[pose_message(p) for p in path], frame_id="world"),
                "current_odom": pose_message(before["base_pose"]),
            },
        )
        if not accepted:
            raise RuntimeError("Holonomic task rejected the apartment path")
        report["motion_started"] = True
        length = float(np.linalg.norm(np.diff(np.asarray(path)[:, :2], axis=0), axis=1).sum())
        deadline = time.monotonic() + 90 + length / 0.05
        last_pose = np.asarray(before["base_pose"])
        progressed = time.monotonic()
        last_sim_time = before["sim_time"]
        updated = time.monotonic()
        while True:
            self._pause(0.05)
            state = self._sim.primitive_state()
            if state["error"]:
                raise RuntimeError(state["error"])
            if state["sim_time"] > last_sim_time:
                updated, last_sim_time = time.monotonic(), state["sim_time"]
            if time.monotonic() - updated > 2:
                raise RuntimeError("Apartment simulation stopped updating during navigation")
            deviation = navigation_tracking_error(path, state["base_pose"])
            report["max_navigation_tracking_error_m"] = max(
                report.get("max_navigation_tracking_error_m", 0.0), deviation
            )
            arrival_error = np.asarray(state["base_pose"]) - path[-1]
            arrival_error[2] = np.arctan2(np.sin(arrival_error[2]), np.cos(arrival_error[2]))
            if (
                arrival_tolerance is not None
                and np.linalg.norm(arrival_error[:2]) <= arrival_tolerance
                and abs(arrival_error[2]) <= 0.005
            ):
                # Close-range manipulation can absorb measured arrival error.
                # Stop here; verify the actual footprint before the arms replan.
                self._control.task_invoke(task_name, "cancel", {})
                break
            if deviation > tracking_limit:
                raise RuntimeError(
                    f"Navigation exceeded its checked tracking allowance ({deviation:.3f} m); "
                    "stopping before continuing the route"
                )
            task = self._control.task_invoke(task_name, "get_state", {})
            if task == "arrived":
                break
            if task in ("aborted", "idle"):
                raise RuntimeError(f"Navigation task stopped: {task}")
            pose = np.asarray(state["base_pose"])
            if np.linalg.norm(pose - last_pose) > 0.002:
                last_pose, progressed = pose, time.monotonic()
            if time.monotonic() - progressed > 12 or time.monotonic() > deadline:
                raise RuntimeError("Navigation made no progress or exceeded its deadline")
        self._sim.stop_primitive_base()
        report.setdefault("paths", []).append(path)
        self._pause(0.5)
        final = np.asarray(self._sim.primitive_state()["base_pose"])
        error = final - path[-1]
        error[2] = np.arctan2(np.sin(error[2]), np.cos(error[2]))
        tolerance = 0.025 if arrival_tolerance is None else arrival_tolerance
        if np.linalg.norm(error[:2]) > tolerance or abs(error[2]) > 0.03:
            raise RuntimeError("Measured navigation endpoint is outside the docking tolerance")
        if arrival_tolerance is not None:
            self._sim.validate_primitive_base_plan(
                JointTrajectory(
                    joint_names=list(R1PRO_PLANAR_BASE.joint_names),
                    points=[TrajectoryPoint(positions=final.tolist(), time_from_start=0.0)],
                )
            )
            report.setdefault("base_arrivals", []).append(
                dict(
                    target=path[-1],
                    measured=final.tolist(),
                    position_error_m=float(np.linalg.norm(error[:2])),
                )
            )

    def _prepare_carry(
        self,
        report: dict[str, Any],
        *,
        phase: str = "prepare_carry",
        expected_held: dict[str, str | None] | None = None,
    ) -> None:
        """Retract without base motion or release; verify both hands before declaring ready."""
        self._phase(phase, report)
        self._pause(0)
        before = self._sim.primitive_state()
        if before["error"]:
            raise RuntimeError(before["error"])
        held = dict(before["held_objects"])
        if expected_held is not None and held != expected_held:
            raise RuntimeError("Cargo ownership changed before retracting to the carrying posture")
        points = self._sim.classical_carry_posture()
        self._drive(points, report)
        self._pause(0.5)
        after = self._sim.primitive_state()
        if after["error"]:
            raise RuntimeError(after["error"])
        if after["held_objects"] != held:
            raise RuntimeError("Cargo ownership changed while retracting to the carrying posture")
        report["carry_posture"] = dict(verified=True, held_objects=held, waypoints=len(points))

    def _navigate(
        self,
        destination: str,
        arm: str,
        report: dict[str, Any],
        stance: list[float] | None = None,
        *,
        carry_tray: bool = False,
    ) -> None:
        self._stop_control()
        if carry_tray:
            self._phase("navigate", report)
            plan = self._sim.prepare_tray_navigation(destination)
        else:
            self._prepare_carry(report)
            self._phase("navigate", report)
            plan = self._sim.prepare_object_navigation(destination, arm, stance)
        report["navigation"] = plan
        try:
            self._position_base(
                {"base_waypoints": plan["departure"]}, report, arrival_tolerance=0.005
            )
            self._navigation.request_object_route(
                plan["goal"], plan["footprint_offset"], plan["cloud"]
            )
            deadline = time.monotonic() + 90
            while True:
                self._pause(0.1)
                route = self._navigation.object_route_status()
                if route.get("error"):
                    raise RuntimeError(route["error"])
                if route["ready"]:
                    break
                if time.monotonic() > deadline:
                    raise RuntimeError("KronkNav did not produce an apartment route")
            report["native_route"] = route["path"]
            self._follow(route["path"], report)
            self._follow(plan["arrival"], report)
            self._sim.finish_object_navigation()
        finally:
            self._control.task_invoke(APARTMENT_NAV_TASK, "cancel", {})
            self._sim.stop_primitive_base()

    def _dock(self, dock: dict[str, Any], report: dict[str, Any], *, carry_tray: bool) -> None:
        """Bring the base to a tray dock pose, navigating first when it is far."""
        desired = np.asarray(dock["base_pose"], dtype=float)

        def error(desired: NDArray[np.float64]) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
            current = np.asarray(self._sim.primitive_state()["base_pose"], dtype=float)
            delta = desired - current
            delta[2] = np.arctan2(np.sin(delta[2]), np.cos(delta[2]))
            return current, delta

        current, delta = error(desired)
        if np.linalg.norm(delta[:2]) > 0.7:
            self._navigate(dock["region"], "right", report, desired.tolist(), carry_tray=carry_tray)
            current, delta = error(desired)
        if not dock["clear"]:
            raise RuntimeError(f"No clear docking pose beside {dock['region']}")
        if np.linalg.norm(delta[:2]) <= 0.01 and abs(delta[2]) <= 0.01:
            return
        self._phase("dock", report)
        if carry_tray:
            self._sim.prepare_tray_navigation(dock["region"])
        else:
            self._sim.prepare_tray_approach()
        self._position_base(
            {"base_waypoints": [current.tolist(), desired.tolist()]}, report, arrival_tolerance=0.01
        )
        self._sim.finish_object_navigation()

    def _position_base(
        self, selection: dict[str, Any], report: dict[str, Any], *, arrival_tolerance: float = 0.03
    ) -> None:
        report["base_plan_ids"] = []
        waypoints = selection["base_waypoints"][1:]
        for index, waypoint in enumerate(waypoints):
            tolerance = arrival_tolerance if index == len(waypoints) - 1 else 0.005
            target = np.asarray(waypoint)
            if np.max(np.abs(target - self._sim.primitive_state()["base_pose"])) <= 0.004:
                continue
            plan = self._manipulation.plan_to_joints(
                {
                    "moving_base": JointState(
                        name=list(R1PRO_PLANAR_BASE.joint_names), position=target.tolist()
                    )
                },
                speed_scale=0.7,
            )
            if not plan.succeeded or plan.plan is None:
                raise RuntimeError(f"SDK prepositioning plan failed: {plan.message}")
            self._sim.validate_primitive_base_plan(plan.plan.trajectory)
            report["base_plan_ids"].append(plan.plan.plan_id)
            trajectory = plan.plan.trajectory
            columns = [trajectory.joint_names.index(name) for name in R1PRO_PLANAR_BASE.joint_names]
            path = [[float(point.positions[i]) for i in columns] for point in trajectory.points]
            # SDK planning still supplies and checks the path. Execute by
            # measured progress so slow physics cannot advance a wall-time
            # reference beyond the robot's physical pose.
            self._execute_base_path(
                path,
                report,
                tracking_limit=0.015,
                task_name=CLASSICAL_POSITION_TASK,
                arrival_tolerance=tolerance,
            )
            self._sim.stop_primitive_base()
            self._pause(1.0)
            actual = np.asarray(self._sim.primitive_state()["base_pose"])
            delta = target - actual
            delta[2] = np.arctan2(np.sin(delta[2]), np.cos(delta[2]))
            if np.linalg.norm(delta[:2]) > tolerance or abs(delta[2]) > 0.03:
                raise RuntimeError(
                    "Measured base is outside the classical prepositioning tolerance"
                )

    def _assess_pick(
        self, index: int, candidates: GraspCandidateArray, arm: str
    ) -> list[dict[str, Any]]:
        """Rank proposals out of process so a slow search fails cleanly and never stalls physics."""
        self._sim.begin_classical_pick_assessment(index, candidates, arm)
        deadline = time.monotonic() + ASSESSMENT_TIMEOUT_S
        try:
            while True:
                status = self._sim.classical_pick_assessment()
                if status["state"] == "done":
                    return list(status["options"])
                if status["state"] == "failed":
                    raise RuntimeError(status["error"])
                if time.monotonic() > deadline:
                    raise RuntimeError(
                        f"Could not find a comfortable grasp within {ASSESSMENT_TIMEOUT_S:.0f} s; "
                        "free the other hand or ask again from closer"
                    )
                self._pause(1.0)
        finally:
            try:
                self._sim.cancel_classical_pick_assessment()
            except Exception:
                logger.exception("Could not cancel the grasp assessment")

    def _prepare_posture(self, selection: dict[str, Any], report: dict[str, Any]) -> None:
        stance = selection["reachability"]
        positions = stance["ready_joints"]
        actual = self._sim.primitive_state()["joint_positions"]
        error = max(
            abs(actual[name] - positions[i]) for i, name in enumerate(R1PRO_PICK_PLACE_JOINTS[:18])
        )
        report["initial_posture_error_rad"] = error
        index = int(selection["object"].removeprefix("object_")) - 1
        target = np.asarray(stance.get("pregrasp", stance.get("preplace")))
        if error > 0.02:
            points = self._sim.classical_posture(
                index,
                stance["arm"],
                positions,
                target.tolist() if report.get("phase") == "preplace" else None,
            )
            report["posture_waypoints"] = len(points)
            self._drive(points, report)
        for attempt in range(3):
            actual_pose = np.asarray(self._sim.primitive_state()["tcp_poses"][stance["arm"]])
            position_error = float(np.linalg.norm(actual_pose[:3, 3] - target[:3, 3]))
            orientation_error = float(np.linalg.norm(actual_pose[:3, :3] - target[:3, :3]))
            report["staging_error"] = dict(
                position_m=position_error, rotation_matrix=orientation_error
            )
            if position_error <= 0.004 and orientation_error <= 0.015:
                break
            if attempt == 2:
                raise RuntimeError("Measured TCP did not reach the staged pose")
            self._drive(self._sim.classical_align(index, stance["arm"], target.tolist()), report)
        actual = self._sim.primitive_state()["joint_positions"]
        report["final_posture_error_rad"] = max(
            abs(actual[name] - positions[i]) for i, name in enumerate(R1PRO_PICK_PLACE_JOINTS[:18])
        )

    def _phase(self, phase: str, report: dict[str, Any]) -> None:
        self._pause(0)
        report["phase"] = phase
        with self._lock:
            self._action["phase"] = phase

    def _stop_control(self) -> None:
        self._control.task_invoke(APARTMENT_NAV_TASK, "cancel", {})
        self._control.task_invoke(CLASSICAL_POSITION_TASK, "cancel", {})
        result = self._manipulation.cancel()
        if result.status in (ExecutionStatus.UNCERTAIN, ExecutionStatus.FAULT):
            raise RuntimeError(result.message)
        for task in ("joint_trajectory", "primitive_left", "primitive_right"):
            if not self._control.cancel_trajectory(task).safe:
                raise RuntimeError(f"Cannot confirm {task} stopped")
        self._sim.stop_primitive_base()

    def _drive(self, points: list[list[float]], report: dict[str, Any]) -> None:
        self._pause(0)
        if not points:
            raise RuntimeError("Planner returned no executable waypoints")
        before = self._sim.primitive_state()
        if before.get("error"):
            raise RuntimeError(before["error"])
        if len(points) == 1:
            # The SDK may return a single pose when start and goal coincide.
            # Represent its hold explicitly; the caller still verifies TCP
            # staging and contacts before proceeding to the next phase.
            points = [points[0], points[0]]
        carrying = any(row.get("held_by") for row in before.get("objects", []))
        active = before.get("active")
        placing_arm = (
            active[1]
            if active
            and active[0] == "place"
            and report.get("phase") in ("lower_to_support", "release", "retreat")
            else None
        )
        # A supported object is no longer "held_by" even before the pads
        # release it. The selected placement uses the two-pad contact guard
        # below until release; every other held object keeps its ownership.
        protected_cargo = {
            row["index"]: row["held_by"]
            for row in before.get("objects", [])
            if row.get("held_by") and row["held_by"] != placing_arm
        }
        trajectory = joint_trajectory(points, carrying=carrying)
        self._pause(0)
        accepted = self._control.execute_trajectory(trajectory, "joint_trajectory")
        if accepted.status is not TrajectoryExecutionStatus.ACCEPTED:
            raise RuntimeError(f"Cartesian trajectory rejected: {accepted}")
        report["motion_started"] = True
        duration = trajectory.points[-1].time_from_start
        started = time.monotonic()
        sim_start = before.get("sim_time")
        progressed = started
        last_joints: NDArray[Any] | None = None
        last_commands: NDArray[Any] | None = None
        last_velocity = float("inf")
        protected_grasp = None
        if active and report.get("phase") in ("lift", "preplace", "lower_to_support"):
            side = active[1]
            protected_grasp = next(
                (
                    row["index"]
                    for row in before["objects"]
                    if row["grasped"] and side in row["contacting_arms"]
                ),
                None,
            )
        fresh, stamp, stable = time.monotonic(), -1.0, 0
        settle_velocity = (
            0.0025
            if report.get("phase") in ("stage_pregrasp", "approach", "preplace", "lower_to_support")
            else 0.01
        )
        while True:
            self._pause(0.05)
            state = self._sim.primitive_state()
            if state["error"]:
                raise RuntimeError(state["error"])
            if state["sim_time"] < max(stamp, sim_start if sim_start is not None else stamp):
                raise RuntimeError("Simulation clock moved backwards during Cartesian execution")
            if state["sim_time"] > stamp:
                fresh, stamp = time.monotonic(), state["sim_time"]
                if sim_start is None:
                    sim_start = stamp
                error = max(
                    abs(state["joint_positions"][n] - points[-1][i])
                    for i, n in enumerate(R1PRO_PICK_PLACE_JOINTS[:18])
                )
                velocities = state.get("joint_velocities", {})
                moving = max(abs(velocities.get(n, 0.0)) for n in R1PRO_PICK_PLACE_JOINTS[:18])
                commands = state.get("joint_commands")
                joints = np.asarray([state["joint_positions"][n] for n in R1PRO_PICK_PLACE_JOINTS])
                command_values = (
                    None
                    if commands is None
                    else np.asarray([commands[n] for n in R1PRO_PICK_PLACE_JOINTS])
                )
                if (
                    last_joints is None
                    or np.max(np.abs(joints - last_joints)) > 1e-4
                    or (
                        command_values is not None
                        and (
                            last_commands is None
                            or np.max(np.abs(command_values - last_commands)) > 1e-5
                        )
                    )
                    or moving < last_velocity - 1e-4
                ):
                    progressed = fresh
                    last_joints, last_commands, last_velocity = joints, command_values, moving
                delivered = (
                    commands is None
                    or max(
                        abs(commands[n] - points[-1][i])
                        for i, n in enumerate(R1PRO_PICK_PLACE_JOINTS)
                    )
                    <= 1e-5
                )
                # Millimetre contact moves need a settled mechanism. A 0.03
                # rad/s endpoint can still move the TCP several mm per second
                # while the next IK query assumes its measured pose is static.
                stable = (
                    stable + 1 if delivered and error <= 0.02 and moving <= settle_velocity else 0
                )
                report["last_trajectory_tracking"] = dict(
                    phase=report.get("phase"),
                    joint_error_rad=float(error),
                    max_joint_velocity=float(moving),
                    commands_delivered=bool(delivered),
                    stable_samples=stable,
                    wall_elapsed_s=time.monotonic() - started,
                    sim_elapsed_s=stamp - sim_start,
                    planned_duration_s=duration,
                )
            if protected_grasp is not None and not state["objects"][protected_grasp]["grasped"]:
                raise RuntimeError("Selected object lost two-finger contact during transfer")
            for index, side in protected_cargo.items():
                if state["objects"][index]["held_by"] != side:
                    raise RuntimeError("Held object lost contact or changed hands during transfer")
            if time.monotonic() - fresh > 10:
                raise RuntimeError("Simulation stopped updating during Cartesian execution")
            task_state = self._control.task_invoke("joint_trajectory", "get_state", {})
            if task_state not in (TrajectoryState.EXECUTING, TrajectoryState.COMPLETED):
                raise RuntimeError("Cartesian task stopped before completing")
            if task_state == TrajectoryState.COMPLETED and stable >= 3:
                # Gripper commands terminate against the object; joint error is
                # not a grasp test. The caller verifies actual finger contacts.
                report.setdefault("trajectory_checks", []).append(
                    dict(
                        phase=report.get("phase"),
                        target=points[-1],
                        commanded=state.get("joint_commands"),
                        measured=state["joint_positions"],
                        tcp=state.get("tcp_poses"),
                    )
                )
                return
            # Physics may run slower than real time: a moving robot must not
            # fail just because wall time exceeded the nominal trajectory.
            if sim_start is not None and stamp - sim_start > duration + 25:
                raise RuntimeError(
                    "Measured Cartesian trajectory did not settle within its simulation-time budget"
                )
            if time.monotonic() - progressed > 30 and time.monotonic() - started > duration:
                raise RuntimeError(
                    "Cartesian motion stalled: no joint, command or settling progress for 30 s"
                )

    def _line(
        self, index: int, arm: str, target: list[list[float]], report: dict[str, Any]
    ) -> None:
        self._drive(self._sim.classical_line(index, arm, target), report)

    def _gripper(self, arm: str, opening: float, report: dict[str, Any]) -> None:
        state = self._sim.primitive_state()
        start = [state["joint_commands"][n] for n in R1PRO_PICK_PLACE_JOINTS]
        target = list(start)
        target[18 if arm == "left" else 19] = opening
        self._drive([start, target], report)
        self._pause(0.3)

    def _seek_support(self, chosen: dict[str, Any], report: dict[str, Any]) -> None:
        """Confirm the intended physical support before releasing, within a 10 mm descent."""
        expected = set(chosen["region"]["support_geoms"])
        target = None
        initial_z = None
        for step in range(21):
            self._pause(0.2)
            state = self._sim.primitive_state()
            row = state["objects"][chosen["index"]]
            actual = np.asarray(state["tcp_poses"][chosen["arm"]], dtype=float)
            if initial_z is None:
                initial_z = float(actual[2, 3])
            report.setdefault("support_samples", []).append(
                dict(
                    step=step,
                    tcp=state.get("tcp_poses", {}).get(chosen["arm"]),
                    object_position=row.get("position"),
                    support_geoms=row["support_geoms"],
                    commanded_target=None if target is None else target.tolist(),
                )
            )
            supports = set(row["support_geoms"])
            if supports & expected:
                if not row["upright"]:
                    raise RuntimeError("Object tipped before release")
                report["support_before_release"] = sorted(supports & expected)
                report["support_descent_m"] = initial_z - float(actual[2, 3])
                return
            if supports:
                raise RuntimeError("Object contacted a different surface before release")
            if step < 20:
                target = actual.copy()
                target[2, 3] -= 0.0005
                if initial_z - float(target[2, 3]) > 0.01:
                    break
                self._line(chosen["index"], chosen["arm"], target.tolist(), report)
        raise RuntimeError("No intended support contact within the bounded descent; holding grip")

    @skill
    def get_scene(self) -> str:
        """Inspect fresh object IDs, types, colors, robot-relative sides and held objects.

        This demo uses simulation instance labels and virtual depth scans.
        """
        state = self._sim.primitive_state()
        tray = self._sim.tray_state()
        # Keep this small: the agent re-reads it often and every field lands in its context.
        objects = [
            dict(
                id=row["id"],
                kind=row["kind"],
                color=color_name(row["rgba"]),
                position=[round(float(v), 3) for v in row["position"]],
                forward_m=round(float(row["forward_m"]), 2),
                left_m=round(float(row["left_m"]), 2),
                distance_m=round(float(row["distance_m"]), 2),
                on=(
                    "tray"
                    if row["inside"]
                    else next(
                        (
                            name
                            for name, region in state["defined_regions"].items()
                            if set(region["support_geoms"]) & set(row["support_geoms"])
                        ),
                        None,
                    )
                ),
                held_by=row["held_by"],
                upright=row["upright"],
                support_geoms=row["support_geoms"],
                grasping_arms=row["grasping_arms"],
                contacting_arms=row["contacting_arms"],
                inside=row["inside"],
            )
            for row in state["objects"]
        ]
        return json.dumps(
            dict(
                objects=objects,
                held_objects=state["held_objects"],
                tray=dict(
                    station=tray["station"],
                    held=tray["held"],
                    cargo=tray["cargo"],
                    position=[round(float(v), 3) for v in tray["position"]],
                    support_geoms=tray["support_geoms"],
                    tilt_radians=round(float(tray["tilt_radians"]), 3),
                ),
                regions=list(state["defined_regions"]),
                base_pose=[round(float(v), 3) for v in state["base_pose"]],
                tcp_poses=state.get("tcp_poses", {}),
                sim_time=round(float(state["sim_time"]), 1),
                error=state["error"],
                action=self._status(),
                controller="classical_graspgenx",
            )
        )

    @skill
    def get_surfaces(self) -> str:
        """Inspect measured support regions before choosing a placement."""
        return json.dumps(self._sim.primitive_state()["defined_regions"])

    @skill
    def pick_object(self, object: str = "nearest", arm: str = "auto") -> str:
        """Approach, grasp and lift the item, then HOLD it without opening either hand.

        Return/retraction is a separate choice, not a prerequisite or part of this action.

        Args:
            object: Exact ID or combined attributes, e.g. blue carton on the left.
            arm: auto compares free hands; left/right strictly preserves that hand.
        """
        state = self._sim.primitive_state()
        try:
            if arm not in (*ARMS, "auto"):
                raise ValueError("Choose auto, left or right")
            if self._sim.tray_state()["held"]:
                raise ValueError("Both hands hold the tray; put_down_tray first")
            if arm in ARMS and state["held_objects"][arm]:
                raise ValueError(f"The {arm} hand is occupied")
            index = resolve_classical_object(state["objects"], object)
        except ValueError as exc:
            return json.dumps(dict(accepted=False, reason=str(exc)))

        def operation(report: dict[str, Any]) -> None:
            self._stop_control()
            report["requested_object"] = object
            report["selected_object"] = f"object_{index + 1}"
            self._phase("perceive", report)
            cloud = self._sim.classical_object_cloud(index)
            self._phase("generate_grasps", report)
            candidates = self._grasp_generator.propose_grasps(cloud)
            report["grasp_candidates"] = len(candidates)
            self._phase("assess_reachability", report)
            options = self._assess_pick(index, candidates, arm)
            if not options:
                raise RuntimeError(
                    "No checked approach/grasp/lift plan for the requested hand; "
                    "see assessment-worker.log for collision, reach or IK rejection details"
                )
            chosen = options[0]
            side = chosen["arm"]
            if arm not in ("auto", side):
                raise RuntimeError("Reachability attempted to substitute the requested arm")
            report["grasp"] = chosen
            current = np.asarray(self._sim.primitive_state()["base_pose"])
            desired = np.asarray(chosen["base_pose"])
            if np.linalg.norm(current[:2] - desired[:2]) > 0.7:
                self._phase("navigate", report)
                self._navigate(f"object_{index + 1}", side, report, chosen["base_pose"])
            stance = dict(chosen, target=chosen["source_position"])
            selection = self._sim.prepare_reachable_primitive("pick", side, index, "", stance)
            report["selection"] = selection
            self._phase("position_body", report)
            self._position_base(selection, report)
            self._phase("stage_pregrasp", report)
            self._prepare_posture(selection, report)
            self._phase("approach", report)
            self._line(index, side, chosen["tcp"], report)
            self._phase("close_gripper", report)
            self._gripper(side, 0.0, report)
            closed = self._sim.primitive_state()["objects"][index]
            if side not in closed["grasping_arms"]:
                raise RuntimeError("The requested hand did not establish two-pad object contact")
            self._phase("lift", report)
            target = np.asarray(chosen["tcp"])
            target[2, 3] += 0.12
            self._line(index, side, target.tolist(), report)
            self._pause(0.5)
            final = self._sim.primitive_state()
            if final["held_objects"][side] != f"object_{index + 1}" or not final["complete"]:
                raise RuntimeError("The selected object did not survive the verified lift")
            report["lift_verified"] = True
            self._phase("holding", report)

        return self._start("pick", operation)

    @skill
    def place_object(self, region: str, arm: str = "auto") -> str:
        """Place the held item on an empty supported region, then release and retreat.

        Args:
            region: A region returned by get_surfaces, or tray/table.
            arm: Holding hand; auto requires exactly one occupied hand.
        """
        held = self._sim.primitive_state()["held_objects"]
        choices = [side for side in ARMS if held[side] and arm in ("auto", side)]
        if len(choices) != 1:
            return json.dumps(dict(accepted=False, reason="Specify one occupied hand"))
        side = choices[0]
        tray = self._sim.tray_state()
        if tray["held"]:
            return json.dumps(dict(accepted=False, reason="Both hands hold the tray"))
        if region == "tray" and tray["station"] is None:
            return json.dumps(dict(accepted=False, reason="The tray is not resting on a platform"))

        def operation(report: dict[str, Any]) -> None:
            self._stop_control()
            self._phase("find_empty_support", report)
            goals = self._sim.classical_place_goals(side, region)
            if not goals:
                raise RuntimeError(
                    "No empty spot has a clear placement and release corridor with this hand"
                )
            chosen = goals[0]
            report["placement"] = chosen
            current = np.asarray(self._sim.primitive_state()["base_pose"])
            desired = np.asarray(chosen["base_pose"])
            if np.linalg.norm(current[:2] - desired[:2]) > 0.7:
                self._phase("navigate", report)
                if region == "tray":
                    destination = str(self._sim.tray_state()["station"])
                else:
                    destination = "worktable" if region == "table" else region
                self._navigate(destination, side, report, chosen["base_pose"])
                # Carrying and turning change the measured grasp transform.
                # Choose the final support corridor from the arrived state.
                goals = self._sim.classical_place_goals(side, region)
                if not goals:
                    raise RuntimeError("No supported placement corridor after arrival")
                chosen = goals[0]
                report["placement"] = chosen
            selection = self._sim.prepare_reachable_primitive(
                "place", side, chosen["index"], region, chosen
            )
            self._phase("position_body", report)
            self._position_base(selection, report)
            self._phase("preplace", report)
            self._prepare_posture(selection, report)
            self._phase("lower_to_support", report)
            self._line(chosen["index"], side, chosen["tcp"], report)
            self._seek_support(chosen, report)
            self._phase("release", report)
            # Open just enough to free the item; a full opening inside the tray
            # can press a pad against the rim and block the retreat plan.
            closed = self._sim.primitive_state()["joint_positions"][f"r1pro/{side}_gripper"]
            self._gripper(side, min(0.05, closed + 0.015), report)
            self._phase("retreat", report)
            self._line(chosen["index"], side, chosen["preplace"], report)
            self._pause(0.5)
            if not self._sim.primitive_state()["complete"]:
                raise RuntimeError("The released item did not settle inside the requested support")
            self._phase("placed", report)

        return self._start("place", operation)

    @skill
    def go_to(self, destination: str) -> str:
        """Navigate to a named region from get_surfaces while keeping all held objects. Never release.

        A held tray travels with the robot and docks where put_down_tray would set it down.
        """

        if self._sim.tray_state()["held"]:
            try:
                self._sim.tray_dock_pose(destination)
            except (ValueError, RuntimeError) as exc:
                return json.dumps(dict(accepted=False, reason=str(exc)))

        def operation(report: dict[str, Any]) -> None:
            self._phase("navigate", report)
            if self._sim.tray_state()["held"]:
                dock = self._sim.tray_dock_pose(destination)
                report["dock"] = dock
                self._dock(dock, report, carry_tray=True)
            else:
                held = self._sim.primitive_state()["held_objects"]
                arm = next((side for side in ARMS if held[side]), "right")
                self._navigate(destination, arm, report)
            self._phase("arrived", report)

        return self._start("navigate", operation)

    @skill
    def pick_up_tray(self) -> str:
        """Dock beside the resting tray and lift it with both hands, keeping its contents."""
        if any(self._sim.primitive_state()["held_objects"].values()):
            return json.dumps(
                dict(accepted=False, reason="Both hands must be free to lift the tray")
            )
        tray = self._sim.tray_state()
        if tray["held"]:
            return json.dumps(dict(accepted=False, reason="The tray is already held"))
        if tray["finger_contacts"] or tray["station"] is None:
            return json.dumps(
                dict(accepted=False, reason="The tray must rest untouched on a known platform")
            )

        def operation(report: dict[str, Any]) -> None:
            self._stop_control()
            dock = self._sim.tray_dock_pose(None)
            report["dock"] = dock
            self._dock(dock, report, carry_tray=False)
            self._phase("plan_tray_pickup", report)
            self._sim.prepare_tray_holding()
            support = set(self._sim.tray_state()["support_geoms"])
            waypoints = self._sim.plan_tray_motion("pickup", None)
            final = run_tray_motion(
                self._control,
                self._sim,
                waypoints,
                report,
                self._pause,
                lambda name: self._phase(name, report),
                allowed_support=support,
            )
            if not final["held"]:
                raise RuntimeError("The tray did not leave its support in both hands")
            report["tray"] = final
            self._phase("holding_tray", report)

        return self._start("pick_up_tray", operation)

    @skill
    def put_down_tray(self, region: str) -> str:
        """Carry the held tray to a named platform, set it down, release both hands and retreat.

        Args:
            region: A platform name from get_surfaces.
        """
        if not self._sim.tray_state()["held"]:
            return json.dumps(dict(accepted=False, reason="Both hands must hold the tray first"))
        try:
            self._sim.tray_dock_pose(region)
        except (ValueError, RuntimeError) as exc:
            return json.dumps(dict(accepted=False, reason=str(exc)))

        def operation(report: dict[str, Any]) -> None:
            self._stop_control()
            dock = self._sim.tray_dock_pose(region)
            report["dock"] = dock
            self._dock(dock, report, carry_tray=True)
            self._phase("plan_tray_placement", report)
            support = set(dock["support_geoms"])
            waypoints = self._sim.plan_tray_motion("place", dock["target"])
            final = run_tray_motion(
                self._control,
                self._sim,
                waypoints,
                report,
                self._pause,
                lambda name: self._phase(name, report),
                allowed_support=support,
                destination_support=support,
            )
            if not (final["released"] and set(final["support_geoms"]) & support):
                raise RuntimeError("Tray was not released onto the requested platform")
            report["tray"] = final
            self._phase("tray_placed", report)

        return self._start("put_down_tray", operation)
