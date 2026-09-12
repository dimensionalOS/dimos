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

"""Cancellable, individually requested house actions exposed to an LLM agent."""

from collections import deque
from collections.abc import Callable
from concurrent.futures import CancelledError
from contextlib import suppress
import json
from pathlib import Path
import threading
import time
from typing import Any

import numpy as np
from pydantic import Field

from dimos.agents.annotation import skill
from dimos.control.tasks.trajectory_task.trajectory_task import TrajectoryExecutionStatus
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.robot.galaxea.r1pro.home_spec import (
    HomeReadyControlSpec,
    InteractiveSimSpec,
    PackingPolicySpec,
)
from dimos.robot.galaxea.r1pro.home_surfaces import station_name
from dimos.robot.galaxea.r1pro.learning import R1PRO_PICK_PLACE_JOINTS
from dimos.robot.galaxea.r1pro.navigation_delivery import (
    prepare_navigation_map,
    run_navigation_transport,
)
from dimos.robot.galaxea.r1pro.navigation_sim import NAV_TASK
from dimos.robot.galaxea.r1pro.packing_run import run_bottle_pick
from dimos.robot.galaxea.r1pro.tray_delivery import _arm_motion
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def select_object(rows: list[dict[str, Any]], selector: str) -> int:
    """Resolve a stable ID or an unambiguous spatial selector in the robot frame."""
    if not rows:
        raise ValueError("No eligible bottles remain")
    selector = selector.strip().lower().replace(" ", "_")
    for row in rows:
        if selector in (row["id"], str(row["bottle"])):
            return int(row["bottle"]) - 1
    keys: dict[str, Callable[[dict[str, Any]], float]] = {
        "nearest": lambda r: r["distance_m"],
        "furthest": lambda r: -r["distance_m"],
        "farthest": lambda r: -r["distance_m"],
        "rightmost": lambda r: r["left_m"],
        "leftmost": lambda r: -r["left_m"],
    }
    if selector not in keys:
        raise ValueError("Choose bottle_1..bottle_5, nearest, furthest, rightmost, or leftmost")
    return int(min(rows, key=lambda row: (keys[selector](row), row["bottle"]))["bottle"]) - 1


class R1ProHomeSkillsConfig(ModuleConfig):
    pick_timeout: float = Field(default=50.0, gt=0, le=120)


class R1ProHomeSkills(Module):
    """One exclusive action at a time; the robot stays idle between requests."""

    config: R1ProHomeSkillsConfig

    _sim: InteractiveSimSpec
    _control: HomeReadyControlSpec
    _policy: PackingPolicySpec

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._lock = threading.RLock()
        self._cancel = threading.Event()
        self._done = threading.Event()
        self._done.set()
        self._thread: threading.Thread | None = None
        self._action: dict[str, Any] = {"state": "idle"}
        self._counter = 0
        self._station = "worktop"
        self._delivered: dict[int, str] = {}
        self._arm_command: list[float] | None = None
        self._needs_recovery = False
        self._failed_unload: dict[str, Any] | None = None

    def _pause(self, seconds: float) -> None:
        if self._cancel.wait(seconds):
            raise CancelledError("Action cancelled")

    def _status(self) -> dict[str, Any]:
        with self._lock:
            return {key: value for key, value in self._action.items() if key != "report"}

    def _start_action(self, name: str, operation: Callable[[dict[str, Any]], None]) -> str:
        with self._lock:
            if not self._done.is_set():
                return json.dumps({"accepted": False, "reason": "busy", **self._status()})
            if self._needs_recovery and name not in ("recover", "reset scene"):
                return json.dumps(
                    {
                        "accepted": False,
                        "reason": "recovery_required",
                        "next_action": "recover_action or reset_scene",
                        **self._status(),
                    }
                )
            self._counter += 1
            self._cancel.clear()
            self._done.clear()
            self._action = {"id": self._counter, "name": name, "state": "running", "success": False}
            self._thread = threading.Thread(
                target=self._run, args=(operation,), name="r1pro-house-action", daemon=True
            )
            self._thread.start()
            return json.dumps({"accepted": True, **self._status()})

    def _run(self, operation: Callable[[dict[str, Any]], None]) -> None:
        report: dict[str, Any] = {"stages": []}
        try:
            self._sim.home_session()
            deadline = time.monotonic() + 120
            while (
                not self._sim.is_simulation_running() or NAV_TASK not in self._control.list_tasks()
            ):
                if time.monotonic() > deadline:
                    raise RuntimeError("House simulation did not become ready")
                self._pause(0.1)
            deadline = time.monotonic() + 10
            while not self._control.base_connection_status()["ready"]:
                if time.monotonic() > deadline:
                    raise RuntimeError("Base adapter received no odometry; check transport wiring")
                self._pause(0.1)
            deadline = time.monotonic() + 10
            while not self._sim.packing_state()["ready_for_pick"]:
                if time.monotonic() > deadline:
                    raise RuntimeError("House scene did not settle before the action")
                self._pause(0.05)
            report["initial_snapshot"] = self._sim.simulation_snapshot()
            operation(report)
            self._needs_recovery = False
            terminal = {"state": "completed", "success": True}
        except CancelledError as error:
            self._arm_command = None
            self._needs_recovery = True
            terminal = {
                "state": "cancelled",
                "success": False,
                "error": str(error),
                "next_action": "recover_action or reset_scene",
            }
        except Exception as error:
            logger.exception("House action failed", error=str(error))
            moved = bool(
                report.get("history") or report["stages"] or report.get("unload", {}).get("stages")
            )
            self._needs_recovery = moved or self._needs_recovery
            self._failed_unload = report.get("unload", self._failed_unload)
            terminal = {"state": "failed", "success": False, "error": str(error)}
            if moved and self._action["name"].startswith("pick ") and self._station == "worktop":
                try:
                    self._stop_act_for_recovery()
                    self._recover_pick(report)
                    self._needs_recovery = False
                    terminal["recovery"] = (
                        "Released supported contacts and returned to the calibrated ACT home posture"
                    )
                except Exception as recovery_error:
                    terminal["recovery"] = f"Holding position: {recovery_error}"
            if self._needs_recovery:
                terminal["next_action"] = (
                    "Call recover_action; reset_scene restarts the simulation if recovery is blocked"
                )
        finally:
            for cleanup in (
                self._policy.stop_rollout,
                lambda: self._control.cancel_trajectory("tray_manipulation"),
                lambda: self._control.task_invoke(NAV_TASK, "cancel", {}),
                self._sim.stop_navigation_base,
            ):
                with suppress(Exception):
                    cleanup()
            with suppress(Exception):
                report["final_snapshot"] = self._sim.simulation_snapshot()
                report["final"] = self._sim.inventory()
                output = (
                    Path(self._sim.home_session()["output"]) / f"action-{self._counter:03d}.json"
                )
                output.write_text(
                    json.dumps({**report, **self._status(), **terminal}, indent=2) + "\n"
                )
                with self._lock:
                    self._action["evidence"] = str(output)
            with self._lock:
                self._action.update(terminal)
                self._done.set()

    def _move_arms(self, waypoints: list[dict[str, Any]], report: dict[str, Any]) -> dict[str, Any]:
        if self._arm_command is not None:
            report["last_arm_command"] = self._arm_command
        state = _arm_motion(self._control, self._sim, waypoints, report, pause=self._pause)
        self._arm_command = report.get("last_arm_command")
        return state

    def _pickup(self, report: dict[str, Any]) -> None:
        inventory = self._sim.inventory()
        if inventory["tray"]["bimanual_grasp"] and not inventory["tray"]["support_geoms"]:
            return
        if not inventory["tray"]["released"] or not inventory["tray"]["support_geoms"]:
            raise RuntimeError("Tray must be released and resting on a surface before pickup")
        cargo = [row["bottle"] - 1 for row in inventory["bottles"] if row["inside_bin"]]
        self._sim.set_carried_bottles(cargo)
        report["cargo"] = cargo
        report["initial"] = self._sim.task_state()
        if self._station != "worktop":
            source = report.get("source") or self._sim.station(self._station)
            report["pickup_support_geoms"] = source.get("support_geoms", [source["support_geom"]])
        self._sim.prepare_tray_holding()
        report["pickup"] = self._move_arms(self._sim.plan_tray_motion("pickup"), report)
        report["pickup_snapshot"] = self._sim.simulation_snapshot()
        self._sim.set_tray_delivery_view()

    def _put_down(self, report: dict[str, Any]) -> None:
        state = self._sim.task_state()
        if state["tray"]["released"] and state["tray"]["support_geoms"]:
            return
        if self._station == "worktop":
            raise RuntimeError("Navigate to a placement surface before putting the tray down")
        destination = self._sim.station(self._station)
        pose_error = np.asarray(state["base_pose"]) - destination["base_position"]
        pose_error[2] = (pose_error[2] + np.pi) % (2 * np.pi) - np.pi
        if np.linalg.norm(pose_error[:2]) > 0.02 or abs(pose_error[2]) > 0.02:
            raise RuntimeError("Base is outside the station docking tolerance")
        report["initial"] = state
        report["destination"] = destination
        final = self._move_arms(
            self._sim.plan_tray_motion("place", destination["tray_position"]), report
        )
        if not (
            final["tray"]["released"]
            and set(destination.get("support_geoms", [destination["support_geom"]])).intersection(
                final["tray"]["support_geoms"]
            )
        ):
            raise RuntimeError("Tray was not released onto the destination surface")

    @skill
    def get_scene(self) -> str:
        """Read bottle IDs, tray contents, robot-relative coordinates, station, and action status.

        Positive left_m is robot-left; rightmost has the smallest left_m.
        Coordinates are simulator ground truth. dining_table is the laptop table.
        """
        return json.dumps(
            {
                **self._sim.inventory(),
                "station": self._station,
                "delivered": self._delivered,
                "action": self._status(),
                "recovery_required": self._needs_recovery,
            }
        )

    @skill
    def get_surfaces(self) -> str:
        """List bed, floor, kitchen counter and laptop table with measured placement points.

        Geometry detection does not guarantee a clear route or reachable loaded pose;
        go_to and put_down_tray validate those against the current robot and cargo.
        """
        return json.dumps(self._sim.surfaces())

    @skill
    def pick_bottle(self, bottle: str = "nearest") -> str:
        """Start one ACT bottle-to-tray pick at the starting worktop.

        Args:
            bottle: bottle_1..bottle_5, nearest, furthest, rightmost, or leftmost.
        Wait for completion before requesting another action. Stops when the tray is full.
        """

        def operation(report: dict[str, Any]) -> None:
            if self._station != "worktop":
                raise RuntimeError(
                    "This ACT checkpoint picks source bottles at the starting worktop"
                )
            inventory = self._sim.inventory()
            if not inventory["tray"]["released"] or not inventory["tray"]["support_geoms"]:
                raise RuntimeError("Put the tray on the worktop before requesting an ACT pick")
            rows = [
                r
                for r in inventory["bottles"]
                if not r["inside_bin"] and r["bottle"] - 1 not in self._delivered
            ]
            index = select_object(rows, bottle)
            with self._lock:
                self._action["selected_bottle"] = f"bottle_{index + 1}"
            result = run_bottle_pick(
                self._policy,
                self._sim,
                index,
                report,
                seconds=self.config.pick_timeout,
                pause=self._pause,
            )
            self._arm_command = None
            if not result["success"]:
                raise RuntimeError(result.get("reason", "ACT did not complete the selected pick"))

        return self._start_action(f"pick {bottle}", operation)

    @skill
    def pick_up_tray(self) -> str:
        """Start a two-handed pickup of the tray and the bottles currently inside it."""
        return self._start_action("pick up tray", self._pickup)

    @skill
    def go_to(self, destination: str) -> str:
        """Carry the tray to a measured surface using KronkNav and holonomic control.

        Picks the tray up first if it is supported. Keeps holding it on arrival.
        Args:
            destination: dining_table, kitchen, bed, or floor; get_surfaces lists geometry.
        Reach and collision checks can reject a blocked destination.
        """

        destination = station_name(destination)

        def operation(report: dict[str, Any]) -> None:
            station = self._sim.station(destination)
            if destination == self._station:
                report["already_there"] = True
                return
            if self._station != "worktop":
                report["source"] = self._sim.station(self._station)
            prepare_navigation_map(self._sim, Path(self._sim.home_session()["cloud"]), self._pause)
            self._pickup(report)
            report["destination"] = station
            run_navigation_transport(
                self._control,
                self._sim,
                report,
                Path(self._sim.home_session()["cloud"]),
                self._pause,
            )
            self._station = destination

        return self._start_action(f"go to {destination}", operation)

    @skill
    def put_down_tray(self) -> str:
        """Place the held tray on the current station's support and release both hands."""
        return self._start_action("put down tray", self._put_down)

    @skill
    def place_bottle(self, bottle: str = "nearest") -> str:
        """Unload one bottle from the tray onto the current dining or kitchen surface.

        Sets the tray down first so the hands are free; leaves it supported afterward.
        Args:
            bottle: bottle_1..bottle_5, nearest, furthest, rightmost, or leftmost.
        """

        def operation(report: dict[str, Any]) -> None:
            if self._station not in ("dining_table", "kitchen"):
                raise RuntimeError("Navigate to dining_table or kitchen before unloading")
            rows = [row for row in self._sim.inventory()["bottles"] if row["inside_bin"]]
            index = select_object(rows, bottle)
            self._put_down(report)
            plan = self._sim.plan_bottle_unload(index, self._station)
            report["unload"] = {"bottle": index + 1, **plan, "stages": []}
            self._unload(index, plan, report["unload"])
            self._delivered[index] = self._station
            remaining = [
                row["bottle"] - 1 for row in self._sim.inventory()["bottles"] if row["inside_bin"]
            ]
            self._sim.set_carried_bottles(remaining)

        return self._start_action(f"place {bottle}", operation)

    def _unload(self, index: int, plan: dict[str, Any], report: dict[str, Any]) -> None:
        remaining = [
            row["bottle"]
            for row in self._sim.inventory()["bottles"]
            if row["inside_bin"] and row["bottle"] != index + 1
        ]
        joints = list(R1PRO_PICK_PLACE_JOINTS)
        pending = deque(plan["waypoints"])
        descent_steps = 0
        while pending:
            point = pending.popleft()
            self._pause(0)
            if point["phase"] == "unload_release":
                if plan["support_geom"] not in self._sim.bottle_state(index)["support_geoms"]:
                    if descent_steps >= 8:
                        raise RuntimeError(
                            "Bottle still lacks support after 24 mm of measured descent; holding for recovery"
                        )
                    refinement = self._sim.plan_bottle_descent(index, plan["support_geom"])
                    if not refinement["supported"]:
                        pending.appendleft(point)
                        point = refinement["waypoint"]
                        descent_steps += 1
                if point["phase"] == "unload_release" and self._arm_command is not None:
                    positions = self._arm_command.copy()
                    positions[-1] = 0.05
                    point = {**point, "positions": positions}
            measured = self._control.get_joint_positions()
            start = self._arm_command or [measured[name] for name in joints]
            duration = point["seconds"]
            trajectory = JointTrajectory(
                joint_names=joints,
                points=[
                    TrajectoryPoint(positions=start, velocities=[0.0] * 20, time_from_start=0.0),
                    TrajectoryPoint(
                        positions=point["positions"],
                        velocities=[0.0] * 20,
                        time_from_start=duration,
                    ),
                    TrajectoryPoint(
                        positions=point["positions"],
                        velocities=[0.0] * 20,
                        time_from_start=duration + 0.5,
                    ),
                ],
            )
            accepted = self._control.execute_trajectory(trajectory, "tray_manipulation")
            if accepted.status is not TrajectoryExecutionStatus.ACCEPTED:
                raise RuntimeError(f"Unloading trajectory rejected: {accepted}")
            stage: dict[str, Any] = {"phase": point["phase"], "history": []}
            report["stages"].append(stage)
            began = time.monotonic()
            try:
                while True:
                    bottle = self._sim.bottle_state(index)
                    stage["history"].append(bottle)
                    state = self._sim.task_state()
                    if state["robot_obstacles"]:
                        raise RuntimeError(f"Unloading arm contacted {state['robot_obstacles']}")
                    if plan["support_geom"] not in state["tray"]["support_geoms"]:
                        raise RuntimeError("Tray lost tabletop support while unloading")
                    if any(
                        not (r["inside_bin"] and r["upright"])
                        for r in state["bottles"]
                        if r["bottle"] in remaining
                    ):
                        raise RuntimeError("Unloading disturbed a bottle remaining in the tray")
                    elapsed = time.monotonic() - began
                    if elapsed >= duration + 0.5:
                        measured = self._control.get_joint_positions()
                        if (
                            max(
                                abs(measured[n] - q)
                                for n, q in zip(joints, point["positions"], strict=True)
                            )
                            < 0.05
                        ):
                            break
                    if elapsed > 2 * duration + 10:
                        raise RuntimeError(f"{point['phase']} did not reach its target")
                    self._pause(0.05)
            finally:
                self._control.cancel_trajectory("tray_manipulation")
            self._arm_command = point["positions"]
            if (
                point["phase"] in ("unload_grasp", "unload_lift", "unload_transfer")
                and not bottle["grasped"]
            ):
                raise RuntimeError(f"Bottle grasp failed during {point['phase']}")
        final = self._sim.bottle_state(index)
        report["final"] = final
        report["success"] = bool(
            final["released"]
            and final["upright"]
            and final["velocity_norm"] < 0.03
            and plan["support_geom"] in final["support_geoms"]
            and np.linalg.norm(np.array(final["position"][:2]) - plan["target"][:2]) < 0.03
        )
        if not report["success"]:
            raise RuntimeError("Bottle did not settle upright on the requested surface")

    def _stop_act_for_recovery(self) -> None:
        stopped = self._policy.stop_rollout()
        if stopped["active"]:
            raise RuntimeError("ACT is still active; cannot start recovery")
        cancelled = self._control.cancel_trajectory("policy_rollout")
        if not cancelled.safe:
            raise RuntimeError(f"ACT trajectory cancellation was uncertain: {cancelled.message}")
        # This also verifies the inference thread has exited. last_error can
        # describe the failed observation/inference, even after a successful stop.
        self._policy.clear_rollout_observations()

    def _recover_pick(self, report: dict[str, Any]) -> None:
        points = deque(self._sim.plan_pick_recovery())
        descents = 0
        baseline = self._sim.inventory()["bottles"]
        joints = list(R1PRO_PICK_PLACE_JOINTS)
        while points:
            point = points.popleft()
            if point["phase"] == "recovery_seek_support":
                if descents >= 8:
                    raise RuntimeError(
                        "Recovery found no worktop support after 24 mm; holding position"
                    )
                descents += 1
            measured = self._control.get_joint_positions()
            start = self._arm_command or [measured[n] for n in joints]
            duration = point["seconds"]
            trajectory = JointTrajectory(
                joint_names=joints,
                points=[
                    TrajectoryPoint(
                        positions=start, velocities=[0.0] * len(joints), time_from_start=0.0
                    ),
                    TrajectoryPoint(
                        positions=point["positions"],
                        velocities=[0.0] * len(joints),
                        time_from_start=duration,
                    ),
                    TrajectoryPoint(
                        positions=point["positions"],
                        velocities=[0.0] * len(joints),
                        time_from_start=duration + 1.5,
                    ),
                ],
            )
            accepted = self._control.execute_trajectory(trajectory, "tray_manipulation")
            if accepted.status is not TrajectoryExecutionStatus.ACCEPTED:
                raise RuntimeError(f"Recovery trajectory rejected: {accepted}")
            report["stages"].append({"phase": point["phase"]})
            began = time.monotonic()
            try:
                while True:
                    state = self._sim.task_state()
                    obstacles = set(state["robot_obstacles"]) - set(
                        point.get("allowed_contacts", [])
                    )
                    if obstacles or not state["tray"]["support_geoms"]:
                        raise RuntimeError("Recovery lost tray support or contacted an obstacle")
                    for initial, current in zip(
                        baseline, self._sim.inventory()["bottles"], strict=True
                    ):
                        if (
                            not current["upright"]
                            or np.linalg.norm(
                                np.asarray(current["bottle_position"]) - initial["bottle_position"]
                            )
                            > 0.025
                        ):
                            raise RuntimeError("Recovery disturbed a bottle; holding position")
                    measured = self._control.get_joint_positions()
                    if time.monotonic() - began >= duration + 0.5 and max(
                        abs(measured[n] - q)
                        for n, q in zip(joints, point["positions"], strict=True)
                    ) < (0.005 if point["phase"] == "recovery_posture" else 0.05):
                        break
                    if time.monotonic() - began > duration + 10:
                        raise RuntimeError("Recovery did not reach its target")
                    self._pause(0.05)
            finally:
                self._control.cancel_trajectory("tray_manipulation")
            self._arm_command = point["positions"]
            if point["phase"] == "recovery_seek_support":
                points.extend(self._sim.plan_pick_recovery())
        report["recovery"] = {"success": True, "kind": "supported_release_and_retreat"}

    @skill
    def recover_action(self) -> str:
        """Recover a failed action using measured support; preserves delivered bottles.

        Releases and retreats after supported worktop contacts, or finishes a bottle
        placement that stopped just above its support. Never retries ACT or drops an
        unsupported object. If blocked, reset_scene can restart the simulation.
        """

        def operation(report: dict[str, Any]) -> None:
            self._stop_act_for_recovery()
            failed = self._failed_unload
            if (
                failed is not None
                and failed.get("stages")
                and failed["stages"][-1]["phase"] in ("unload_place", "unload_seek_support")
            ):
                plan = {
                    **failed,
                    "waypoints": [
                        p
                        for p in failed["waypoints"]
                        if p["phase"] in ("unload_release", "unload_retreat")
                    ],
                }
                report["unload"] = {**plan, "stages": []}
                index = int(failed["bottle"]) - 1
                self._unload(index, plan, report["unload"])
                self._delivered[index] = self._station
                self._sim.set_carried_bottles(
                    [r["bottle"] - 1 for r in self._sim.inventory()["bottles"] if r["inside_bin"]]
                )
            elif (
                self._sim.inventory()["tray"]["bimanual_grasp"]
                and not self._sim.inventory()["tray"]["support_geoms"]
            ):
                state = self._sim.task_state()
                if state["robot_obstacles"] or not state["inside_bin"] or not state["upright"]:
                    raise RuntimeError("The carried tray is unstable; keeping the robot stopped")
                report["recovery"] = {"success": True, "kind": "stable_tray_hold"}
            elif self._station == "worktop":
                self._recover_pick(report)
            else:
                raise RuntimeError(
                    "No supported recovery is available at this pose; use reset_scene to restart"
                )
            self._failed_unload = None

        return self._start_action("recover", operation)

    @skill
    def reset_scene(self) -> str:
        """Restart this simulation with all five bottles on the worktop and clear progress.

        Use only when the user asks to reset/restart. Cannot run during another action;
        stop_action then wait_for_action first. This is an explicit simulation reset.
        """

        def operation(report: dict[str, Any]) -> None:
            self._stop_act_for_recovery()
            self._control.cancel_trajectory("tray_manipulation")
            self._control.task_invoke(NAV_TASK, "cancel", {})
            self._sim.stop_navigation_base()
            if not self._sim.reset():
                raise RuntimeError("Simulation reset did not complete")
            self._control.task_invoke(NAV_TASK, "reset", {})
            self._policy.clear_rollout_observations()
            self._sim.restore_packing_observations()
            self._station = "worktop"
            self._delivered.clear()
            self._arm_command = None
            self._failed_unload = None
            deadline = time.monotonic() + 10
            while not self._sim.packing_state()["ready_for_pick"]:
                if time.monotonic() > deadline:
                    raise RuntimeError("Reset scene did not settle")
                self._pause(0.05)
            report["reset"] = True

        return self._start_action("reset scene", operation)

    @skill
    def wait_for_action(self, seconds: float = 20.0) -> str:
        """Wait up to 20 seconds and report the action outcome; repeat while running."""
        self._done.wait(max(0.0, min(float(seconds), 20.0)))
        return json.dumps(self._status())

    @skill
    def stop_action(self) -> str:
        """Cancel the current action and hold the robot; it does not release held objects."""
        self._cancel.set()
        return json.dumps(self._status())

    @rpc
    def stop(self) -> None:
        self._cancel.set()
        if self._thread is not None:
            self._thread.join(timeout=3)
        super().stop()
