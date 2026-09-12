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

"""Interactive selected-object ACT commands with explicit outcomes and recovery."""

from collections.abc import Callable
from concurrent.futures import CancelledError
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
from dimos.robot.galaxea.r1pro.home_spec import HomeControlSpec, PackingPolicySpec
from dimos.robot.galaxea.r1pro.learning import R1PRO_PICK_PLACE_JOINTS
from dimos.robot.galaxea.r1pro.object_packing_run import (
    ObjectPackingSimSpec,
    run_object_pick,
    run_object_place,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def resolve_object(rows: list[dict[str, Any]], selector: str) -> int:
    """Select from eligible source objects using measured robot-frame coordinates."""
    available = [r for r in rows if not r["inside"] and r["upright"] and r["released"]]
    value = selector.strip().lower().replace("-", "_").replace(" ", "_")
    if not available:
        raise ValueError("No eligible objects remain outside the tray")
    for row in available:
        if value in (row["id"], row["object"]):
            return int(row["index"])
    keys = {
        "nearest": ("distance_m", 1),
        "furthest": ("distance_m", -1),
        "farthest": ("distance_m", -1),
        "rightmost": ("left_m", 1),
        "leftmost": ("left_m", -1),
    }
    # A bare shape is accepted only when it identifies one source object.
    candidates = [r for r in available if r["shape"] == value]
    if len(candidates) == 1:
        return int(candidates[0]["index"])
    if value not in keys:
        raise ValueError(
            "Select a source object ID, a unique shape, nearest, furthest, rightmost or leftmost; ambiguous requests need an ID"
        )
    key, sign = keys[value]
    return int(min(available, key=lambda r: (sign * r[key], r["index"]))["index"])


class R1ProObjectSkillsConfig(ModuleConfig):
    pick_timeout: float = Field(default=50.0, gt=0, le=120)
    auto_recover: bool = True


class R1ProObjectSkills(Module):
    config: R1ProObjectSkillsConfig
    _sim: ObjectPackingSimSpec
    _control: HomeControlSpec
    _policy: PackingPolicySpec

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._lock = threading.RLock()
        self._cancel = threading.Event()
        self._done = threading.Event()
        self._done.set()
        self._thread: threading.Thread | None = None
        self._action: dict[str, Any] = {"state": "idle", "recovery_required": False}
        self._counter = 0
        self._closing = False

    def _status(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._action)

    def _pause(self, seconds: float) -> None:
        if self._cancel.wait(seconds):
            raise CancelledError("Action cancelled; holding position")

    def _start(
        self, name: str, operation: Callable[[dict[str, Any]], None], *, recovery: bool = False
    ) -> str:
        with self._lock:
            if self._closing or not self._done.is_set():
                return json.dumps(dict(accepted=False, reason="busy_or_stopping", **self._action))
            if self._action.get("recovery_required") and not recovery:
                return json.dumps(dict(accepted=False, reason="recovery_required", **self._action))
            self._counter += 1
            needs_recovery = self._action.get("recovery_required", False)
            self._action = dict(
                id=self._counter,
                name=name,
                state="running",
                success=False,
                recovery_required=needs_recovery,
            )
            self._cancel.clear()
            self._done.clear()
            self._thread = threading.Thread(
                target=self._run, args=(operation,), name="r1pro-object-action", daemon=True
            )
            self._thread.start()
            return json.dumps(dict(accepted=True, **self._action))

    def _stop_control(self) -> None:
        stopped = self._policy.stop_rollout()
        if stopped["active"]:
            raise RuntimeError("ACT has not stopped")
        for task in ("policy_rollout", "tray_manipulation"):
            result = self._control.cancel_trajectory(task)
            if not result.safe:
                raise RuntimeError(f"Cannot confirm control stopped: {result.message}")

    def _run(self, operation: Callable[[dict[str, Any]], None]) -> None:
        report: dict[str, Any] = {}
        terminal: dict[str, Any] = dict(state="failed", success=False, recovery_required=True)
        try:
            deadline = time.monotonic() + 120
            while (
                not self._sim.is_simulation_running()
                or "policy_rollout" not in self._control.list_tasks()
            ):
                if time.monotonic() > deadline:
                    raise RuntimeError("Object simulation did not become ready")
                self._pause(0.1)
            self._pause(0)
            operation(report)
            terminal = dict(state="completed", success=True, recovery_required=False)
        except CancelledError as exc:
            terminal.update(state="cancelled", error=str(exc))
        except Exception as exc:
            logger.exception("Object action failed", error=str(exc))
            terminal["error"] = str(exc)
            moved = any(report.get(phase, {}).get("history") for phase in ("pick", "place"))
            terminal["recovery_required"] = moved or bool(self._status().get("recovery_required"))
            if moved and self.config.auto_recover and not self._cancel.is_set():
                try:
                    self._recover(report)
                    terminal.update(
                        recovery_required=False,
                        recovery="Supported release and return home completed; requested pick still failed",
                    )
                except Exception as recovery_error:
                    terminal.update(recovery_required=True, recovery=str(recovery_error))
        finally:
            try:
                self._stop_control()
            except Exception as cleanup_error:
                terminal.update(
                    state="failed",
                    success=False,
                    recovery_required=True,
                    cleanup_error=str(cleanup_error),
                )
            try:
                report["final"] = self._sim.object_state()
                path = (
                    Path(self._sim.prepare_object_session()["output"])
                    / f"action-{self._counter:03d}.json"
                )
                path.write_text(
                    json.dumps({**self._status(), **report, **terminal}, indent=2) + "\n"
                )
                terminal["evidence"] = str(path)
            except Exception as report_error:
                terminal["report_error"] = str(report_error)
                logger.exception("Object action report failed")
            with self._lock:
                self._action.update(terminal)
                self._done.set()

    @skill
    def get_scene(self) -> str:
        """Read object IDs, shapes, colors, robot-frame positions, tray contents and current held-object state.

        Coordinates are simulator ground truth. Rightmost has the smallest left_m;
        nearest has the smallest distance_m. Current learned grasps support the right arm.
        """
        return json.dumps(dict(**self._sim.object_state(), action=self._status()))

    @skill
    def pick_object(self, object: str = "nearest", arm: str = "right") -> str:
        """Use ACT to grasp and lift one selected object, then STOP with it held. Never place or release.

        Args:
            object: Stable object_1..object_5 ID, unique shape, nearest, furthest, rightmost or leftmost.
            arm: Requested hand. This checkpoint supports right; left requests are rejected without moving.
        """
        if arm.strip().lower() != "right":
            return json.dumps(
                dict(
                    accepted=False,
                    reason="unsupported_arm",
                    supported_arms=["right"],
                    requested_arm=arm,
                )
            )
        state = self._sim.object_state()
        if state.get("held_object") or any(not row["released"] for row in state["objects"]):
            return json.dumps(
                dict(accepted=False, reason="Place the held object before another pick")
            )
        try:
            index = resolve_object(state["objects"], object)
        except ValueError as exc:
            return json.dumps(dict(accepted=False, reason=str(exc)))

        def operation(report: dict[str, Any]) -> None:
            deadline = time.monotonic() + 10
            while True:
                state = self._sim.object_state()
                if state["at_home"] and state["sim_time"] >= 0.6:
                    break
                if time.monotonic() > deadline:
                    raise RuntimeError("Wait for recovery to return the arm home before picking")
                self._pause(0.05)
            report.update(requested_object=object, selected_index=index, arm="right", pick={})
            run_object_pick(
                self._policy,
                self._sim,
                index,
                report["pick"],
                seconds=self.config.pick_timeout,
                pause=self._pause,
                grasp_only=True,
            )
            if not report["pick"]["success"]:
                raise RuntimeError(
                    report["pick"].get("reason", "Grasp did not remain held after stopping")
                )

        return self._start(f"pick object_{index + 1} with right", operation)

    @skill
    def place_object(self, destination: str = "tray", arm: str = "right") -> str:
        """Use ACT to place the held object, release on support, and return home.

        Args:
            destination: Requested destination. This checkpoint currently supports the tray only.
            arm: Hand holding the object. Currently right only; never substitutes another arm.
        """
        if arm.strip().lower() != "right":
            return json.dumps(
                dict(accepted=False, reason="unsupported_arm", supported_arms=["right"])
            )
        if destination.strip().lower() != "tray":
            return json.dumps(
                dict(
                    accepted=False,
                    reason="unsupported_destination",
                    supported_destinations=["tray"],
                )
            )
        state = self._sim.object_state()
        if not state.get("holding"):
            return json.dumps(dict(accepted=False, reason="Pick and hold an object before placing"))

        def operation(report: dict[str, Any]) -> None:
            report.update(
                destination=destination, arm="right", held_object=state["held_object"], place={}
            )
            run_object_place(
                self._policy,
                self._sim,
                report["place"],
                seconds=self.config.pick_timeout,
                pause=self._pause,
            )
            if not report["place"]["success"]:
                raise RuntimeError(
                    report["place"].get("reason", "Placement did not remain complete")
                )

        return self._start(
            f"place {state['held_object']} in tray with right", operation, recovery=True
        )

    def _recover(self, report: dict[str, Any]) -> None:
        self._stop_control()
        self._policy.clear_rollout_observations()
        points = self._sim.plan_object_recovery()
        baseline = self._sim.object_state()["objects"]
        stages = report.setdefault("recovery_stages", [])
        joints = list(R1PRO_PICK_PLACE_JOINTS)
        limits = np.asarray(self._sim.prepare_object_session()["limits"][: len(joints)])
        for point in points:
            self._pause(0)
            measured = self._control.get_joint_positions()
            # Simulator feedback can exceed an exact stop by nanoradians.
            start = np.clip([measured[n] for n in joints], limits[:, 0], limits[:, 1]).tolist()
            target = np.asarray(point["positions"])
            if np.any(target < limits[:, 0] - 1e-5) or np.any(target > limits[:, 1] + 1e-5):
                raise RuntimeError("Recovery goal exceeds joint limits")
            point = {**point, "positions": np.clip(target, limits[:, 0], limits[:, 1]).tolist()}
            duration = max(
                point["seconds"], float(np.max(np.abs(np.asarray(point["positions"]) - start))) * 2
            )
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
                        time_from_start=duration + 1.0,
                    ),
                ],
            )
            accepted = self._control.execute_trajectory(trajectory, "tray_manipulation")
            if accepted.status is not TrajectoryExecutionStatus.ACCEPTED:
                raise RuntimeError(f"Recovery trajectory rejected: {accepted}")
            stages.append(point)
            began = time.monotonic()
            try:
                while True:
                    self._pause(0.05)
                    if not self._sim.is_simulation_running():
                        raise RuntimeError("Simulation stopped during recovery")
                    state = self._sim.object_state()
                    if state["robot_obstacles"]:
                        raise RuntimeError(
                            f"Recovery contacted an obstacle: {state['robot_obstacles']}"
                        )
                    rows = state["objects"]
                    for before, after in zip(baseline, rows, strict=True):
                        if (
                            not after["upright"]
                            or not after["support_geoms"]
                            or np.linalg.norm(np.asarray(after["position"]) - before["position"])
                            > 0.02
                        ):
                            raise RuntimeError(
                                "Recovery disturbed or lost support for an object; holding"
                            )
                    measured = self._control.get_joint_positions()
                    if (
                        time.monotonic() - began >= duration + 0.3
                        and max(
                            abs(measured[n] - q)
                            for n, q in zip(joints, point["positions"], strict=True)
                        )
                        < 0.01
                    ):
                        break
                    if time.monotonic() - began > duration + 10:
                        raise RuntimeError("Recovery did not reach its waypoint")
            finally:
                self._control.cancel_trajectory("tray_manipulation")
        report["recovered"] = self._sim.finish_object_recovery()

    @skill
    def recover_action(self) -> str:
        """Release only supported contacts and return home after a failure; never retry the grasp.

        Preserves tray contents. If an object is unsupported or the retreat is blocked,
        holds position and reports why. Scene reset is a separate explicit command.
        """
        return self._start("recover", self._recover, recovery=True)

    @skill
    def reset_scene(self) -> str:
        """Explicitly restore this seeded simulation and clear all packing progress.

        Use only when the user asks to reset. Stop and wait for any action first.
        """

        def operation(report: dict[str, Any]) -> None:
            self._stop_control()
            if not self._sim.reset():
                raise RuntimeError("Scene reset did not complete")
            self._policy.clear_rollout_observations()
            deadline = time.monotonic() + 10
            while True:
                state = self._sim.object_state()
                if state["at_home"] and state["sim_time"] >= 0.6:
                    break
                if time.monotonic() > deadline:
                    raise RuntimeError("Reset scene did not settle")
                self._pause(0.05)
            report["reset"] = True

        return self._start("reset scene", operation, recovery=True)

    @skill
    def wait_for_action(self, seconds: float = 20.0) -> str:
        """Wait up to 20 seconds for the current action; repeat while its state is running."""
        self._done.wait(max(0, min(float(seconds), 20)))
        return json.dumps(self._status())

    @skill
    def stop_action(self) -> str:
        """Cancel the current action and hold position without opening the gripper."""
        self._cancel.set()
        return json.dumps(self._status())

    @rpc
    def stop(self) -> None:
        with self._lock:
            self._closing = True
            self._cancel.set()
            thread = self._thread
        if thread is not None:
            thread.join(timeout=10)
            if thread.is_alive():
                logger.error("Object action did not finish before shutdown")
        super().stop()
