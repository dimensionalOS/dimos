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

"""Cancellable selected-object ACT execution through the native coordinator."""

from collections.abc import Callable
import time
from typing import Any, Literal, Protocol

from dimos.robot.galaxea.r1pro.home_spec import PackingPolicySpec
from dimos.spec.utils import Spec


class ObjectPackingSimSpec(Spec, Protocol):
    def select_object(self, index: int, grasp_only: bool = False) -> dict[str, Any]: ...
    def prepare_object_place(self) -> dict[str, Any]: ...
    def object_state(self) -> dict[str, Any]: ...
    def is_simulation_running(self) -> bool: ...
    def reset(self) -> bool: ...
    def prepare_object_session(self) -> dict[str, Any]: ...
    def plan_object_recovery(self) -> list[dict[str, Any]]: ...
    def finish_object_recovery(self) -> dict[str, Any]: ...


def run_object_pick(
    policy: PackingPolicySpec,
    sim: ObjectPackingSimSpec,
    index: int,
    report: dict[str, Any],
    *,
    seconds: float = 50.0,
    pause: Callable[[float], None] = time.sleep,
    grasp_only: bool = False,
) -> dict[str, Any]:
    """Assign one goal, execute ACT, stop, and verify its physical outcome."""
    report.update(index=index, success=False, history=[])
    pause(0)
    selection = (
        sim.select_object(index, grasp_only=True) if grasp_only else sim.select_object(index)
    )
    report["selection"] = selection
    if not selection["selected"]:
        report["reason"] = selection["reason"]
        return report
    return run_object_action(
        policy,
        sim,
        report,
        completion="holding" if grasp_only else "pick_complete",
        seconds=seconds,
        pause=pause,
    )


def run_object_place(
    policy: PackingPolicySpec,
    sim: ObjectPackingSimSpec,
    report: dict[str, Any],
    *,
    seconds: float = 50.0,
    pause: Callable[[float], None] = time.sleep,
) -> dict[str, Any]:
    """Start a new rollout only after an explicit placement request for the held object."""
    report.update(success=False, history=[])
    pause(0)
    selection = sim.prepare_object_place()
    report["selection"] = selection
    if not selection["selected"]:
        report["reason"] = selection["reason"]
        return report
    return run_object_action(
        policy, sim, report, completion="pick_complete", seconds=seconds, pause=pause
    )


def run_object_action(
    policy: PackingPolicySpec,
    sim: ObjectPackingSimSpec,
    report: dict[str, Any],
    *,
    completion: Literal["holding", "pick_complete"],
    seconds: float,
    pause: Callable[[float], None],
) -> dict[str, Any]:
    """Execute until the requested physical predicate, stop, and verify it while holding."""
    report["completion"] = completion
    policy.clear_rollout_observations()
    deadline = time.monotonic() + 45
    while True:
        status = policy.preflight_rollout()
        if status["policy_ready"] and status["observations_ready"] and not status["last_error"]:
            break
        if time.monotonic() > deadline:
            raise RuntimeError(f"Object policy preflight failed: {status}")
        pause(0.1)
    pause(0)
    try:
        status = policy.start_rollout()
        if not status["active"]:
            raise RuntimeError(f"Object policy did not start: {status}")
        deadline = time.monotonic() + seconds
        stable_since = None
        while time.monotonic() < deadline:
            pause(0.05)
            if not sim.is_simulation_running():
                raise RuntimeError("Simulation viewer or physics loop stopped")
            status = policy.rollout_status()
            if status["last_error"] or not status["active"]:
                raise RuntimeError(f"Object policy stopped unexpectedly: {status}")
            state = sim.object_state()
            report["history"].append(state)
            if state["error"]:
                raise RuntimeError(state["error"])
            if state[completion]:
                stable_since = time.monotonic() if stable_since is None else stable_since
                if time.monotonic() - stable_since >= 0.1:
                    report["success"] = True
                    break
            else:
                stable_since = None
        if not report["success"]:
            report["reason"] = "action_timeout"
    finally:
        report["stopped"] = policy.stop_rollout()
        # A cancellation must still leave a measured, stationary outcome.
        time.sleep(0.5)
        report["final"] = sim.object_state()
        report["success"] = bool(report["success"] and report["final"][completion])
    if report["stopped"]["active"] or report["stopped"]["last_error"]:
        raise RuntimeError(f"Policy failed to stop cleanly: {report['stopped']}")
    return report
