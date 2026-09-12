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

"""Shared, cancellable ACT packing and tray delivery sequence."""

from collections.abc import Callable
from dataclasses import dataclass
import json
from pathlib import Path
import time
from typing import Any

from dimos.robot.galaxea.r1pro.home_spec import HomeControlSpec, HomeSimSpec, PackingPolicySpec
from dimos.robot.galaxea.r1pro.navigation_delivery import prepare_navigation_map
from dimos.robot.galaxea.r1pro.packing_checks import validate_other_bottles
from dimos.robot.galaxea.r1pro.tray_delivery import run_tray_delivery


@dataclass(frozen=True)
class PackingRunConfig:
    artifact: Path
    output: Path
    seed: int = 5000
    random_order: bool = False
    seconds: float = 30.0
    deliver_to_laptop: bool = True


def run_bottle_pick(
    policy: PackingPolicySpec,
    sim: HomeSimSpec,
    index: int,
    pick: dict[str, Any],
    *,
    seconds: float = 30.0,
    pause: Callable[[float], None] = time.sleep,
) -> dict[str, Any]:
    """Execute one selected ACT goal and verify grasp, release, and return home."""
    pick.update(bottle=index + 1, history=[], success=False)
    pause(0)
    selected = sim.select_bottle(index)
    pick["selection"] = selected
    if not selected["selected"]:
        pick["reason"] = selected["reason"]
        return pick
    initial_bottles = sim.packing_state()["bottles"]
    policy.clear_rollout_observations()
    deadline = time.monotonic() + 45
    while True:
        status = policy.preflight_rollout()
        if status["policy_ready"] and status["observations_ready"] and not status["last_error"]:
            break
        if time.monotonic() > deadline:
            raise RuntimeError(f"Packing preflight failed: {status}")
        pause(0.1)
    pause(0)
    status = policy.start_rollout()
    if not status["active"]:
        raise RuntimeError(f"Packing policy did not start: {status}")
    deadline = time.monotonic() + seconds
    stable_since = None
    try:
        while time.monotonic() < deadline:
            status = policy.rollout_status()
            if status["last_error"] or not status["active"]:
                raise RuntimeError(f"Packing rollout failed: {status}")
            state = sim.packing_state()
            pick["history"].append(state)
            validate_other_bottles(initial_bottles, state["bottles"], index)
            if state["selected"]["pick_complete"]:
                stable_since = time.monotonic() if stable_since is None else stable_since
                if time.monotonic() - stable_since >= 0.1:
                    pick["success"] = True
                    break
            else:
                stable_since = None
            pause(0.05)
    finally:
        pick["stopped"] = policy.stop_rollout()
        # Cancel at arrival, then verify while the coordinator holds.
        # Keeping ACT active here can start another approach to the old goal.
        pause(0.5)
        pick["final"] = sim.packing_state()
        validate_other_bottles(initial_bottles, pick["final"]["bottles"], index)
        pick["success"] = bool(pick["success"] and pick["final"]["selected"]["pick_complete"])
    if pick["stopped"]["active"] or pick["stopped"]["last_error"]:
        raise RuntimeError(f"Packing policy did not stop cleanly: {pick['stopped']}")
    print(
        json.dumps({key: pick[key] for key in ("bottle", "success", "stopped", "final")}),
        flush=True,
    )
    return pick


def run_packing_sequence(
    control: HomeControlSpec,
    policy: PackingPolicySpec,
    sim: HomeSimSpec,
    config: PackingRunConfig,
    report: dict[str, Any],
    *,
    navigation_cloud: Path | None = None,
    pause: Callable[[float], None] = time.sleep,
) -> dict[str, Any]:
    """Run on already-started modules, retaining physical evidence on failure."""
    report.update(
        artifact=str(config.artifact),
        seed=config.seed,
        order_mode="random" if config.random_order else "left_to_right",
        success=False,
        delivery_requested=config.deliver_to_laptop,
        picks=[],
    )
    try:
        if navigation_cloud is not None:
            prepare_navigation_map(sim, navigation_cloud, pause=pause)
        deadline = time.monotonic() + 30
        while not sim.packing_state()["ready_for_pick"]:
            if time.monotonic() >= deadline:
                raise RuntimeError("Simulation did not settle during startup")
            pause(0.05)
        report["order"] = sim.packing_order(config.seed if config.random_order else None)
        report["initial"] = sim.packing_state()
        for index in report["order"]:
            pick: dict[str, Any] = {"bottle": index + 1, "history": [], "success": False}
            report["picks"].append(pick)
            run_bottle_pick(policy, sim, index, pick, seconds=config.seconds, pause=pause)
            if pick.get("reason"):
                report["completion_reason"] = pick["reason"]
                break
            if not pick["success"]:
                report["completion_reason"] = "pick_failed"
                break
        else:
            report["completion_reason"] = "completed"
        report["final"] = sim.packing_state()
        report["success"] = (
            report["final"]["success"] and report["completion_reason"] == "completed"
        )
        report["packing_success"] = report["success"]
        if report["packing_success"] and config.deliver_to_laptop:
            report["packing_final"] = report["final"]
            report["success"] = False
            report["completion_reason"] = "delivery_in_progress"
            report["delivery"] = {}
            (config.output / "result.json").write_text(json.dumps(report, indent=2) + "\n")
            run_tray_delivery(
                control,
                sim,
                report["delivery"],
                navigation_cloud=navigation_cloud,
                pause=pause,
            )
            report["final"] = sim.task_state()
            report["success"] = bool(report["delivery"]["success"] and report["final"]["success"])
            report["completion_reason"] = "delivered" if report["success"] else "delivery_failed"
        print(
            json.dumps(
                {key: value for key, value in report.items() if key not in ("picks", "delivery")},
                indent=2,
            ),
            flush=True,
        )
        (config.output / "result.json").write_text(json.dumps(report, indent=2) + "\n")
        return report
    except Exception as error:
        report.update(success=False, error=str(error))
        if report.get("completion_reason") == "delivery_in_progress":
            report["completion_reason"] = "delivery_failed"
        raise
    finally:
        (config.output / "result.json").write_text(json.dumps(report, indent=2) + "\n")
