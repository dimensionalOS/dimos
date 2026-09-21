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

"""Verify KronkNav and planned arm execution in one BEHAVIOR R1 Pro run.

python -m dimos.simulation.behavior.demo_r1pro --headless --report /tmp/r1pro.json
"""

import argparse
import json
import math
from pathlib import Path
import time
from typing import Any

from scipy.spatial.transform import Rotation

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_spec import ExecutionStatus, PlanStatus
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.simulation.behavior.blueprints import behavior_r1pro
from dimos.simulation.behavior.connection import BehaviorConnection
from dimos.simulation.behavior.demo_integration import check_runtime
from dimos.simulation.behavior.r1pro_bridge import BehaviorR1ProBridge
from dimos.simulation.behavior.types import TaskSelection
from dimos.visualization.rerun.bridge import RerunBridgeModule


def wait_ready(sim: Any, bridge: Any, manip: Any) -> dict[str, Any]:
    deadline = time.monotonic() + 60
    while time.monotonic() < deadline:
        check_runtime(sim)
        state: dict[str, Any] = bridge.snapshot()
        if (
            state["position"] is not None
            and state["joints"]
            and all(group.joints is not None for group in manip.get_state().groups.values())
        ):
            return state
        time.sleep(0.1)
    raise TimeoutError("Missing simulator/coordinator/planner feedback")


def check_kinematics(sim: Any, manip: Any) -> dict[str, Any]:
    links = sim.get_ground_truth()["links"]
    groups = manip.get_state().groups
    results = {}
    for side in ("left", "right"):
        pose = groups[f"{side}_arm"].end_effector_pose
        if pose is None:
            raise AssertionError("Planner has no measured end-effector pose")
        actual = links[f"{side}_gripper_link"]
        error = math.dist([pose.position.x, pose.position.y, pose.position.z], actual["position"])
        q = pose.orientation
        orientation_error = float(
            (
                Rotation.from_quat([q.x, q.y, q.z, q.w]).inv()
                * Rotation.from_quat(actual["orientation"])
            ).magnitude()
        )
        results[side] = {"position_error_m": error, "orientation_error_rad": orientation_error}
        if error > 0.01 or orientation_error > math.radians(2):
            raise AssertionError(f"{side} arm model disagrees with simulation: {results[side]}")
    return results


def check_navigation(sim: Any, bridge: Any, goal: list[float] | None) -> dict[str, Any]:
    before = bridge.snapshot()["position"]
    if goal is None:
        q = sim.get_ground_truth()["links"]["base_link"]["orientation"]
        forward = Rotation.from_quat(q).apply([0.8, 0, 0])
        goal = [before[0] + float(forward[0]), before[1] + float(forward[1]), 0.0]
    if math.dist(before[:2], goal[:2]) < 0.5:
        raise ValueError("Navigation proof requires a goal at least 0.5 m from the start")
    bridge.set_goal(*goal)
    deadline = time.monotonic() + 120
    while time.monotonic() < deadline:
        check_runtime(sim)
        state = bridge.snapshot()
        current = state["position"]
        if math.dist(current[:2], goal[:2]) <= 0.2:
            # Require measured stationarity, not just a goal-reached notification.
            held = current
            held_yaw = state["yaw"]
            settle = time.monotonic() + 2
            while time.monotonic() < settle:
                time.sleep(0.1)
                state = bridge.snapshot()
                current = state["position"]
                if math.dist(held[:2], current[:2]) > 0.01 or abs(
                    math.remainder(state["yaw"] - held_yaw, 2 * math.pi)
                ) > math.radians(1):
                    break
            else:
                displacement = math.dist(before[:2], current[:2])
                if displacement < 0.5:
                    raise AssertionError(
                        f"Navigation moved only {displacement} m; at least 0.5 m required"
                    )
                return {
                    "start": before,
                    "goal": goal,
                    "end": current,
                    "error_m": math.dist(current[:2], goal[:2]),
                    "displacement_m": displacement,
                }
        time.sleep(0.1)
    raise TimeoutError(f"Navigation did not reach and stop at {goal}; last position {current}")


def check_arms(sim: Any, bridge: Any, manip: Any) -> dict[str, Any]:
    results = {}
    for side in ("left", "right"):
        before = bridge.snapshot()
        names = [f"r1pro/{side}_arm_joint{i}" for i in range(1, 8)]
        target = [before["joints"][name] for name in names]
        target[-1] += 0.10
        plan = manip.plan_to_joints(
            {f"{side}_arm": JointState(name=names, position=target)}, speed_scale=0.2
        )
        if plan.status != PlanStatus.SUCCEEDED:
            raise AssertionError(f"{side} arm planning failed: {plan}")
        executed = manip.execute(timeout=60)
        if executed.status != ExecutionStatus.COMPLETED:
            raise AssertionError(f"{side} arm execution failed: {executed}")
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            after = bridge.snapshot()
            error = max(
                abs(after["joints"][name] - value)
                for name, value in zip(names, target, strict=True)
            )
            if error <= 0.05:
                break
            time.sleep(0.1)
        else:
            raise AssertionError(f"{side} arm tracking error: {error}")
        if abs(after["joints"][names[-1]] - before["joints"][names[-1]]) < 0.05:
            raise AssertionError("Arm did not measurably move")
        if math.dist(before["position"][:2], after["position"][:2]) > 0.02:
            raise AssertionError("Base moved during arm execution")
        results[side] = {"max_joint_error_rad": error, "kinematics": check_kinematics(sim, manip)}
    return results


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--goal", nargs=3, type=float)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()
    report: dict[str, Any] = {
        "passed": False,
        "localization": "simulator_ground_truth",
        "benchmark_score": False,
    }
    app = None
    try:
        blueprint = autoconnect(
            behavior_r1pro,
            BehaviorConnection.blueprint(
                headless=args.headless,
                task=TaskSelection(),
                publish_scan=True,
                allow_task_changes=False,
            ),
        )
        if args.headless and any(a.module is RerunBridgeModule for a in blueprint.blueprints):
            blueprint = autoconnect(
                blueprint, RerunBridgeModule.blueprint(rerun_open="none", rerun_web=False)
            )
        app = ModuleCoordinator.build(blueprint)
        sim = app.get_instance(BehaviorConnection)
        bridge = app.get_instance(BehaviorR1ProBridge)
        manip = app.get_instance(ManipulationModule)
        report["initial_state"] = wait_ready(sim, bridge, manip)
        report["capabilities"] = sim.describe()
        report["task"] = sim.get_status().episode.task.model_dump()
        report["initial_kinematics"] = check_kinematics(sim, manip)
        print("Initial FK verified; navigating to the test goal", flush=True)
        report["navigation"] = check_navigation(sim, bridge, args.goal)
        report["post_navigation_kinematics"] = check_kinematics(sim, manip)
        print("Navigation and stopped-base FK verified; executing both arms", flush=True)
        report["arms"] = check_arms(sim, bridge, manip)
        print("Both arms verified; checking cancellation and hold", flush=True)
        report["cancellation"] = check_cancellation(sim, bridge, manip)
        report["passed"] = True
    except Exception as error:
        report["error"] = str(error)
        raise
    finally:
        if app is not None:
            app.stop()
        args.report.write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report, indent=2))


def check_cancellation(sim: Any, bridge: Any, manip: Any) -> dict[str, Any]:
    names = [f"r1pro/left_arm_joint{i}" for i in range(1, 8)]
    before = bridge.snapshot()["joints"]
    target = [before[name] for name in names]
    target[-1] -= 0.2
    plan = manip.plan_to_joints(
        {"left_arm": JointState(name=names, position=target)}, speed_scale=0.05
    )
    if plan.status != PlanStatus.SUCCEEDED:
        raise AssertionError(f"Cancellation test planning failed: {plan}")
    execution = manip.execute(blocking=False)
    if execution.status not in (ExecutionStatus.ACCEPTED, ExecutionStatus.EXECUTING):
        raise AssertionError(f"Cancellation test did not start: {execution}")
    deadline = time.monotonic() + 10
    while abs(bridge.snapshot()["joints"][names[-1]] - before[names[-1]]) < 0.01:
        check_runtime(sim)
        if time.monotonic() >= deadline:
            raise TimeoutError("Cancellation test never produced measured motion")
        time.sleep(0.05)
    result = manip.cancel()
    if result.status != ExecutionStatus.ABORTED:
        raise AssertionError(f"Cancellation was not confirmed: {result}")
    time.sleep(0.5)
    held = bridge.snapshot()["joints"]
    maximum_drift = 0.0
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        check_runtime(sim)
        current = bridge.snapshot()["joints"]
        maximum_drift = max(maximum_drift, *(abs(current[n] - held[n]) for n in names))
        time.sleep(0.1)
    if maximum_drift > 0.02:
        raise AssertionError(f"Cancelled arm failed to hold: {maximum_drift} rad drift")
    return {"status": result.status.name, "max_hold_drift_rad": maximum_drift}


if __name__ == "__main__":
    main()
