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

"""Run explicit GPU integration checks through public DimOS streams and RPCs."""

import argparse
from collections.abc import Callable
import json
import math
from pathlib import Path
import time
from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.simulation.behavior.blueprints import behavior_task, behavior_teleop
from dimos.simulation.behavior.connection import BehaviorConnection
from dimos.simulation.behavior.probe import BehaviorProbe
from dimos.simulation.behavior.types import ControlMode


def check_runtime(connection: Any) -> None:
    status = connection.get_status()
    if status.error:
        raise RuntimeError(f"BEHAVIOR runtime failed: {status.error}")


def wait_operation(connection: Any, operation: str, timeout: float = 600) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        check_runtime(connection)
        result = connection.get_operation(operation)
        if result.state != "running":
            if result.state != "succeeded":
                raise RuntimeError(result.model_dump_json())
            return
        time.sleep(0.1)
    connection.cancel_operation(operation)
    raise TimeoutError(f"Operation {operation} exceeded {timeout}s; cancellation requested")


def check_sensors(connection: Any, probe: Any, timeout: float = 30) -> dict[str, Any]:
    required = {
        "color_image",
        "depth_image",
        "camera_info",
        "left_wrist_image",
        "right_wrist_image",
        "left_wrist_depth",
        "right_wrist_depth",
        "left_wrist_camera_info",
        "right_wrist_camera_info",
        "joint_state",
        "odometry",
        "tf",
    }
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        check_runtime(connection)
        snapshot: dict[str, Any] = probe.snapshot()
        if required <= set(snapshot["received"]):
            for prefix in ("", "left_wrist_", "right_wrist_"):
                rgb = snapshot["sensors"]["color_image" if not prefix else prefix + "image"]
                depth = snapshot["sensors"]["depth_image" if not prefix else prefix + "depth"]
                calibration = snapshot["sensors"][prefix + "camera_info"]
                assert rgb["shape"] == [calibration["height"], calibration["width"], 3]
                assert depth["shape"] == rgb["shape"][:2]
                assert depth["dtype"] == "float32" and depth["valid_depth_pixels"] > 0
                assert rgb["frame"] == depth["frame"] == calibration["frame"]
                assert calibration["K"][0] > 0 and calibration["K"][4] > 0
                assert time.time() - rgb["ts"] < 5, "Camera stream is stale"
            return snapshot
        time.sleep(0.1)
    raise TimeoutError(f"Missing streams: {sorted(required - set(probe.snapshot()['received']))}")


def run_for_simulation_time(
    connection: Any, command: Callable[[], None], duration: float = 1.0
) -> None:
    """Keep commands fresh while allowing first-use compilation and slow rendering."""
    start = connection.get_status().episode.step
    steps = math.ceil(duration * connection.describe()["action_hz"])
    deadline = time.monotonic() + 120
    while time.monotonic() < deadline:
        check_runtime(connection)
        if connection.get_status().episode.step - start >= steps:
            return
        command()
        time.sleep(0.05)
    raise TimeoutError(f"Simulation did not advance {steps} steps in 120 seconds")


def drive(connection: Any, probe: Any, duration: float = 1.0) -> None:
    try:
        run_for_simulation_time(connection, lambda: probe.drive(x=0.1), duration)
    finally:
        probe.drive()


def check_joints(connection: Any, probe: Any) -> dict[str, Any]:
    description = connection.describe()
    results = {}
    for group in ("arm_left", "arm_right", "gripper_left", "gripper_right"):
        names = description["joint_groups"][group]
        before = probe.snapshot()["joints"]
        targets = {}
        for name in names:
            low, high = description["joint_limits"][name]
            amount = 0.005 if group.startswith("gripper") else 0.05
            targets[name] = (
                min(high, before[name] + amount)
                if before[name] + amount <= high
                else max(low, before[name] - amount)
            )
        probe.set_joints(targets)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            measured = probe.snapshot()["joints"]
            if all(
                abs(measured[n] - value) < (0.002 if group.startswith("gripper") else 0.02)
                for n, value in targets.items()
            ):
                break
            time.sleep(0.05)
        else:
            raise AssertionError(
                f"{group} did not reach named joint targets: {targets}; got {measured}"
            )
        results[group] = {"targets": targets, "measured": {n: measured[n] for n in names}}
        probe.set_joints({n: before[n] for n in names})
        time.sleep(0.5)
    return results


def check_native(connection: Any, probe: Any) -> dict[str, Any]:
    wait_operation(connection, connection.take_control(ControlMode.NATIVE))
    description = connection.describe()
    measured = probe.snapshot()["joints"]
    action = [0.0] * len(description["action_bounds"])
    for group, indices in description["action_layout"].items():
        if group.startswith("gripper"):
            # The evaluator's smooth gripper controller has one normalized action.
            action[indices[0]] = 1.0  # Keep both grippers open.
        elif group != "base":
            for index, name in zip(indices, description["joint_groups"][group], strict=True):
                action[index] = measured[name]
    # Native base actions are normalized velocities, not per-step displacement.
    action[description["action_layout"]["base"][0]] = 0.2
    before = probe.snapshot()["position"]
    run_for_simulation_time(connection, lambda: probe.send_native(action))
    # Let the command expire before checking the measured hold.
    time.sleep(0.5)
    after = probe.snapshot()["position"]
    time.sleep(0.5)
    held = probe.snapshot()["position"]
    assert math.dist(before[:2], after[:2]) >= 0.02, "Native action did not move the base"
    assert abs(after[2] - before[2]) < 0.1, "Native action lost floor support"
    assert math.dist(after, held) < 0.03, "Expired native action kept moving"
    wait_operation(connection, connection.take_control(ControlMode.DIMOS))
    return {"before": before, "after": after, "held": held}


def check_task(connection: Any, kind: str) -> list[dict[str, Any]]:
    results = []
    for _ in range(2):
        wait_operation(connection, connection.reset_task())
        wait_operation(connection, connection.take_control(ControlMode.PRIMITIVE))
        objects = connection.get_ground_truth()["objects"]
        cans = sorted(key for key in objects if key.startswith("can__of__soda.n.01_"))
        if len(cans) != 3 or "ashcan.n.01_1" not in objects:
            raise RuntimeError(
                "The pinned picking_up_trash fixture requires three cans and one bin"
            )
        for target in cans:
            wait_operation(connection, connection.start_primitive(kind, "GRASP", target))
            wait_operation(
                connection, connection.start_primitive(kind, "PLACE_INSIDE", "ashcan.n.01_1")
            )
        episode = connection.get_status().episode
        if not episode.success:
            raise AssertionError(
                f"BEHAVIOR evaluator did not report success: {episode.model_dump_json()}"
            )
        results.append(episode.model_dump(mode="json"))
    if results[0]["id"] == results[1]["id"]:
        raise AssertionError("Reset did not create a new episode")
    return results


def check_handoff(connection: Any, probe: Any) -> dict[str, Any]:
    wait_operation(connection, connection.reset_task())
    wait_operation(connection, connection.take_control(ControlMode.PRIMITIVE))
    operation = connection.start_primitive("physical", "GRASP", "can__of__soda.n.01_1")
    probe.drive(x=0.1)
    if connection.get_status().control != ControlMode.PRIMITIVE:
        raise AssertionError("Command stream implicitly stole primitive control")
    wait_operation(connection, connection.take_control(ControlMode.DIMOS))
    if connection.get_operation(operation).state != "cancelled":
        raise AssertionError("Takeover did not cancel the primitive")
    before = check_sensors(connection, probe)["position"]
    time.sleep(0.5)
    held = probe.snapshot()["position"]
    if math.dist(before[:2], held[:2]) > 0.03:
        raise AssertionError("Robot moved after takeover without a fresh command")
    drive(connection, probe)
    after = probe.snapshot()["position"]
    if math.dist(held[:2], after[:2]) < 0.02:
        raise AssertionError("Fresh direct commands did not move the robot")
    return {"cancelled": operation, "start": before, "held": held, "end": after}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("demo", choices=["sensors", "task", "handoff"])
    parser.add_argument("--kind", choices=["physical", "symbolic"], default="physical")
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    blueprint = behavior_task if args.demo in ("task", "handoff") else behavior_teleop
    coordinator = ModuleCoordinator.build(autoconnect(blueprint, BehaviorProbe.blueprint()))
    try:
        connection = coordinator.get_instance(BehaviorConnection)
        probe = coordinator.get_instance(BehaviorProbe)
        report: dict[str, Any] = {
            "demo": args.demo,
            "capabilities": connection.describe(),
            "streams": check_sensors(connection, probe),
        }
        if args.demo == "task":
            report["episodes"] = check_task(connection, args.kind)
        elif args.demo == "handoff":
            report["handoff"] = check_handoff(connection, probe)
        elif args.demo == "sensors":
            before = report["streams"]["position"]
            drive(connection, probe)
            check_runtime(connection)
            after = probe.snapshot()["position"]
            assert abs(after[2] - before[2]) < 0.1, "Base lost floor support"
            if math.dist(before[:2], after[:2]) < 0.02:
                raise AssertionError(
                    f"Base failed to move at least 2 cm: {before} -> {after}; "
                    f"status={connection.get_status().model_dump_json()}"
                )
            report["motion"] = {"before": before, "after": after}
            report["joints"] = check_joints(connection, probe)
            report["native"] = check_native(connection, probe)
        report["passed"] = True
        output = json.dumps(report, indent=2)
        print(output)
        if args.report:
            args.report.write_text(output + "\n")
    finally:
        coordinator.stop()


if __name__ == "__main__":
    main()
