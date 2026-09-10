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

"""Run a bounded R1Pro ACT deployment check, with a native MuJoCo viewer.

Requires the diagnostic artifact made by dimos_lerobot.demo_r1pro_checkpoint.
This tests the live rollout path and stopping; it does not test learned grasping.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time
from typing import Any

from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.imitation.policy.module import POLICY_ROLLOUT_INSTANCE_NAME
from dimos.robot.galaxea.r1pro.sim_act import build_r1pro_act_sim, prepare_r1pro_act_scene


def run_check(args: argparse.Namespace) -> dict[str, Any]:
    """Start an isolated stack, validate motion and stopping, then clean it up."""
    if not (args.artifact / "DIAGNOSTIC_ONLY.txt").is_file():
        raise ValueError("This command only accepts the labelled R1Pro diagnostic checkpoint")
    args.output.mkdir(parents=True, exist_ok=True)
    scene = prepare_r1pro_act_scene(args.output / "scene.xml", scene_package=args.scene_package)
    blueprint = build_r1pro_act_sim(
        scene_path=scene,
        artifact=str(args.artifact),
        headless=args.headless,
    ).global_config(viewer="none", n_workers=3, zenoh_scout_addr=args.zenoh_scout_addr)
    coordinator: ModuleCoordinator | None = None
    result: dict[str, Any] = {"kind": "diagnostic_act_deployment", "grasp_success_tested": False}
    try:
        coordinator = ModuleCoordinator.build(blueprint)
        policy = coordinator.get_instance(POLICY_ROLLOUT_INSTANCE_NAME)
        control = coordinator.get_instance(ControlCoordinator)
        deadline = time.monotonic() + 30
        while True:
            status = policy.preflight_rollout()
            if status["policy_ready"] and status["observations_ready"]:
                break
            if time.monotonic() > deadline:
                raise RuntimeError(f"Preflight failed: {status}")
            time.sleep(0.1)
        result["preflight"] = status
        result["initial_joints"] = control.get_joint_positions()
        status = policy.start_rollout()
        if not status["active"]:
            raise RuntimeError(f"Policy did not start: {status}")
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            status = policy.rollout_status()
            if status["last_error"] or not status["active"]:
                raise RuntimeError(f"Rollout failed: {status}")
            time.sleep(0.1)
        started = time.monotonic()
        result["stopped"] = policy.stop_rollout()
        result["stop_seconds"] = time.monotonic() - started
        result["final_joints"] = control.get_joint_positions()
        expected = {"r1pro/left_arm_joint7": 0.05, "r1pro/right_arm_joint7": -0.05}
        result["max_wrist_error_rad"] = max(
            abs(result["final_joints"][key] - target) for key, target in expected.items()
        )
        if result["stopped"]["active"] or result["stopped"]["last_error"]:
            raise RuntimeError(f"Stop failed: {result['stopped']}")
        if result["stopped"]["chunks_accepted"] < 3 or result["max_wrist_error_rad"] > 0.01:
            raise RuntimeError(f"Motion did not match diagnostic targets: {result}")
        if result["stop_seconds"] > 0.5:
            raise RuntimeError(f"Stop exceeded 0.5 seconds: {result['stop_seconds']}")
        deadline = time.monotonic() + 0.5
        result["max_post_stop_drift_rad"] = 0.0
        while time.monotonic() < deadline:
            positions = control.get_joint_positions()
            drift = max(
                abs(positions[key] - value) for key, value in result["final_joints"].items()
            )
            result["max_post_stop_drift_rad"] = max(result["max_post_stop_drift_rad"], drift)
            time.sleep(0.05)
        after_stop = policy.rollout_status()
        if (
            after_stop["active"]
            or after_stop["chunks_accepted"] != result["stopped"]["chunks_accepted"]
        ):
            raise RuntimeError(f"Policy kept publishing after stop: {after_stop}")
        if result["max_post_stop_drift_rad"] > 0.01:
            raise RuntimeError(f"Joints kept moving after stop: {result}")
        result["passed"] = True
        return result
    except Exception as error:
        result.update(passed=False, error=str(error))
        raise
    finally:
        try:
            if coordinator is not None:
                coordinator.stop()
        finally:
            (args.output / "result.json").write_text(json.dumps(result, indent=2) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--artifact", required=True, type=Path)
    parser.add_argument(
        "--output", required=True, type=Path, help="Unique local scene and result directory"
    )
    parser.add_argument(
        "--zenoh-scout-addr", required=True, help="Separate multicast address:port for this check"
    )
    parser.add_argument("--scene-package", type=Path)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--seconds", type=float, default=6.0)
    args = parser.parse_args()
    if args.seconds < 2 or args.seconds > 60:
        parser.error("--seconds must be between 2 and 60")
    print(json.dumps(run_check(args), indent=2))


if __name__ == "__main__":
    main()
