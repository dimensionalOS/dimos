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

"""Evaluate a trained ACT checkpoint against physical bottle containment in simulation."""

import argparse
import json
from pathlib import Path
import time
from typing import Any, cast

import numpy as np

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.imitation.policy.skills import PolicySkills
from dimos.manipulation.grasp_verification import GraspVerificationConfig
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.robot.manipulators.dual_openyam.blueprints.sim_learning import (
    build_dual_openyam_sim_rollout,
)
from dimos.robot.manipulators.dual_openyam.learning import DUAL_OPENYAM_SIM_TASK
from dimos.robot.manipulators.dual_openyam.sim_demo import SimDemoSkills, inside_bin
from dimos.robot.manipulators.dual_openyam.tool_generate_demos import require
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--artifact", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--episodes", type=int, default=10)
    parser.add_argument("--timeout", type=float, default=45.0)
    parser.add_argument("--jitter", type=float, default=0.015)
    parser.add_argument("--seed", type=int, default=1000)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--zenoh-scout-addr", default="")
    args = parser.parse_args()
    if args.episodes < 1 or args.timeout <= 0 or args.jitter < 0:
        parser.error("episodes and timeout must be positive; jitter must be nonnegative")
    if args.report.exists():
        parser.error("report already exists")
    if not args.artifact.is_dir():
        parser.error("checkpoint directory does not exist")
    args.report.parent.mkdir(parents=True, exist_ok=True)
    blueprint = build_dual_openyam_sim_rollout(
        artifact=str(args.artifact.resolve()), task=DUAL_OPENYAM_SIM_TASK, device=args.device
    ).global_config(viewer="none", n_workers=4, zenoh_scout_addr=args.zenoh_scout_addr)
    rng = np.random.default_rng(args.seed)
    coordinator = None
    successes = 0
    try:
        coordinator = ModuleCoordinator.build(blueprint)
        sim = cast("MujocoSimModule", coordinator.get_instance("MujocoSimModule"))
        demo = cast("SimDemoSkills", coordinator.get_instance(SimDemoSkills))
        policy = cast("PolicySkills", coordinator.get_instance(PolicySkills))
        manipulation = cast("ManipulationModule", coordinator.get_instance(ManipulationModule))
        verification = GraspVerificationConfig()
        open_threshold = verification.open_position - verification.open_tolerance

        def gripper_is_open() -> bool:
            position = manipulation.get_state().groups["right_manipulator"].gripper_position
            return position is not None and position >= open_threshold

        time.sleep(2.0)
        with args.report.open("x") as report:
            for episode in range(args.episodes):
                row: dict[str, Any] = {"episode": episode, "success": False}
                start = time.monotonic()
                try:
                    require(demo.reset_scene())
                    pose = sim.get_body_poses(["bottle_1"])["bottle_1"]
                    pose[:2] = (
                        np.asarray(pose[:2]) + rng.uniform(-args.jitter, args.jitter, 2)
                    ).tolist()
                    if not sim.set_body_pose("bottle_1", pose[:3], pose[3:]):
                        raise RuntimeError("Could not randomize target")
                    time.sleep(1.0)
                    initial_z = sim.get_body_poses(["bottle_1"])["bottle_1"][2]
                    row["spawn"] = pose
                    status = json.loads(policy.run_policy())
                    if not status["active"]:
                        raise RuntimeError(status["last_error"] or "Policy did not start")
                    deadline = time.monotonic() + args.timeout
                    peak_lift = 0.0
                    contained_since = None
                    stable_containment = False
                    while time.monotonic() < deadline:
                        current = sim.get_body_poses(["bottle_1"])["bottle_1"]
                        peak_lift = max(peak_lift, current[2] - initial_z)
                        contained = inside_bin(
                            sim.sample_body_surface("bottle_1", 4096),
                            sim.sample_body_surface("bin_container", 4096),
                        )
                        if contained and gripper_is_open():
                            contained_since = contained_since or time.monotonic()
                            if time.monotonic() - contained_since >= 1.0:
                                stable_containment = True
                                break
                        else:
                            contained_since = None
                        status = json.loads(policy.policy_status())
                        if not status["active"]:
                            raise RuntimeError(
                                status["last_error"] or "Policy stopped before completion"
                            )
                        time.sleep(0.2)
                    row["peak_lift_m"] = peak_lift
                    row["stable_containment"] = stable_containment
                    row["policy"] = json.loads(policy.stop_policy())
                    time.sleep(1.0)
                    row["final_pose"] = sim.get_body_poses(["bottle_1"])["bottle_1"]
                    row["right_gripper_position"] = (
                        manipulation.get_state().groups["right_manipulator"].gripper_position
                    )
                    row["right_gripper_open"] = gripper_is_open()
                    row["final_inside_bin"] = inside_bin(
                        sim.sample_body_surface("bottle_1", 8192),
                        sim.sample_body_surface("bin_container", 8192),
                    )
                    row["success"] = (
                        peak_lift >= 0.05
                        and stable_containment
                        and row["right_gripper_open"]
                        and row["final_inside_bin"]
                    )
                    if not row["success"]:
                        row["error"] = (
                            "Bottle was not physically lifted, released, and contained within the time limit"
                        )
                except Exception as exc:
                    row["error"] = str(exc)
                finally:
                    row["final_policy"] = json.loads(policy.stop_policy())
                if row["success"]:
                    successes += 1
                row["duration_s"] = time.monotonic() - start
                report.write(json.dumps(row) + "\n")
                report.flush()
                print(json.dumps(row), flush=True)
    finally:
        if coordinator is not None:
            coordinator.stop()
    print(json.dumps({"successes": successes, "episodes": args.episodes}), flush=True)
    return 0 if successes / args.episodes >= 0.7 else 1


if __name__ == "__main__":
    raise SystemExit(main())
