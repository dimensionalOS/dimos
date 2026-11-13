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

"""Exercise the real JoyLo -> coordinator -> connection -> sim2 path without hardware.

Run after preparing the model: python -m dimos.robot.galaxea.r1pro.sim.demo_joylo
Uses a unique Zenoh domain, stops its workers, and never connects to vendor ROS.
"""

import json
import math
import time
from typing import Any
import uuid

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.robot.galaxea.r1pro.blueprints.teleop.r1pro_joylo import _robot_blueprint
from dimos.robot.galaxea.r1pro.blueprints.teleop.r1pro_joylo_inputs import _transports
from dimos.robot.galaxea.r1pro.joylo import ARMS
from dimos.robot.galaxea.r1pro.joylo_module import R1ProJoylo
from dimos.robot.galaxea.r1pro.sim.config import SCENE
from dimos.robot.galaxea.r1pro.sim.connection import R1ProSimConnection
from dimos.robot.galaxea.r1pro.sim.joylo_input import JoyloSimInput
from dimos.sim2.module import SimModule


def main() -> None:
    # r1pro-openarm-joylo-sim2's robot on the plain scene, with scripted leaders
    # and Joy-Cons in place of the devices, the cameras and the recorder.
    blueprint = (
        autoconnect(_robot_blueprint(SCENE), JoyloSimInput.blueprint())
        .transports(_transports)
        .global_config(
            viewer="none",
            transport="zenoh",
            n_workers=6,
            zenoh_domain="joylo-check-" + uuid.uuid4().hex,
            serve_coordinator_rpc=False,
        )
    )
    run = ModuleCoordinator.build(blueprint)
    try:
        sim = run.get_instance(SimModule)
        inputs = run.get_instance(JoyloSimInput)
        profile = run.get_instance(R1ProJoylo)
        connection = run.get_instance(R1ProSimConnection)

        def send(
            seconds: float,
            joints: list[float],
            left: list[str] | None = None,
            right: list[str] | None = None,
            axes: list[float] | None = None,
        ) -> None:
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                inputs.send(joints, left or [], right or [], axes or [0.0] * 4)
                time.sleep(0.02)

        def positions() -> dict[str, float]:
            truth = sim.inspect().joint_truth
            return dict(zip(truth.name, truth.position, strict=True))

        neutral = [0.0] * len(ARMS)
        send(1.0, neutral)
        start = positions()
        assert not profile.status()["active"], profile.status()
        send(0.15, neutral, right=["X"])
        assert profile.status()["active"], profile.status()
        target = neutral.copy()
        # Joint 4 flexion stays away from the torso in the model's zero pose.
        target[3], target[10] = 0.18, 0.15
        send(2.0, target, left=["ZL", "down"], right=["ZR"])
        moved = positions()
        left_change = moved[ARMS[3]] - start[ARMS[3]]
        right_change = moved[ARMS[10]] - start[ARMS[10]]
        assert left_change > 0.08 and right_change > 0.06, (left_change, right_change)
        apertures = {
            side: moved[f"r1pro/{side}_gripper_finger_joint1"]
            - moved[f"r1pro/{side}_gripper_finger_joint2"]
            for side in ("left", "right")
        }
        assert min(apertures.values()) > 0.06, apertures

        def height(q: dict[str, float]) -> float:
            a, b = q["r1pro/torso_joint1"], q["r1pro/torso_joint2"]
            return 0.4 * math.cos(a) + 0.3 * math.cos(a + b)

        lowered = height(start) - height(moved)
        assert lowered > 0.005, (
            height(start),
            height(moved),
            {k: v for k, v in moved.items() if "torso" in k},
            sim.status(),
            profile.status(),
        )

        # Moving the leader while paused must not establish a new zero on enable.
        resume_errors = []
        for elbows in ((0.35, 0.32), (0.18, 0.15)):
            send(0.15, target, right=["home"])
            assert not profile.status()["active"], profile.status()
            held = positions()
            target[3], target[10] = elbows
            send(0.3, target)
            paused = positions()
            assert max(abs(paused[n] - held[n]) for n in ARMS) < 0.02, (held, paused)
            send(0.15, target, right=["X"])
            send(2.0, target)
            assert profile.status()["active"], profile.status()
            resumed = positions()
            error = max(abs(resumed[n] - q) for n, q in zip(ARMS, target, strict=True))
            assert error < 0.02, (error, resumed, target)
            resume_errors.append(error)

        def base_pose() -> list[float]:
            body = next(b for b in sim.inspect().bodies if b.body_id == "r1pro/base_link")
            return [body.pose.position.x, body.pose.position.y, body.pose.position.z]

        before = base_pose()
        send(2.0, target, axes=[0.0, -0.6, 0.0, 0.0])
        after = base_pose()
        distance = math.dist(before[:2], after[:2])
        assert distance > 0.03, (before, after)
        # Stop the input stream. The receiver must latch a pause, including the base.
        deadline = time.monotonic() + 3
        while profile.status()["active"] and time.monotonic() < deadline:
            time.sleep(0.02)
        assert not profile.status()["active"], profile.status()
        send(0.5, target)  # Fresh, released input must not re-enable following.
        assert not profile.status()["active"], profile.status()
        stopped = base_pose()
        send(0.5, target)
        drift = math.dist(stopped[:2], base_pose()[:2])
        assert drift < 0.01, drift
        status = sim.status()
        assert not status["error"] and not status["publication_error"], status
        assert not connection.status().last_error, connection.status()
        result: dict[str, Any] = {
            "left_arm_delta_rad": left_change,
            "right_arm_delta_rad": right_change,
            "gripper_aperture_m": apertures,
            "base_distance_m": distance,
            "torso_lowered_m": lowered,
            "paused_base_drift_m": drift,
            "resume_max_joint_error_rad": resume_errors,
            "timeout_latched": not profile.status()["active"],
            "sim": status,
        }
        print(json.dumps(result, indent=2))
    finally:
        run.stop()


if __name__ == "__main__":
    main()
