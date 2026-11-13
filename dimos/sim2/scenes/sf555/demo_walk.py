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

"""Run the complete G1 office blueprint, walk, and save actual camera evidence."""

from collections import deque
from contextlib import ExitStack
import json
import time
import uuid

from PIL import Image as PillowImage

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_sim2_sf555 import (
    ASSETS,
    unitree_g1_sim2_sf555,
)
from dimos.robot.unitree.g1.sim2_policy import G1PolicyModule
from dimos.sim2.geometry.poses import poses_by_id
from dimos.sim2.module import SimModule
from dimos.sim2.sensors.camera.module import SimCameraModule
from dimos.sim2.sensors.lidar.module import SimLidarModule
from dimos.utils.testing.waiting import wait_until


def main() -> None:
    blueprint = unitree_g1_sim2_sf555.global_config(
        zenoh_domain="sf555-validation-" + uuid.uuid4().hex
    )
    started = time.monotonic()
    deployment = ModuleCoordinator.build(
        blueprint, parsed_config=BlueprintConfigParser(blueprint).parse()
    )
    with ExitStack() as cleanup:
        cleanup.callback(deployment.stop)
        sim = deployment.get_instance(SimModule)
        policy = deployment.get_instance(G1PolicyModule)
        camera = deployment.get_instance(SimCameraModule)
        lidar = deployment.get_instance(SimLidarModule)
        images: deque[Image] = deque(maxlen=10)
        transport = deployment.transports[("color_image", Image)]
        cleanup.callback(transport.stop)
        cleanup.callback(transport.subscribe(images.append))

        def ready() -> bool:
            assert not sim.status()["error"], sim.status()
            assert not camera.statistics()["error"], camera.statistics()
            assert not lidar.statistics()["error"], lidar.statistics()
            return (
                sim.status()["time_ns"] > 2_000_000_000
                and len(images) >= 3
                and lidar.statistics()["acquisitions"] >= 1
            )

        wait_until(ready, timeout=600)
        before = sim.inspect()
        before_pose = poses_by_id(before)["g1/pelvis"].position
        assert before_pose.z > 0.68, before_pose
        before_captures = camera.statistics()["acquisitions"]
        walk_start = time.monotonic()
        policy.move(Twist(linear=Vector3(0.2, 0, 0)), duration=120)
        wait_until(
            lambda: sim.status()["time_ns"] > before.header.time_ns + 4_000_000_000, timeout=120
        )
        after = sim.inspect()
        policy.move(Twist(), duration=1)
        after_pose = poses_by_id(after)["g1/pelvis"].position
        displacement = after_pose.x - before_pose.x
        assert after_pose.z > 0.68 and displacement > 0.3, (before_pose, after_pose)
        destination = ASSETS / "validation"
        destination.mkdir(exist_ok=True)
        image = images[-1]
        assert image.ts >= before.header.time_ns / 1e9
        assert (
            camera.statistics()["error"]
            == lidar.statistics()["error"]
            == sim.status()["error"]
            == ""
        )
        PillowImage.fromarray(image.as_numpy()).save(destination / "walking-camera.png")
        report = {
            "displacement_x_m": displacement,
            "pelvis_height_m": after_pose.z,
            "simulated_walk_s": (after.header.time_ns - before.header.time_ns) / 1e9,
            "wall_walk_s": time.monotonic() - walk_start,
            "total_wall_s": time.monotonic() - started,
            "camera_frames_during_walk": camera.statistics()["acquisitions"] - before_captures,
            "camera": camera.statistics(),
            "lidar": lidar.statistics(),
            "physics": sim.status(),
            "policy": policy.status(),
            "passed": True,
        }
        (destination / "walking.json").write_text(json.dumps(report, indent=2))
        print(report, flush=True)


if __name__ == "__main__":
    main()
