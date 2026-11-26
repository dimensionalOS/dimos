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

"""Run G1 through Sim2 on a bounded map patch, optionally following a nav route."""

import argparse
from collections import deque
from collections.abc import Callable
from contextlib import ExitStack
from functools import partial
import json
import math
from pathlib import Path
import struct
import threading
import time
from typing import Any
import uuid

import numpy as np
from PIL import Image as PILImage
from scipy.spatial.transform import Rotation

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.experimental.gta_sim2.blueprint import gta_g1_blueprint
from dimos.experimental.gta_sim2.prepare import ground_height
from dimos.experimental.gta_sim2.renderer import SixNGRenderer
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.unitree.g1.sim2_policy import G1PolicyModule
from dimos.robot.unitree.g1.sim2_scene import PELVIS
from dimos.sim2.geometry.poses import poses_by_id
from dimos.sim2.module import SimModule
from dimos.sim2.sensors.camera.config import CameraRenderer
from dimos.sim2.sensors.camera.module import SimCameraModule
from dimos.sim2.sensors.camera.renderers.mujoco import MujocoRenderer
from dimos.sim2.sensors.lidar.module import SimLidarModule
from dimos.utils.testing.waiting import wait_until


class Captures:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.images: deque[Image] = deque(maxlen=600)
        self.depths: deque[Image] = deque(maxlen=2)

    def color(self, image: Image) -> None:
        with self.lock:
            self.images.append(image)

    def depth(self, image: Image) -> None:
        with self.lock:
            self.depths.append(image)

    def save(self, output: Path) -> dict[str, Any]:
        with self.lock:
            frames = list(self.images)
            depths = list(self.depths)
        if frames:
            PILImage.fromarray(frames[-1].as_numpy()).save(output.with_suffix(".png"))
        if depths:
            np.save(output.with_suffix(".depth.npy"), depths[-1].as_numpy())
        # Preserve acquired timestamps, not the consumer's wall-clock receipt time.
        return {
            "color_frames_retained": len(frames),
            "color_timestamps": [m.ts for m in frames],
            "depth_timestamps": [m.ts for m in depths],
        }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seconds", type=float, default=8)
    parser.add_argument("--route", type=Path)
    parser.add_argument("--renderer-executable", type=Path)
    parser.add_argument("--game", type=Path)
    parser.add_argument("--work", type=Path)
    parser.add_argument("--gpu", help="Optional SixNG adapter-name filter")
    args = parser.parse_args()
    manifest = json.loads(args.scene.with_name("manifest.json").read_text())
    origin = np.asarray(manifest["origin_rage_m"])
    renderer: Callable[[], CameraRenderer] = MujocoRenderer
    if args.renderer_executable:
        if args.game is None or args.work is None:
            parser.error("--renderer-executable requires --game and --work")
        renderer = partial(
            SixNGRenderer,
            executable=args.renderer_executable,
            game=args.game,
            work=args.work,
            runtime=args.output.parent / "renderer",
            origin=tuple(origin),
            gpu=args.gpu,
        )
    blueprint = (
        gta_g1_blueprint(args.scene, renderer)
        .global_config(zenoh_domain="gta-sim2-" + uuid.uuid4().hex)
        .solidify()
    )
    route = (
        np.asarray(json.loads(args.route.read_text())["waypoints_rage_m"]) - origin
        if args.route
        else None
    )
    report: dict[str, Any] = {
        "scene": str(args.scene),
        "trajectory": [],
        "renderer": "SixNG" if args.renderer_executable else "MuJoCo",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    captures = Captures()
    surfaces = np.load(args.scene.with_name("surfaces.npz"))["triangles"]
    with ExitStack() as cleanup:
        deployment = ModuleCoordinator.build(blueprint)
        cleanup.callback(deployment.stop)
        for (name, kind), transport in deployment.transports.items():
            if kind is Image and name.endswith(("color_image", "depth_image")):
                cleanup.callback(transport.stop)
                cleanup.callback(
                    transport.subscribe(
                        captures.color if name.endswith("color_image") else captures.depth
                    )
                )
        sim = deployment.get_instance(SimModule)
        policy = deployment.get_instance(G1PolicyModule)
        camera = deployment.get_instance(SimCameraModule)
        lidar = deployment.get_instance(SimLidarModule)

        def ready() -> bool:
            stats = camera.statistics()
            if stats["error"]:
                raise RuntimeError(stats["error"])
            return bool(sim.status()["time_ns"] >= 2_000_000_000 and stats["acquisitions"] >= 3)

        wait_until(ready, timeout=90)
        first = sim.inspect()
        start = poses_by_id(first)[PELVIS].position
        policy.arm_joint("g1/left_elbow_joint", 1.1)
        started = time.monotonic()
        target = 1
        reached = False
        trace = cleanup.enter_context(args.output.with_suffix(".worldstates").open("wb"))
        while time.monotonic() - started < max(120, args.seconds * 3):
            tick = time.monotonic()
            sample = sim.inspect()
            pose = poses_by_id(sample)[PELVIS]
            xyz = np.array([pose.position.x, pose.position.y, pose.position.z])
            q = pose.orientation
            yaw = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler("xyz")[2]
            encoded = sample.lcm_encode()
            trace.write(struct.pack("<I", len(encoded)))
            trace.write(encoded)
            ground = ground_height(surfaces, (float(xyz[0]), float(xyz[1])), float(xyz[2] - 0.3))
            report["trajectory"].append(
                {
                    "t": sample.header.time_ns / 1e9,
                    "xyz": xyz.tolist(),
                    "yaw": float(yaw),
                    "pelvis_clearance_m": float(xyz[2] - ground),
                }
            )
            if route is not None:
                delta = route[target, :2] - xyz[:2]
                if np.linalg.norm(delta) < 0.25:
                    if target == len(route) - 1:
                        reached = True
                        break
                    target += 1
                    delta = route[target, :2] - xyz[:2]
                heading = math.atan2(delta[1], delta[0])
                error = math.atan2(math.sin(heading - yaw), math.cos(heading - yaw))
                velocity = 0.2 if abs(error) < 0.5 else 0.0
                policy.move(
                    Twist(
                        linear=Vector3(velocity, 0, 0),
                        angular=Vector3(0, 0, float(np.clip(error, -0.5, 0.5))),
                    ),
                    duration=1,
                )
            else:
                policy.move(Twist(linear=Vector3(0.2, 0, 0)), duration=1)
            if sample.header.time_ns - first.header.time_ns >= args.seconds * 1e9:
                break
            time.sleep(max(0, 0.1 - (time.monotonic() - tick)))
        policy.move(Twist(), duration=1)
        report.update(
            sim=sim.status(),
            policy=policy.status(),
            camera=camera.statistics(),
            lidar=lidar.statistics(),
            captures=captures.save(args.output),
            route_reached=reached,
            wall_seconds=time.monotonic() - started,
            displacement_x_m=pose.position.x - start.x,
            displacement_xy_m=float(np.linalg.norm(xyz[:2] - [start.x, start.y])),
            minimum_pelvis_z_m=min(p["xyz"][2] for p in report["trajectory"]),
            minimum_pelvis_clearance_m=min(p["pelvis_clearance_m"] for p in report["trajectory"]),
            simulated_seconds=(sample.header.time_ns - first.header.time_ns) / 1e9,
        )
        args.output.write_text(json.dumps(report, indent=2, default=str) + "\n")
        summary = {k: v for k, v in report.items() if k not in ("trajectory", "captures")}
        assert reached if route is not None else report["displacement_x_m"] > 0.5, summary
        assert report["minimum_pelvis_clearance_m"] > 0.6, summary
        assert report["camera"]["acquisitions"] >= 3 and report["lidar"]["acquisitions"] >= 3, (
            summary
        )
        assert (
            not report["sim"]["error"]
            and not report["camera"]["error"]
            and not report["lidar"]["error"]
        ), summary
        print(json.dumps(summary, indent=2, default=str))


if __name__ == "__main__":
    main()
