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

"""Small sensor-only reference controller for the sim2 mobile cases; not a learned policy."""

from __future__ import annotations

from contextlib import ExitStack
from functools import partial
import json
import math
from pathlib import Path
import re
from threading import Event, RLock
import time
from typing import Any

import numpy as np

from dimos.core.introspection.module.info import StreamInfo
from dimos.evals.agents.base import Agent
from dimos.evals.agents.lib.trajectory_builder import TrajectoryBuilder
from dimos.evals.types import RunningEnvironment, Trajectory
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.porcelain.dimos import Dimos


def public_output(streams: list[StreamInfo], name: str) -> StreamInfo:
    outputs = [s for s in streams if s.name == name and s.direction == "out"]
    channels = {s.channel for s in outputs}
    if len(channels) != 1 or None in channels:
        raise ValueError(f"expected one public {name} channel")
    return outputs[0]


def green_box(color: Image, depth: Image, info: CameraInfo, transform: Transform) -> Vector3 | None:
    """Colour segmentation plus measured depth. No scene names, geometry or simulator state."""
    if len({color.frame_id, depth.frame_id, info.frame_id, transform.child_frame_id}) != 1:
        return None
    stamps = [color.ts, depth.ts, info.ts, transform.ts]
    if max(stamps) - min(stamps) > 0.03 or time.time() - min(stamps) > 1:
        return None
    rgb = color.data.astype(np.float32)
    mask = (
        (rgb[:, :, 1] > 60)
        & (rgb[:, :, 1] > 1.6 * rgb[:, :, 0])
        & (rgb[:, :, 1] > 1.6 * rgb[:, :, 2])
    )
    ys, xs = np.nonzero(mask & np.isfinite(depth.data) & (depth.data > 0) & (depth.data < 8))
    if len(xs) < 30:
        return None
    u, v = float(np.median(xs)), float(np.median(ys))
    z = float(np.median(depth.data[ys, xs]))
    point = Vector3((u - info.K[2]) * z / info.K[0], (v - info.K[5]) * z / info.K[4], z)
    return transform.translation + transform.rotation.rotate_vector(point)


class MobileScripted(Agent):
    """Uses ordinary navigation RPCs, locomotion and camera/odometry streams only.

    Handles the suite's coordinate-goal prompt and its explicitly green-box
    search prompt. This limited colour baseline is not general object recognition.
    """

    def run(
        self, inputs: str, env: RunningEnvironment, run_dir: Path, *, timeout_s: float
    ) -> Trajectory:
        trace = TrajectoryBuilder(inputs, name=type(self).__name__)
        request, response = run_dir / "script_request.json", run_dir / "script_response.json"
        request.write_text(json.dumps({"inputs": inputs, "timeout_s": timeout_s}))
        calls: list[dict[str, Any]] = []
        ended, message = "answer", "Completed requested movement."
        started = time.monotonic()
        try:
            self._drive(inputs, started + timeout_s, calls)
        except TimeoutError as error:
            ended, message = "timeout", str(error)
        response.write_text(json.dumps({"calls": calls, "result": message}, indent=2))
        trace.step(
            message=message,
            request=request,
            response=response,
            latency_s=time.monotonic() - started,
        )
        return trace.build("timeout" if ended == "timeout" else "answer")

    def _drive(self, inputs: str, deadline: float, calls: list[dict[str, Any]]) -> None:
        latest: dict[str, Any] = {}
        lock, changed = RLock(), Event()

        def receive(name: str, value: Any) -> None:
            with lock:
                latest[name] = value
            changed.set()

        def receive_tf(value: TFMessage) -> None:
            for transform in value.transforms:
                if transform.frame_id == "world":
                    receive(f"tf/{transform.child_frame_id}", transform)

        with ExitStack() as resources:
            app = Dimos.connect()
            resources.callback(app.stop)
            planner: Any = app.get_module("ReplanningAStarPlanner")
            robot_names = [
                m.instance_name or m.class_name
                for m in app.list_modules()
                if m.class_name in {"G1Connection", "GO2Connection"}
            ]
            if len(robot_names) != 1:
                raise ValueError("mobile reference controller needs exactly one G1 or Go2")
            robot: Any = app.get_module(robot_names[0])
            resources.callback(robot.move, Twist())
            resources.callback(planner.cancel_goal)
            streams = app.list_streams()
            for name in ("odom", "color_image", "depth_image", "camera_info"):
                owner = public_output(streams, name).module_name
                assert owner is not None
                module = app.get_module(owner)
                resources.callback(module.get_stream(name).subscribe(partial(receive, name)))
                if name == "color_image":
                    resources.callback(module.get_stream("tf").subscribe(receive_tf))

            def observations() -> dict[str, Any]:
                if time.monotonic() >= deadline:
                    raise TimeoutError("mobile reference controller exhausted the case budget")
                changed.wait(0.1)
                changed.clear()
                with lock:
                    data = dict(latest)
                pose = data.get("odom")
                if pose is not None and time.time() - pose.ts > 1:
                    raise RuntimeError("odometry became stale")
                return data

            def navigate(x: float, y: float) -> None:
                goal = PoseStamped(position=Vector3(x, y, 0), frame_id="world")
                if not planner.set_goal(goal):
                    raise RuntimeError("navigation rejected the goal")
                calls.append({"rpc": "set_goal", "xy": [x, y], "ts": time.time()})
                since: float | None = None
                while True:
                    data = observations()
                    pose = data.get("odom")
                    near = (
                        pose is not None
                        and math.hypot(pose.position.x - x, pose.position.y - y) < 0.3
                    )
                    since = (time.monotonic() if since is None else since) if near else None
                    if since is not None and time.monotonic() - since >= 2:
                        planner.cancel_goal()
                        return

            goal_match = re.search(r"world goal \(([-\d.]+), ([-\d.]+)\)", inputs)
            if goal_match:
                navigate(float(goal_match[1]), float(goal_match[2]))
                return
            if not inputs.startswith("Find the green box"):
                raise ValueError("unsupported instruction for the mobile reference controller")
            previous: Vector3 | None = None
            for _ in range(2):
                while True:
                    data = observations()
                    target = None
                    if all(
                        k in data for k in ("color_image", "depth_image", "camera_info", "odom")
                    ):
                        transform = data.get(f"tf/{data['camera_info'].frame_id}")
                        if transform is not None:
                            target = green_box(
                                data["color_image"],
                                data["depth_image"],
                                data["camera_info"],
                                transform,
                            )
                    if target is not None and (
                        previous is None
                        or math.hypot(target.x - previous.x, target.y - previous.y) > 0.8
                    ):
                        robot.move(Twist())
                        pose = data["odom"].position
                        dx, dy = target.x - pose.x, target.y - pose.y
                        distance = math.hypot(dx, dy)
                        if distance < 0.1:
                            raise RuntimeError("camera target overlaps the robot")
                        calls.append(
                            {
                                "observation": "green box",
                                "xyz": target.to_tuple(),
                                "ts": data["color_image"].ts,
                            }
                        )
                        navigate(target.x - 0.95 * dx / distance, target.y - 0.95 * dy / distance)
                        previous = target
                        break
                    robot.move(Twist(angular=Vector3(0, 0, 0.45)))
