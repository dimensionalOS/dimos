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

"""Run the existing G1/GR00T/Point-LIO application with a native Unreal camera.

Example:
    python -m dimos.sim2.unreal.demo_g1 --executable /path/to/DimosRenderer --walk --reset
"""

from __future__ import annotations

import argparse
from collections import OrderedDict, deque
from contextlib import ExitStack
from dataclasses import asdict
from functools import partial
from importlib import import_module
import json
import os
from pathlib import Path
import signal
import threading
import time
from types import FrameType
from typing import Any, cast
from uuid import uuid4

import numpy as np
from PIL import Image as PillowImage
import rerun.blueprint as rrb
from scipy.spatial.transform import Rotation

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.global_config import global_config
from dimos.core.transport import PubSubTransport
from dimos.core.transport_factory import make_transport
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.sim2.messages import FrameMetadata
from dimos.sim2.module import SimulationModule
from dimos.sim2.sensors.camera.spec import Camera
from dimos.sim2.sensors.spec import Mount
from dimos.sim2.unreal.module import UnrealCameraModule
from dimos.visualization.vis_module import vis_module


def live_layout() -> rrb.Blueprint:
    """Show the actual Unreal sensor images beside the retained G1/LIO view."""
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(origin="world", name="G1 / Point-LIO"),
            rrb.Vertical(
                rrb.Spatial2DView(origin="world/color_image", name="Unreal RGB"),
                rrb.Spatial2DView(origin="world/depth_image", name="Unreal depth (metres)"),
            ),
            column_shares=[1, 1],
        ),
        rrb.TimePanel(state="collapsed"),
        collapse_panels=True,
    )


def observer_camera() -> Camera:
    """A world camera aimed at the G1; the robot itself is defined by its usual stack."""
    position = np.array([2.4, -2.4, 1.8])
    forward = np.array([0.0, 0.0, 0.75]) - position
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, [0, 0, 1])
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    rpy = Rotation.from_matrix(np.column_stack((right, up, -forward))).as_euler("xyz")
    return Camera(
        "unreal",
        Mount("world", xyz=tuple(position), rpy=tuple(rpy)),
        width=640,
        height=480,
        rate_hz=10,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--executable", required=True, type=Path)
    parser.add_argument("--project", type=Path, help="Only needed when using UnrealEditor -game")
    parser.add_argument("--output", type=Path, default=Path("/tmp/dimos-g1-unreal"))
    parser.add_argument(
        "--seconds", type=float, default=30, help="Run duration; 0 watches until Ctrl-C"
    )
    parser.add_argument("--viewer", choices=("none", "rerun"), default="none")
    parser.add_argument(
        "--viewer-open",
        choices=("web", "native"),
        default="web",
        help="Where to open --viewer rerun",
    )
    parser.add_argument("--camera", choices=("observer", "front"), default="observer")
    parser.add_argument("--walk", action="store_true", help="Walk forward briefly in simulation")
    parser.add_argument("--reset", action="store_true", help="Reset world and controller halfway")
    args = parser.parse_args()
    if args.seconds != 0 and args.seconds < 10:
        parser.error("--seconds must be 0 (until Ctrl-C) or at least 10")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)

    # Isolate the demo from other stacks before any worker or transport starts.
    domain = f"unreal-g1-{uuid4().hex[:10]}"
    bucket = int(uuid4().hex[:4], 16) % 5000
    os.environ.update(
        SIMULATION="mujoco",
        VIEWER="none",
        ZENOH_DOMAIN=domain,
        ZENOH_SCOUT_ADDR=f"224.0.0.224:{37000 + bucket}",
        LCM_DEFAULT_URL=f"udpm://239.255.76.67:{27000 + bucket}?ttl=0",
    )
    global_config.simulation = "mujoco"
    global_config.viewer = "none"
    global_config.transport = "zenoh"
    global_config.zenoh_domain = domain
    global_config.zenoh_scout_addr = os.environ["ZENOH_SCOUT_ADDR"]
    # This blueprint selects its operating profile at import, after the flags above.
    g1 = import_module("dimos.robot.unitree.g1.blueprints.basic.unitree_g1_groot_wbc")
    sensor = observer_camera() if args.camera == "observer" else Camera("unreal", "front_camera")
    blueprint = autoconnect(
        g1.unitree_g1_groot_wbc,
        UnrealCameraModule.blueprint(
            executable=args.executable.resolve(),
            project=args.project.resolve() if args.project else None,
            sensor=sensor,
            robot_id=None if args.camera == "observer" else "g1",
            log_path=output / "renderer.log",
            rpc_timeouts={"start": 210.0},
        ),
        vis_module(
            args.viewer,
            rerun_config={
                **g1._rerun_config,
                "blueprint": live_layout,
                "rerun_open": args.viewer_open,
                "memory_limit": "1GB",
            },
        ),
    )
    parsed = BlueprintConfigParser(blueprint).parse(
        [],
        environ={},
        global_overrides={
            "simulation": "mujoco",
            "viewer": args.viewer,
            "transport": "zenoh",
            "zenoh_domain": domain,
            "zenoh_scout_addr": global_config.zenoh_scout_addr,
        },
    )
    counts: dict[str, int] = {}
    recent: dict[str, Any] = {}
    channels: dict[str, OrderedDict[float, Any]] = {
        name: OrderedDict() for name in ("color_image", "depth_image", "camera_info")
    }
    first_episode: int | None = None
    last_episode: int | None = None
    epochs_monotonic = True
    lock = threading.Lock()
    frame_available = threading.Event()

    def receive(name: str, message: Any) -> None:
        nonlocal first_episode, last_episode, epochs_monotonic
        with lock:
            counts[name] = counts.get(name, 0) + 1
            recent[name] = message
            if name in channels:
                channel = channels[name]
                channel[message.ts] = message
                while len(channel) > 8:
                    channel.popitem(last=False)
            if name == "frame_metadata":
                if first_episode is None:
                    first_episode = message.episode_id
                if last_episode is not None and message.episode_id < last_episode:
                    epochs_monotonic = False
                last_episode = message.episode_id
            if set.intersection(*(set(channel) for channel in channels.values())):
                frame_available.set()

    with ExitStack() as resources:
        for name, kind in (
            ("color_image", Image),
            ("depth_image", Image),
            ("camera_info", CameraInfo),
            ("frame_metadata", FrameMetadata),
            ("odometry", Odometry),
            ("lidar_raw", PointCloud2),
            ("global_map", PointCloud2),
        ):
            transport = make_transport(name, kind)
            transport.start()
            resources.callback(transport.stop)
            resources.callback(transport.subscribe(partial(receive, name)))
        app = ModuleCoordinator.build(blueprint, parsed)
        resources.callback(app.stop)
        # Use the application's resolved wire, including its selected backend.
        velocity_commands = cast("PubSubTransport[Twist]", app.transports["cmd_vel", Twist])
        velocity_commands.start()
        resources.callback(velocity_commands.stop)
        world = app.get_instance(SimulationModule)
        camera = app.get_instance(UnrealCameraModule)
        coordinator = app.get_instance("ControlCoordinator")
        connection = app.get_instance(g1.G1WholeBodyConnection)
        lidar = app.get_instance("mid360")
        if not frame_available.wait(timeout=60):
            raise RuntimeError(f"No aligned RGB-D frame: {camera.renderer_status()}")
        print("First aligned RGB-D frame received; recording G1 run.", flush=True)
        stopped = threading.Event()

        def request_stop(_signal: int, _frame: FrameType | None) -> None:
            stopped.set()

        for stop_signal in (signal.SIGINT, signal.SIGTERM):
            previous_handler = signal.signal(stop_signal, request_stop)
            resources.callback(signal.signal, stop_signal, previous_handler)
        if args.viewer == "rerun":
            print(
                "Rerun shows Unreal RGB, depth and G1/Point-LIO. Press Ctrl-C to stop.", flush=True
            )
        start = time.monotonic()
        baseline = world.status()
        first_frame = camera.renderer_status()["frames"]
        first_tick = coordinator.get_tick_count()
        reset_done = False
        records: deque[dict[str, Any]] = deque(maxlen=300)
        preview: deque[PillowImage.Image] = deque(maxlen=60)
        displacement = 0.0
        next_record = 0.0
        reset_at = args.seconds / 2 if args.seconds else 15.0
        walk_until = min(8, args.seconds / 3) if args.seconds else 8.0
        while not stopped.is_set() and (
            not args.seconds or (time.monotonic() - start) < args.seconds
        ):
            elapsed = time.monotonic() - start
            velocity = 0.15 if args.walk and 3 < elapsed < walk_until else 0.0
            velocity_commands.publish(Twist(linear=[velocity, 0, 0]))
            if args.reset and not reset_done and elapsed > reset_at:
                world.set_paused(True)
                world.reset()
                result = coordinator.reset_runtime_state(reactivate=True)
                if not result or not all(result.values()):
                    raise RuntimeError(f"Controller reset failed: {result}")
                world.set_paused(False)
                reset_done = True
            if elapsed >= next_record:
                state, render_state = world.status(), camera.renderer_status()
                if state["error"] or render_state["error"]:
                    raise RuntimeError({"physics": state, "renderer": render_state})
                with lock:
                    common = set.intersection(*(set(channel) for channel in channels.values()))
                    stamp = max(common) if common else None
                    matched = (
                        {name: channel[stamp] for name, channel in channels.items()}
                        if stamp
                        else {}
                    )
                    count_snapshot = dict(counts)
                if matched:
                    color, depth, info = (matched[key] for key in channels)
                    rgb = PillowImage.fromarray(color.data)
                    rgb.save(output / "rgb.png")
                    np.save(output / "depth.npy", depth.data)
                    (output / "calibration.json").write_text(
                        json.dumps(
                            {
                                "timestamp": stamp,
                                "frame_id": info.frame_id,
                                "width": info.width,
                                "height": info.height,
                                "K": info.K,
                            },
                            indent=2,
                        )
                    )
                    preview.append(rgb.resize((480, 360)))
                record = {
                    "elapsed": elapsed,
                    "world": state,
                    "renderer": render_state,
                    "messages": count_snapshot,
                }
                displacement = max(
                    displacement,
                    float(
                        np.linalg.norm(
                            np.asarray(state["robots"]["g1"])[:2]
                            - np.asarray(baseline["robots"]["g1"])[:2]
                        )
                    ),
                )
                records.append(record)
                print(json.dumps(record), flush=True)
                next_record += 1
            stopped.wait(0.1)
        final = world.status()
        connection_state = connection.status()
        render_state = camera.renderer_status()
        lidar_state = lidar.sensor_status()
        duration = time.monotonic() - start
        with lock:
            final_counts = dict(counts)
            epoch_order_valid = epochs_monotonic
            reset_observed = first_episode != last_episode
        checks = {
            "physics_running": final["running"] and final["error"] is None,
            "robot_standing": final["robots"]["g1"][2] > 0.5,
            "commands_accepted": final["rejected_commands"] == {},
            "connection_healthy": connection_state.connected
            and not connection_state.deadman_fired
            and connection_state.last_error is None
            and connection_state.last_rejection is None,
            "lidar_healthy": lidar_state.get("error") is None
            and final_counts.get("lidar_raw", 0) > 0,
            "odometry_received": final_counts.get("odometry", 0) > 0,
            "rgbd_received": render_state["frames"] > 0 and render_state["error"] is None,
            "epochs_monotonic": epoch_order_valid,
            "reset_observed": not args.reset or reset_observed,
            "walk_observed": not args.walk or displacement > 0.15,
        }
        report = {
            "interrupted": stopped.is_set(),
            "checks": checks,
            "duration": duration,
            "physics_steps_per_second": (final["steps"] - baseline["steps"]) / duration,
            "controller_ticks_per_second": (coordinator.get_tick_count() - first_tick) / duration,
            "rgbd_frames_per_second": (render_state["frames"] - first_frame) / duration,
            "max_displacement_m": displacement,
            "final_connection": asdict(connection_state),
            "samples": list(records),
        }
        (output / "report.json").write_text(json.dumps(report, indent=2))
        if preview:
            preview[0].save(
                output / "preview.gif",
                save_all=True,
                append_images=list(preview)[1:],
                duration=1000,
                loop=0,
            )
        if not all(checks.values()) and not stopped.is_set():
            raise RuntimeError(f"G1 Unreal validation failed: {checks}")
        print(f"{'STOPPED' if stopped.is_set() else 'PASS'}: {output / 'report.json'}", flush=True)


if __name__ == "__main__":
    main()
