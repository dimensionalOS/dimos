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

"""Run an isolated browser-controlled G1 and a separate latest-state observer.

The browser never supplies joint targets or teleports. Its bounded Twist goes
through the selected robot policy. Reset rebuilds the deployment and policy history.
Observer images have no sensor publication role; Sim2 sensors keep their clocks.
"""

import argparse
from contextlib import ExitStack
from functools import partial
import json
import math
from pathlib import Path
import signal
import threading
import time
from types import FrameType
from typing import Any
import uuid

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.experimental.gta_sim2.blueprint import gta_g1_blueprint
from dimos.experimental.gta_sim2.m20 import M20PolicyModule, m20_blueprint
from dimos.experimental.gta_sim2.prepare import ground_height
from dimos.experimental.gta_sim2.prepare_m20 import BASE as M20_BASE
from dimos.experimental.gta_sim2.renderer import ObserverOutput, SixNGRenderer
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.unitree.g1.sim2_policy import G1PolicyModule
from dimos.robot.unitree.g1.sim2_scene import PELVIS, TORSO
from dimos.sim2.description import compose_scene
from dimos.sim2.geometry.poses import poses_by_id
from dimos.sim2.module import SimModule
from dimos.sim2.sensors.camera.config import CameraConfig
from dimos.utils.testing.waiting import wait_until

CONTROL_TIMEOUT = 0.35


def read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text())
        return value if isinstance(value, dict) else {}
    except (OSError, ValueError):
        return {}


def write_json(path: Path, value: dict[str, Any]) -> None:
    pending = path.with_suffix(".tmp")
    pending.write_text(json.dumps(value))
    pending.replace(path)


def bounded_velocity(
    command: dict[str, Any],
    now: float,
    xy: NDArray[np.float64],
    yaw: float,
    lo: NDArray[np.float64],
    hi: NDArray[np.float64],
    speed: tuple[float, float, float] = (0.2, 0.15, 0.4),
) -> tuple[float, float, float]:
    """Expire browser input and remove motion directed out of the contact patch."""
    values = [command.get(k, 0.0) for k in ("issued", "forward", "right", "turn")]
    if any(
        isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x)
        for x in values
    ):
        return 0.0, 0.0, 0.0
    stamp, forward, right, turn = values
    if not 0 <= now - stamp <= CONTROL_TIMEOUT:
        return 0.0, 0.0, 0.0
    vx, vy, wz = (
        float(np.clip(forward, -1, 1)) * speed[0],
        -float(np.clip(right, -1, 1)) * speed[1],
        -float(np.clip(turn, -1, 1)) * speed[2],
    )
    rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    world = rotation @ [vx, vy]
    for axis in range(2):
        if (xy[axis] <= lo[axis] and world[axis] < 0) or (xy[axis] >= hi[axis] and world[axis] > 0):
            world[axis] = 0
    body = rotation.T @ world
    return float(body[0]), float(body[1]), wz


def physics_ready(sim: Any) -> bool:
    return bool(sim.status()["time_ns"] >= 2_000_000_000)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--runtime", type=Path, required=True)
    parser.add_argument("--executable", type=Path, required=True)
    parser.add_argument("--game", type=Path, required=True)
    parser.add_argument("--work", type=Path, required=True)
    parser.add_argument("--camera-port", type=int, required=True)
    parser.add_argument("--gpu")
    parser.add_argument("--robot", choices=("g1", "m20"), default="g1")
    parser.add_argument("--policy", type=Path)
    parser.add_argument("--policy-profile", choices=("sdk57", "dreamwaq342"), default="sdk57")
    args = parser.parse_args()
    if args.robot == "m20" and args.policy is None:
        parser.error("--robot m20 requires the explicit --policy checkpoint")
    is_m20 = args.robot == "m20"
    focus_body = M20_BASE if is_m20 else PELVIS
    camera_frame = (M20_BASE if is_m20 else TORSO) + "/camera_mount/camera/camera"
    speed = (1.5, 0.3, 0.7) if is_m20 else (0.2, 0.15, 0.4)
    args.runtime.mkdir(parents=True, exist_ok=True)
    manifest = read_json(args.scene.with_name("manifest.json"))
    origin = np.asarray(manifest["origin_rage_m"], dtype=np.float64)
    bounds = np.asarray(manifest["bounds_rage_m"], dtype=np.float64) - origin
    lo, hi = bounds[0, :2] + 2, bounds[1, :2] - 2
    surfaces = np.load(args.scene.with_name("surfaces.npz"))["triangles"]
    done = threading.Event()

    def shutdown(signum: int, frame: FrameType | None) -> None:
        done.set()

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    observer = SixNGRenderer(
        args.executable,
        args.game,
        args.work,
        args.runtime / "renderer",
        tuple(origin),
        gpu=args.gpu,
        observer=ObserverOutput(args.runtime, args.camera_port),
    )
    deployment: ModuleCoordinator | None = None
    last_action = read_json(args.runtime / "action.json").get("id")
    last_seen = time.monotonic()
    next_status = 0.0
    generation = 0
    observer_loaded = False
    state: dict[str, Any] = {
        "state": "idle",
        "message": "Open the viewer to start",
        "generation": generation,
    }
    write_json(args.runtime / "simulation.json", state)
    with ExitStack() as cleanup:
        cleanup.callback(observer.close)
        try:
            while not done.is_set():
                tick = time.monotonic()
                try:
                    visible = time.time() - (args.runtime / "viewer").stat().st_mtime < 4
                except OSError:
                    visible = False
                if visible:
                    last_seen = tick
                action = read_json(args.runtime / "action.json")
                reset = action.get("id") != last_action and action.get("action") == "reset"
                last_action = action.get("id")
                if deployment is not None and (reset or tick - last_seen > 60):
                    write_json(
                        args.runtime / "simulation.json",
                        {
                            "state": "resetting" if reset else "idle",
                            "message": "Resetting robot…" if reset else "Waiting for a viewer",
                            "generation": generation,
                        },
                    )
                    deployment.stop()
                    deployment = None
                    if not reset:
                        observer.close()
                        observer_loaded = False
                if deployment is None:
                    if not visible:
                        done.wait(0.1)
                        continue
                    write_json(
                        args.runtime / "simulation.json",
                        {
                            "state": "starting",
                            "message": "Loading physics and walking policy…",
                            "generation": generation,
                        },
                    )
                    blueprint = (
                        (
                            m20_blueprint(args.scene, args.policy, args.policy_profile)
                            if is_m20
                            else gta_g1_blueprint(args.scene)
                        )
                        .global_config(zenoh_domain="gta-interactive-" + uuid.uuid4().hex)
                        .solidify()
                    )
                    deployment = ModuleCoordinator.build(blueprint)
                    sim = deployment.get_instance(SimModule)
                    policy = deployment.get_instance(M20PolicyModule if is_m20 else G1PolicyModule)
                    wait_until(partial(physics_ready, sim), timeout=40)
                    if not observer_loaded:
                        scene = compose_scene(
                            [
                                deployment.get_instance(atom.name)
                                for atom in blueprint.active_blueprints
                            ]
                        )
                        camera = CameraConfig(
                            camera_frame,
                            width=960,
                            height=540,
                            fx=570,
                            fy=570,
                            cx=480,
                            cy=270,
                        )
                        observer.load(scene, (camera,))
                        observer_loaded = True
                    generation += 1
                    if not is_m20:
                        policy.arm_joint("g1/left_elbow_joint", 1.1)
                    state = {
                        "state": "running",
                        "message": "Ready",
                        "generation": generation,
                        "robot": args.robot,
                        "speed_limits": speed,
                    }
                    # Never replay input received during a long reset/load.
                    accept_after = time.monotonic()
                sample = sim.inspect()
                pose = poses_by_id(sample)[focus_body]
                p, q = pose.position, pose.orientation
                xyz = np.array([p.x, p.y, p.z])
                yaw = float(Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler("xyz")[2])
                command = read_json(args.runtime / "control.json")
                if command.get("issued", 0) < accept_after:
                    command = {}
                vx, vy, wz = bounded_velocity(
                    command, time.monotonic(), xyz[:2], yaw, lo, hi, speed
                )
                try:
                    clearance = p.z - ground_height(surfaces, (p.x, p.y), p.z - 0.3)
                except ValueError:
                    clearance = 0.0
                upright = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()[2, 2]
                fallen = clearance < (0.23 if is_m20 else 0.5) or p.z < -0.2 or upright < 0.3
                if fallen or not visible:
                    vx = vy = wz = 0.0
                policy.move(
                    Twist(linear=Vector3(vx, vy, 0), angular=Vector3(0, 0, wz)),
                    duration=CONTROL_TIMEOUT,
                )
                observer.publish_observer(sample, focus_body)
                if tick >= next_status:
                    state.update(
                        state="fallen" if fallen else "running",
                        message="Robot fell — use Reset" if fallen else "Ready",
                        position=xyz.tolist(),
                        yaw=yaw,
                        command=[vx, vy, wz],
                        clearance_m=clearance,
                        patch_limit_m=[lo.tolist(), hi.tolist()],
                        near_boundary=bool(
                            np.any(xyz[:2] < lo + 0.5) or np.any(xyz[:2] > hi - 0.5)
                        ),
                        sim=sim.status(),
                        policy=policy.status(),
                        updated_unix=time.time(),
                    )
                    write_json(args.runtime / "simulation.json", state)
                    next_status = tick + 0.25
                done.wait(max(0, 1 / 30 - (time.monotonic() - tick)))
        except Exception as error:
            write_json(
                args.runtime / "simulation.json",
                {"state": "error", "message": str(error), "generation": generation},
            )
            raise
        finally:
            if deployment is not None:
                deployment.stop()


if __name__ == "__main__":
    main()
