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

"""Measure the actual M20 policy through the composed Sim2 deployment."""

import argparse
from functools import partial
import json
from pathlib import Path
import struct
import time
from typing import BinaryIO
import uuid

import cbor2
import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics
from scipy.spatial.transform import Rotation

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.experimental.gta_sim2.demo_interactive import physics_ready
from dimos.experimental.gta_sim2.m20 import M20PolicyModule, m20_blueprint
from dimos.experimental.gta_sim2.prepare_m20 import BASE
from dimos.experimental.gta_sim2.world_streaming import streaming_backend
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.sim2.description import compose_scene
from dimos.sim2.geometry.poses import poses_by_id
from dimos.sim2.module import SimModule
from dimos.sim2.physics.mujoco import MujocoBackend
from dimos.utils.testing.waiting import wait_until


def fixture(scene: Path, output: Path, terrain: str, step_height: float = 0.1) -> Path:
    """Synthetic evaluation only; never replace the interactive map collision."""
    stage = Usd.Stage.Open(str(scene.resolve()))
    stage.RemovePrim("/world/gta_patch")
    # Evaluation fixtures run along +X regardless of the street spawn heading.
    body = UsdGeom.Xformable(stage.GetPrimAtPath("/" + BASE))
    transform = Gf.Matrix4d(1.0)
    transform.SetTranslate(body.GetLocalTransformation().ExtractTranslation())
    body.MakeMatrixXform().Set(transform)
    boxes: list[tuple[str, tuple[float, float, float], tuple[float, float, float]]] = [
        ("ground", (0, 0, -0.05), (500, 100, 0.1))
    ]
    if terrain == "stairs":
        for i in range(5):
            left = 1.5 + 0.35 * i
            length = 0.35 if i < 4 else 4.0
            height = step_height * (i + 1)
            boxes.append((f"step_{i}", (left + length / 2, 0, height / 2), (length, 6, height)))
    for name, center, size in boxes:
        cube = UsdGeom.Cube.Define(stage, "/world/evaluation/" + name)
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*center))
        cube.AddScaleOp().Set(Gf.Vec3f(*size))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    result = output.with_suffix(".usda").resolve()
    stage.Export(str(result))
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--policy", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--seconds", type=float, default=4)
    parser.add_argument("--stop-after-s", type=float, help="Release input, then measure braking")
    parser.add_argument("--yaw-rate", type=float, default=0)
    parser.add_argument("--turn-after-s", type=float, default=0)
    parser.add_argument("--turning-forward-speed", type=float)
    parser.add_argument("--step-height", type=float, default=0.1)
    parser.add_argument("--stream-catalog", type=Path)
    parser.add_argument("--compiler-delay-s", type=float, default=0)
    parser.add_argument(
        "--reverse-after-s", type=float, help="Reverse the policy command for revisit evaluation"
    )
    parser.add_argument(
        "--record", action="store_true", help="Save composed USD and measured body poses"
    )
    parser.add_argument("--profile", choices=("sdk57", "dreamwaq342"), default="sdk57")
    parser.add_argument("--terrain", choices=("road", "flat", "stairs"), default="road")
    args = parser.parse_args()
    if not np.isfinite([args.speed, args.seconds, args.yaw_rate, args.turn_after_s]).all():
        parser.error("Evaluation commands and duration must be finite")
    if args.seconds <= 0 or abs(args.yaw_rate) > 0.7 or args.turn_after_s < 0:
        parser.error("Positive duration, nonnegative turn delay and |yaw-rate| <= .7 required")
    if args.stop_after_s is not None and not 0 < args.stop_after_s < args.seconds:
        parser.error("stop-after-s must be between zero and seconds")
    if not 0 < args.step_height <= 0.3:
        parser.error("step-height must be in (0, .3] metres")
    if args.stream_catalog and args.terrain != "road":
        parser.error("Streamed world evaluation requires road terrain")
    scene = args.scene.resolve()
    if args.terrain != "road":
        scene = fixture(scene, args.output, args.terrain, args.step_height)
    blueprint = (
        m20_blueprint(
            scene,
            args.policy.resolve(),
            args.profile,
            backend=(
                partial(
                    streaming_backend,
                    catalog=args.stream_catalog.resolve(),
                    focus_body=BASE,
                    metrics=args.output.with_suffix(".streaming.json"),
                    compiler_delay_s=args.compiler_delay_s,
                )
                if args.stream_catalog
                else MujocoBackend
            ),
            streaming=bool(args.stream_catalog and args.stream_catalog.suffix == ".json"),
            command_velocity_limits=(max(1.5, abs(args.speed)), 0.5, 0.7),
            turning_forward_speed=args.turning_forward_speed,
        )
        .global_config(zenoh_domain="m20-eval-" + uuid.uuid4().hex)
        .solidify()
    )
    deployment = ModuleCoordinator.build(blueprint)
    recording: BinaryIO | None = None
    try:
        if args.record:
            composed = compose_scene(
                [deployment.get_instance(atom.name) for atom in blueprint.active_blueprints]
            )
            args.output.with_suffix(".composed.usda").write_text(composed.usd)
            recording = args.output.with_suffix(".worldstates").open("wb")
        sim = deployment.get_instance(SimModule)
        policy = deployment.get_instance(M20PolicyModule)
        wait_until(partial(physics_ready, sim), timeout=40)
        start = time.monotonic()
        rows: list[list[float]] = []
        forward_axis = np.zeros(2)
        reached_goal_at: float | None = None
        residency_rows = []
        command_rows = []
        outcome = "timeout"
        while time.monotonic() - start < args.seconds:
            tick = time.monotonic()
            elapsed = tick - start
            direction = (
                -1 if args.reverse_after_s is not None and elapsed >= args.reverse_after_s else 1
            )
            stopped = reached_goal_at is not None or (
                args.stop_after_s is not None and elapsed >= args.stop_after_s
            )
            vx = 0.0 if stopped else args.speed * direction
            wz = args.yaw_rate if not stopped and elapsed >= args.turn_after_s else 0.0
            policy.move(Twist(linear=Vector3(vx, 0, 0), angular=Vector3(0, 0, wz)), duration=0.35)
            sample = sim.inspect()
            for observation in sample.cbor_observations:
                if observation.entity_id == "world" and observation.key == "gta/streaming":
                    residency_rows.append(cbor2.loads(bytes(observation.value)))
            if recording is not None:
                encoded = sample.lcm_encode()
                recording.write(struct.pack("<I", len(encoded)))
                recording.write(encoded)
            body = poses_by_id(sample)[BASE]
            q, p = body.orientation, body.position
            rotation = Rotation.from_quat([q.x, q.y, q.z, q.w])
            if not rows:
                forward_axis = rotation.as_matrix()[:2, 0]
                forward_axis /= np.linalg.norm(forward_axis)
            rows.append(
                [
                    sample.header.time_ns / 1e9,
                    p.x,
                    p.y,
                    p.z,
                    rotation.as_matrix()[2, 2],
                    rotation.as_euler("xyz")[2],
                ]
            )
            command_rows.append([rows[-1][0], elapsed, vx, wz])
            if rows[-1][4] < 0.5 or p.z < 0.2:
                outcome = "fallen"
                break
            if args.terrain == "stairs":
                on_landing = (
                    p.x >= 4
                    and abs(p.y) < 2.4
                    and p.z >= rows[0][3] + 5 * args.step_height - 0.12
                    and rows[-1][4] > 0.85
                )
                if reached_goal_at is None and on_landing:
                    reached_goal_at = rows[-1][0]
                if reached_goal_at is not None and rows[-1][0] - reached_goal_at >= 2:
                    outcome = "goal" if on_landing else "lost_landing"
                    break
            time.sleep(max(0, 0.05 - (time.monotonic() - tick)))
        policy.move(Twist(), duration=0.35)
        a = np.asarray(rows)
        commands = np.asarray(command_rows)
        dt = np.diff(a[:, 0])
        travel = np.linalg.norm(np.diff(a[:, 1:3], axis=0), axis=1)
        measured_speed = travel / np.maximum(dt, 1e-9)
        steady = (commands[1:, 1] >= 3) & (commands[1:, 2] != 0) & (dt > 0)
        turning = (commands[1:, 1] >= args.turn_after_s + 3) & (commands[1:, 3] != 0) & (dt > 0)
        yaw_rate = np.diff(np.unwrap(a[:, 5])) / np.maximum(dt, 1e-9)
        stop_indices = np.flatnonzero(commands[:, 2] == 0)
        stop_index = int(stop_indices[0]) if len(stop_indices) else None
        report = {
            "terrain": args.terrain,
            "outcome": outcome,
            "stairs_passed": args.terrain == "stairs" and outcome == "goal",
            "step_height_m": args.step_height if args.terrain == "stairs" else None,
            "goal_sim_time_s": reached_goal_at,
            "command_mps": args.speed,
            "command_yaw_rate": args.yaw_rate,
            "steady_speed_mps": float(np.median(measured_speed[steady])) if steady.any() else None,
            "peak_speed_mps": float(measured_speed.max()) if len(measured_speed) else 0.0,
            "turn_yaw_rate_rad_s": float(np.median(yaw_rate[turning])) if turning.any() else None,
            "stop_path_m": float(travel[stop_index:].sum()) if stop_index is not None else None,
            "final_2s_drift_m": float(travel[a[1:, 0] >= a[-1, 0] - 2].sum()),
            "duration_s": float(a[-1, 0] - a[0, 0]),
            "distance_xy_m": float(np.linalg.norm(a[-1, 1:3] - a[0, 1:3])),
            "forward_axis_xy": forward_axis.tolist(),
            "forward_mps": float(
                np.dot(a[-1, 1:3] - a[0, 1:3], forward_axis) / max(a[-1, 0] - a[0, 0], 1e-9)
            ),
            "min_upright": float(a[:, 4].min()),
            "z_range_m": [float(a[:, 3].min()), float(a[:, 3].max())],
            "sim": sim.status(),
            "policy": policy.status(),
            "trajectory": rows,
            "commands": command_rows,
            "residency": residency_rows,
        }
        args.output.write_text(json.dumps(report, indent=2))
        print(
            json.dumps(
                {
                    k: v
                    for k, v in report.items()
                    if k not in ("trajectory", "residency", "commands")
                }
            ),
            flush=True,
        )
    except Exception:
        print(
            json.dumps({"simulation_failure": deployment.get_instance(SimModule).status()}),
            flush=True,
        )
        raise
    finally:
        if recording is not None:
            recording.close()
        deployment.stop()


if __name__ == "__main__":
    main()
