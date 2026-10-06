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

"""Isolated recovered-policy trials. No transport, hardware, planner or live blueprint."""

from __future__ import annotations

import argparse
from contextlib import ExitStack
import ctypes
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from typing import Any
from uuid import uuid4

import imageio.v3 as iio
import mujoco
import numpy as np

from dimos.control.go2_freewalk.policy import JOINT_NAMES, FreePolicy, Proprioception
from dimos.hardware.whole_body.spec import MotorCommand
from dimos.robot.unitree.go2.sim2 import GO2_FREEWALK
from dimos.sim2.control.adapters import WholeBodyAdapter
from dimos.sim2.runtime import SimulationRuntime
from dimos.sim2.spec import RobotInstance, WorldConfig

ROOT = Path(__file__).resolve().parents[2]
FPTR = ctypes.POINTER(ctypes.c_float)


def reference_library(root: Path) -> ctypes.CDLL:
    source = Path(__file__).with_name("reference_ffi.rs")
    key = hashlib.sha256(source.read_bytes() + str(root).encode()).hexdigest()[:12]
    build = Path(tempfile.gettempdir()) / f"go2-policy-reference-{key}"
    build.mkdir(exist_ok=True)
    manifest = (
        '[package]\nname = "go2-reference"\nversion = "0.0.0"\nedition = "2021"\n'
        f'[lib]\npath = {json.dumps(str(source))}\ncrate-type = ["cdylib"]\n'
        f"[dependencies]\ngo2-policy = {{path = {json.dumps(str(root / 'policy'))}}}\n"
    )
    (build / "Cargo.toml").write_text(manifest)
    subprocess.run(
        ["cargo", "build", "--quiet", "--release", "--manifest-path", str(build / "Cargo.toml")],
        check=True,
    )
    suffix = "dylib" if sys.platform == "darwin" else "so"
    lib = ctypes.CDLL(str(build / f"target/release/libgo2_reference.{suffix}"))
    lib.create.argtypes = [ctypes.c_uint32, ctypes.c_void_p, ctypes.c_size_t, FPTR]
    lib.create.restype = ctypes.c_void_p
    lib.tick.argtypes = [ctypes.c_void_p, FPTR, FPTR]
    lib.tick.restype = None
    lib.destroy.argtypes = [ctypes.c_void_p]
    lib.destroy.restype = None
    return lib


def run(args: argparse.Namespace) -> dict[str, Any]:
    with ExitStack() as resources:
        return _run(args, resources)


def _run(args: argparse.Namespace, resources: ExitStack) -> dict[str, Any]:
    ref = args.reference_root
    weights = (
        ref
        / "policy/assets"
        / ("b_v2_blind.bin" if args.policy == "rust-blind" else "freewalk_mcf.bin")
    )
    blob = weights.read_bytes()
    policy = FreePolicy(blob) if args.policy == "python-free" else None
    lib = reference_library(ref) if policy is None else None
    handle = None
    if lib is not None:
        home = np.zeros(12, dtype=np.float32)
        handle = lib.create(
            int(args.policy == "rust-blind"), blob, len(blob), home.ctypes.data_as(FPTR)
        )
        if not handle:
            raise ValueError("reference policy initialization failed")
        resources.callback(lib.destroy, handle)
        home = home.astype(float)
    else:
        assert policy is not None
        home = policy.default_pose

    if args.sim2 and (
        args.policy != "python-free" or args.robot != ROOT / "data/go2_menagerie/go2.xml"
    ):
        raise ValueError("sim2 probe uses the shipped GO2_FREEWALK definition and Python policy")
    spec = mujoco.MjSpec() if args.sim2 else mujoco.MjSpec.from_file(str(args.robot))
    for key in list(spec.keys):
        spec.delete(key)
    spec.option.timestep = args.dt
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    if args.cone is not None:
        spec.option.cone = getattr(mujoco.mjtCone, "mjCONE_" + args.cone.upper())
    if args.impratio is not None:
        spec.option.impratio = args.impratio
    spec.worldbody.add_geom(
        name="ground",
        type=mujoco.mjtGeom.mjGEOM_PLANE,
        size=[20, 20, 0.1],
        rgba=[0.35, 0.4, 0.42, 1],
        friction=[0.8, 0.02, 0.01],
    )
    if args.height:
        for step in range(args.steps):
            length = args.tread if step < args.steps - 1 else 6.0
            x = args.start + step * args.tread
            h = args.height * (step + 1)
            spec.worldbody.add_geom(
                name=f"step_{step}",
                type=mujoco.mjtGeom.mjGEOM_BOX,
                pos=[x + length / 2, 0, h / 2],
                size=[length / 2, args.width / 2, h / 2],
                rgba=[0.45 + 0.08 * (step % 2), 0.52, 0.6, 1],
                friction=[0.8, 0.02, 0.01],
            )
    spec.worldbody.add_light(pos=[0, -3, 5], dir=[0.2, 0.5, -1])
    world = None
    device = None
    fixture = Path(resources.enter_context(tempfile.TemporaryDirectory(prefix="go2-stairs-")))
    if args.sim2:
        scene = fixture / "scene.xml"
        spec.compile()
        spec.to_file(str(scene))
        world = SimulationRuntime(
            WorldConfig(
                scene,
                {
                    "go2": RobotInstance(
                        GO2_FREEWALK,
                        xyz=(0, args.lateral_offset, 0.33),
                        rpy=(0, 0, float(np.deg2rad(args.yaw_degrees))),
                    )
                },
                timestep=args.dt,
            ),
            "stair-study-" + uuid4().hex,
        )
        resources.callback(world.close)
        model, data = world.model, world.data
        device = WholeBodyAdapter(
            address=world.snapshot_descriptor.sim_id + "/go2", dof=12, definition=GO2_FREEWALK
        )
        resources.callback(device.disconnect)
        device.connect()
    else:
        model = spec.compile()
        data = mujoco.MjData(model)
    prefix = "go2/" if args.sim2 else ""
    jids = np.array([model.joint(prefix + name).id for name in JOINT_NAMES])
    qids, dids = model.jnt_qposadr[jids], model.jnt_dofadr[jids]
    aids = np.array(
        [next(i for i in range(model.nu) if model.actuator_trnid[i, 0] == jid) for jid in jids]
    )
    if args.damping is not None:
        model.dof_damping[dids] = args.damping
    if args.frictionloss is not None:
        model.dof_frictionloss[dids] = args.frictionloss
    yaw = np.deg2rad(args.yaw_degrees)
    data.qpos[:7] = [0, args.lateral_offset, 0.33, np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
    data.qpos[qids] = home
    mujoco.mj_forward(model, data)
    base = int(model.jnt_bodyid[0])
    feet = np.array([model.geom(prefix + name).id for name in ("FL", "FR", "RL", "RR")])
    kp = policy.kp if policy is not None else np.full(12, 40.0)
    kd = policy.kd if policy is not None else np.ones(12)
    target = home.copy()
    command = np.zeros(3)
    desired = np.array(args.command)
    stride = round(1 / args.hz / args.dt)
    if not np.isclose(stride * args.dt * args.hz, 1):
        raise ValueError("physics dt must divide policy period exactly")
    frames = []
    trace = []
    states = []
    commands = []
    fallen = False
    top = args.start + (args.steps - 1) * args.tread
    reached_at = None
    start = time.perf_counter()
    renderer = mujoco.Renderer(model, 480, 640) if args.video else None
    if renderer is not None:
        resources.callback(renderer.close)
    camera = mujoco.MjvCamera()
    camera.azimuth, camera.elevation, camera.distance = 130, -20, 3
    render_options = mujoco.MjvOption()
    render_options.geomgroup[3] = False
    try:
        for tick in range(round(args.seconds / args.dt)):
            q = data.qpos[qids].copy()
            dq = data.qvel[dids].copy()
            rot = data.xmat[base].reshape(3, 3)
            quat = data.xquat[base].copy()
            gyro = data.qvel[3:6].copy()
            gravity = -rot[2].copy()
            if tick % stride == 0:
                if device is not None:
                    motors = device.read_motor_states()
                    imu = device.read_imu()
                    q = np.array([m.q for m in motors])
                    dq = np.array([m.dq for m in motors])
                    gyro = np.array(imu.gyroscope)
                    quat = np.array(imu.quaternion)
                    rotation = np.zeros(9)
                    mujoco.mju_quat2Mat(rotation, quat)
                    gravity = -rotation.reshape(3, 3)[2]
                if data.time >= args.hold:
                    requested = desired if data.time >= args.hold + args.settle else np.zeros(3)
                    if args.height and data.qpos[0] > top + 0.7 and reached_at is None:
                        reached_at = float(data.time)
                    if reached_at is not None:
                        requested = np.zeros(3)
                    command += np.clip(requested - command, [-0.05, -0.04, -0.1], [0.05, 0.04, 0.1])
                    if policy is not None:
                        target = policy.act(Proprioception(gyro, gravity, q, dq), command)
                    else:
                        x = np.concatenate([quat, gyro, q, dq, command]).astype(np.float32)
                        y = np.zeros(36, dtype=np.float32)
                        lib.tick(handle, x.ctypes.data_as(FPTR), y.ctypes.data_as(FPTR))
                        target, kp, kd = np.split(y.astype(float), 3)
                if device is not None:
                    device.write_motor_commands(
                        [
                            MotorCommand(
                                q=float(t),
                                dq=0,
                                kp=float(p * args.kp_scale),
                                kd=float(d * args.kd_scale),
                            )
                            for t, p, d in zip(target, kp, kd, strict=True)
                        ]
                    )
                local_v = rot.T @ data.qvel[:3]
                trace.append(
                    [
                        float(data.time),
                        *data.qpos[:3].tolist(),
                        *local_v.tolist(),
                        float(gyro[2]),
                        float(np.arccos(np.clip(-gravity[2], -1, 1))),
                        *target.tolist(),
                    ]
                )
                states.append(data.qpos.copy())
                commands.append(command.copy())
                if gravity[2] > -0.5:
                    fallen = True
                    break
                if reached_at is not None and data.time - reached_at >= 3:
                    break
            if world is not None:
                world.step()
            else:
                torque = kp * args.kp_scale * (target - q) - kd * args.kd_scale * dq
                data.ctrl[aids] = np.clip(
                    torque, model.actuator_ctrlrange[aids, 0], model.actuator_ctrlrange[aids, 1]
                )
                mujoco.mj_step(model, data)
            if renderer is not None and tick % round(1 / 25 / args.dt) == 0:
                camera.lookat = data.qpos[:3]
                renderer.update_scene(data, camera, scene_option=render_options)
                frames.append(renderer.render().copy())
    finally:
        resources.close()
    samples = np.array(trace)
    mujoco.mj_forward(model, data)
    late = samples[samples[:, 0] >= max(args.hold + args.settle, data.time - 4)]
    if len(late) == 0:
        late = samples
    foot_positions = data.geom_xpos[feet]
    feet_on_top = bool(
        (foot_positions[:, 0] > top).all()
        and (abs(foot_positions[:, 1]) < args.width / 2).all()
        and (foot_positions[:, 2] > args.steps * args.height - 0.02).all()
    )
    success = bool(
        reached_at is not None
        and data.time - reached_at >= 3
        and not fallen
        and data.qpos[2] > args.steps * args.height + 0.20
        and feet_on_top
    )
    result = {
        "policy": args.policy,
        "weights_sha256": hashlib.sha256(blob).hexdigest(),
        "reference_revision": subprocess.check_output(
            ["git", "-C", str(ref), "rev-parse", "HEAD"], text=True
        ).strip(),
        "robot": str(args.robot),
        "robot_xml_sha256": hashlib.sha256(args.robot.read_bytes()).hexdigest(),
        "parameters": {
            k: v
            for k, v in vars(args).items()
            if k not in {"reference_root", "robot", "output", "video"}
        },
        "seconds": float(data.time),
        "wall_seconds": time.perf_counter() - start,
        "fallen": fallen,
        "stairs_reached_top": success,
        "reached_at_seconds": reached_at,
        "final_position": data.qpos[:3].tolist(),
        "final_foot_positions": foot_positions.tolist(),
        "max_x": float(samples[:, 1].max()),
        "max_tilt_deg": float(np.rad2deg(samples[:, 8].max())),
        "late_body_velocity": late[:, 4:7].mean(axis=0).tolist(),
        "late_yaw_rate": float(late[:, 7].mean()),
        "final_tilt_deg": float(np.rad2deg(samples[-1, 8])),
        "late_lateral_velocity_rms": float(np.sqrt(np.mean(late[:, 5] ** 2))),
        "physics_options": {
            "cone": mujoco.mjtCone(model.opt.cone).name,
            "impratio": float(model.opt.impratio),
            "timestep": float(model.opt.timestep),
        },
        "passive_damping": model.dof_damping[dids].tolist(),
    }
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + "\n")
        np.savez_compressed(
            args.output.with_suffix(".npz"),
            trace=samples,
            qpos=np.array(states),
            command=np.array(commands),
        )
    if args.video:
        iio.imwrite(args.video, np.array(frames), plugin="pyav", codec="h264", fps=25)
    return result


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--reference-root", type=Path, required=True)
    p.add_argument("--robot", type=Path, default=ROOT / "data/go2_menagerie/go2.xml")
    p.add_argument(
        "--policy", choices=["python-free", "rust-free", "rust-blind"], default="python-free"
    )
    p.add_argument(
        "--sim2",
        action="store_true",
        help="Use the real sim2 physics owner and SHM motor/IMU device",
    )
    p.add_argument("--height", type=float, default=0)
    p.add_argument("--steps", type=int, default=4)
    p.add_argument("--tread", type=float, default=0.30)
    p.add_argument("--width", type=float, default=1.2)
    p.add_argument("--start", type=float, default=1.0)
    p.add_argument("--seconds", type=float, default=14)
    p.add_argument("--hold", type=float, default=1)
    p.add_argument("--settle", type=float, default=2)
    p.add_argument("--command", type=float, nargs=3, default=[0.4, 0, 0])
    p.add_argument("--hz", type=float, default=50)
    p.add_argument("--dt", type=float, default=0.0025)
    p.add_argument("--damping", type=float)
    p.add_argument("--frictionloss", type=float)
    p.add_argument("--kp-scale", type=float, default=1)
    p.add_argument("--kd-scale", type=float, default=1)
    p.add_argument("--cone", choices=["elliptic", "pyramidal"])
    p.add_argument("--impratio", type=float)
    p.add_argument("--lateral-offset", type=float, default=0)
    p.add_argument("--yaw-degrees", type=float, default=0)
    p.add_argument("--output", type=Path)
    p.add_argument("--video", type=Path)
    print(json.dumps(run(p.parse_args()), indent=2))


if __name__ == "__main__":
    main()
