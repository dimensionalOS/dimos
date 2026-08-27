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

"""Standalone GR00T-in-MuJoCo replay. No dimos modules, no transports.

Runs the two-model GR00T ONNX policy on the G1 MJCF, feeds it the twist
commands recorded on hardware, and draws a green transparent box at the
Point-LIO pelvis pose so sim body motion can be eyeballed against the real
robot's.

    .venv/bin/python -m dimos.robot.unitree.g1.sysid.groot_mujoco \
        data/g1_groot_characterization_2026-08-27.db

The box starts welded to the robot: Point-LIO poses are taken relative to
the pose at command t0, so what you see afterwards is pure divergence.
"""

from __future__ import annotations

import argparse
import bisect
from pathlib import Path
import time
from typing import Any, Protocol

import mujoco
import mujoco.viewer
import numpy as np
from numpy.typing import NDArray
import onnxruntime as ort

from dimos.control.tasks.g1_groot_wbc_task.g1_groot_wbc_task import (
    _DEFAULT_POSITIONS_29,
    G1_GROOT_KD,
    G1_GROOT_KP,
)
from dimos.memory.cli.dataset import open_store
from dimos.utils.data import LfsPath

ROBOT_MJCF = Path(__file__).resolve().parents[1] / "assets" / "g1_29dof.xml"


class Mid360Odometry(Protocol):
    x: float
    y: float
    z: float

    @property
    def orientation(self) -> Any: ...


# The Mid-360 is mounted rolled 180 deg relative to the URDF convention.
_MID360_MOUNT_UNROLL = np.diag([1.0, -1.0, -1.0])


def world_T_pelvis(odom: Mid360Odometry) -> NDArray[np.float64]:
    """Point-LIO mid360 odometry -> world_T_pelvis, at the zero waist pose."""
    from dimos.robot.unitree.g1.g1_tf_publisher import base_to_torso, torso_to_mid360

    pelvis_T_physical = (base_to_torso(0.0, 0.0, 0.0) + torso_to_mid360()).to_matrix()
    pelvis_T_mid360 = np.asarray(pelvis_T_physical, dtype=np.float64).copy()
    pelvis_T_mid360[:3, :3] = pelvis_T_physical[:3, :3] @ _MID360_MOUNT_UNROLL

    world_T_mid360 = np.eye(4, dtype=np.float64)
    world_T_mid360[:3, :3] = odom.orientation.to_rotation_matrix() @ _MID360_MOUNT_UNROLL
    world_T_mid360[:3, 3] = (odom.x, odom.y, odom.z)
    return world_T_mid360 @ np.linalg.inv(pelvis_T_mid360)


# Policy contract (see G1GrootWBCTask docstring). Changing these drifts the
# policy away from what it was trained for.
OBS_DIM = 86
OBS_HISTORY = 6
NUM_ACTIONS = 15
NUM_MOTORS = 29
ACTION_SCALE = 0.25
ANG_VEL_SCALE = 0.5
DOF_POS_SCALE = 1.0
DOF_VEL_SCALE = 0.05
CMD_SCALE = np.array([2.0, 2.0, 0.5], dtype=np.float32)
CMD_NORM_THRESHOLD = 0.05
HEIGHT_CMD = 0.74
POLICY_HZ = 50.0

DEFAULT_29 = np.asarray(_DEFAULT_POSITIONS_29, dtype=np.float32)
KP = np.asarray(G1_GROOT_KP, dtype=np.float64)
KD = np.asarray(G1_GROOT_KD, dtype=np.float64)


def name2id(model: mujoco.MjModel, objtype: int, name: str) -> int:
    """mj_name2id that raises instead of returning -1 and poisoning an index."""
    i = int(mujoco.mj_name2id(model, objtype, name))
    if i < 0:
        raise KeyError(f"{name!r} not found in the composed model")
    return i


def build_model(ghost: bool, mjcf: Path = ROBOT_MJCF) -> mujoco.MjModel:
    """The blueprint's own empty scene + robot MJCF + a green mocap ghost box."""
    spec = mujoco.MjSpec.from_file(str(LfsPath("mujoco_sim/scene_empty.xml")))
    robot = mujoco.MjSpec.from_file(str(mjcf))
    robot.meshdir = str(LfsPath("g1_urdf/meshes"))
    spec.option.timestep = robot.option.timestep
    # prefix="" keeps MJCF names unprefixed so name lookups below stay valid.
    spec.attach(robot, frame=spec.worldbody.add_frame(), prefix="")

    if ghost:
        body = spec.worldbody.add_body()
        body.name = "ghost"
        body.mocap = True
        geom = body.add_geom()
        geom.name = "ghost_box"
        geom.type = mujoco.mjtGeom.mjGEOM_BOX
        geom.size = [0.13, 0.11, 0.30]
        geom.pos = [0.0, 0.0, 0.18]
        geom.rgba = [0.1, 1.0, 0.2, 0.28]
        geom.contype = 0  # visual only: never collides, never adds mass
        geom.conaffinity = 0
    return spec.compile()


def touchdown_z(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    """Pelvis height at which the default pose first touches the floor.

    Bisection on ncon rather than geom bounds: rbound is a bounding sphere and
    overstates foot extent, so it would spawn the robot too high. Keeps the
    drop identical across plants with different foot geometry.
    """

    def touches(z: float) -> bool:
        mujoco.mj_resetData(model, data)  # type: ignore[attr-defined]
        data.qpos[7 : 7 + NUM_MOTORS] = DEFAULT_29
        data.qpos[2] = z
        mujoco.mj_forward(model, data)
        return bool(data.ncon)

    lo, hi = 0.3, float(model.body_pos[1][2])
    if touches(hi) or not touches(lo):
        return hi  # nothing sane to bisect between; leave the MJCF height alone
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        if touches(mid):
            lo = mid
        else:
            hi = mid
    return lo


class GrootPolicy:
    """Balance + walk ONNX pair behind one step() call."""

    def __init__(self, model_dir: Path) -> None:
        providers = ["CPUExecutionProvider"]
        if "CUDAExecutionProvider" in ort.get_available_providers():
            providers.insert(0, "CUDAExecutionProvider")
        self._balance = ort.InferenceSession(str(model_dir / "balance.onnx"), providers=providers)
        self._walk = ort.InferenceSession(str(model_dir / "walk.onnx"), providers=providers)
        self._balance_in = self._balance.get_inputs()[0].name
        self._walk_in = self._walk.get_inputs()[0].name
        self._last_action = np.zeros(NUM_ACTIONS, dtype=np.float32)
        self._buf = np.zeros((1, OBS_DIM * OBS_HISTORY), dtype=np.float32)
        self._first = True

    def step(
        self,
        cmd: NDArray[np.float32],
        gyro: NDArray[np.float32],
        quat_wxyz: NDArray[np.float64],
        q29: NDArray[np.float32],
        dq29: NDArray[np.float32],
    ) -> NDArray[np.float32]:
        w, x, y, z = quat_wxyz
        gravity = np.array(
            [
                2.0 * (-x * z + w * y),
                2.0 * (-y * z - w * x),
                -(w * w - x * x - y * y + z * z),
            ],
            dtype=np.float32,
        )

        obs: NDArray[np.float32] = np.zeros(OBS_DIM, dtype=np.float32)
        obs[0:3] = cmd * CMD_SCALE
        obs[3] = HEIGHT_CMD
        obs[7:10] = gyro * ANG_VEL_SCALE
        obs[10:13] = gravity
        obs[13:42] = (q29 - DEFAULT_29) * DOF_POS_SCALE
        obs[42:71] = dq29 * DOF_VEL_SCALE
        obs[71:86] = self._last_action

        if self._first:
            self._buf[0, :] = np.tile(obs, OBS_HISTORY)
            self._first = False
        else:
            self._buf[0, : OBS_DIM * (OBS_HISTORY - 1)] = self._buf[0, OBS_DIM:]
            self._buf[0, OBS_DIM * (OBS_HISTORY - 1) :] = obs

        if float(np.linalg.norm(cmd)) <= CMD_NORM_THRESHOLD:
            raw = self._balance.run(None, {self._balance_in: self._buf})[0]
        else:
            raw = self._walk.run(None, {self._walk_in: self._buf})[0]

        self._last_action[:] = raw[0, :NUM_ACTIONS].astype(np.float32)
        return self._last_action * ACTION_SCALE + DEFAULT_29[:NUM_ACTIONS]


def load_commands(db: Path, stream: str) -> tuple[list[float], NDArray[np.float32]]:
    """(timestamps, Nx3 vx/vy/wz) for one recorded Twist stream."""
    with open_store(db) as store:
        obs: list[Any] = list(iter(store.stream(stream)))
        ts = [float(o.ts) for o in obs]
        cmds = np.array(
            [[o.data.linear.x, o.data.linear.y, o.data.angular.z] for o in obs], dtype=np.float32
        )
    return ts, cmds


def load_ghost(db: Path) -> tuple[list[float], NDArray[np.float64]]:
    """(timestamps, Nx4x4 world_T_pelvis) from Point-LIO odometry."""
    with open_store(db) as store:
        obs: list[Any] = list(iter(store.stream("pointlio_odometry")))
        ts = [float(o.ts) for o in obs]
        poses = np.array([world_T_pelvis(o.data) for o in obs], dtype=np.float64)
    return ts, poses


def at(ts: list[float], t: float) -> int | None:
    """Index of the last sample at or before *t* (None before the first)."""
    i = bisect.bisect_right(ts, t) - 1
    return None if i < 0 else i


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("recording", type=Path)
    ap.add_argument("--stream", default="cmd_vel", help="twist stream to replay")
    ap.add_argument("--lead-in-s", type=float, default=3.0, help="settle before commands start")
    ap.add_argument("--duration-s", type=float, default=None)
    ap.add_argument("--speed", type=float, default=1.0, help="0 = as fast as it computes")
    ap.add_argument("--no-ghost", action="store_true")
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--mjcf", type=Path, default=ROBOT_MJCF, help="plant under test")
    ap.add_argument("--trace-hz", type=float, default=0.0, help="print pelvis pose")
    ap.add_argument("--drop-m", type=float, default=0.01, help="spawn height above touchdown")
    args = ap.parse_args()

    cmd_ts, cmds = load_commands(args.recording, args.stream)
    ghost_ts, ghost_poses = (
        ([], np.empty((0, 4, 4))) if args.no_ghost else load_ghost(args.recording)
    )
    print(f"{len(cmd_ts)} twists on {args.stream}, {len(ghost_ts)} Point-LIO poses")

    model = build_model(ghost=not args.no_ghost and len(ghost_ts) > 0, mjcf=args.mjcf)
    data = mujoco.MjData(model)
    policy = GrootPolicy(Path(str(LfsPath("groot"))))

    # Stand at the policy's zero-offset pose, dropped from a fixed height so
    # every plant gets the same landing transient instead of whatever clearance
    # its MJCF happens to spawn with.
    ground = touchdown_z(model, data)
    mujoco.mj_resetData(model, data)  # type: ignore[attr-defined]
    data.qpos[7 : 7 + NUM_MOTORS] = DEFAULT_29
    data.qpos[2] = ground + args.drop_m
    mujoco.mj_forward(model, data)
    print(f"touchdown z={ground:.4f} m, spawning at {data.qpos[2]:.4f} m")

    gyro_adr = model.sensor_adr[
        name2id(model, int(mujoco.mjtObj.mjOBJ_SENSOR), "imu-angular-velocity")  # type: ignore[attr-defined]
    ]
    ghost_id = (
        model.body_mocapid[name2id(model, int(mujoco.mjtObj.mjOBJ_BODY), "ghost")]
        if len(ghost_ts)
        else -1
    )

    t0 = cmd_ts[0] - args.lead_in_s  # recording clock at sim t=0
    # Weld the ghost to the robot's start pose: only divergence is visible.
    ghost_ref = np.eye(4)
    if ghost_id >= 0:
        i = at(ghost_ts, cmd_ts[0])
        if i is not None:
            ghost_ref = np.linalg.inv(ghost_poses[i])
    robot_start = np.eye(4)
    robot_start[:3, 3] = data.qpos[:3]

    steps_per_policy = max(1, round(1.0 / (POLICY_HZ * model.opt.timestep)))
    end_t = (cmd_ts[-1] - t0) if args.duration_s is None else args.duration_s
    target = DEFAULT_29[:NUM_ACTIONS].copy()
    ctrl = np.zeros(NUM_MOTORS)
    step_i = 0
    wall0 = time.time()

    viewer = (
        None
        if args.headless
        else mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False)
    )
    try:
        while data.time < end_t and (viewer is None or viewer.is_running()):
            now = t0 + data.time

            if step_i % steps_per_policy == 0:
                i = at(cmd_ts, now)
                cmd = cmds[i] if i is not None else np.zeros(3, dtype=np.float32)
                target = policy.step(
                    cmd=cmd,
                    gyro=data.sensordata[gyro_adr : gyro_adr + 3].astype(np.float32),
                    quat_wxyz=data.qpos[3:7],
                    q29=data.qpos[7 : 7 + NUM_MOTORS].astype(np.float32),
                    dq29=data.qvel[6 : 6 + NUM_MOTORS].astype(np.float32),
                )

            # Legs+waist track the policy; arms hold the relaxed default.
            q = data.qpos[7 : 7 + NUM_MOTORS]
            dq = data.qvel[6 : 6 + NUM_MOTORS]
            desired = DEFAULT_29.astype(np.float64).copy()
            desired[:NUM_ACTIONS] = target
            ctrl[:] = KP * (desired - q) - KD * dq
            data.ctrl[:] = ctrl

            if ghost_id >= 0:
                i = at(ghost_ts, now)
                if i is not None:
                    pose = robot_start @ ghost_ref @ ghost_poses[i]
                    data.mocap_pos[ghost_id] = pose[:3, 3]
                    quat = np.zeros(4)
                    mujoco.mju_mat2Quat(  # type: ignore[attr-defined]
                        quat, pose[:3, :3].flatten()
                    )
                    data.mocap_quat[ghost_id] = quat

            mujoco.mj_step(model, data)
            step_i += 1

            if (
                args.trace_hz
                and step_i % max(1, round(1 / (args.trace_hz * model.opt.timestep))) == 0
            ):
                print(
                    f"t={data.time:6.2f}  x={data.qpos[0]:+.3f} y={data.qpos[1]:+.3f} "
                    f"z={data.qpos[2]:.3f}",
                    flush=True,
                )

            if viewer is not None:
                viewer.sync()
            if args.speed > 0:
                lag = data.time / args.speed - (time.time() - wall0)
                if lag > 0:
                    time.sleep(lag)

        print(
            f"stopped at t={data.time:.1f}s  pelvis={np.round(data.qpos[:3], 3)}  "
            f"height={data.qpos[2]:.3f}m"
        )
    finally:
        if viewer is not None:
            viewer.close()


if __name__ == "__main__":
    main()
