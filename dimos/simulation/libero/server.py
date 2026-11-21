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

"""LIBERO native: a LIBERO(-PRO) task with LIBERO's own Panda, stepped live and published on zenoh.

Runs in LIBERO's own Python (3.10, robosuite 1.4), so no dimos imports; ``dimos_lcm``
provides the encoders. Reads one JSON line on stdin: ``topics`` (port -> zenoh key),
``config``, ``session``.

LIBERO builds the scene from the BDDL file, robot included, and starts it from one of
the task's recorded initial states (``seed`` picks which) when LIBERO ships them, else
from a layout it samples with ``seed``. The Panda's torque motors then track the joint
targets on ``sim_command`` with a joint-space impedance law (robosuite's
``JOINT_POSITION`` controller, run every physics step); robosuite's own controllers
never run. Published, in the shapes ``MujocoSimModule`` uses:

- ``sim_state``: joint1..7 then the gripper opening in metres (0 closed .. 0.08 open);
- ``color_image`` / ``depth_image`` / ``camera_info`` from the hand camera, and ``tf``
  for the camera frames under ``link7`` plus ``world`` -> every LIBERO object;
- ``task_status``: LIBERO's own goal check as JSON, ``{"success", "predicates", ...}``.

Poses are published in a ``world`` frame shifted so the Panda's ``link0`` is the
origin, so the planning model needs no per-scene base pose.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import pickle
import random
import sys
import threading
import time
from typing import Any
import zipfile

from dimos_lcm.geometry_msgs.Quaternion import Quaternion
from dimos_lcm.geometry_msgs.Transform import Transform
from dimos_lcm.geometry_msgs.TransformStamped import TransformStamped
from dimos_lcm.geometry_msgs.Vector3 import Vector3
from dimos_lcm.sensor_msgs.CameraInfo import CameraInfo as LCMCameraInfo
from dimos_lcm.sensor_msgs.Image import Image as LCMImage
from dimos_lcm.sensor_msgs.JointState import JointState as LCMJointState
from dimos_lcm.std_msgs.Header import Header
from dimos_lcm.std_msgs.String import String as LCMString
from dimos_lcm.tf2_msgs.TFMessage import TFMessage as LCMTFMessage
import mujoco
import numpy as np
import zenoh

PREFIX = "robot0_"
ARM_JOINTS = tuple(f"joint{i}" for i in range(1, 8))
GRIPPER_JOINT = "gripper"
# The gripper position: the opening between the fingers, each 0..0.04 m.
GRIPPER_RANGE = (0.0, 0.08)
FINGERS = ("gripper0_finger_joint1", "gripper0_finger_joint2")
FINGER_ACTUATORS = ("gripper0_gripper_finger_joint1", "gripper0_gripper_finger_joint2")
# Joint-space impedance on the arm's torque motors: tau = M (kp e - kd qdot) + bias.
KP = 400.0
KD = 2.0 * math.sqrt(KP)
# robosuite's hand camera; published under dimos's wrist-camera frame names.
SIM_CAMERA = PREFIX + "eye_in_hand"
CAMERA = "wrist_camera"


def log(msg: str) -> None:
    """stdout: NativeModule logs it as INFO."""
    sys.stdout.write(f"[libero] {msg}\n")
    sys.stdout.flush()


def warn(msg: str) -> None:
    """stderr: NativeModule logs it as WARNING."""
    sys.stderr.write(f"[libero] {msg}\n")
    sys.stderr.flush()


# -- encoders -----------------------------------------------------------------


def _header(frame_id: str, ts: float) -> Header:
    h = Header()
    h.seq = 0
    h.stamp.sec = int(ts)
    h.stamp.nsec = int((ts - int(ts)) * 1e9)
    h.frame_id = frame_id
    return h


def image_msg(array: np.ndarray, encoding: str, frame_id: str, ts: float) -> bytes:
    m = LCMImage()
    m.header = _header(frame_id, ts)
    m.height, m.width = int(array.shape[0]), int(array.shape[1])
    m.encoding = encoding
    m.is_bigendian = False
    channels = 1 if array.ndim == 2 else array.shape[2]
    m.step = m.width * array.dtype.itemsize * channels
    view = memoryview(np.ascontiguousarray(array)).cast("B")
    m.data_length = len(view)
    m.data = view
    return bytes(m.lcm_encode())


def camera_info_msg(width: int, height: int, fovy_deg: float, frame_id: str, ts: float) -> bytes:
    """Pinhole intrinsics from the MJCF camera, as MujocoSimModule computes them."""
    fy = height / (2.0 * math.tan(math.radians(fovy_deg) / 2.0))
    fx, cx, cy = fy, width / 2.0, height / 2.0
    m = LCMCameraInfo()
    m.header = _header(frame_id, ts)
    m.width, m.height = width, height
    m.distortion_model = "plumb_bob"
    m.D = [0.0] * 5
    m.D_length = 5
    m.K = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    m.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    m.P = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    m.binning_x = m.binning_y = 0
    return bytes(m.lcm_encode())


def joint_state_msg(
    names: list[str], pos: list[float], vel: list[float], eff: list[float], ts: float
) -> bytes:
    m = LCMJointState()
    m.header = _header("", ts)
    m.name, m.name_length = list(names), len(names)
    m.position, m.position_length = list(pos), len(pos)
    m.velocity, m.velocity_length = list(vel), len(vel)
    m.effort, m.effort_length = list(eff), len(eff)
    return bytes(m.lcm_encode())


def transform(
    parent: str, child: str, pos: np.ndarray, quat_wxyz: np.ndarray, ts: float
) -> TransformStamped:
    t = TransformStamped()
    t.header = _header(parent, ts)
    t.child_frame_id = child
    t.transform = Transform()
    t.transform.translation = Vector3()
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = (
        float(v) for v in pos
    )
    t.transform.rotation = Quaternion()
    w, x, y, z = (float(v) for v in quat_wxyz)
    (
        t.transform.rotation.x,
        t.transform.rotation.y,
        t.transform.rotation.z,
        t.transform.rotation.w,
    ) = x, y, z, w
    return t


def tf_msg(transforms: list[TransformStamped]) -> bytes:
    m = LCMTFMessage()
    m.transforms, m.transforms_length = transforms, len(transforms)
    return bytes(m.lcm_encode())


def string_msg(text: str) -> bytes:
    m = LCMString()
    m.data = text
    return bytes(m.lcm_encode())


# -- zenoh --------------------------------------------------------------------

# Mirrors _ZENOH_KEYS in dimos/protocol/service/zenohservice.py.
_ZENOH_KEYS = {
    "mode": "mode",
    "connect": "connect/endpoints",
    "listen": "listen/endpoints",
    "multicast": "scouting/multicast/enabled",
    "scout_addr": "scouting/multicast/address",
    "interface": "scouting/multicast/interface",
    "gossip": "scouting/gossip/enabled",
    "connect_timeout_ms": "connect/timeout_ms",
}
# Empty means zenoh's own default.
_ZENOH_DEFAULTED_WHEN_EMPTY = ("connect", "listen", "scout_addr", "connect_timeout_ms")


def zenoh_session(session: dict[str, Any]) -> Any:
    """Open a zenoh session on the settings NativeModule handed us."""
    if not session.get("mode"):
        warn("no zenoh session on stdin; the LIBERO native is zenoh-only (DIMOS_TRANSPORT=zenoh)")
        sys.exit(2)
    cfg = zenoh.Config()
    for name, value in session.items():
        key = _ZENOH_KEYS.get(name)
        if key is None or (not value and name in _ZENOH_DEFAULTED_WHEN_EMPTY):
            continue
        cfg.insert_json5(key, json.dumps(value))
    return zenoh.open(cfg)


# -- the sim ------------------------------------------------------------------


def quat_mat(mat: np.ndarray) -> np.ndarray:
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.ascontiguousarray(mat).reshape(9))  # type: ignore[attr-defined]
    return q


def init_states(bddl: Path) -> Any:
    """LIBERO's recorded initial states for a task it ships, or None.

    ``init_files/<suite>/<task>.pruned_init`` mirrors ``bddl_files/<suite>/<task>.bddl``;
    the file is a ``torch.save`` of one numpy array, read here without torch.
    """
    from libero.libero import get_libero_path  # type: ignore[import-not-found]

    try:
        relative = bddl.relative_to(Path(get_libero_path("bddl_files")).resolve())
    except ValueError:
        return None
    path = Path(get_libero_path("init_states")) / relative.with_suffix(".pruned_init")
    if not path.exists():
        return None
    with zipfile.ZipFile(path) as archive:
        (name,) = [n for n in archive.namelist() if n.endswith("data.pkl")]
        return pickle.loads(archive.read(name))


class LiberoSim:
    """One LIBERO task: LIBERO builds and resets it, then the Panda is driven directly."""

    def __init__(self, cfg: dict[str, Any]) -> None:
        from libero.libero.envs import TASK_MAPPING  # type: ignore[import-not-found]
        import libero.libero.envs.bddl_utils as bddl_utils  # type: ignore[import-not-found]

        bddl = Path(cfg["bddl"]).resolve()
        seed = int(cfg.get("seed", 0))
        random.seed(seed)
        np.random.seed(seed)
        self.problem = bddl_utils.robosuite_parse_problem(str(bddl))
        self.env = TASK_MAPPING[self.problem["problem_name"]](
            bddl_file_name=str(bddl),
            robots=["Panda"],
            has_renderer=False,
            has_offscreen_renderer=False,
            use_camera_obs=False,
            ignore_done=True,
        )
        self.env.reset()
        states = init_states(bddl)
        if states is not None:
            # As LIBERO's own evaluation does (env_wrapper.set_init_state): episode i
            # starts from recorded state i.
            self.env.sim.set_state_from_flattened(states[seed % len(states)])
            self.env.sim.forward()
        self.model: Any = self.env.sim.model._model
        self.data: Any = self.env.sim.data._data
        m, d = self.model, self.data

        self.arm_act = [m.actuator(f"{PREFIX}torq_j{i}").id for i in range(1, 8)]
        self.arm_qpos = [m.jnt_qposadr[m.joint(PREFIX + j).id] for j in ARM_JOINTS]
        self.arm_dof = [m.jnt_dofadr[m.joint(PREFIX + j).id] for j in ARM_JOINTS]
        self.torque_range = m.actuator_ctrlrange[self.arm_act]
        self.finger_act = [m.actuator(a).id for a in FINGER_ACTUATORS]
        self.finger_qpos = [m.jnt_qposadr[m.joint(f).id] for f in FINGERS]
        self.mass = np.zeros((m.nv, m.nv))
        d.qvel[:] = 0.0
        mujoco.mj_forward(m, d)
        # Hold the start pose until the first command.
        self.target = d.qpos[self.arm_qpos].copy()
        self.set_gripper(self.gripper_opening())

        base = d.body(PREFIX + "link0").xpos.copy()
        self.shift = -base
        self.link7 = m.body(PREFIX + "link7").id
        self.camera = m.camera(SIM_CAMERA).id
        self.fovy = float(m.cam_fovy[self.camera])
        # Every LIBERO object and fixture, by the BDDL's own names.
        self.objects = {
            name: self.env.obj_body_id[name]
            for name in (*self.env.objects_dict, *self.env.fixtures_dict)
        }
        self.lock = threading.Lock()
        self.command: list[float] | None = None
        start = "recorded state" if states is not None else "sampled layout"
        log(
            f"ready: {bddl.name} '{' '.join(self.problem['language_instruction'])}' "
            f"{start} {seed} base={np.round(base, 3).tolist()} timestep={m.opt.timestep}"
        )

    # Commands arrive on the zenoh thread; they are applied before the next step.
    def set_command(self, positions: list[float]) -> None:
        with self.lock:
            self.command = positions

    def set_gripper(self, opening: float) -> None:
        self.data.ctrl[self.finger_act] = finger_targets(opening)

    def gripper_opening(self) -> float:
        left, right = (float(self.data.qpos[a]) for a in self.finger_qpos)
        return left - right

    def step(self) -> None:
        with self.lock:
            command, self.command = self.command, None
        if command is not None and len(command) >= len(ARM_JOINTS):
            self.target = np.asarray(command[: len(ARM_JOINTS)], dtype=float)
            if len(command) > len(ARM_JOINTS):
                self.set_gripper(command[len(ARM_JOINTS)])
        m, d = self.model, self.data
        mujoco.mj_fullM(m, self.mass, d.qM)  # type: ignore[attr-defined]
        mass = self.mass[np.ix_(self.arm_dof, self.arm_dof)]
        error = self.target - d.qpos[self.arm_qpos]
        torque = mass @ (KP * error - KD * d.qvel[self.arm_dof]) + d.qfrc_bias[self.arm_dof]
        d.ctrl[self.arm_act] = np.clip(torque, self.torque_range[:, 0], self.torque_range[:, 1])
        mujoco.mj_step(m, d)

    def joint_state(self) -> tuple[list[str], list[float], list[float], list[float]]:
        d = self.data
        pos = [float(d.qpos[a]) for a in self.arm_qpos]
        vel = [float(d.qvel[a]) for a in self.arm_dof]
        eff = [float(d.qfrc_actuator[a]) for a in self.arm_dof]
        pos.append(self.gripper_opening())
        vel.append(0.0)
        eff.append(0.0)
        return [*ARM_JOINTS, GRIPPER_JOINT], pos, vel, eff

    def camera_transforms(self, ts: float) -> list[TransformStamped]:
        """link7 -> wrist camera link and optical frames, as MujocoSimModule publishes them."""
        d = self.data
        base_rot = d.xmat[self.link7].reshape(3, 3)
        base_pos = d.xpos[self.link7]
        cam_rot = d.cam_xmat[self.camera].reshape(3, 3)
        cam_pos = d.cam_xpos[self.camera]
        optical_rot = cam_rot @ np.diag([1.0, -1.0, -1.0])  # Rx(180): GL camera -> optical
        rel_pos = base_rot.T @ (cam_pos - base_pos)
        link = quat_mat(base_rot.T @ cam_rot)
        optical = quat_mat(base_rot.T @ optical_rot)
        return [
            transform("link7", f"{CAMERA}_color_optical_frame", rel_pos, optical, ts),
            transform("link7", f"{CAMERA}_depth_optical_frame", rel_pos, optical, ts),
            transform("link7", f"{CAMERA}_link", rel_pos, link, ts),
        ]

    def object_transforms(self, ts: float) -> list[TransformStamped]:
        d = self.data
        return [
            transform("world", name, d.xpos[body] + self.shift, d.xquat[body], ts)
            for name, body in self.objects.items()
        ]

    def status(self) -> dict[str, Any]:
        """LIBERO's goal check on the current state: each predicate, and all of them."""
        predicates = [
            [*state, bool(self.env._eval_predicate(state))] for state in self.problem["goal_state"]
        ]
        return {
            "success": all(p[-1] for p in predicates),
            "predicates": predicates,
            "language": " ".join(self.problem["language_instruction"]),
            "sim_time": float(self.data.time),
        }


def finger_targets(opening: float) -> tuple[float, float]:
    """Gripper opening (0 closed .. 0.08 open) -> the two finger servos, mirrored."""
    lo, hi = GRIPPER_RANGE
    half = min(max(opening, lo), hi) / 2.0
    return half, -half


def main() -> None:
    launch = json.loads(sys.stdin.readline())
    topics: dict[str, str] = launch["topics"]
    cfg: dict[str, Any] = launch.get("config") or {}
    session = launch.get("session") or {}
    log(f"topics: {sorted(topics)}")

    z = zenoh_session(session)
    pubs = {name: z.declare_publisher(key) for name, key in topics.items() if name != "sim_command"}

    def put(name: str, payload: bytes) -> None:
        pub = pubs.get(name)
        if pub is not None:
            pub.put(payload)

    sim = LiberoSim(cfg)

    def on_command(sample: Any) -> None:
        try:
            msg = LCMJointState.lcm_decode(bytes(sample.payload.to_bytes()))
            sim.set_command(list(msg.position))
        except Exception as exc:
            warn(f"bad sim_command: {exc}")

    if "sim_command" in topics:
        z.declare_subscriber(topics["sim_command"], on_command)

    width, height = int(cfg.get("width", 640)), int(cfg.get("height", 480))
    enable_depth = bool(cfg.get("enable_depth", True))
    color = mujoco.Renderer(sim.model, height, width)
    depth = mujoco.Renderer(sim.model, height, width) if enable_depth else None
    if depth is not None:
        depth.enable_depth_rendering()

    dt = float(sim.model.opt.timestep)
    frame_period = 1.0 / float(cfg.get("fps", 15.0))
    state_period = 1.0 / float(cfg.get("joint_state_hz", 100.0))
    info_period = 1.0 / float(cfg.get("camera_info_fps", 1.0))
    status_period = 1.0 / float(cfg.get("status_hz", 5.0))
    next_frame = next_state = next_info = next_status = 0.0
    optical = f"{CAMERA}_color_optical_frame"
    last_success: bool | None = None

    start = time.time()
    steps = 0
    next_report, report_sim, report_wall = time.time() + 30.0, 0.0, time.time()
    timings = {"step": 0.0, "render": 0.0, "publish": 0.0}
    while True:
        t0 = time.perf_counter()
        sim.step()
        timings["step"] += time.perf_counter() - t0
        steps += 1
        now = time.time()
        if now >= next_report:
            sim_s = float(sim.data.time) - report_sim
            wall_s = now - report_wall
            log(
                f"real-time factor {sim_s / wall_s:.2f} "
                + " ".join(f"{k}={v / wall_s * 100:.0f}%" for k, v in timings.items())
            )
            next_report, report_sim, report_wall = now + 30.0, float(sim.data.time), now
            timings = dict.fromkeys(timings, 0.0)
        if now >= next_state:
            next_state = now + state_period
            put("sim_state", joint_state_msg(*sim.joint_state(), now))
        if now >= next_frame:
            next_frame = now + frame_period
            t0 = time.perf_counter()
            color.update_scene(sim.data, camera=sim.camera)
            rgb = color.render()
            depth_m = None
            if depth is not None:
                depth.update_scene(sim.data, camera=sim.camera)
                depth_m = depth.render().astype(np.float32)
            t1 = time.perf_counter()
            put("color_image", image_msg(rgb, "rgb8", optical, now))
            if depth_m is not None:
                put("depth_image", image_msg(depth_m, "32FC1", optical, now))
            put("tf", tf_msg(sim.camera_transforms(now) + sim.object_transforms(now)))
            timings["render"] += t1 - t0
            timings["publish"] += time.perf_counter() - t1
        if now >= next_info:
            next_info = now + info_period
            info = camera_info_msg(width, height, sim.fovy, optical, now)
            put("camera_info", info)
            put("depth_camera_info", info)
        if now >= next_status:
            next_status = now + status_period
            status = sim.status()
            put("task_status", string_msg(json.dumps(status)))
            if status["success"] != last_success:
                log(f"success={status['success']} {status['predicates']}")
                last_success = status["success"]
        # Real time: sleep off whatever the step budget has left.
        lag = start + steps * dt - time.time()
        if lag > 0:
            time.sleep(lag)
        elif lag < -1.0:
            start, steps = time.time(), 0  # fell behind (rendering); do not try to catch up


if __name__ == "__main__":
    main()
