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

"""Plays an episode back: its body motion in a MuJoCo viewer, its replay file in Rerun."""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass
import json
from pathlib import Path
import subprocess
import time

import mujoco
import numpy as np
from numpy.typing import NDArray

from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.navigation.bench.driver import TERMINAL_FILE
from dimos.navigation.bench.runner import RECORDING_FILE, REPLAY_FILE, RUN_FILE
from dimos.navigation.bench.suite import Manifest
from dimos.simulation.go2_legged.robot import BASE_QUAT
from dimos.simulation.go2_sim.world import build_model, open_viewer
from dimos.simulation.scenes.procedural import Scene


@dataclass(frozen=True)
class Frame:
    """The body at one recorded instant."""

    t: float
    position: NDArray[np.float64]
    quaternion: NDArray[np.float64]
    joints: dict[str, float]


def frames(recording: Path) -> list[Frame]:
    """Recorded body poses paired with the joint state nearest in time."""
    store = SqliteStore(path=str(recording))
    store.start()
    try:
        poses = store.stream("ground_truth", PoseStamped).to_list()
        joints = store.stream("joint_state", JointState).to_list()
        if not poses or not joints:
            raise ValueError(f"{recording} holds no ground_truth and joint_state streams")
        joint_t = np.array([float(j.ts) for j in joints])
        out = []
        for pose in poses:
            t = float(pose.ts)
            k = int(np.clip(np.searchsorted(joint_t, t), 0, len(joints) - 1))
            if k > 0 and abs(joint_t[k - 1] - t) < abs(joint_t[k] - t):
                k -= 1
            state = joints[k].data
            q = pose.data.orientation
            out.append(
                Frame(
                    t,
                    np.array(tuple(pose.data.position), dtype=np.float64),
                    np.array([q.w, q.x, q.y, q.z], dtype=np.float64),
                    dict(zip(state.name, state.position, strict=True)),
                )
            )
    finally:
        store.stop()
    return out


class Puppet:
    """The scene's model posed from frames, with physics left out."""

    def __init__(self, scene: Scene) -> None:
        self.model = build_model(scene)
        self.data = mujoco.MjData(self.model)
        self._trunk = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "base")
        self._qpos_adr = {
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, j): int(
                self.model.jnt_qposadr[j]
            )
            for j in range(self.model.njnt)
            if self.model.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE
        }

    def pose(self, frame: Frame) -> None:
        self.data.qpos[0:3] = frame.position
        self.data.qpos[BASE_QUAT] = frame.quaternion
        for name, angle in frame.joints.items():
            self.data.qpos[self._qpos_adr[name]] = angle
        mujoco.mj_forward(self.model, self.data)

    def trunk_position(self) -> NDArray[np.float64]:
        position: NDArray[np.float64] = self.data.xpos[self._trunk].copy()
        return position


def episode_scene(episode_dir: Path) -> Scene:
    """The scene of the case the episode ran, from the run's suite."""
    case_id = json.loads((episode_dir / TERMINAL_FILE).read_text())["case_id"]
    meta = json.loads((episode_dir.parent / RUN_FILE).read_text())
    manifest = Manifest.load(Path(meta["suite"]))
    case = next(c for c in manifest.cases if c.id == case_id)
    return case.scene()


def paced(items: list[Frame], speed: float) -> Iterator[Frame]:
    """Frames on the wall clock at the recorded pace times speed."""
    start, t0 = time.monotonic(), items[0].t
    for frame in items:
        delay = (frame.t - t0) / speed - (time.monotonic() - start)
        if delay > 0:
            time.sleep(delay)
        yield frame


def open_rerun(replay: Path) -> subprocess.Popen[bytes]:
    """The Rerun viewer on a replay file, our own build when it is installed."""
    for command in ("dimos-viewer", "rerun"):
        try:
            return subprocess.Popen(
                [command, str(replay)], stdin=subprocess.DEVNULL, start_new_session=True
            )
        except FileNotFoundError:
            continue
    raise FileNotFoundError("no rerun viewer on PATH")


def play(episode_dir: Path, speed: float = 1.0, loop: bool = True, rerun: bool = True) -> None:
    """Move the recorded body through the episode's scene, with its replay open in Rerun."""
    puppet = Puppet(episode_scene(episode_dir))
    recorded = frames(episode_dir / RECORDING_FILE)
    puppet.pose(recorded[0])
    replay = episode_dir / REPLAY_FILE
    if rerun and replay.exists():
        open_rerun(replay)
    viewer = open_viewer(puppet.model, puppet.data, puppet.trunk_position())
    try:
        while viewer.is_running():
            for frame in paced(recorded, speed):
                if not viewer.is_running():
                    return
                puppet.pose(frame)
                viewer.sync()
            if not loop:
                return
    finally:
        viewer.close()
