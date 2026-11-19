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

"""RoboLab's side of the dimos bridge: a RoboLab InferenceClient that dimos drives.

Runs in RoboLab's own Python (Isaac Sim), so it imports nothing from dimos. Each step it
publishes one camera's RGB-D frame and the robot's state, and steps with the latest joint
and gripper target dimos sent. The sim only advances while the robot has somewhere to go:
when the arm has reached its target and settled, the client blocks until a new target
arrives, so time spent thinking on the dimos side does not count against the episode.

Wire format: msgpack over zenoh, under ``<prefix>/``:
    obs     RoboLab -> dimos, one per step (and repeated while paused)
    status  RoboLab -> dimos, episode start and end; never the task's success
    cmd     dimos -> RoboLab, {"seq", "q": [7], "gripper": 0..1} or {"release": true}
"""

from __future__ import annotations

import json
import threading
import time
from typing import Any

import msgpack
import numpy as np
import zenoh
from robolab.eval.base_client import InferenceClient

ARM_DOF = 7


def open_session(listen: str | None, connect: str | None) -> zenoh.Session:
    config = zenoh.Config()
    config.insert_json5("scouting/multicast/enabled", "false")
    if listen:
        config.insert_json5("listen/endpoints", json.dumps([listen]))
    if connect:
        config.insert_json5("connect/endpoints", json.dumps([connect]))
    return zenoh.open(config)


class DimosClient(InferenceClient):
    """Hands each observation to dimos and steps with the target dimos last sent."""

    open_loop_horizon = 1

    def __init__(
        self,
        session: zenoh.Session,
        prefix: str = "robolab",
        camera: str = "over_shoulder_left_camera",
        control_hz: float = 15.0,
        settle_steps: int = 15,
        joint_tolerance: float = 0.01,
        paused_publish_s: float = 0.2,
    ) -> None:
        super().__init__()
        self._camera = camera
        self._period = 1.0 / control_hz
        self._settle_steps = settle_steps
        self._joint_tolerance = joint_tolerance
        self._paused_publish_s = paused_publish_s
        self._obs_pub = session.declare_publisher(f"{prefix}/obs")
        self._status_pub = session.declare_publisher(f"{prefix}/status")
        self._cmd_sub = session.declare_subscriber(f"{prefix}/cmd", self._on_cmd)
        self._lock = threading.Lock()
        self._new_cmd = threading.Event()
        self._cmd: dict[str, Any] | None = None
        self._start_episode_state()

    def _start_episode_state(self) -> None:
        self._target: np.ndarray | None = None
        self._gripper = 0.0
        self._seq = -1
        self._step = 0
        self._steps_since_change = 0
        self._released = False
        self._last_step_wall = 0.0

    def _on_cmd(self, sample: zenoh.Sample) -> None:
        cmd = msgpack.unpackb(sample.payload.to_bytes())
        with self._lock:
            self._cmd = cmd
        self._new_cmd.set()

    def _publish_status(self, **fields: Any) -> None:
        self._status_pub.put(msgpack.packb({"wall_time": time.time(), **fields}))

    def begin_episode(self, episode_idx: int) -> None:
        super().begin_episode(episode_idx)
        self._start_episode_state()
        self._publish_status(event="episode_start", episode=episode_idx)

    def reset(self, *, env_id: int | None = None) -> None:
        self._publish_status(event="episode_end", episode=self._eval_episode_idx, steps=self._step)
        super().reset(env_id=env_id)

    def _extract_observation(self, raw_obs: Any, *, env_id: int = 0) -> dict:
        image = raw_obs["image_obs"]
        proprio = raw_obs["proprio_obs"]
        cam = self._camera
        return {
            "rgb": self._to_numpy(image[cam], env_id).astype(np.uint8),
            "depth": self._to_numpy(image[f"{cam}_depth"], env_id).astype(np.float32),
            "K": self._to_numpy(image[f"{cam}_K"], env_id).astype(np.float64),
            "cam_pos": self._to_numpy(image[f"{cam}_pos"], env_id).astype(np.float64),
            "cam_quat": self._to_numpy(image[f"{cam}_quat"], env_id).astype(np.float64),
            "q": self._to_numpy(proprio["arm_joint_pos"], env_id).astype(np.float64),
            "gripper": float(self._to_numpy(proprio["gripper_pos"], env_id).reshape(-1)[0]),
            "ee_pos": self._to_numpy(proprio["ee_pos"], env_id).astype(np.float64),
            "ee_quat": self._to_numpy(proprio["ee_quat"], env_id).astype(np.float64),
        }

    def _publish_obs(self, obs: dict, paused: bool) -> None:
        msg = {
            "step": self._step,
            "sim_time": self._step * self._period,
            "paused": paused,
            "seq": self._seq,
            "rgb": {"shape": obs["rgb"].shape, "data": obs["rgb"].tobytes()},
            "depth": {"shape": obs["depth"].shape, "data": obs["depth"].tobytes()},
            **{k: obs[k].reshape(-1).tolist() for k in ("K", "cam_pos", "cam_quat", "q", "ee_pos", "ee_quat")},
            "gripper": obs["gripper"],
        }
        self._obs_pub.put(msgpack.packb(msg))

    def _take_cmd(self) -> bool:
        """Apply the newest command; True if it moved the target."""
        with self._lock:
            cmd, self._cmd = self._cmd, None
            self._new_cmd.clear()
        if cmd is None:
            return False
        if cmd.get("release"):
            self._released = True
            return True
        if int(cmd.get("seq", 0)) <= self._seq:
            return False
        self._seq = int(cmd["seq"])
        target = np.asarray(cmd["q"], dtype=np.float64)[:ARM_DOF]
        gripper = float(cmd.get("gripper", self._gripper))
        changed = (
            self._target is None
            or not np.allclose(target, self._target, atol=1e-6)
            or gripper != self._gripper
        )
        self._target, self._gripper = target, gripper
        return changed

    def _settled(self, q: np.ndarray) -> bool:
        assert self._target is not None
        at_target = float(np.max(np.abs(q[:ARM_DOF] - self._target))) < self._joint_tolerance
        return at_target and self._steps_since_change >= self._settle_steps

    def infer(self, obs: Any, instruction: Any, *, env_id: int = 0) -> dict:
        extracted = self._extract_observation(obs, env_id=env_id)
        q = extracted["q"].reshape(-1)
        if self._target is None:
            self._target = q[:ARM_DOF].copy()
        if self._take_cmd():
            self._steps_since_change = 0
        # Paused: nothing left to move, so wait for dimos, showing it the same frame.
        while not self._released and self._settled(q):
            self._publish_obs(extracted, paused=True)
            if self._new_cmd.wait(self._paused_publish_s) and self._take_cmd():
                self._steps_since_change = 0
        self._publish_obs(extracted, paused=False)
        # Pace the sim to the wall clock, so dimos's trajectory timing holds.
        delay = self._last_step_wall + self._period - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        self._last_step_wall = time.monotonic()
        self._step += 1
        self._steps_since_change += 1
        action = np.concatenate([self._target, [1.0 if self._gripper > 0.5 else 0.0]])
        return {"action": action.astype(np.float32), "viz": None}

    # The base's query-then-chunk flow is replaced by infer above.
    def _pack_request(self, extracted_obs: dict, instruction: Any) -> Any:
        raise NotImplementedError

    def _query_server(self, request: Any) -> Any:
        raise NotImplementedError

    def _unpack_response(self, response: Any) -> np.ndarray:
        raise NotImplementedError
