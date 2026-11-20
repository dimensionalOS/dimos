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

"""The dimos side of the RoboLab bridge's wire format (see runner/dimos_client.py).

msgpack over zenoh, under one key prefix:
    state   RoboLab -> dimos, each sim step and every 0.2 s while paused: joints, gripper
    camera  RoboLab -> dimos, with each state: one RGB-D frame, its intrinsics and pose
    status  RoboLab -> dimos, episode start and end; never the task's success
    cmd     dimos -> RoboLab, {"seq", "q": [7], "gripper": 0..1} or {"release": true}

The runner keeps its own copy of these names: it runs in RoboLab's Python, without dimos.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
from typing import Any

import msgpack  # type: ignore[import-untyped]
import numpy as np
from numpy.typing import NDArray
import zenoh

DEFAULT_ENDPOINT = "tcp/127.0.0.1:7448"
DEFAULT_PREFIX = "robolab"
ARM_DOF = 7
# RoboLab's Robotiq finger_joint: 0 open, pi/4 closed (rad).
GRIPPER_CLOSED_RAD = float(np.pi / 4)


def open_session(endpoint: str) -> zenoh.Session:
    """A zenoh session that reaches RoboLab's runner at ``endpoint`` only.

    RoboLab may still be starting Isaac: the session opens at once and keeps trying to
    connect in the background.
    """
    config = zenoh.Config()
    config.insert_json5("scouting/multicast/enabled", "false")
    config.insert_json5("mode", json.dumps("peer"))
    config.insert_json5("connect/endpoints", json.dumps([endpoint]))
    config.insert_json5("connect/timeout_ms", "0")
    config.insert_json5("connect/exit_on_failure", "false")
    return zenoh.open(config)


def unpack(sample: zenoh.Sample) -> dict[str, Any]:
    message: dict[str, Any] = msgpack.unpackb(sample.payload.to_bytes())
    return message


def pack_command(seq: int, q: list[float], gripper: float) -> bytes:
    payload: bytes = msgpack.packb({"seq": seq, "q": list(q), "gripper": float(gripper)})
    return payload


def pack_release() -> bytes:
    payload: bytes = msgpack.packb({"release": True})
    return payload


@dataclass(frozen=True)
class RobotState:
    step: int
    sim_time: float
    paused: bool
    seq: int
    q: NDArray[np.float64]
    gripper: float  # 0 open .. 1 closed, as RoboLab measures it

    @classmethod
    def decode(cls, message: dict[str, Any]) -> RobotState:
        return cls(
            step=int(message["step"]),
            sim_time=float(message["sim_time"]),
            paused=bool(message["paused"]),
            seq=int(message["seq"]),
            q=np.asarray(message["q"], dtype=np.float64)[:ARM_DOF],
            gripper=float(message["gripper"]),
        )


@dataclass(frozen=True)
class CameraFrame:
    step: int
    sim_time: float
    camera: str
    rgb: NDArray[np.uint8]  # H x W x 3
    depth: NDArray[np.float32]  # H x W, metres; inf where nothing is in range
    K: NDArray[np.float64]  # 3 x 3
    # Camera pose in RoboLab's world, OpenGL convention: looking down -z, y up.
    position: NDArray[np.float64]
    quat_xyzw: NDArray[np.float64]

    @classmethod
    def decode(cls, message: dict[str, Any]) -> CameraFrame:
        rgb = np.frombuffer(message["rgb"]["data"], np.uint8).reshape(message["rgb"]["shape"])
        depth = np.frombuffer(message["depth"]["data"], np.float32).reshape(
            message["depth"]["shape"]
        )
        w, x, y, z = message["cam_quat"]
        return cls(
            step=int(message["step"]),
            sim_time=float(message["sim_time"]),
            camera=str(message["camera"]),
            rgb=rgb,
            depth=depth.reshape(depth.shape[0], depth.shape[1]),
            K=np.asarray(message["K"], dtype=np.float64).reshape(3, 3),
            position=np.asarray(message["cam_pos"], dtype=np.float64),
            quat_xyzw=np.array([x, y, z, w], dtype=np.float64),
        )
