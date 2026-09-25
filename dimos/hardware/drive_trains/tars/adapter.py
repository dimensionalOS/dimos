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

"""TwistBaseAdapter for the TARS slab robot via tars_sdk (MuJoCo simulation).

Install the SDK into the dimos venv first:
    uv pip install -e experimental/tars_sdk
"""

from __future__ import annotations

from pathlib import Path
import threading
import time
from typing import Any

from tars_sdk import TarsClient

from dimos.utils.data import get_data
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def _resolve_scene(scene: str | None) -> Path | None:
    """dimos scene name ("office1") -> data/mujoco_sim/scene_office1.xml, or a path as-is."""
    if scene is None:
        return None
    if scene.endswith(".xml") or "/" in scene:
        return Path(scene)
    return get_data("mujoco_sim") / f"scene_{scene}.xml"


class TarsTwistAdapter:
    """3 DOF velocity [vx, vy, wz] for TARS.

    TARS cannot strafe: vy commands are ignored (a warning is logged once) and vy reads
    back as the measured sideways drift. `write_enable(True)` stands the robot up,
    `write_enable(False)` stops and sits it down.

    Odometry defaults to the SDK's leg-odometry estimate; pass odom_source="ground_truth"
    to read the simulator's true pose instead.

    scene: a dimos MuJoCo scene name ("office1" -> data/mujoco_sim/scene_office1.xml, the
    room the Go2 sim uses) or a path to any scene MJCF; None = empty floor.
    viewer: open a MuJoCo window (separate process, safe from a worker thread / macOS).
    """

    def __init__(
        self,
        dof: int = 3,
        realtime: bool = True,
        odom_source: str = "estimated",
        cmd_timeout: float = 0.5,
        scene: str | None = None,
        spawn: tuple[float, float, float] = (0.0, 0.0, 0.0),
        viewer: bool = False,
        **_: Any,
    ) -> None:
        if dof != 3:
            raise ValueError(f"TARS supports 3 DOF (vx, vy, wz), got {dof}")
        if odom_source not in ("estimated", "ground_truth"):
            raise ValueError(
                f"odom_source must be 'estimated' or 'ground_truth', got {odom_source!r}"
            )
        self._realtime = realtime
        self._odom_gt = odom_source == "ground_truth"
        self._cmd_timeout = cmd_timeout
        self._scene = scene
        self._spawn = (float(spawn[0]), float(spawn[1]), float(spawn[2]))
        self._viewer = viewer
        self._client: TarsClient | None = None
        self._enabled = False
        self._warned_vy = False
        self._lock = threading.Lock()

    @property
    def client(self) -> TarsClient:
        """Underlying SDK client (for sim stepping, camera, joint state)."""
        if self._client is None:
            raise RuntimeError("TARS adapter not connected")
        return self._client

    def connect(self) -> bool:
        with self._lock:
            if self._client is not None:
                return True
            try:
                client = TarsClient(
                    realtime=self._realtime,
                    cmd_timeout=self._cmd_timeout,
                    scene=_resolve_scene(self._scene),
                    spawn=self._spawn,
                    viewer=self._viewer,
                )
                client.connect()
            except Exception as e:
                logger.error(f"[TARS] Failed to start: {e}")
                return False
            self._client = client
            logger.info(f"[TARS] Connected (MuJoCo sim, scene={self._scene or 'empty'})")
            return True

    def disconnect(self) -> None:
        with self._lock:
            if self._client is None:
                return
            self._client.disconnect()
            self._client = None
            self._enabled = False

    def is_connected(self) -> bool:
        return self._client is not None

    def get_dof(self) -> int:
        return 3

    def read_velocities(self) -> list[float]:
        """Measured body-frame [vx, vy, wz]."""
        if self._client is None:
            return [0.0, 0.0, 0.0]
        o = self._client.get_odometry(ground_truth=self._odom_gt)
        return [o.vx, o.vy, o.wz]

    def read_odometry(self) -> list[float] | None:
        """Pose [x, y, yaw] in the odom frame (start pose = origin)."""
        if self._client is None:
            return None
        o = self._client.get_odometry(ground_truth=self._odom_gt)
        return [o.x, o.y, o.yaw]

    def write_velocities(self, velocities: list[float]) -> bool:
        if len(velocities) != 3 or self._client is None or not self._enabled:
            return False
        vx, vy, wz = velocities
        if vy != 0.0 and not self._warned_vy:
            logger.warning("[TARS] vy is not supported (TARS cannot strafe); ignoring it")
            self._warned_vy = True
        self._client.move(vx, wz)
        return True

    def write_stop(self) -> bool:
        if self._client is None:
            return False
        self._client.stop()
        return True

    def write_enable(self, enable: bool) -> bool:
        if self._client is None:
            return False
        if enable:
            self._client.stand()
        else:
            self._client.stop()
            self._client.sit()
        self._enabled = enable
        return True

    def read_enabled(self) -> bool:
        return self._enabled

    def wait_until_ready(self, timeout: float = 5.0) -> bool:
        """Block until the robot has stood up and accepts motion."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.client.get_state().mode not in ("sit", "rise"):
                return True
            if self._realtime:
                time.sleep(0.02)
            else:
                self.client.step(0.02)
        return False
