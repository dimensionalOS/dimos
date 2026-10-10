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

"""TARS robot connection (MuJoCo sim via tars_sdk), Go2-style: owns the robot and
publishes its sensors as streams, takes cmd_vel.

Install the SDK into the dimos venv first:
    uv pip install -e experimental/tars_sdk
"""

from __future__ import annotations

from collections.abc import Callable
import math
import threading
import time
from typing import Any

import numpy as np
from numpy.typing import NDArray
from reactivex.disposable import Disposable
from tars_sdk import JOINT_NAMES, TarsClient
from tars_sdk.model.params import Params

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.hardware.drive_trains.tars.adapter import resolve_scene
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class TarsConnectionConfig(ModuleConfig):
    # dimos MuJoCo scene name ("office1" -> data/mujoco_sim/scene_office1.xml), an MJCF path,
    # or None for an empty floor
    scene: str | None = "office1"
    # (x, y, yaw) in the scene; the "world" frame has its origin here
    spawn: tuple[float, float, float] = (-4.5, 1.1, 0.0)
    mujoco_viewer: bool = True  # MuJoCo window (separate process)
    odom_hz: float = 50.0
    lidar_hz: float = 10.0
    camera: bool = True
    camera_hz: float = 5.0
    camera_size: tuple[int, int] = (320, 240)


class TarsConnection(Module):
    """TARS in MuJoCo. Odometry and TF come from simulator truth.

    The lidar rides on top of a slab and tilts with the gait. Its cloud is published in its
    own frame (lidar_link) with TF world -> base_link -> slab_N_upper -> lidar_link; every
    scan also publishes the TF of its exact instant, so a TF lookup at the scan timestamp
    registers it without interpolation error (see LidarRegistration). Timestamps are
    simulation time mapped onto the wall clock.
    """

    dedicated_worker = True

    config: TarsConnectionConfig
    cmd_vel: In[Twist]
    odom: Out[PoseStamped]
    lidar: Out[PointCloud2]
    color_image: Out[Image]
    tf: Out[TFMessage]

    _client: TarsClient | None = None

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._stop_event = threading.Event()
        self._threads: list[threading.Thread] = []
        p = Params().resolved()
        self._slab_frame = f"slab_{p.lidar_slab}_upper"
        self._slab_y = p.slab_y(p.lidar_slab)
        self._hinge = JOINT_NAMES.index(f"slab_{p.lidar_slab}_hinge")
        self._lidar_height = p.lidar_height
        self._clock_offset = 0.0

    @rpc
    def start(self) -> None:
        super().start()
        cfg = self.config
        client = TarsClient(
            realtime=True,
            scene=resolve_scene(cfg.scene),
            spawn=cfg.spawn,
            viewer=cfg.mujoco_viewer,
        )
        client.connect()
        client.stand()
        self._clock_offset = time.time() - client.get_state().time
        self._client = client
        logger.info(f"[TARS] Connected (MuJoCo sim, scene={cfg.scene or 'empty'})")

        self.register_disposable(Disposable(self.cmd_vel.subscribe(self.move)))
        self._stop_event.clear()
        loops: list[tuple[str, float, Callable[[], None]]] = [
            ("odom", cfg.odom_hz, self._publish_odom),
            ("lidar", cfg.lidar_hz, self._publish_lidar),
        ]
        if cfg.camera:
            loops.append(("camera", cfg.camera_hz, self._publish_camera))
        for name, hz, fn in loops:
            t = threading.Thread(
                target=self._run_at, args=(hz, fn), name=f"tars-{name}", daemon=True
            )
            t.start()
            self._threads.append(t)

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        for t in self._threads:
            t.join(timeout=2.0)
        self._threads.clear()
        if self._client is not None:
            self._client.disconnect()
            self._client = None
        super().stop()

    @rpc
    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        """Body-frame velocity (linear.x, angular.z); TARS cannot strafe."""
        if self._client is None:
            return False
        self._client.move(twist.linear.x, twist.angular.z)
        return True

    @rpc
    def stand(self) -> None:
        if self._client is not None:
            self._client.stand()

    @rpc
    def sit(self) -> None:
        if self._client is not None:
            self._client.stop()
            self._client.sit()

    def _run_at(self, hz: float, fn: Callable[[], None]) -> None:
        period = 1.0 / hz
        while not self._stop_event.is_set():
            start = time.monotonic()
            try:
                fn()
            except Exception:
                logger.exception(f"[TARS] {fn.__name__} failed")
            self._stop_event.wait(max(0.0, period - (time.monotonic() - start)))

    def _stamp(self, sim_time: float) -> float:
        return self._clock_offset + sim_time

    def _robot_tf(
        self,
        ts: float,
        base_position: NDArray[np.float64],
        base_quat: NDArray[np.float64],
        hinge: float,
    ) -> tuple[PoseStamped, TFMessage]:
        """odom pose + TF world -> base_link -> slab (hinge) -> lidar_link at one instant."""
        w, x, y, z = base_quat
        pose = PoseStamped(
            ts=ts,
            frame_id="world",
            position=base_position.tolist(),
            orientation=Quaternion(x, y, z, w),
        )
        tf = TFMessage(
            Transform.from_pose("base_link", pose),
            Transform(
                translation=Vector3(0.0, self._slab_y, 0.0),
                rotation=Quaternion(0.0, math.sin(hinge / 2), 0.0, math.cos(hinge / 2)),
                frame_id="base_link",
                child_frame_id=self._slab_frame,
                ts=ts,
            ),
            Transform(
                translation=Vector3(0.0, 0.0, self._lidar_height),
                rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
                frame_id=self._slab_frame,
                child_frame_id="lidar_link",
                ts=ts,
            ),
        )
        return pose, tf

    def _publish_odom(self) -> None:
        assert self._client is not None
        s = self._client.get_state()
        assert s.odom_gt is not None
        o = s.odom_gt
        pose, tf = self._robot_tf(
            self._stamp(s.time),
            np.array([o.x, o.y, o.z]),
            o.quat,
            float(s.measurement.joint_q[self._hinge]),
        )
        self.odom.publish(pose)
        self.tf.publish(tf)

    def _publish_lidar(self) -> None:
        assert self._client is not None
        scan = self._client.get_lidar()
        ts = self._stamp(scan.time)
        # TF of the scan instant first, so the lookup at `ts` finds it
        _, tf = self._robot_tf(
            ts, scan.base_position, scan.base_quat, float(scan.joint_q[self._hinge])
        )
        self.tf.publish(tf)
        self.lidar.publish(PointCloud2.from_numpy(scan.points, frame_id="lidar_link", timestamp=ts))

    def _publish_camera(self) -> None:
        assert self._client is not None
        w, h = self.config.camera_size
        frame = self._client.get_camera(w, h, depth=False)
        self.color_image.publish(
            Image.from_numpy(
                frame.rgb, format=ImageFormat.RGB, frame_id="camera", ts=self._stamp(frame.time)
            )
        )
