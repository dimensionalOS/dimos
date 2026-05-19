# Copyright 2025-2026 Dimensional Inc.
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

"""MuJoCo-backed Go2 sim connection.

Sibling to ``dimos.robot.unitree.g1.sim:G1SimConnection``. Swaps the WebRTC
transport in ``GO2Connection`` for the existing ``MujocoConnection`` so
sim blueprints expose the same topics (`cmd_vel`, `odom`, `lidar`,
`color_image`, `camera_info`) the real-hardware blueprint does.

``dimos/simulation/mujoco/mujoco_process.py`` already accepts
``robot_model="unitree_go2"`` (it currently maps internally to the
``unitree_go1`` MuJoCo asset; same kinematics). Setting
``global_config.robot_model="unitree_go2"`` before construction picks the
right scene.

Example::

    from dimos.core.blueprints import autoconnect
    from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import with_vis
    from dimos.robot.unitree.go2.sim import go2_sim_connection

    unitree_go2_basic_sim = (
        autoconnect(with_vis, go2_sim_connection())
        .global_config(robot_model="unitree_go2")
    )
"""

from __future__ import annotations

import threading
import time
from threading import Thread
from typing import TYPE_CHECKING, Any

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.global_config import GlobalConfig, global_config
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs import (
    PoseStamped,
    Quaternion,
    Transform,
    Twist,
    Vector3,
)
from dimos.msgs.sensor_msgs import CameraInfo, Image, PointCloud2
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.robot.unitree.type.odometry import Odometry as SimOdometry
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.robot.unitree.mujoco_connection import MujocoConnection

logger = setup_logger()


class Go2SimConnection(GO2Connection):
    """MuJoCo-backed Go2 connection.

    Same In/Out interface as :class:`GO2Connection` so the rest of the
    Go2 blueprint stack (perception, planner, viz, navigation) is
    unchanged when swapping real hardware for sim. Internally we delegate
    to :class:`MujocoConnection`, which spawns a MuJoCo process running
    the Go1 model (Go2 shares kinematics with Go1 in the upstream sim
    assets).

    The ``GO2Connection.__init__`` is intentionally bypassed because it
    opens a WebRTC client — there's no real robot to connect to here.
    """

    cmd_vel: In[Twist]
    lidar: Out[PointCloud2]
    odom: Out[PoseStamped]
    color_image: Out[Image]
    camera_info: Out[CameraInfo]

    ip: str | None
    _global_config: GlobalConfig
    _camera_info_thread: Thread | None = None

    def __init__(
        self,
        ip: str | None = None,
        cfg: GlobalConfig = global_config,
        *args: Any,
        **kwargs: Any,
    ) -> None:
        self._global_config = cfg
        # Make sure MuJoCo brings up the Go2 (mapped to go1 internally).
        if not getattr(cfg, "robot_model", None):
            try:
                cfg.robot_model = "unitree_go2"
            except Exception:
                # global_config may be frozen — ignore; the sim will fall
                # back to its own default (currently go1, same asset).
                pass
        self.ip = ip if ip is not None else getattr(self._global_config, "robot_ip", None)
        self.connection: MujocoConnection | None = None
        self._stop_event = threading.Event()
        # Skip GO2Connection.__init__ (WebRTC handshake). Go straight to Module.
        Module.__init__(self, *args, **kwargs)

    @rpc
    def start(self) -> None:
        # Bypass GO2Connection.start (WebRTC handshake) — Module.start
        # is the only thing we need from upstream.
        Module.start(self)

        from dimos.robot.unitree.mujoco_connection import MujocoConnection

        self.connection = MujocoConnection(self._global_config)
        assert self.connection is not None
        self.connection.start()

        self._disposables.add(Disposable(self.cmd_vel.subscribe(self.move)))
        self._disposables.add(self.connection.odom_stream().subscribe(self._publish_sim_odom))
        self._disposables.add(self.connection.lidar_stream().subscribe(self.lidar.publish))
        self._disposables.add(self.connection.video_stream().subscribe(self.color_image.publish))

        self._camera_info_thread = Thread(
            target=self._publish_camera_info_loop,
            name="go2-sim-camera-info",
            daemon=True,
        )
        self._camera_info_thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self.connection is not None:
            self.connection.stop()
        if self._camera_info_thread and self._camera_info_thread.is_alive():
            self._camera_info_thread.join(timeout=1.0)
        Module.stop(self)

    def _publish_camera_info_loop(self) -> None:
        assert self.connection is not None
        info = self.connection.camera_info_static
        while not self._stop_event.is_set():
            self.camera_info.publish(info)
            self._stop_event.wait(1.0)

    def _publish_tf(self, msg: PoseStamped) -> None:
        self.odom.publish(msg)

        self.tf.publish(Transform.from_pose("base_link", msg))

        # camera_link / camera_optical chain so any blueprint that
        # renders the lidar pointcloud at base_link sees the right
        # camera placement.
        camera_link = Transform(
            translation=Vector3(0.05, 0.0, 0.4),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id="base_link",
            child_frame_id="camera_link",
            ts=time.time(),
        )
        camera_optical = Transform(
            translation=Vector3(0.0, 0.0, 0.0),
            rotation=Quaternion(-0.5, 0.5, -0.5, 0.5),
            frame_id="camera_link",
            child_frame_id="camera_optical",
            ts=time.time(),
        )
        map_to_world = Transform(
            translation=Vector3(0.0, 0.0, 0.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id="map",
            child_frame_id="world",
            ts=time.time(),
        )
        self.tf.publish(camera_link, camera_optical, map_to_world)

    def _publish_sim_odom(self, msg: SimOdometry) -> None:
        self._publish_tf(
            PoseStamped(
                ts=msg.ts,
                frame_id=msg.frame_id,
                position=msg.position,
                orientation=msg.orientation,
            )
        )

    @rpc
    def move(self, twist: Twist, duration: float = 0.0) -> None:
        assert self.connection is not None
        self.connection.move(twist, duration)

    @rpc
    def publish_request(self, topic: str, data: dict[str, Any]) -> dict[Any, Any]:
        logger.info(f"Publishing request to topic: {topic} with data: {data}")
        assert self.connection is not None
        return self.connection.publish_request(topic, data)


go2_sim_connection = Go2SimConnection.blueprint


__all__ = ["Go2SimConnection", "go2_sim_connection"]
