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

"""Booster K1 humanoid connection module (built on the booster-rpc SDK).

Scope: the K1 over booster-rpc exposes a camera (JPEG over WebSocket) and base
velocity control (+ stand/sit mode changes). It has no world-frame odometry or
lidar, so this connection implements only the `Camera` spec: no `odom`/`lidar`/
`pointcloud` ports, and therefore no mapping/navigation tier.
"""

from threading import Event, Thread
from typing import Any

from pydantic import Field
from reactivex.disposable import Disposable
import rerun.blueprint as rrb

from dimos.agents.annotation import skill
from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.global_config import GlobalConfig
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.booster.booster_rpc import BoosterRPCConnection
from dimos.spec.perception import Camera
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# K1 camera intrinsics — placeholders until a measured calibration replaces them.
CAMERA_WIDTH = 544
CAMERA_HEIGHT = 306
CAMERA_FX = 400.0
CAMERA_FY = 400.0
CAMERA_CX = 272.0
CAMERA_CY = 153.0
CAMERA_INFO_REPUBLISH_S = 1.0  # re-emit static intrinsics on a timer for late subscribers


class ConnectionConfig(ModuleConfig):
    ip: str = Field(default_factory=lambda m: m["g"].robot_ip)


def make_connection(ip: str, cfg: GlobalConfig) -> BoosterRPCConnection:
    # Real hardware only; add sim/replay branches (keyed off cfg) here when they exist.
    return BoosterRPCConnection(ip)


def _camera_info_static() -> CameraInfo:
    # TODO: replace with measured K1 camera intrinsics (these are placeholders).
    return CameraInfo(
        frame_id="camera_optical",
        height=CAMERA_HEIGHT,
        width=CAMERA_WIDTH,
        distortion_model="plumb_bob",
        D=[0.0, 0.0, 0.0, 0.0, 0.0],
        K=[CAMERA_FX, 0.0, CAMERA_CX, 0.0, CAMERA_FY, CAMERA_CY, 0.0, 0.0, 1.0],
        R=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        P=[CAMERA_FX, 0.0, CAMERA_CX, 0.0, 0.0, CAMERA_FY, CAMERA_CY, 0.0, 0.0, 0.0, 1.0, 0.0],
        binning_x=0,
        binning_y=0,
    )


class K1Connection(Module, Camera):
    """Booster K1 humanoid: exposes camera + velocity control as DimOS streams/RPCs."""

    dedicated_worker = True

    config: ConnectionConfig

    # input: velocity command from MovementManager / teleop
    cmd_vel: In[Twist]
    # outputs: the Camera spec (color_image + camera_info)
    color_image: Out[Image]
    camera_info: Out[CameraInfo]

    camera_info_static: CameraInfo = _camera_info_static()
    _latest_frame: Image | None = None
    _camera_info_thread: Thread | None = None

    @classmethod
    def rerun_views(cls):  # type: ignore[no-untyped-def]
        """Rerun view blueprint for the K1 camera."""
        return [
            rrb.Spatial2DView(name="Camera", origin="world/robot/camera/rgb"),
        ]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._stop_event = Event()
        self.hw = make_connection(self.config.ip, self.config.g)

    @rpc
    def start(self) -> None:
        super().start()
        self.hw.start()

        def on_image(image: Image) -> None:  # publish AND cache for observe()
            self.color_image.publish(image)
            self._latest_frame = image

        self.register_disposable(self.hw.camera_stream().subscribe(on_image))
        self.register_disposable(Disposable(self.cmd_vel.subscribe(self.move)))

        logger.warning(
            "K1 camera intrinsics are placeholders; 3D projection/perception will be "
            "inaccurate until replaced with a measured calibration."
        )
        self._camera_info_thread = Thread(target=self._publish_camera_info, daemon=True)
        self._camera_info_thread.start()

        # Arm the robot so it accepts velocity commands.
        self.standup()
        logger.info("K1Connection started (ip=%s)", self.config.ip)

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._camera_info_thread and self._camera_info_thread.is_alive():
            self._camera_info_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        self.hw.stop()
        super().stop()

    def _publish_camera_info(self) -> None:
        while not self._stop_event.is_set():
            self.camera_info.publish(self.camera_info_static)
            self._stop_event.wait(CAMERA_INFO_REPUBLISH_S)

    @rpc
    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        """Send a base velocity command to the robot."""
        return self.hw.move(twist, duration)

    @rpc
    def standup(self) -> bool:
        """Arm the robot for walking (DAMPING -> PREPARE -> WALKING)."""
        return self.hw.standup()

    @rpc
    def sit(self) -> bool:
        """Make the robot lie down."""
        return self.hw.sit()

    @skill
    def walk(self, x: float, y: float = 0.0, yaw: float = 0.0, duration: float = 0.0) -> str:
        """Walk at the given velocity for `duration` seconds, then stop (blocks until stopped).

        A positive `duration` is required; pick it from the distance and speed.

        Args:
            x: Forward velocity (m/s)
            y: Left/right velocity (m/s)
            yaw: Rotational velocity (rad/s)
            duration: How long to move (seconds); must be > 0
        """
        if duration <= 0:
            return "Specify a positive duration (seconds); compute it from the distance and speed."
        twist = Twist(linear=Vector3(x, y, 0.0), angular=Vector3(0.0, 0.0, yaw))
        if not self.move(twist, duration=duration):
            return "Failed to move."
        return f"Moved at velocity=({x}, {y}, {yaw}) for {duration}s then stopped."

    @skill
    def stand(self) -> str:
        """Make the robot stand up from a sitting or damping position."""
        return "Robot is now standing." if self.standup() else "Failed to stand up."

    @skill
    def liedown(self) -> str:
        """Make the robot lie down."""
        return "Robot is now sitting." if self.sit() else "Failed to sit down."

    @skill
    def observe(self) -> Image | None:
        """Returns the latest camera frame. Use this for any visual world queries.

        Returns None if no frame has been captured yet.
        """
        return self._latest_frame
