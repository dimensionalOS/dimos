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

"""Full Boston Dynamics Spot control: velocity driving plus sensor streaming.

`SpotHighLevel` is the single Spot hardware module. Over the one robot
connection it owns it: acquires the lease/E-stop, powers on and stands, drives
from a `cmd_vel` Twist stream, and streams every onboard camera plus body
odometry:

- `grayscale_image_1..5` — the five fisheye body cameras (front-left, front-right,
  left, right, back), in that order.
- `depth_image_1..5` — the matching depth cameras, same ordering.
- `odom` — body pose + velocity in Spot's `vision` frame, also broadcast on TF.

`bosdyn` is an optional extra (`uv sync --extra spot`); its imports live inside
methods so this file stays importable — and blueprint discovery keeps working —
on hosts where the SDK isn't installed.
"""

from __future__ import annotations

import asyncio
from collections.abc import AsyncIterator
from dataclasses import field
import time
from typing import Any

import numpy as np

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.robot.bosdyn.spot.config import (
    BODY_FRAME,
    DEPTH_SOURCES,
    GRAYSCALE_SOURCES,
    POWER_OFF_TIMEOUT_S,
    POWER_ON_TIMEOUT_S,
    SIT_TIMEOUT_S,
    STAND_TIMEOUT_S,
    VISION_FRAME,
    default_candidate_ips,
    resolve_credentials,
    resolve_ip,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class SpotHighLevelConfig(ModuleConfig):
    """Connection, credentials, safety gating, and sensor capture for a Spot."""

    # Explicit address always wins (`-o spothighlevel.ip=<addr>`). When left
    # blank, main() probes `candidate_ips` and uses the first that answers on the
    # API port — so plugging in over Ethernet or joining Spot's WiFi both work.
    ip: str = ""
    candidate_ips: list[str] = field(default_factory=default_candidate_ips)

    # Auth — required. Startup fails fast if either is missing.
    username: str | None = None
    password: str | None = None

    # Safety / startup gating.
    enable_estop: bool = True
    acquire_lease: bool = True
    power_on_at_start: bool = True
    stand_at_start: bool = True

    # Spot rejects velocity commands without an `end_time_secs`. This duration
    # is added to now() for each command and doubles as the auto-stop window:
    # if the cmd_vel stream stalls for longer than this, the robot halts.
    cmd_vel_timeout: float = 0.5

    # Spot E-stops itself if it doesn't see a keep-alive check-in within this
    # window. 9.0 s matches the bosdyn-client default.
    estop_timeout: float = 9.0

    # Which sources feed grayscale_image_N / depth_image_N (index N-1). Trim these
    # to capture fewer cameras.
    grayscale_sources: list[str] = field(default_factory=lambda: list(GRAYSCALE_SOURCES))
    depth_sources: list[str] = field(default_factory=lambda: list(DEPTH_SOURCES))

    # Clockwise rotation (degrees, multiple of 90) applied to a camera's
    # grayscale + depth before publishing, correcting for physically rotated
    # fisheye mounts. Keyed by 1-based camera index.
    image_rotations_cw: dict[int, int] = field(default_factory=lambda: {1: 90, 2: 90, 4: 180})

    image_rate_hz: float = 5.0
    odom_rate_hz: float = 20.0


class SpotHighLevel(Module):
    """Drives Spot and streams its fisheye cameras, depth cameras, and odometry."""

    # A hardware-driving module gets its own worker process, matching the other
    # robot connection modules (go2/b1/drone). Sharing a process with a GUI
    # module like KeyboardTeleop wedges this module's RPC server at startup.
    dedicated_worker = True

    cmd_vel: In[Twist]

    grayscale_image_1: Out[Image]
    grayscale_image_2: Out[Image]
    grayscale_image_3: Out[Image]
    grayscale_image_4: Out[Image]
    grayscale_image_5: Out[Image]

    depth_image_1: Out[Image]
    depth_image_2: Out[Image]
    depth_image_3: Out[Image]
    depth_image_4: Out[Image]
    depth_image_5: Out[Image]

    odom: Out[Odometry]

    config: SpotHighLevelConfig

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._robot: Any = None
        self._command_client: Any = None
        self._state_client: Any = None
        self._image_client: Any = None
        self._estop_keepalive: Any = None
        self._lease_keepalive: Any = None
        self._standing = False
        self._image_task: asyncio.Task[None] | None = None
        self._odom_task: asyncio.Task[None] | None = None
        # cmd_vel handlers wait on this so a velocity command issued mid-setup
        # never reaches a half-initialised SDK.
        self._ready = asyncio.Event()

    async def main(self) -> AsyncIterator[None]:
        username, password = resolve_credentials(self.config.username, self.config.password)
        ip = self.config.ip or await resolve_ip(self.config.candidate_ips)

        from bosdyn.client import create_standard_sdk  # type: ignore[import-not-found]
        from bosdyn.client.estop import (  # type: ignore[import-not-found]
            EstopClient,
            EstopEndpoint,
            EstopKeepAlive,
        )
        from bosdyn.client.image import ImageClient  # type: ignore[import-not-found]
        from bosdyn.client.lease import (  # type: ignore[import-not-found]
            LeaseClient,
            LeaseKeepAlive,
        )
        from bosdyn.client.robot_command import (  # type: ignore[import-not-found]
            RobotCommandClient,
            blocking_stand,
        )
        from bosdyn.client.robot_state import (  # type: ignore[import-not-found]
            RobotStateClient,
        )

        logger.info(f"Connecting to Spot at {ip}")
        sdk = await asyncio.to_thread(create_standard_sdk, "dimos-spot")
        self._robot = await asyncio.to_thread(sdk.create_robot, ip)
        await asyncio.to_thread(self._robot.authenticate, username, password)
        await asyncio.to_thread(self._robot.time_sync.wait_for_sync)

        if self.config.enable_estop:
            estop_client = self._robot.ensure_client(EstopClient.default_service_name)
            endpoint = EstopEndpoint(
                client=estop_client,
                name="dimos-spot",
                estop_timeout=self.config.estop_timeout,
            )
            await asyncio.to_thread(endpoint.force_simple_setup)
            self._estop_keepalive = EstopKeepAlive(endpoint)

        if self.config.acquire_lease:
            lease_client = self._robot.ensure_client(LeaseClient.default_service_name)
            await asyncio.to_thread(lease_client.take)
            self._lease_keepalive = LeaseKeepAlive(lease_client)

        self._command_client = self._robot.ensure_client(RobotCommandClient.default_service_name)
        self._state_client = self._robot.ensure_client(RobotStateClient.default_service_name)
        self._image_client = self._robot.ensure_client(ImageClient.default_service_name)

        if self.config.power_on_at_start:
            logger.info("Powering on Spot motors")
            await asyncio.to_thread(self._robot.power_on, timeout_sec=POWER_ON_TIMEOUT_S)

        if self.config.stand_at_start:
            logger.info("Standing Spot")
            await asyncio.to_thread(
                blocking_stand, self._command_client, timeout_sec=STAND_TIMEOUT_S
            )
            self._standing = True

        self.tf.start()
        self._image_task = asyncio.create_task(self._poll_images())
        self._odom_task = asyncio.create_task(self._poll_odom())

        self._ready.set()
        logger.info("Spot control + sensors ready")

        yield

        self._ready.clear()
        for task in (self._image_task, self._odom_task):
            if task is not None:
                task.cancel()

        if self._standing:
            try:
                from bosdyn.client.robot_command import (  # type: ignore[import-not-found]
                    blocking_sit,
                )

                await asyncio.to_thread(
                    blocking_sit, self._command_client, timeout_sec=SIT_TIMEOUT_S
                )
            except Exception as error:
                logger.error(f"Spot sit during teardown failed: {error}")
            self._standing = False

        if self.config.power_on_at_start and self._robot is not None:
            try:
                await asyncio.to_thread(
                    self._robot.power_off, cut_immediately=False, timeout_sec=POWER_OFF_TIMEOUT_S
                )
            except Exception as error:
                logger.error(f"Spot power_off during teardown failed: {error}")

        for keepalive_name, keepalive in (
            ("lease", self._lease_keepalive),
            ("estop", self._estop_keepalive),
        ):
            if keepalive is not None:
                try:
                    await asyncio.to_thread(keepalive.shutdown)
                except Exception as error:
                    logger.error(f"Spot {keepalive_name} shutdown failed: {error}")
        self._lease_keepalive = None
        self._estop_keepalive = None
        logger.info("Spot control torn down")

    async def handle_cmd_vel(self, msg: Twist) -> None:
        if not self._ready.is_set():
            return
        await self._send_velocity(msg.linear.x, msg.linear.y, msg.angular.z)

    async def _send_velocity(
        self, forward: float, strafe: float, yaw: float, duration: float = 0.0
    ) -> bool:
        from bosdyn.client.robot_command import (  # type: ignore[import-not-found]
            RobotCommandBuilder,
        )

        command = RobotCommandBuilder.synchro_velocity_command(v_x=forward, v_y=strafe, v_rot=yaw)
        window = duration if duration > 0 else self.config.cmd_vel_timeout
        try:
            await asyncio.to_thread(
                self._command_client.robot_command,
                command,
                end_time_secs=time.time() + window,
            )
            return True
        except Exception as error:
            logger.error(f"Spot velocity command failed: {error}")
            return False

    def _grayscale_outputs(self) -> list[Out[Image]]:
        return [
            self.grayscale_image_1,
            self.grayscale_image_2,
            self.grayscale_image_3,
            self.grayscale_image_4,
            self.grayscale_image_5,
        ]

    def _depth_outputs(self) -> list[Out[Image]]:
        return [
            self.depth_image_1,
            self.depth_image_2,
            self.depth_image_3,
            self.depth_image_4,
            self.depth_image_5,
        ]

    async def _poll_images(self) -> None:
        period = 1.0 / self.config.image_rate_hz
        sources = self.config.grayscale_sources + self.config.depth_sources
        # source name -> (Out stream that carries it, np.rot90 k applied before publish).
        routing: dict[str, tuple[Out[Image], int]] = {}
        # Fewer configured sources than output streams is fine — extra streams
        # just stay silent, so pair only as many as the shorter list.
        for index, (source, out) in enumerate(
            zip(self.config.grayscale_sources, self._grayscale_outputs(), strict=False)
        ):
            routing[source] = (out, self._rotation_k(index + 1))
        for index, (source, out) in enumerate(
            zip(self.config.depth_sources, self._depth_outputs(), strict=False)
        ):
            routing[source] = (out, self._rotation_k(index + 1))

        while True:
            start = time.monotonic()
            try:
                responses = await asyncio.to_thread(
                    self._image_client.get_image_from_sources, sources
                )
            except Exception as error:
                logger.error(f"Spot image capture failed: {error}")
                await asyncio.sleep(period)
                continue

            for response in responses:
                source_name = response.source.name
                route = routing.get(source_name)
                if route is None:
                    continue
                out, rotation_k = route
                image = _decode_image(response, source_name)
                if image is None:
                    continue
                if rotation_k:
                    image = _rotate_image(image, rotation_k)
                out.publish(image)

            await asyncio.sleep(max(0.0, period - (time.monotonic() - start)))

    def _rotation_k(self, camera_number: int) -> int:
        """Convert a clockwise degree rotation into an np.rot90 k (0-3)."""
        degrees_cw = self.config.image_rotations_cw.get(camera_number, 0)
        return (-degrees_cw // 90) % 4

    async def _poll_odom(self) -> None:
        from bosdyn.client.frame_helpers import (  # type: ignore[import-not-found]
            BODY_FRAME_NAME,
            VISION_FRAME_NAME,
            get_a_tform_b,
        )

        period = 1.0 / self.config.odom_rate_hz
        while True:
            start = time.monotonic()
            try:
                state = await asyncio.to_thread(self._state_client.get_robot_state)
            except Exception as error:
                logger.error(f"Spot state capture failed: {error}")
                await asyncio.sleep(period)
                continue

            kinematic_state = state.kinematic_state
            vision_tform_body = get_a_tform_b(
                kinematic_state.transforms_snapshot, VISION_FRAME_NAME, BODY_FRAME_NAME
            )
            velocity = kinematic_state.velocity_of_body_in_vision
            self._publish_odom(vision_tform_body, velocity)

            await asyncio.sleep(max(0.0, period - (time.monotonic() - start)))

    def _publish_odom(self, vision_tform_body: Any, velocity: Any) -> None:
        from dimos.msgs.geometry_msgs.Quaternion import Quaternion
        from dimos.msgs.geometry_msgs.Transform import Transform

        now = time.time()
        pose = Pose(
            position=[vision_tform_body.x, vision_tform_body.y, vision_tform_body.z],
            orientation=[
                vision_tform_body.rot.x,
                vision_tform_body.rot.y,
                vision_tform_body.rot.z,
                vision_tform_body.rot.w,
            ],
        )
        twist = Twist(
            linear=[velocity.linear.x, velocity.linear.y, velocity.linear.z],
            angular=[velocity.angular.x, velocity.angular.y, velocity.angular.z],
        )
        odometry = Odometry(
            ts=now,
            frame_id=VISION_FRAME,
            child_frame_id=BODY_FRAME,
            pose=pose,
            twist=twist,
        )
        self.odom.publish(odometry)
        self.tf.publish(
            Transform(
                translation=Vector3(vision_tform_body.x, vision_tform_body.y, vision_tform_body.z),
                rotation=Quaternion(
                    vision_tform_body.rot.x,
                    vision_tform_body.rot.y,
                    vision_tform_body.rot.z,
                    vision_tform_body.rot.w,
                ),
                frame_id=VISION_FRAME,
                child_frame_id=BODY_FRAME,
                ts=now,
            )
        )

    @rpc
    async def move(self, twist: Twist, duration: float = 0.0) -> bool:
        """Send a Twist as a body velocity command, optionally for `duration` seconds."""
        return await self._send_velocity(twist.linear.x, twist.linear.y, twist.angular.z, duration)

    @rpc
    async def get_state(self) -> str:
        if self._state_client is None:
            return "DISCONNECTED"
        try:
            state = await asyncio.to_thread(self._state_client.get_robot_state)
            return str(state.power_state.motor_power_state)
        except Exception as error:
            logger.error(f"Spot get_state failed: {error}")
            return "UNKNOWN"

    @skill
    async def move_velocity(
        self, x: float, y: float = 0.0, yaw: float = 0.0, duration: float = 0.0
    ) -> str:
        """Move Spot with a direct body velocity command.

        Args:
            x: Forward velocity (m/s).
            y: Left/right velocity (m/s).
            yaw: Rotational velocity (rad/s).
            duration: Seconds to move. 0 uses one `cmd_vel_timeout` window.
        """
        twist = Twist(linear=Vector3(x, y, 0), angular=Vector3(0, 0, yaw))
        if await self.move(twist, duration=duration):
            return f"Moving with velocity=({x}, {y}, {yaw}) for {duration} seconds"
        return f"Failed to move with velocity=({x}, {y}, {yaw})"

    @skill
    async def stand(self) -> str:
        """Make Spot stand up. Spot must already be powered on."""
        if self._command_client is None:
            return "Spot is not connected."
        try:
            from bosdyn.client.robot_command import (  # type: ignore[import-not-found]
                blocking_stand,
            )

            await asyncio.to_thread(
                blocking_stand, self._command_client, timeout_sec=STAND_TIMEOUT_S
            )
            self._standing = True
            return "Spot is standing."
        except Exception as error:
            logger.error(f"Spot stand failed: {error}")
            return f"Stand failed: {error}"

    @skill
    async def sit(self) -> str:
        """Make Spot sit down."""
        if self._command_client is None:
            return "Spot is not connected."
        try:
            from bosdyn.client.robot_command import (  # type: ignore[import-not-found]
                blocking_sit,
            )

            await asyncio.to_thread(blocking_sit, self._command_client, timeout_sec=SIT_TIMEOUT_S)
            self._standing = False
            return "Spot is sitting."
        except Exception as error:
            logger.error(f"Spot sit failed: {error}")
            return f"Sit failed: {error}"


def _rotate_image(image: Image, rotation_k: int) -> Image:
    """Rotate an image counterclockwise by rotation_k * 90 degrees."""
    rotated = np.ascontiguousarray(np.rot90(image.data, rotation_k))
    return Image(data=rotated, format=image.format, frame_id=image.frame_id, ts=image.ts)


def _decode_image(response: Any, source_name: str) -> Image | None:
    """Turn a bosdyn ImageResponse into a dimos Image, or None if unsupported."""
    from bosdyn.api import image_pb2  # type: ignore[import-not-found]

    shot = response.shot.image
    pixel_format = shot.pixel_format
    now = time.time()

    if shot.format == image_pb2.Image.FORMAT_JPEG:
        import cv2

        buffer = np.frombuffer(shot.data, dtype=np.uint8)
        decoded = cv2.imdecode(buffer, cv2.IMREAD_UNCHANGED)
        if decoded is None:
            logger.error(f"Failed to decode JPEG image from {source_name}")
            return None
        image_format = ImageFormat.GRAY if decoded.ndim == 2 else ImageFormat.BGR
        return Image.from_numpy(decoded, format=image_format, frame_id=source_name, ts=now)

    if shot.format != image_pb2.Image.FORMAT_RAW:
        logger.error(f"Unsupported Spot image encoding {shot.format} from {source_name}")
        return None

    dtype, channels, image_format = _raw_layout(pixel_format)
    if dtype is None:
        logger.error(f"Unsupported Spot pixel format {pixel_format} from {source_name}")
        return None

    array = np.frombuffer(shot.data, dtype=dtype)
    array = (
        array.reshape(shot.rows, shot.cols)
        if channels == 1
        else array.reshape(shot.rows, shot.cols, channels)
    )
    return Image.from_numpy(array, format=image_format, frame_id=source_name, ts=now)


def _raw_layout(pixel_format: int) -> tuple[Any, int, ImageFormat]:
    from bosdyn.api import image_pb2  # type: ignore[import-not-found]

    layouts: dict[int, tuple[Any, int, ImageFormat]] = {
        image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U8: (np.uint8, 1, ImageFormat.GRAY),
        image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U16: (np.uint16, 1, ImageFormat.GRAY16),
        image_pb2.Image.PIXEL_FORMAT_DEPTH_U16: (np.uint16, 1, ImageFormat.DEPTH16),
        image_pb2.Image.PIXEL_FORMAT_RGB_U8: (np.uint8, 3, ImageFormat.RGB),
        image_pb2.Image.PIXEL_FORMAT_RGBA_U8: (np.uint8, 4, ImageFormat.RGBA),
    }
    return layouts.get(pixel_format, (None, 0, ImageFormat.GRAY))
