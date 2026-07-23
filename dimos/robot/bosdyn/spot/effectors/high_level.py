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

- `grayscale_image_{front_left,front_right,left,right,back}` — the five fisheye
  body cameras.
- `depth_image_{front_left,front_right,left,right,back}` — the matching depth cameras.
- `odometry` — base pose + velocity, also published live as `odom`->`base_link`
  on TF (frame names configurable via `odom_frame_id` / `base_frame_id`).

The fixed camera mounts (`base_link`->`{pos}_camera_optical`) come from the URDF
at `SPOT_URDF_PATH` instead of Spot's live snapshot: `SpotHighLevel` subclasses
`StaticTfPublisher`, which republishes those static extrinsics on an interval so
the moving odom edge and rigid mounts together anchor every recorded frame.

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

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.protocol.tf.static_tf_publisher import StaticTfPublisher, StaticTfPublisherConfig
from dimos.robot.bosdyn.spot.config import (
    CAMERA_MAX_HZ,
    FRONT_CAMERA_MIRROR_HALF_TURN,
    FRONT_CAMERA_ROTATE_UPRIGHT,
    IP_LABELS,
    MAX_ANGULAR_VELOCITY,
    MAX_LINEAR_VELOCITY,
    POWER_OFF_TIMEOUT_S,
    POWER_ON_TIMEOUT_S,
    REACHABILITY_PROBE_TIMEOUT_S,
    RIGHT_CAMERA_ROTATE_UPRIGHT,
    SIT_TIMEOUT_S,
    SPOT_API_PORT,
    SPOT_URDF_PATH,
    STAND_TIMEOUT_S,
)
from dimos.robot.bosdyn.spot.utils import (
    camera_info_from_response,
    camera_mount_transforms,
    clamp,
    decode_image,
    roll_optical_frame,
    rotate_camera_info_quarter_turns,
    rotate_image_quarter_turns,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class SpotHighLevelConfig(StaticTfPublisherConfig):
    """Cmd vel, sensors, credentials, and safety gating a Spot robot."""

    # When left blank, main() probes `candidate_ips` and uses the first that answers on the
    # API port — so plugging in over Ethernet or joining Spot's WiFi both work.
    ip: str = ""
    candidate_ips: list[str] = field(default_factory=lambda: list(IP_LABELS))

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

    # frame_id's
    odom_frame_id: str = "odom"
    base_frame_id: str = "base_link"
    frontleft_camera_frame_id: str = "frontleft_camera_optical"
    frontright_camera_frame_id: str = "frontright_camera_optical"
    left_camera_frame_id: str = "left_camera_optical"
    right_camera_frame_id: str = "right_camera_optical"
    back_camera_frame_id: str = "back_camera_optical"

    image_rate_hz: float = CAMERA_MAX_HZ
    odom_rate_hz: float = 60.0


class SpotHighLevel(StaticTfPublisher):
    """Drives Spot and streams its fisheye cameras, depth cameras, and odometry."""

    config: SpotHighLevelConfig

    cmd_vel: In[Twist]

    grayscale_image_front_left: Out[Image]
    grayscale_image_front_right: Out[Image]
    grayscale_image_left: Out[Image]
    grayscale_image_right: Out[Image]
    grayscale_image_back: Out[Image]

    depth_image_front_left: Out[Image]
    depth_image_front_right: Out[Image]
    depth_image_left: Out[Image]
    depth_image_right: Out[Image]
    depth_image_back: Out[Image]

    # All five grayscale cameras share one lens model and all five depth cameras share another
    grayscale_info: Out[CameraInfo]
    depth_info: Out[CameraInfo]

    odometry: Out[Odometry]

    dedicated_worker = True

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

    def transforms(self) -> list[Transform]:
        """Static base_link -> camera-optical extrinsics parsed from the URDF.

        `StaticTfPublisher` republishes these on a fixed interval; the moving
        odom->base_link edge stays live (see `_publish_odom`). The front optical
        frames are rolled to match the upright-rotated front images so recorded
        tf, intrinsics, and pixels stay mutually consistent for depth reprojection.
        """
        mounts = camera_mount_transforms(
            SPOT_URDF_PATH,
            self.config.base_frame_id,
            [
                self.config.frontleft_camera_frame_id,
                self.config.frontright_camera_frame_id,
                self.config.left_camera_frame_id,
                self.config.right_camera_frame_id,
                self.config.back_camera_frame_id,
            ],
        )
        front_frame_rolls = {
            self.config.frontleft_camera_frame_id: FRONT_CAMERA_ROTATE_UPRIGHT,
            self.config.frontright_camera_frame_id: (
                FRONT_CAMERA_ROTATE_UPRIGHT + FRONT_CAMERA_MIRROR_HALF_TURN
            ),
        }
        return [
            roll_optical_frame(mount, front_frame_rolls.get(mount.child_frame_id, 0))
            for mount in mounts
        ]

    async def main(self) -> AsyncIterator[None]:
        username, password = self.config.username, self.config.password
        if not username or not password:
            raise ValueError(
                "Spot credentials missing — pass username/password in config "
                "(-o <module>.username=... -o <module>.password=...)"
            )

        # Any setup failure past this point must undo the lease/E-stop keepalives and
        # motor power it already acquired, else Spot is left powered with no manager.
        try:
            ip = await self.resolve_ip()

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
            await self.sync_clocks()

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

            self._command_client = self._robot.ensure_client(
                RobotCommandClient.default_service_name
            )
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
        except BaseException:
            await self._teardown()
            raise

        yield

        await self._teardown()

    async def _teardown(self) -> None:
        self._ready.clear()
        for task in (self._image_task, self._odom_task):
            if task is not None:
                task.cancel()

        if self._standing:
            await self.lie_down()

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
        await self.move(msg)

    # the spot API can only poll - no callbacks
    async def _poll_images(self) -> None:
        period = 1.0 / self.config.image_rate_hz
        config = self.config
        # bosdyn source name -> (image Out, CameraInfo Out, frame_id, quarter_turns).
        routing: dict[str, tuple[Out[Image], Out[CameraInfo], str, int]] = {
            "frontleft_fisheye_image": (
                self.grayscale_image_front_left,
                self.grayscale_info,
                config.frontleft_camera_frame_id,
                FRONT_CAMERA_ROTATE_UPRIGHT,
            ),
            "frontright_fisheye_image": (
                self.grayscale_image_front_right,
                self.grayscale_info,
                config.frontright_camera_frame_id,
                FRONT_CAMERA_ROTATE_UPRIGHT,
            ),
            "left_fisheye_image": (
                self.grayscale_image_left,
                self.grayscale_info,
                config.left_camera_frame_id,
                0,
            ),
            "right_fisheye_image": (
                self.grayscale_image_right,
                self.grayscale_info,
                config.right_camera_frame_id,
                RIGHT_CAMERA_ROTATE_UPRIGHT,
            ),
            "back_fisheye_image": (
                self.grayscale_image_back,
                self.grayscale_info,
                config.back_camera_frame_id,
                0,
            ),
            "frontleft_depth": (
                self.depth_image_front_left,
                self.depth_info,
                config.frontleft_camera_frame_id,
                FRONT_CAMERA_ROTATE_UPRIGHT,
            ),
            "frontright_depth": (
                self.depth_image_front_right,
                self.depth_info,
                config.frontright_camera_frame_id,
                FRONT_CAMERA_ROTATE_UPRIGHT,
            ),
            "left_depth": (
                self.depth_image_left,
                self.depth_info,
                config.left_camera_frame_id,
                0,
            ),
            "right_depth": (
                self.depth_image_right,
                self.depth_info,
                config.right_camera_frame_id,
                RIGHT_CAMERA_ROTATE_UPRIGHT,
            ),
            "back_depth": (
                self.depth_image_back,
                self.depth_info,
                config.back_camera_frame_id,
                0,
            ),
        }
        sources = list(routing)
        # Sensor capture time of the last frame published per source. Polling above
        # the sensor's frame rate re-returns the same frame; matching acquisition
        # time means it's a repeat, so skip it and never publish a frame twice.
        last_published_ts: dict[str, float] = {}

        while True:
            start = time.monotonic()
            try:
                responses = await asyncio.to_thread(
                    self._image_client.get_image_from_sources, sources
                )
                time_converter = self._robot.time_sync.get_robot_time_converter()
            except Exception as error:
                logger.error(f"Spot image capture failed: {error}")
                await asyncio.sleep(period)
                continue

            for response in responses:
                source_name = response.source.name
                route = routing.get(source_name)
                if route is None:
                    continue
                out, info_out, frame_id, quarter_turns = route
                image = decode_image(response, frame_id, time_converter)
                if image is None:
                    continue
                if last_published_ts.get(source_name) == image.ts:
                    continue
                last_published_ts[source_name] = image.ts
                camera_info = camera_info_from_response(response, frame_id, image.ts)
                if quarter_turns:
                    image = rotate_image_quarter_turns(image, quarter_turns)
                    if camera_info is not None:
                        camera_info = rotate_camera_info_quarter_turns(camera_info, quarter_turns)
                out.publish(image)
                if camera_info is not None:
                    info_out.publish(camera_info)

            await asyncio.sleep(max(0.0, period - (time.monotonic() - start)))

    async def _poll_odom(self) -> None:
        from bosdyn.client.frame_helpers import (  # type: ignore[import-not-found]
            BODY_FRAME_NAME,
            VISION_FRAME_NAME,
            get_a_tform_b,
        )
        from bosdyn.client.math_helpers import SE3Pose  # type: ignore[import-not-found]

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
            # note:  creating new time_converter's is intenional, time_sync changes itself over time, time_converter doesn't
            time_converter = self._robot.time_sync.get_robot_time_converter()
            ts = time_converter.local_seconds_from_robot_timestamp(
                kinematic_state.acquisition_timestamp
            )
            vision_tform_body = get_a_tform_b(
                kinematic_state.transforms_snapshot, VISION_FRAME_NAME, BODY_FRAME_NAME
            )
            # No vision->body: fall back to identity so odom->base_link keeps publishing and the
            # tf tree stays connected (cameras keep resolving). Odom pose reads zero until it returns.
            if vision_tform_body is None:
                logger.warning(
                    "Spot state has no vision->body transform; using identity "
                    "odom->base_link until it returns (odometry pose reads zero)."
                )
                vision_tform_body = SE3Pose.from_identity()
            velocity = kinematic_state.velocity_of_body_in_vision
            self._publish_odom(vision_tform_body, velocity, ts)

            await asyncio.sleep(max(0.0, period - (time.monotonic() - start)))

    def _publish_odom(self, vision_tform_body: Any, velocity: Any, ts: float) -> None:
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
            ts=ts,
            frame_id=self.config.odom_frame_id,
            child_frame_id=self.config.base_frame_id,
            pose=pose,
            twist=twist,
        )
        self.odometry.publish(odometry)
        self.tf.publish(
            Transform(
                translation=Vector3(vision_tform_body.x, vision_tform_body.y, vision_tform_body.z),
                rotation=Quaternion(
                    vision_tform_body.rot.x,
                    vision_tform_body.rot.y,
                    vision_tform_body.rot.z,
                    vision_tform_body.rot.w,
                ),
                frame_id=self.config.odom_frame_id,
                child_frame_id=self.config.base_frame_id,
                ts=ts,
            )
        )

    @rpc
    async def move(self, twist: Twist, duration: float = 0.0) -> bool:
        """Send a Twist as a body velocity command, optionally for `duration` seconds."""
        if self._command_client is None:
            return False
        from bosdyn.client.robot_command import (  # type: ignore[import-not-found]
            RobotCommandBuilder,
        )

        command = RobotCommandBuilder.synchro_velocity_command(
            v_x=clamp(twist.linear.x, -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY),
            v_y=clamp(twist.linear.y, -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY),
            v_rot=clamp(twist.angular.z, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY),
        )
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
    async def get_battery_soc(self) -> str:
        """Report Spot's battery state of charge as a percentage."""
        if self._state_client is None:
            return "Spot is not connected."
        try:
            state = await asyncio.to_thread(self._state_client.get_robot_state)
        except Exception as error:
            logger.error(f"Spot get_battery_soc failed: {error}")
            return "Failed to read Spot battery state."
        for battery in state.battery_states:
            if battery.HasField("charge_percentage"):
                return f"Battery is at {battery.charge_percentage.value:.0f}%."
        return "Battery charge is unavailable."

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
    async def lie_down(self) -> str:
        """Make Spot lie down."""
        if self._command_client is None:
            return "Spot is not connected."
        try:
            from bosdyn.client.robot_command import (  # type: ignore[import-not-found]
                blocking_sit,
            )

            await asyncio.to_thread(blocking_sit, self._command_client, timeout_sec=SIT_TIMEOUT_S)
            self._standing = False
            return "Spot is lying down."
        except Exception as error:
            logger.error(f"Spot lie_down failed: {error}")
            return f"Lie down failed: {error}"

    async def resolve_ip(self) -> str:
        """The Spot IP to connect to: explicit `config.ip`, else the first reachable candidate.

        With no `config.ip`, each candidate gets a short TCP connect to the API
        port and the first successful handshake wins — so Ethernet or Spot's WiFi
        both work without configuration. Raises `ConnectionError` if none answer.
        """
        if self.config.ip:
            return self.config.ip
        for candidate in self.config.candidate_ips:
            try:
                _, writer = await asyncio.wait_for(
                    asyncio.open_connection(candidate, SPOT_API_PORT),
                    timeout=REACHABILITY_PROBE_TIMEOUT_S,
                )
            except (OSError, asyncio.TimeoutError):
                continue
            # A successful TCP handshake is all we need. Close without awaiting
            # wait_closed(); the API port speaks TLS and never completes a clean
            # plaintext close, which would otherwise hang the probe.
            writer.close()
            logger.info(f"Spot reachable at {candidate}")
            return candidate
        described = " or ".join(
            f"{candidate} ({IP_LABELS[candidate]})" if candidate in IP_LABELS else candidate
            for candidate in self.config.candidate_ips
        )
        raise ConnectionError(
            f"I'm unable to connect to {described}. Did you forget to connect to "
            "Spot's WiFi or plug in an Ethernet cable to Spot?"
        )

    async def sync_clocks(self) -> None:
        """Establish time sync so robot-clock image timestamps convert to local time.

        Touching `robot.time_sync` starts bosdyn's background sync thread, which keeps
        re-estimating clock skew on an interval; this blocks until the first estimate
        lands. `_poll_images` then pulls a live `RobotTimeConverter` each cycle rather
        than freezing one offset here, since the skew drifts and would go stale.
        """
        await asyncio.to_thread(self._robot.time_sync.wait_for_sync)
