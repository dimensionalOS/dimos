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

"""Quest teleop module extensions and subclasses.

Available subclasses:
    - ArmTeleopModule: Per-hand press-and-hold engage (X/A hold to track)
    - HandTeleopModule: Pinch-to-toggle arm teleop using WebXR hand tracking
    - TwistTeleopModule: Outputs Twist instead of PoseStamped
    - VideoArmTeleopModule: ArmTeleopModule + JPEG frames pushed to the Quest over /ws
    - MobileVideoArmTeleopModule: Video arm teleop + thumbstick base velocity
    - Go2TeleopModule: Thumbstick → Twist velocity for the Go2 + camera over /ws
"""

import asyncio
from typing import Any, Literal

from fastapi import WebSocket

from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.teleop.quest.quest_teleop_module import QuestTeleopConfig, QuestTeleopModule
from dimos.teleop.quest.quest_types import Buttons, Hand, QuestControllerState
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


async def _ws_send_jpeg(ws: WebSocket, data: bytes) -> None:
    try:
        await ws.send_bytes(data)
    except Exception:
        # Client closed or write failed — drop the frame; the base /ws
        # disconnect handler evicts the dead client.
        pass


def _push_jpeg(module: QuestTeleopModule, msg: Image, quality: int) -> None:
    """JPEG-encode an Image and push it to all of module's connected /ws clients.

    Runs on the RX thread; sends are scheduled on the asyncio loop captured by
    QuestTeleopModule when the first client connected.
    """
    # Snapshot clients under the lock to avoid concurrent set mutation from
    # the uvicorn thread. Skip the encode entirely if nobody is listening.
    loop = module._ws_loop
    if loop is None:
        return
    with module._clients_lock:
        clients = tuple(module._connected_clients)
    if not clients:
        return

    try:
        jpeg = msg.to_jpeg_bytes(quality=quality)
    except Exception:
        logger.exception("Failed to encode camera frame")
        return

    for ws in clients:
        asyncio.run_coroutine_threadsafe(_ws_send_jpeg(ws, jpeg), loop)


class TwistTeleopConfig(QuestTeleopConfig):
    """Configuration for TwistTeleopModule."""

    linear_scale: float = 1.0
    angular_scale: float = 1.0


# Example implementation to show how to extend QuestTeleopModule for different teleop behaviors and outputs.
class TwistTeleopModule(QuestTeleopModule):
    """Quest teleop that outputs TwistStamped instead of PoseStamped.

    Config:
        - linear_scale: Scale factor for linear (position) values. Default 1.0.
        - angular_scale: Scale factor for angular (orientation) values. Default 1.0.

    Outputs:
        - left_twist: TwistStamped (linear + angular velocity)
        - right_twist: TwistStamped (linear + angular velocity)
        - buttons: Buttons (inherited)
    """

    config: TwistTeleopConfig

    left_twist: Out[TwistStamped]
    right_twist: Out[TwistStamped]

    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()

    def _publish_msg(self, hand: Hand, output_msg: PoseStamped) -> None:
        """Convert PoseStamped to TwistStamped, apply scaling, and publish."""
        twist = TwistStamped(
            ts=output_msg.ts,
            frame_id=output_msg.frame_id,
            linear=output_msg.position * self.config.linear_scale,
            angular=output_msg.orientation.to_euler() * self.config.angular_scale,
        )
        if hand == Hand.LEFT:
            self.left_twist.publish(twist)
        else:
            self.right_twist.publish(twist)


class ArmTeleopModule(QuestTeleopModule):
    """Quest teleop with per-hand press-and-hold engage.

    Each controller's primary button (X for left, A for right)
    engages that hand while held, disengages on release. Each hand's
    output port is wired to its consuming task's coordinator port in
    the blueprint; no addressing happens in the message.

    Unlike the base module, this publishes absolute controller poses. The
    control task owns controller-to-robot reference capture so one task can
    establish a coherent reference for both arms.

    Outputs:
        - left_controller_output: PoseStamped (inherited)
        - right_controller_output: PoseStamped (inherited)
        - teleop_buttons: Buttons (inherited)
        - left_gripper_command: Float32 normalized opening
        - right_gripper_command: Float32 normalized opening
    """

    left_gripper_command: Out[Float32]
    right_gripper_command: Out[Float32]

    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()

    def _get_output_pose(self, hand: Hand) -> PoseStamped | None:
        """Return the current absolute controller pose."""
        return self._current_poses.get(hand)

    def _publish_button_state(
        self,
        left: QuestControllerState | None,
        right: QuestControllerState | None,
    ) -> None:
        """Publish Buttons with analog triggers packed into bits 16-29."""
        buttons = Buttons.from_controllers(left, right)
        buttons.pack_analog_triggers(
            left=left.trigger if left is not None else 0.0,
            right=right.trigger if right is not None else 0.0,
        )
        self.teleop_buttons.publish(buttons)
        self._publish_gripper_commands(left, right)

    def _publish_gripper_commands(
        self,
        left: QuestControllerState | None,
        right: QuestControllerState | None,
    ) -> None:
        """Publish normalized opening for each currently engaged hand."""
        controllers = {Hand.LEFT: left, Hand.RIGHT: right}
        outputs = {
            Hand.LEFT: self.left_gripper_command,
            Hand.RIGHT: self.right_gripper_command,
        }
        for hand, controller in controllers.items():
            if controller is None or not self._is_engaged[hand]:
                continue
            outputs[hand].publish(Float32(data=1.0 - float(controller.trigger)))


class HandTeleopModule(ArmTeleopModule):
    """WebXR hand teleop with pinch-to-toggle engage and task name routing.

    A thumb-and-index pinch is sent as the primary button by the browser. Each
    pinch edge toggles that hand between engaged and disengaged, so movement
    continues after releasing the pinch.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._primary_was_pressed: dict[Hand, bool] = {Hand.LEFT: False, Hand.RIGHT: False}

    def _handle_engage(self) -> None:
        """Toggle each hand on the rising edge of its primary button."""
        for hand in Hand:
            controller = self._controllers.get(hand)
            is_pressed = controller is not None and controller.primary
            if is_pressed and not self._primary_was_pressed[hand]:
                if self._is_engaged[hand]:
                    self._disengage(hand)
                else:
                    self._engage(hand)
            self._primary_was_pressed[hand] = is_pressed

    def _publish_button_state(
        self,
        left: QuestControllerState | None,
        right: QuestControllerState | None,
    ) -> None:
        """Keep downstream press-and-hold teleop tasks engaged between pinches."""
        buttons = Buttons.from_controllers(left, right)
        buttons.pack_analog_triggers(
            left=left.trigger if left is not None else 0.0,
            right=right.trigger if right is not None else 0.0,
        )
        buttons.left_primary = self._is_engaged[Hand.LEFT]
        buttons.right_primary = self._is_engaged[Hand.RIGHT]
        self.teleop_buttons.publish(buttons)
        self._publish_gripper_commands(left, right)


class VideoArmTeleopConfig(QuestTeleopConfig):
    """Configuration for VideoArmTeleopModule."""

    video_jpeg_quality: int = 70


class VideoArmTeleopModule(ArmTeleopModule):
    """ArmTeleopModule + camera frames pushed to the Quest as JPEG over /ws.

    Subscribes to color_image, JPEG-encodes each frame, and broadcasts raw
    JPEG bytes to every connected /ws client as a binary message. The client
    decodes via createObjectURL and uploads to a WebGL texture.

    Inputs:
        - color_image: In[Image] (required — wire to a camera output)

    Outputs:
        - left_controller_output: PoseStamped (inherited)
        - right_controller_output: PoseStamped (inherited)
        - buttons: Buttons (inherited)
    """

    config: VideoArmTeleopConfig

    color_image: In[Image]

    async def handle_color_image(self, msg: Image) -> None:
        _push_jpeg(self, msg, self.config.video_jpeg_quality)


class MobileVideoArmTeleopConfig(VideoArmTeleopConfig):
    """Configuration for combined arm, video, and mobile-base teleoperation."""

    linear_scale: float = 0.3
    yaw_scale: float = 0.3
    strafe_scale: float = 0.3
    right_stick_mode: Literal["yaw", "strafe"] = "yaw"
    deadzone: float = 0.18


class MobileVideoArmTeleopModule(VideoArmTeleopModule):
    """Video arm teleop with thumbstick velocity for a mobile manipulator."""

    dedicated_worker = True

    config: MobileVideoArmTeleopConfig

    cmd_vel: Out[Twist]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._cmd_vel_moving = False
        self._right_stick_pressed = False

    def _on_joy_bytes(self, data: bytes) -> None:
        super()._on_joy_bytes(data)
        with self._lock:
            left = self._controllers.get(Hand.LEFT)
            right = self._controllers.get(Hand.RIGHT)
        self._publish_cmd_vel(left, right)

    def _publish_cmd_vel(
        self,
        left: QuestControllerState | None,
        right: QuestControllerState | None,
    ) -> None:
        """Publish operator motion and one definitive stop per stop transition."""

        def deadzone(value: float) -> float:
            return 0.0 if abs(value) < self.config.deadzone else value

        right_stick_pressed = right is not None and right.thumbstick_press
        if right_stick_pressed:
            if not self._right_stick_pressed:
                self.cmd_vel.publish(Twist.zero())
            self._right_stick_pressed = True
            self._cmd_vel_moving = False
            return
        self._right_stick_pressed = False

        left_x = deadzone(left.thumbstick.x if left is not None else 0.0)
        left_y = deadzone(left.thumbstick.y if left is not None else 0.0)
        right_x = deadzone(right.thumbstick.x if right is not None else 0.0)

        vx = -left_y * self.config.linear_scale
        vy = 0.0
        yaw_rate = 0.0
        if self.config.right_stick_mode == "strafe":
            vy = -right_x * self.config.strafe_scale
            yaw_rate = -left_x * self.config.yaw_scale
        else:
            yaw_rate = -right_x * self.config.yaw_scale

        moving = any(value != 0.0 for value in (vx, vy, yaw_rate))
        if moving:
            self.cmd_vel.publish(
                Twist(
                    linear=Vector3(vx, vy, 0.0),
                    angular=Vector3(0.0, 0.0, yaw_rate),
                )
            )
        elif self._cmd_vel_moving:
            self.cmd_vel.publish(Twist.zero())
        self._cmd_vel_moving = moving

    @rpc
    def stop(self) -> None:
        try:
            self.cmd_vel.publish(Twist.zero())
        except Exception:
            logger.exception("Failed to publish stop Twist")
        super().stop()


class Go2TeleopConfig(QuestTeleopConfig):
    """Configuration for Go2TeleopModule."""

    linear_speed: float = 0.5  # m/s at full stick deflection
    angular_speed: float = 0.8  # rad/s at full stick deflection
    deadzone: float = 0.1
    video_jpeg_quality: int = 70


class Go2TeleopModule(QuestTeleopModule):
    """Quest teleop for the Unitree Go2: thumbstick driving + camera in the headset.

    Velocity is derived from the controller thumbsticks as each Joy message
    arrives (left stick → forward/strafe, right stick → yaw) and published on
    cmd_vel for GO2Connection.move. The Go2 camera (color_image) is JPEG-encoded
    and pushed to the headset over /ws. A deadzone suppresses stick drift.

    Inputs:
        - color_image: In[Image] (wire to the Go2 camera output)

    Outputs:
        - cmd_vel: Twist (base velocity command)
    """

    config: Go2TeleopConfig

    color_image: In[Image]
    cmd_vel: Out[Twist]

    def _publish_safe_command(self) -> None:
        self.cmd_vel.publish(Twist.zero())

    def _deadzone(self, v: float) -> float:
        return 0.0 if abs(v) < self.config.deadzone else v

    def _on_joy_bytes(self, data: bytes) -> bool:
        try:
            valid = super()._on_joy_bytes(data)
        except ValueError:
            self._publish_safe_command()
            raise
        if not valid:
            self._publish_safe_command()
            return False
        with self._lock:
            left = self._controllers.get(Hand.LEFT)
            right = self._controllers.get(Hand.RIGHT)
        twist = Twist()
        twist.linear = Vector3(0.0, 0.0, 0.0)
        twist.angular = Vector3(0.0, 0.0, 0.0)
        if left is not None:
            twist.linear.x = -self._deadzone(left.thumbstick.y) * self.config.linear_speed
            twist.linear.y = -self._deadzone(left.thumbstick.x) * self.config.linear_speed
        if right is not None:
            twist.angular.z = -self._deadzone(right.thumbstick.x) * self.config.angular_speed
        self.cmd_vel.publish(twist)
        return True

    async def handle_color_image(self, msg: Image) -> None:
        _push_jpeg(self, msg, self.config.video_jpeg_quality)
