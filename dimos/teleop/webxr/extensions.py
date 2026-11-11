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

"""WebXR teleop module extensions and subclasses.

Available subclasses:
    - ArmTeleopModule: Raw arm poses with middle-finger grip deadman buttons
    - HandTeleopModule: Pinch-to-toggle arm teleop using WebXR hand tracking
    - TwistTeleopModule: Outputs Twist instead of PoseStamped
    - VideoArmTeleopModule: ArmTeleopModule + JPEG frames pushed to the headset over /ws
    - Go2TeleopModule: Thumbstick → Twist velocity for the Go2 + camera over /ws
"""

import asyncio
import threading
from typing import Any

from fastapi import WebSocket

from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.manipulation.manipulation_spec import ManipulationSpec
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.teleop.webxr.controller_types import (
    BUTTON_ALIASES,
    Buttons,
    Hand,
    WebXRControllerState,
)
from dimos.teleop.webxr.module import WebXRTeleopConfig, WebXRTeleopModule
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


async def _ws_send_jpeg(ws: WebSocket, data: bytes) -> None:
    try:
        await ws.send_bytes(data)
    except Exception:
        # Client closed or write failed — drop the frame; the base /ws
        # disconnect handler evicts the dead client.
        pass


def _push_jpeg(module: WebXRTeleopModule, msg: Image, quality: int) -> None:
    """JPEG-encode an Image and push it to all of module's connected /ws clients.

    Runs on the RX thread; sends are scheduled on the asyncio loop captured by
    WebXRTeleopModule when the first client connected.
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


class TwistTeleopConfig(WebXRTeleopConfig):
    """Configuration for TwistTeleopModule."""

    linear_scale: float = 1.0
    angular_scale: float = 1.0


# Example implementation to show how to extend WebXRTeleopModule for different teleop behaviors and outputs.
class TwistTeleopModule(WebXRTeleopModule):
    """WebXR teleop that outputs TwistStamped instead of PoseStamped.

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


class ThumbstickDriveConfig(WebXRTeleopConfig):
    """Config for teleop modules that drive the base with the thumbsticks.

    Attributes:
        linear_speed: m/s of translation at full left-stick deflection.
        angular_speed: rad/s of yaw at full right-stick deflection.
        deadzone: Stick magnitudes below this are treated as centred.
    """

    linear_speed: float = 0.5
    angular_speed: float = 0.8
    deadzone: float = 0.1


def _deadzone(value: float, threshold: float) -> float:
    """``value``, or 0.0 when its magnitude is below ``threshold``."""
    return 0.0 if abs(value) < threshold else value


def thumbstick_twist(
    left: WebXRControllerState | None,
    right: WebXRControllerState | None,
    *,
    deadzone: float,
    linear_speed: float,
    angular_speed: float,
) -> Twist:
    """The base Twist the two controllers' thumbsticks ask for, in the body frame.

    The left stick translates (push forward → ``+x``, push right → ``-y``),
    the right stick's x turns (push right → ``-yaw``). This is the one
    stick→Twist mapping shared by every thumbstick-driven teleop module.

    Args:
        left: The left controller, or ``None`` when it has no fresh input.
        right: The right controller, or ``None`` when it has no fresh input.
        deadzone: Stick magnitudes below this are treated as centred.
        linear_speed: m/s at full left-stick deflection.
        angular_speed: rad/s at full right-stick deflection.

    Returns:
        A Twist; all zeros when a stick is centred or its controller absent,
        never the previous speed.
    """
    twist = Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))
    if left is not None:
        twist.linear.x = -_deadzone(left.thumbstick.y, deadzone) * linear_speed
        twist.linear.y = -_deadzone(left.thumbstick.x, deadzone) * linear_speed
    if right is not None:
        twist.angular.z = -_deadzone(right.thumbstick.x, deadzone) * angular_speed
    return twist


class ArmTeleopModule(WebXRTeleopModule):
    """Quest arm input that leaves engagement to the coordinator task.

    Controller poses and buttons are published as raw operator input. The
    consuming TeleopIKTask owns the middle-finger grip deadman and reference
    capture, leaving the face buttons available for stack controls.

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

    _manipulation: ManipulationSpec | None = None

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._grip_held: dict[Hand, bool] = {Hand.LEFT: False, Hand.RIGHT: False}

    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_buttons_pressed(self, pressed: Buttons) -> None:
        """Move to the zero joint pose, off the control loop, when zero_button is pressed."""
        button = self.config.zero_button
        if button is not None and getattr(pressed, BUTTON_ALIASES.get(button, button)):
            threading.Thread(target=self._move_to_zero, daemon=True).start()

    def _move_to_zero(self) -> None:
        if self._manipulation is None:
            return
        targets = {
            group.id: JointState(
                name=list(group.joint_names), position=[0.0] * len(group.joint_names)
            )
            for group in self._manipulation.list_planning_groups()
        }
        plan = self._manipulation.plan_to_joints(targets)
        if plan.plan is not None:
            self._manipulation.execute(blocking=False, plan_id=plan.plan.plan_id)

    def _get_output_pose(self, hand: Hand) -> PoseStamped | None:
        """Return the current absolute controller pose."""
        return self._current_poses.get(hand)

    def _handle_engage(self) -> None:
        """Leave arm engagement to TeleopIKTask."""

    def _should_publish(self, hand: Hand) -> bool:
        """Publish every fresh absolute controller pose."""
        return self._controllers.get(hand) is not None

    def _publish_button_state(
        self,
        left: WebXRControllerState | None,
        right: WebXRControllerState | None,
    ) -> None:
        """Publish Buttons with analog triggers packed into bits 16-29."""
        buttons = Buttons.from_controllers(left, right)
        buttons.pack_analog_triggers(
            left=left.trigger if left is not None else 0.0,
            right=right.trigger if right is not None else 0.0,
        )
        self._publish_buttons(buttons)
        self._publish_gripper_commands(left, right, buttons)
        self._log_engagement(buttons)

    def _log_engagement(self, buttons: Buttons) -> None:
        """Log a hand as engaged when its grip is pressed, and disengaged when let go."""
        for hand, held in ((Hand.LEFT, buttons.left_grip), (Hand.RIGHT, buttons.right_grip)):
            if held != self._grip_held[hand]:
                self._grip_held[hand] = held
                logger.info(f"{hand.name} {'engaged' if held else 'disengaged'}.")

    def _publish_gripper_commands(
        self,
        left: WebXRControllerState | None,
        right: WebXRControllerState | None,
        buttons: Buttons,
    ) -> None:
        """Publish normalized opening while the classified deadman is held."""
        controllers = {Hand.LEFT: left, Hand.RIGHT: right}
        outputs = {
            Hand.LEFT: self.left_gripper_command,
            Hand.RIGHT: self.right_gripper_command,
        }
        for hand, controller in controllers.items():
            if controller is None or not self._gripper_is_enabled(hand, buttons):
                continue
            outputs[hand].publish(Float32(data=1.0 - float(controller.trigger)))

    def _gripper_is_enabled(self, hand: Hand, buttons: Buttons) -> bool:
        """Use the same classified grip bit consumed by TeleopIKTask."""
        return buttons.left_grip if hand == Hand.LEFT else buttons.right_grip


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

    def _should_publish(self, hand: Hand) -> bool:
        return self._is_engaged[hand]

    def _gripper_is_enabled(self, hand: Hand, buttons: Buttons) -> bool:
        return self._is_engaged[hand]

    def _publish_button_state(
        self,
        left: WebXRControllerState | None,
        right: WebXRControllerState | None,
    ) -> None:
        """Keep downstream press-and-hold teleop tasks engaged between pinches."""
        buttons = Buttons.from_controllers(left, right)
        buttons.pack_analog_triggers(
            left=left.trigger if left is not None else 0.0,
            right=right.trigger if right is not None else 0.0,
        )
        buttons.left_grip = self._is_engaged[Hand.LEFT]
        buttons.right_grip = self._is_engaged[Hand.RIGHT]
        self._publish_buttons(buttons)
        self._publish_gripper_commands(left, right, buttons)


class VideoArmTeleopConfig(WebXRTeleopConfig):
    """Configuration for VideoArmTeleopModule."""

    video_jpeg_quality: int = 70


class VideoArmTeleopModule(ArmTeleopModule):
    """ArmTeleopModule + camera frames pushed to the headset as JPEG over /ws.

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


class Go2TeleopConfig(ThumbstickDriveConfig):
    """Configuration for Go2TeleopModule."""

    video_jpeg_quality: int = 70


class Go2TeleopModule(WebXRTeleopModule):
    """WebXR teleop for the Unitree Go2: thumbstick driving + camera in the headset.

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
        self.cmd_vel.publish(
            thumbstick_twist(
                left,
                right,
                deadzone=self.config.deadzone,
                linear_speed=self.config.linear_speed,
                angular_speed=self.config.angular_speed,
            )
        )
        return True

    async def handle_color_image(self, msg: Image) -> None:
        _push_jpeg(self, msg, self.config.video_jpeg_quality)
