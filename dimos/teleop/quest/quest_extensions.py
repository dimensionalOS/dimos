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
    - ArmTeleopModule: Per-hand press-and-hold engage (X/A hold to track), task name routing
    - TwistTeleopModule: Outputs Twist instead of PoseStamped
    - VideoArmTeleopModule: ArmTeleopModule + JPEG frames pushed to the Quest over /ws
    - Go2TeleopModule: Thumbstick → Twist velocity for the Go2 + camera over /ws
    - R1LiteQuestTeleopModule: bimanual arms + grippers + chassis for the R1 Lite
"""

import asyncio
import math
import time
from typing import Any

from fastapi import WebSocket
from pydantic import Field

from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.Joy import Joy
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


class ArmTeleopConfig(QuestTeleopConfig):
    """Configuration for ArmTeleopModule.

    Attributes:
        task_names: Mapping of Hand -> coordinator task name. Used to set
            frame_id on output PoseStamped so the coordinator routes each
            hand's commands to the correct TeleopIKTask.
    """

    task_names: dict[str, str] = Field(default_factory=dict)


class ArmTeleopModule(QuestTeleopModule):
    """Quest teleop with per-hand press-and-hold engage and task name routing.

    Each controller's primary button (X for left, A for right)
    engages that hand while held, disengages on release.

    When task_names is configured, output PoseStamped messages have their
    frame_id set to the task name, enabling the coordinator to route
    each hand's commands to the correct TeleopIKTask.

    Outputs:
        - left_controller_output: PoseStamped (inherited)
        - right_controller_output: PoseStamped (inherited)
        - buttons: Buttons (inherited)
    """

    config: ArmTeleopConfig

    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)

        self._task_names: dict[Hand, str] = {
            Hand[k.upper()]: v for k, v in self.config.task_names.items()
        }

    def _publish_msg(self, hand: Hand, output_msg: PoseStamped) -> None:
        """Stamp frame_id with task name and publish."""
        task_name = self._task_names.get(hand)
        if task_name:
            output_msg = PoseStamped(
                position=output_msg.position,
                orientation=output_msg.orientation,
                ts=output_msg.ts,
                frame_id=task_name,
            )
        super()._publish_msg(hand, output_msg)

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


class VideoArmTeleopConfig(ArmTeleopConfig):
    """Configuration for VideoArmTeleopModule."""

    video_jpeg_quality: int = 70
    # JPEG encoding is GIL-heavy and can starve the control loop; turn the
    # headset camera off when smooth tracking matters more than the view.
    video_enabled: bool = True


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
        if not self.config.video_enabled:
            return
        _push_jpeg(self, msg, self.config.video_jpeg_quality)


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

    def _deadzone(self, v: float) -> float:
        return 0.0 if abs(v) < self.config.deadzone else v

    def _on_joy_bytes(self, data: bytes) -> None:
        super()._on_joy_bytes(data)
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

    async def handle_color_image(self, msg: Image) -> None:
        _push_jpeg(self, msg, self.config.video_jpeg_quality)

    @rpc
    def stop(self) -> None:
        # Send one zero Twist so the base halts if teleop dies mid-motion.
        try:
            self.cmd_vel.publish(Twist.zero())
        except Exception:
            logger.exception("Failed to publish stop Twist")
        super().stop()


class R1LiteQuestTeleopConfig(VideoArmTeleopConfig):
    """Configuration for R1LiteQuestTeleopModule."""

    linear_speed: float = 0.2  # m/s at full stick deflection
    angular_speed: float = 0.4  # rad/s at full stick deflection
    deadzone: float = 0.1
    joy_timeout: float = 0.5  # seconds without Joy before a controller stops driving
    motion_gain: float = 1.0  # scales hand position deltas; >1 = arm covers more than the hand
    # Soft radial deadband on the raw hand delta, applied before gain: motion
    # under this radius is ignored and larger motion is shortened by it, so
    # there is no jump at the threshold. Isolates wrist rotation (the
    # controller's tracked origin sweeps an arc when the wrist rolls) and
    # absorbs hand tremor while holding.
    position_deadband_m: float = 0.0
    # The WebXR frame's forward is wherever the operator faces; the arm frame
    # is the robot's forward. Facing the robot means 180.
    operator_yaw_deg: float = 0.0
    # Compose wrist rotation in the hand's own frame (twist about the hand
    # axis maps to twist about the gripper axis, independent of heading).
    # Requires rotation_frame "local" on the teleop tasks.
    local_rotation: bool = False
    # Publish the hand's current orientation instead of a delta. Requires
    # rotation_frame "absolute" on the teleop tasks, which map hand attitude
    # to gripper attitude through a session-fixed alignment: orientation
    # errors cannot accumulate across engages and returning the hand to its
    # reference attitude always returns the gripper. Takes precedence over
    # local_rotation.
    absolute_orientation: bool = False
    gripper_open: float = 100.0
    gripper_closed: float = 0.0


class R1LiteQuestTeleopModule(VideoArmTeleopModule):
    """Quest teleop for the Galaxea R1 Lite: arms, grippers and chassis from one headset.

    Arms: inherited per-hand press-and-hold engage (X/A) with task name routing
    to the per-arm TeleopIKTasks. Grippers: each trigger maps to that side's
    0-100 gripper command, streamed every control tick while the hand is
    engaged (the robot's gripper controller acts only on a continuous stream).
    Chassis: left stick drives forward/strafe, right stick X yaws, any
    thumbstick press commands zero. Twist and gripper commands publish from the
    50 Hz control loop, not per Joy message, so the chassis dead-man stays fed.
    A controller whose Joy stream went stale contributes zero velocity; its
    gripper holds by not publishing.

    Inputs:
        - color_image: In[Image] (inherited; wire to the head camera)

    Outputs:
        - left_controller_output / right_controller_output: PoseStamped (inherited)
        - buttons: Buttons (inherited)
        - cmd_vel: Twist chassis velocity
        - gripper_left_command / gripper_right_command: JointState, position[0] in 0-100
    """

    config: R1LiteQuestTeleopConfig

    cmd_vel: Out[Twist]
    gripper_left_command: Out[JointState]
    gripper_right_command: Out[JointState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._joy_rx_ts: dict[Hand, float] = {Hand.LEFT: 0.0, Hand.RIGHT: 0.0}
        # Telemetry (TELEM lines, 1 Hz from the control loop)
        self._telem_joy_count: dict[Hand, int] = {Hand.LEFT: 0, Hand.RIGHT: 0}
        self._telem_joy_gap_max: dict[Hand, float] = {Hand.LEFT: 0.0, Hand.RIGHT: 0.0}
        self._telem_pose_count = 0
        self._telem_loop_gap_max = 0.0
        self._telem_last_tick = 0.0
        self._telem_last_emit = 0.0

    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        # Send one zero Twist so the base halts if teleop dies mid-motion.
        try:
            self.cmd_vel.publish(Twist.zero())
        except Exception:
            logger.exception("Failed to publish stop Twist")
        super().stop()

    def _get_output_pose(self, hand: Hand) -> PoseStamped | None:
        current = self._current_poses.get(hand)
        initial = self._initial_poses.get(hand)
        if current is None or initial is None:
            return None
        dx = current.position.x - initial.position.x
        dy = current.position.y - initial.position.y
        dz = current.position.z - initial.position.z
        deadband = self.config.position_deadband_m
        if deadband > 0.0:
            norm = math.sqrt(dx * dx + dy * dy + dz * dz)
            if norm <= deadband:
                dx = dy = dz = 0.0
            else:
                shrink = (norm - deadband) / norm
                dx, dy, dz = dx * shrink, dy * shrink, dz * shrink
        gain = self.config.motion_gain
        dx, dy, dz = dx * gain, dy * gain, dz * gain
        yaw = math.radians(self.config.operator_yaw_deg)
        if yaw:
            cos_y, sin_y = math.cos(yaw), math.sin(yaw)
            dx, dy = cos_y * dx - sin_y * dy, sin_y * dx + cos_y * dy
        if self.config.absolute_orientation:
            orientation = current.orientation
        elif self.config.local_rotation:
            orientation = initial.orientation.inverse() * current.orientation
        else:
            orientation = current.orientation * initial.orientation.inverse()
        return PoseStamped(
            position=Vector3(dx, dy, dz),
            orientation=orientation,
            ts=current.ts,
            frame_id=current.frame_id,
        )

    def _on_joy_bytes(self, data: bytes) -> None:
        msg = Joy.lcm_decode(data)
        hand = Hand.LEFT if msg.frame_id == "left" else Hand.RIGHT
        try:
            controller = QuestControllerState.from_joy(msg, is_left=(hand == Hand.LEFT))
        except ValueError:
            logger.warning(
                f"Malformed Joy for {hand.name}: axes={len(msg.axes or [])}, "
                f"buttons={len(msg.buttons or [])}"
            )
            return
        with self._lock:
            self._controllers[hand] = controller
            now = time.monotonic()
            prev = self._joy_rx_ts[hand]
            if prev > 0.0:
                self._telem_joy_gap_max[hand] = max(self._telem_joy_gap_max[hand], now - prev)
            self._telem_joy_count[hand] += 1
            # Local receive time, not msg.ts: headset and robot clocks are not synced.
            self._joy_rx_ts[hand] = now

    def _deadzone(self, v: float) -> float:
        return 0.0 if abs(v) < self.config.deadzone else v

    def _fresh(self, hand: Hand, now: float) -> bool:
        return (now - self._joy_rx_ts[hand]) < self.config.joy_timeout

    def _chassis_twist(
        self,
        left: QuestControllerState | None,
        right: QuestControllerState | None,
        now: float,
    ) -> Twist:
        twist = Twist()
        twist.linear = Vector3(0.0, 0.0, 0.0)
        twist.angular = Vector3(0.0, 0.0, 0.0)
        left_ok = left is not None and self._fresh(Hand.LEFT, now)
        right_ok = right is not None and self._fresh(Hand.RIGHT, now)
        if (left_ok and left.thumbstick_press) or (right_ok and right.thumbstick_press):
            return twist
        if left_ok:
            twist.linear.x = -self._deadzone(left.thumbstick.y) * self.config.linear_speed
            twist.linear.y = -self._deadzone(left.thumbstick.x) * self.config.linear_speed
        if right_ok:
            twist.angular.z = -self._deadzone(right.thumbstick.x) * self.config.angular_speed
        return twist

    def _gripper_command(self, trigger: float) -> JointState:
        clamped = max(0.0, min(1.0, trigger))
        span = self.config.gripper_closed - self.config.gripper_open
        return JointState(position=[self.config.gripper_open + span * clamped])

    def _on_pose_bytes(self, data: bytes) -> None:
        super()._on_pose_bytes(data)
        with self._lock:
            self._telem_pose_count += 1

    def _publish_button_state(
        self,
        left: QuestControllerState | None,
        right: QuestControllerState | None,
    ) -> None:
        super()._publish_button_state(left, right)
        now = time.monotonic()
        if self._telem_last_tick > 0.0:
            self._telem_loop_gap_max = max(self._telem_loop_gap_max, now - self._telem_last_tick)
        self._telem_last_tick = now
        if now - self._telem_last_emit >= 1.0:
            logger.info(
                "TELEM quest: joyL_hz=%d joyR_hz=%d pose_hz=%d "
                "joy_gap_ms L=%.0f R=%.0f loop_gap_ms=%.0f engaged L=%s R=%s",
                self._telem_joy_count[Hand.LEFT],
                self._telem_joy_count[Hand.RIGHT],
                self._telem_pose_count,
                self._telem_joy_gap_max[Hand.LEFT] * 1000.0,
                self._telem_joy_gap_max[Hand.RIGHT] * 1000.0,
                self._telem_loop_gap_max * 1000.0,
                self._is_engaged[Hand.LEFT],
                self._is_engaged[Hand.RIGHT],
            )
            self._telem_last_emit = now
            self._telem_joy_count = {Hand.LEFT: 0, Hand.RIGHT: 0}
            self._telem_joy_gap_max = {Hand.LEFT: 0.0, Hand.RIGHT: 0.0}
            self._telem_pose_count = 0
            self._telem_loop_gap_max = 0.0
        self.cmd_vel.publish(self._chassis_twist(left, right, now))
        if left is not None and self._is_engaged[Hand.LEFT] and self._fresh(Hand.LEFT, now):
            self.gripper_left_command.publish(self._gripper_command(left.trigger))
        if right is not None and self._is_engaged[Hand.RIGHT] and self._fresh(Hand.RIGHT, now):
            self.gripper_right_command.publish(self._gripper_command(right.trigger))
