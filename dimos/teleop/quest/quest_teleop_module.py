#!/usr/bin/env python3
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

"""
Quest Teleoperation Module.

Receives VR controller tracking data via LCM from Deno bridge,
transforms from WebXR to robot frame, computes deltas, and publishes PoseStamped commands.
"""

import threading
import time
from dataclasses import dataclass
from enum import IntEnum
from typing import Any

from reactivex.disposable import Disposable

from dimos.core import In, Module, Out, rpc
from dimos.core.module import ModuleConfig
from dimos.msgs.geometry_msgs import Pose, PoseStamped
from dimos.msgs.sensor_msgs import Joy
from dimos.teleop.base import TeleopProtocol
from dimos.teleop.quest.quest_types import QuestButtons, QuestController
from dimos.teleop.utils.teleop_transforms import webxr_to_robot
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class Hand(IntEnum):
    """Controller hand index."""

    LEFT = 0
    RIGHT = 1


@dataclass
class QuestTeleopConfig(ModuleConfig):
    """Configuration for Quest Teleoperation Module."""

    control_loop_hz: float = 50.0


class QuestTeleopModule(Module[QuestTeleopConfig], TeleopProtocol):
    """Quest Teleoperation Module for Meta Quest controllers.

    Subscribes to controller data from Deno bridge, transforms WebXR→robot frame,
    computes deltas from initial pose, and publishes PoseStamped commands.

    Implements TeleopProtocol.
    """

    default_config = QuestTeleopConfig

    # Inputs from Deno bridge
    vr_left_pose: In[PoseStamped]
    vr_right_pose: In[PoseStamped]
    vr_left_joy: In[Joy]
    vr_right_joy: In[Joy]

    # Outputs: delta poses for each controller
    left_controller_delta: Out[PoseStamped]
    right_controller_delta: Out[PoseStamped]
    buttons: Out[QuestButtons]

    # -------------------------------------------------------------------------
    # Initialization
    # -------------------------------------------------------------------------

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)

        # Engage state
        self._is_engaged = False
        self._initial_poses: dict[Hand, PoseStamped | None] = {
            Hand.LEFT: None,
            Hand.RIGHT: None,
        }
        self._current_poses: dict[Hand, PoseStamped | None] = {
            Hand.LEFT: None,
            Hand.RIGHT: None,
        }
        self._controllers: dict[Hand, QuestController | None] = {
            Hand.LEFT: None,
            Hand.RIGHT: None,
        }
        self._lock = threading.Lock()

        # Control loop
        self._control_loop_thread: threading.Thread | None = None
        self._control_loop_running = False

        logger.info("QuestTeleopModule initialized")

    # -------------------------------------------------------------------------
    # Public RPC Methods
    # -------------------------------------------------------------------------

    @rpc
    def start(self) -> None:
        """Start the Quest teleoperation module."""
        super().start()

        subscriptions = [
            (self.vr_left_pose, lambda msg: self._on_pose_cb(Hand.LEFT, msg)),
            (self.vr_right_pose, lambda msg: self._on_pose_cb(Hand.RIGHT, msg)),
            (self.vr_left_joy, lambda msg: self._on_joy_cb(Hand.LEFT, msg)),
            (self.vr_right_joy, lambda msg: self._on_joy_cb(Hand.RIGHT, msg)),
        ]
        for stream, handler in subscriptions:
            if stream and stream.transport:
                self._disposables.add(Disposable(stream.subscribe(handler)))

        self._start_control_loop()
        logger.info("Quest Teleoperation Module started")

    @rpc
    def stop(self) -> None:
        """Stop the Quest teleoperation module."""
        logger.info("Stopping Quest Teleoperation Module...")
        self._stop_control_loop()
        super().stop()

    @rpc
    def engage(self) -> bool:
        """Engage teleoperation by capturing current poses as reference."""
        logger.info("Engaging...")

        for hand in Hand:
            pose = self._current_poses.get(hand)
            if pose is None:
                logger.error(f"Engage failed: {hand.name.lower()} controller has no data")
                return False
            self._initial_poses[hand] = pose

        self._is_engaged = True
        logger.info("Engaged.")
        return True

    @rpc
    def disengage(self) -> None:
        """Disengage teleoperation."""
        self._is_engaged = False
        logger.info("Disengaged.")

    @rpc
    def get_status(self) -> dict[str, Any]:
        """Get current teleoperation status."""
        left = self._controllers.get(Hand.LEFT)
        right = self._controllers.get(Hand.RIGHT)
        return {
            "is_engaged": self._is_engaged,
            "left_has_data": self._current_poses.get(Hand.LEFT) is not None,
            "right_has_data": self._current_poses.get(Hand.RIGHT) is not None,
            "left_trigger": left.trigger if left else 0.0,
            "right_trigger": right.trigger if right else 0.0,
            "left_grip": left.grip if left else 0.0,
            "right_grip": right.grip if right else 0.0,
        }

    # -------------------------------------------------------------------------
    # Private Internal Methods
    # -------------------------------------------------------------------------

    def _on_pose_cb(self, hand: Hand, pose_stamped: PoseStamped) -> None:
        """Callback for controller pose, converting WebXR to robot frame."""
        is_left = hand == Hand.LEFT
        robot_pose_stamped = webxr_to_robot(pose_stamped, is_left_controller=is_left)
        with self._lock:
            self._current_poses[hand] = robot_pose_stamped

    def _on_joy_cb(self, hand: Hand, msg: Joy) -> None:
        """Callback for Joy message, parsing into QuestController."""
        is_left = hand == Hand.LEFT
        controller = QuestController.from_joy(msg, is_left=is_left)
        with self._lock:
            self._controllers[hand] = controller

    def _start_control_loop(self) -> None:
        """Start the control loop thread."""
        if self._control_loop_running:
            return

        self._control_loop_running = True
        self._control_loop_thread = threading.Thread(
            target=self._control_loop,
            daemon=True,
            name="QuestTeleopControlLoop",
        )
        self._control_loop_thread.start()
        logger.info(f"Control loop started at {self.config.control_loop_hz} Hz")

    def _stop_control_loop(self) -> None:
        """Stop the control loop thread."""
        self._control_loop_running = False
        if self._control_loop_thread is not None:
            self._control_loop_thread.join(timeout=1.0)
            self._control_loop_thread = None
        logger.info("Control loop stopped")

    def _control_loop(self) -> None:
        """Main control loop: compute deltas and publish at fixed rate."""
        period = 1.0 / self.config.control_loop_hz

        while self._control_loop_running:
            loop_start = time.perf_counter()

            # Check engage button (rising edge detection)
            self._handle_engage()

            if self._should_publish():
                for hand in Hand:
                    output_pose = self._get_output_pose(hand)
                    if output_pose is not None:
                        self._publish_pose(hand, output_pose)

                self._publish_button_state(
                    self._controllers.get(Hand.LEFT),
                    self._controllers.get(Hand.RIGHT),
                )

            elapsed = time.perf_counter() - loop_start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    # -------------------------------------------------------------------------
    # Overridable Methods (for subclasses)
    # -------------------------------------------------------------------------

    def _handle_engage(self) -> None:
        """Check for engage button press and toggle engage state.

        Override to customize which button triggers engage, use
        different detection logic (e.g., hold detection), or add haptic feedback.
        Default: Left controller X button (primary) toggles engage.
        """
        left = self._controllers.get(Hand.LEFT)
        if left is None:
            return

        if left.primary:
            if not self._is_engaged:
                self.engage()
        else:
            if self._is_engaged:
                self.disengage()

    def _should_publish(self) -> bool:
        """Check if we should publish commands.

        Override to add custom conditions (e.g., safety checks, workspace limits).
        Default: Returns True if engaged.
        """
        return self._is_engaged

    def _get_output_pose(self, hand: Hand) -> PoseStamped | None:
        """Get the pose to publish for a controller.

        Override to customize pose computation (e.g., send absolute pose,
        apply scaling, add filtering).
        Default: Computes delta from initial pose.
        """
        current_pose = self._current_poses.get(hand)
        initial_pose = self._initial_poses.get(hand)

        if current_pose is None or initial_pose is None:
            return None

        delta_pose = Pose(current_pose) - Pose(initial_pose)

        return PoseStamped(
            position=delta_pose.position,
            orientation=delta_pose.orientation,
            ts=current_pose.ts,
            frame_id=current_pose.frame_id,
        )

    def _publish_pose(self, hand: Hand, pose: PoseStamped) -> None:
        """Publish pose for a controller.

        Override to customize pose output (e.g., convert to Twist, scale values).
        """
        if hand == Hand.LEFT:
            self.left_controller_delta.publish(pose)
        else:
            self.right_controller_delta.publish(pose)

    def _publish_button_state(
        self,
        left: QuestController | None,
        right: QuestController | None,
    ) -> None:
        """Publish button states for both controllers.

        Override to customize button output format (e.g., different bit layout,
        keep analog values, add extra streams).
        """
        buttons = QuestButtons.from_controllers(left, right)
        self.buttons.publish(buttons)


quest_teleop_module = QuestTeleopModule.blueprint

__all__ = ["Hand", "QuestTeleopModule", "QuestTeleopConfig", "quest_teleop_module"]
