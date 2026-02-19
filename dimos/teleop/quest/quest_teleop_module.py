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

from dataclasses import dataclass
from enum import IntEnum
from pathlib import Path
import shutil
import signal
import subprocess
import threading
import time
from typing import Any

from reactivex.disposable import Disposable

from dimos.core import In, Module, Out, rpc
from dimos.core.module import ModuleConfig
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.msgs.sensor_msgs import Joy
from dimos.teleop.quest.quest_types import Buttons, QuestControllerState
from dimos.teleop.utils.teleop_transforms import webxr_to_robot
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class Hand(IntEnum):
    """Controller hand index."""

    LEFT = 0
    RIGHT = 1


@dataclass
class QuestTeleopStatus:
    """Current teleoperation status."""

    left_engaged: bool
    right_engaged: bool
    left_pose: PoseStamped | None
    right_pose: PoseStamped | None
    buttons: Buttons


@dataclass
class QuestTeleopConfig(ModuleConfig):
    """Configuration for Quest Teleoperation Module."""

    control_loop_hz: float = 50.0


class QuestTeleopModule(Module[QuestTeleopConfig]):
    """Quest Teleoperation Module for Meta Quest controllers.

    Gets controller data from Deno bridge, computes output poses, and publishes them. Subclass to customize pose
    computation, output format, and engage behavior.

    Outputs:
        - left_controller_output: PoseStamped (output pose for left hand)
        - right_controller_output: PoseStamped (output pose for right hand)
        - buttons: Buttons (button states for both controllers)
    """

    default_config = QuestTeleopConfig

    # Inputs from Deno bridge
    vr_left_pose: In[PoseStamped]
    vr_right_pose: In[PoseStamped]
    vr_left_joy: In[Joy]
    vr_right_joy: In[Joy]

    # Outputs: delta poses for each controller
    left_controller_output: Out[PoseStamped]
    right_controller_output: Out[PoseStamped]
    buttons: Out[Buttons]

    # -------------------------------------------------------------------------
    # Initialization
    # -------------------------------------------------------------------------

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)

        # Engage state (per-hand)
        self._is_engaged: dict[Hand, bool] = {Hand.LEFT: False, Hand.RIGHT: False}
        self._initial_poses: dict[Hand, PoseStamped | None] = {Hand.LEFT: None, Hand.RIGHT: None}
        self._current_poses: dict[Hand, PoseStamped | None] = {Hand.LEFT: None, Hand.RIGHT: None}
        self._controllers: dict[Hand, QuestControllerState | None] = {
            Hand.LEFT: None,
            Hand.RIGHT: None,
        }
        self._lock = threading.RLock()

        # Control loop
        self._control_loop_thread: threading.Thread | None = None
        self._stop_event = threading.Event()

        # Deno bridge server
        self._server_process: subprocess.Popen[bytes] | None = None
        self._server_script = Path(__file__).parent / "web" / "teleop_server.ts"

    # -------------------------------------------------------------------------
    # Lifecycle
    # -------------------------------------------------------------------------

    @rpc
    def start(self) -> None:
        super().start()

        input_streams = {
            "vr_left_pose": (self.vr_left_pose, lambda msg: self._on_pose(Hand.LEFT, msg)),
            "vr_right_pose": (self.vr_right_pose, lambda msg: self._on_pose(Hand.RIGHT, msg)),
            "vr_left_joy": (self.vr_left_joy, lambda msg: self._on_joy(Hand.LEFT, msg)),
            "vr_right_joy": (self.vr_right_joy, lambda msg: self._on_joy(Hand.RIGHT, msg)),
        }
        connected = []
        for name, (stream, handler) in input_streams.items():
            if not (stream and stream.transport):  # type: ignore[attr-defined]
                logger.warning(f"Stream '{name}' has no transport — skipping")
                continue
            self._disposables.add(Disposable(stream.subscribe(handler)))  # type: ignore[attr-defined]
            connected.append(name)

        if connected:
            logger.info(f"Subscribed to: {', '.join(connected)}")

        self._start_server()
        logger.info("Quest Teleoperation Module started")

    @rpc
    def stop(self) -> None:
        self._stop_control_loop()
        self._stop_server()
        super().stop()

    # -------------------------------------------------------------------------
    # Internal engage/disengage (assumes lock is held)
    # -------------------------------------------------------------------------

    def _engage(self, hand: Hand | None = None) -> bool:
        """Engage a hand. Assumes self._lock is held."""
        hands = [hand] if hand is not None else list(Hand)
        for h in hands:
            pose = self._current_poses.get(h)
            if pose is None:
                logger.error(f"Engage failed: {h.name.lower()} controller has no data")
                return False
            self._initial_poses[h] = pose
            self._is_engaged[h] = True
            logger.info(f"{h.name} engaged.")
        return True

    def _disengage(self, hand: Hand | None = None) -> None:
        """Disengage a hand. Assumes self._lock is held."""
        hands = [hand] if hand is not None else list(Hand)
        for h in hands:
            self._is_engaged[h] = False
            logger.info(f"{h.name} disengaged.")

    def get_status(self) -> QuestTeleopStatus:
        with self._lock:
            left = self._controllers.get(Hand.LEFT)
            right = self._controllers.get(Hand.RIGHT)
            return QuestTeleopStatus(
                left_engaged=self._is_engaged[Hand.LEFT],
                right_engaged=self._is_engaged[Hand.RIGHT],
                left_pose=self._current_poses.get(Hand.LEFT),
                right_pose=self._current_poses.get(Hand.RIGHT),
                buttons=Buttons.from_controllers(left, right),
            )

    # -------------------------------------------------------------------------
    # Callbacks and Control Loop
    # -------------------------------------------------------------------------

    def _on_pose(self, hand: Hand, pose_stamped: PoseStamped) -> None:
        """Callback for controller pose, converting WebXR to robot frame."""
        is_left = hand == Hand.LEFT
        robot_pose_stamped = webxr_to_robot(pose_stamped, is_left_controller=is_left)
        with self._lock:
            self._current_poses[hand] = robot_pose_stamped

    def _on_joy(self, hand: Hand, joy: Joy) -> None:
        """Callback for Joy message, parsing into QuestControllerState."""
        is_left = hand == Hand.LEFT
        try:
            controller = QuestControllerState.from_joy(joy, is_left=is_left)
        except ValueError:
            logger.warning(
                f"Malformed Joy for {hand.name}: axes={len(joy.axes or [])}, buttons={len(joy.buttons or [])}"
            )
            return
        with self._lock:
            self._controllers[hand] = controller

    # -------------------------------------------------------------------------
    # Deno Bridge Server
    # -------------------------------------------------------------------------

    def _start_server(self) -> None:
        """Launch the Deno WebSocket-to-LCM bridge server as a subprocess."""
        if self._server_process is not None and self._server_process.poll() is None:
            logger.warning("Deno bridge already running", pid=self._server_process.pid)
            return

        if shutil.which("deno") is None:
            logger.error(
                "Deno is not installed. Install it with: curl -fsSL https://deno.land/install.sh | sh"
            )
            return

        script = str(self._server_script)
        cmd = [
            "deno",
            "run",
            "--allow-net",
            "--allow-read",
            "--allow-run",
            "--allow-write",
            "--unstable-net",
            script,
        ]
        try:
            self._server_process = subprocess.Popen(cmd)
            logger.info(f"Deno bridge server started (pid {self._server_process.pid})")
        except OSError as e:
            logger.error(f"Failed to start Deno bridge: {e}")

    def _stop_server(self) -> None:
        """Terminate the Deno bridge server subprocess."""
        if self._server_process is None or self._server_process.poll() is not None:
            self._server_process = None
            return

        logger.info("Stopping Deno bridge server", pid=self._server_process.pid)
        self._server_process.send_signal(signal.SIGTERM)
        try:
            self._server_process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            logger.warning(
                "Deno bridge did not exit, sending SIGKILL", pid=self._server_process.pid
            )
            self._server_process.kill()
            try:
                self._server_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                logger.error("Deno bridge did not exit after SIGKILL")
        logger.info("Deno bridge server stopped")
        self._server_process = None

    def _start_control_loop(self) -> None:
        """Start the control loop thread."""
        if self._control_loop_thread is not None and self._control_loop_thread.is_alive():
            return

        self._stop_event.clear()
        self._control_loop_thread = threading.Thread(
            target=self._control_loop,
            daemon=True,
            name="QuestTeleopControlLoop",
        )
        self._control_loop_thread.start()
        logger.info(f"Control loop started at {self.config.control_loop_hz} Hz")

    def _stop_control_loop(self) -> None:
        """Stop the control loop thread."""
        self._stop_event.set()
        if self._control_loop_thread is not None:
            self._control_loop_thread.join(timeout=1.0)
            self._control_loop_thread = None
        logger.info("Control loop stopped")

    def _control_loop(self) -> None:
        """
        Holds self._lock for the entire iteration so overridable methods
        don't need to acquire it themselves.
        """
        period = 1.0 / self.config.control_loop_hz

        while not self._stop_event.is_set():
            loop_start = time.perf_counter()
            try:
                with self._lock:
                    self._handle_engage()

                    for hand in Hand:
                        if not self._should_publish(hand):
                            continue
                        output_pose = self._get_output_pose(hand)
                        if output_pose is not None:
                            self._publish_msg(hand, output_pose)

                    # Always publish buttons regardless of engage state,
                    # so UI/listeners can react to button presses (e.g., trigger engage).
                    left = self._controllers.get(Hand.LEFT)
                    right = self._controllers.get(Hand.RIGHT)
                    self._publish_button_state(left, right)
            except Exception:
                logger.exception("Error in teleop control loop")

            elapsed = time.perf_counter() - loop_start
            sleep_time = period - elapsed
            if sleep_time > 0:
                self._stop_event.wait(sleep_time)

    # -------------------------------------------------------------------------
    # Control Loop Internals
    # -------------------------------------------------------------------------

    def _handle_engage(self) -> None:
        """Check for engage button press and update per-hand engage state.

        Override to customize which button/action triggers engage.
        Default: Each controller's primary button (X/A) hold engages that hand.
        """
        for hand in Hand:
            controller = self._controllers.get(hand)
            if controller is None:
                continue
            if controller.primary:
                if not self._is_engaged[hand]:
                    self._engage(hand)
            else:
                if self._is_engaged[hand]:
                    self._disengage(hand)

    def _should_publish(self, hand: Hand) -> bool:
        """Check if we should publish commands for a hand.

        Override to add custom conditions.
        Default: Returns True if the hand is engaged.
        """
        return self._is_engaged[hand]

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

        delta = current_pose - initial_pose
        return PoseStamped(
            position=delta.position,
            orientation=delta.orientation,
            ts=current_pose.ts,
            frame_id=current_pose.frame_id,
        )

    def _publish_msg(self, hand: Hand, output_msg: PoseStamped) -> None:
        """Publish message for a controller.

        Override to customize output (e.g., convert to Twist, scale values).
        """
        if hand == Hand.LEFT:
            self.left_controller_output.publish(output_msg)
        else:
            self.right_controller_output.publish(output_msg)

    def _publish_button_state(
        self,
        left: QuestControllerState | None,
        right: QuestControllerState | None,
    ) -> None:
        """Publish button states for both controllers.

        Override to customize button output format (e.g., different bit layout,
        keep analog values, add extra streams).
        """
        buttons = Buttons.from_controllers(left, right)
        self.buttons.publish(buttons)


quest_teleop_module = QuestTeleopModule.blueprint

__all__ = [
    "Hand",
    "QuestTeleopConfig",
    "QuestTeleopModule",
    "QuestTeleopStatus",
    "quest_teleop_module",
]
