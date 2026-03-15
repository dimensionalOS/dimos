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

Receives VR controller tracking data from the Quest web app via an embedded
FastAPI WebSocket server.  Transforms from WebXR to robot frame, computes
deltas, and publishes PoseStamped commands.
"""

from dataclasses import dataclass
from enum import IntEnum
from pathlib import Path
import threading
import time
from typing import Any, TypeVar

from dimos_lcm.geometry_msgs import PoseStamped as LCMPoseStamped
from dimos_lcm.sensor_msgs import Joy as LCMJoy
from fastapi import WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.teleop.quest.quest_types import Buttons, QuestControllerState
from dimos.teleop.utils.teleop_transforms import webxr_to_robot
from dimos.utils.logging_config import setup_logger
from dimos.utils.path_utils import get_project_root
from dimos.web.robot_web_interface import RobotWebInterface

logger = setup_logger()

STATIC_DIR = Path(__file__).parent / "web" / "static"


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


class QuestTeleopConfig(ModuleConfig):
    """Configuration for Quest Teleoperation Module."""

    control_loop_hz: float = 50.0
    server_port: int = 8443


_Config = TypeVar("_Config", bound=QuestTeleopConfig)


class QuestTeleopModule(Module[_Config]):
    """Quest Teleoperation Module for Meta Quest controllers.

    Receives controller data from the Quest web app via an embedded WebSocket
    server, computes output poses, and publishes them.  Subclass to customize
    pose computation, output format, and engage behavior.

    Outputs:
        - left_controller_output: PoseStamped (output pose for left hand)
        - right_controller_output: PoseStamped (output pose for right hand)
        - buttons: Buttons (button states for both controllers)
    """

    default_config = QuestTeleopConfig  # type: ignore[assignment]

    # Outputs: delta poses for each controller
    left_controller_output: Out[PoseStamped]
    right_controller_output: Out[PoseStamped]
    buttons: Out[Buttons]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)

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

        # Embedded web server — RobotWebInterface provides FastAPI app + run()/shutdown()
        self._web_server = RobotWebInterface(port=self.config.server_port)
        self._web_server_thread: threading.Thread | None = None

        # Fingerprint-based message dispatch table
        self._decoders: dict[bytes, Any] = {
            LCMPoseStamped._get_packed_fingerprint(): self._on_pose_bytes,
            LCMJoy._get_packed_fingerprint(): self._on_joy_bytes,
        }

        self._setup_routes()

    def _setup_routes(self) -> None:
        """Register teleop routes on the embedded web server."""

        @self._web_server.app.get("/teleop", response_class=HTMLResponse)
        async def teleop_index() -> HTMLResponse:
            index_path = STATIC_DIR / "index.html"
            return HTMLResponse(content=index_path.read_text())

        if STATIC_DIR.is_dir():
            self._web_server.app.mount(
                "/static", StaticFiles(directory=str(STATIC_DIR)), name="teleop_static"
            )

        @self._web_server.app.websocket("/ws")
        async def websocket_endpoint(ws: WebSocket) -> None:
            await ws.accept()
            logger.info("Quest client connected")
            try:
                while True:
                    data = await ws.receive_bytes()
                    fingerprint = data[:8]
                    decoder = self._decoders.get(fingerprint)
                    if decoder:
                        decoder(data)
                    else:
                        logger.warning(f"Unknown message fingerprint: {fingerprint.hex()}")
            except WebSocketDisconnect:
                logger.info("Quest client disconnected")
            except Exception:
                logger.exception("WebSocket error")

    @rpc
    def start(self) -> None:
        super().start()
        self._start_server()
        self._start_control_loop()
        logger.info("Quest Teleoperation Module started")

    @rpc
    def stop(self) -> None:
        self._stop_control_loop()
        self._stop_server()
        super().stop()

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

    @staticmethod
    def _resolve_hand(frame_id: str) -> Hand:
        if frame_id == "left":
            return Hand.LEFT
        elif frame_id == "right":
            return Hand.RIGHT
        raise ValueError(f"Unexpected frame_id: {frame_id!r}, expected 'left' or 'right'")

    def _on_pose_bytes(self, data: bytes) -> None:
        """Decode LCM bytes into PoseStamped, transform to robot frame."""
        msg = PoseStamped.lcm_decode(data)
        hand = self._resolve_hand(msg.frame_id)
        robot_pose = webxr_to_robot(msg, is_left_controller=(hand == Hand.LEFT))
        with self._lock:
            self._current_poses[hand] = robot_pose

    def _on_joy_bytes(self, data: bytes) -> None:
        """Decode LCM bytes into Joy, parse into QuestControllerState."""
        msg = Joy.lcm_decode(data)
        hand = Hand.LEFT if msg.frame_id == "left" else Hand.RIGHT
        try:
            controller = QuestControllerState.from_joy(msg, is_left=(hand == Hand.LEFT))
        except ValueError:
            logger.warning(
                f"Malformed Joy for {hand.name}: axes={len(msg.axes or [])}, buttons={len(msg.buttons or [])}"
            )
            return
        with self._lock:
            self._controllers[hand] = controller

    def _start_server(self) -> None:
        """Start the embedded FastAPI server with HTTPS in a daemon thread."""
        if self._web_server_thread is not None and self._web_server_thread.is_alive():
            logger.warning("Web server already running")
            return

        self._web_server_thread = threading.Thread(
            target=self._web_server.run,
            kwargs={"ssl": True, "ssl_certs_dir": get_project_root() / "assets" / "teleop_certs"},
            daemon=True,
            name="QuestTeleopWebServer",
        )
        self._web_server_thread.start()
        logger.info(f"Quest teleop web server started on https://0.0.0.0:{self.config.server_port}")

    def _stop_server(self) -> None:
        """Shutdown the embedded web server."""
        self._web_server.shutdown()
        if self._web_server_thread is not None:
            self._web_server_thread.join(timeout=3)
            self._web_server_thread = None
        logger.info("Quest teleop web server stopped")

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
