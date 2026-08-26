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
WebXR Teleoperation Module.

Receives VR controller tracking data from the WebXR client via an embedded
FastAPI WebSocket server.  Transforms from WebXR to robot frame, computes
deltas, and publishes PoseStamped commands.
"""

import asyncio
from dataclasses import dataclass
import json
import math
from pathlib import Path
import threading
import time
from typing import Any, TypeVar

from dimos_lcm.geometry_msgs import PoseStamped as LCMPoseStamped
from dimos_lcm.sensor_msgs import Joy as LCMJoy
from fastapi import WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from pydantic import Field
from reactivex.disposable import Disposable

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.imitation.collection.episode_monitor import EpisodeStatus
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.teleop.utils.teleop_transforms import webxr_to_robot

# Hand is re-exported for back-compat; it lives in controller_types.
from dimos.teleop.webxr.controller_types import Buttons, Hand, WebXRControllerState
from dimos.utils.logging_config import setup_logger
from dimos.web.robot_web_interface import RobotWebInterface

logger = setup_logger()

STATIC_DIR = Path(__file__).parent / "web" / "static"


async def _ws_send_text(ws: WebSocket, data: str) -> None:
    try:
        await ws.send_text(data)
    except Exception:
        # The receive loop removes disconnected clients. Outbound status
        # updates should not disrupt teleoperation while that happens.
        pass


@dataclass
class WebXRTeleopStatus:
    """Current teleoperation status."""

    left_engaged: bool
    right_engaged: bool
    left_pose: PoseStamped | None
    right_pose: PoseStamped | None
    buttons: Buttons


class WebXRTeleopConfig(ModuleConfig):
    """Configuration for WebXR Teleoperation Module."""

    control_loop_hz: float = 50.0
    server_port: int = 8443
    input_timeout_s: float = Field(default=1.0, gt=0)


_Config = TypeVar("_Config", bound=WebXRTeleopConfig)


class WebXRTeleopModule(Module):
    """WebXR Teleoperation Module for Meta WebXR controllers.

    Receives controller data from the WebXR client via an embedded WebSocket
    server, computes output poses, and publishes them.  Subclass to customize
    pose computation, output format, and engage behavior.

    Outputs:
        - left_controller_output: PoseStamped (output pose for left hand)
        - right_controller_output: PoseStamped (output pose for right hand)
        - teleop_buttons: Buttons (button states for both controllers)
    """

    config: WebXRTeleopConfig

    # Outputs: delta poses for each controller
    left_controller_output: Out[PoseStamped]
    right_controller_output: Out[PoseStamped]
    teleop_buttons: Out[Buttons]
    status: In[EpisodeStatus]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)

        # Engage state (per-hand)
        self._is_engaged: dict[Hand, bool] = {Hand.LEFT: False, Hand.RIGHT: False}
        self._initial_poses: dict[Hand, PoseStamped | None] = {Hand.LEFT: None, Hand.RIGHT: None}
        self._current_poses: dict[Hand, PoseStamped | None] = {Hand.LEFT: None, Hand.RIGHT: None}
        self._controllers: dict[Hand, WebXRControllerState | None] = {
            Hand.LEFT: None,
            Hand.RIGHT: None,
        }
        self._last_pose_update: dict[Hand, float | None] = {Hand.LEFT: None, Hand.RIGHT: None}
        self._last_controller_update: dict[Hand, float | None] = {
            Hand.LEFT: None,
            Hand.RIGHT: None,
        }
        self._lock = threading.RLock()
        self._translation_scale = 1.0

        # Control loop
        self._control_loop_thread: threading.Thread | None = None
        self._stop_event = threading.Event()

        # Embedded web server, initialized during the module start lifecycle.
        self._web_server: RobotWebInterface | None = None
        self._web_server_thread: threading.Thread | None = None

        # Fingerprint-based message dispatch table
        self._decoders: dict[bytes, Any] = {
            LCMPoseStamped._get_packed_fingerprint(): self._on_pose_bytes,
            LCMJoy._get_packed_fingerprint(): self._on_joy_bytes,
        }

        # Tracked here so subclasses can push from non-asyncio threads.
        # _clients_lock guards add/discard/snapshot of the set across the
        # uvicorn thread and the RX subscriber thread.
        self._connected_clients: set[WebSocket] = set()
        self._clients_lock = threading.Lock()
        self._ws_loop: asyncio.AbstractEventLoop | None = None
        self._latest_episode_status: EpisodeStatus | None = None

    def _setup_routes(self) -> None:
        """Register teleop routes on the embedded web server."""
        assert self._web_server is not None

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
            self._ws_loop = asyncio.get_running_loop()
            if not self._client_connected(ws):
                logger.warning("Rejecting additional WebXR control client")
                await ws.close(code=1008, reason="A WebXR control client is already connected")
                return
            logger.info("WebXR client connected")
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
                logger.info("WebXR client disconnected")
            except Exception:
                logger.exception("WebSocket error")
            finally:
                self._client_disconnected(ws)

    def _client_connected(self, ws: WebSocket) -> bool:
        with self._clients_lock:
            if self._connected_clients:
                return False
            self._connected_clients.add(ws)
            self._reset_controller_state()
        with self._lock:
            status = self._latest_episode_status
        if status is not None:
            self._broadcast_text(self._encode_episode_status(status))
        return True

    def _client_disconnected(self, ws: WebSocket) -> None:
        with self._clients_lock:
            was_connected = ws in self._connected_clients
            self._connected_clients.discard(ws)
            if was_connected:
                self._reset_controller_state()

    def _broadcast_text(self, data: str) -> None:
        """Schedule a text message for the active WebXR client."""
        loop = self._ws_loop
        if loop is None:
            return
        with self._clients_lock:
            clients = tuple(self._connected_clients)
        for ws in clients:
            asyncio.run_coroutine_threadsafe(_ws_send_text(ws, data), loop)

    def _on_episode_status(self, status: EpisodeStatus) -> None:
        with self._lock:
            self._latest_episode_status = status
        self._broadcast_text(self._encode_episode_status(status))

    @staticmethod
    def _encode_episode_status(status: EpisodeStatus) -> str:
        payload = status.model_dump(mode="json")
        payload["type"] = "episode_status"
        payload["elapsed_s"] = (
            max(0.0, time.time() - status.ts) if status.state == "recording" else 0.0
        )
        return json.dumps(payload, separators=(",", ":"))

    @rpc
    def build(self) -> None:
        super().build()
        if self.status.connection is not None or self.status._transport is not None:
            self.register_disposable(Disposable(self.status.subscribe(self._on_episode_status)))

    @rpc
    def start(self) -> None:
        super().start()
        self._web_server = RobotWebInterface(host="0.0.0.0", port=self.config.server_port)
        self._setup_routes()
        self._start_server()
        self._start_control_loop()
        logger.info("WebXR Teleoperation Module started")

    @rpc
    def stop(self) -> None:
        self._stop_control_loop()
        self._reset_controller_state()
        self._stop_server()
        super().stop()

    def _reset_controller_state(self) -> None:
        """Clear stale input and publish the zero-button safe command."""
        with self._lock:
            for hand in Hand:
                self._is_engaged[hand] = False
                self._initial_poses[hand] = None
                self._current_poses[hand] = None
                self._controllers[hand] = None
                self._last_pose_update[hand] = None
                self._last_controller_update[hand] = None
            self._publish_button_state(None, None)
            self._publish_safe_command()

    def _expire_stale_state(self, now: float) -> None:
        """Disengage hands whose pose or controller updates have timed out.

        Assumes ``self._lock`` is held.
        """
        input_expired = False
        for hand in Hand:
            pose_update = self._last_pose_update[hand]
            controller_update = self._last_controller_update[hand]
            pose_stale = pose_update is None or now - pose_update > self.config.input_timeout_s
            controller_stale = (
                controller_update is None or now - controller_update > self.config.input_timeout_s
            )
            input_expired |= (pose_stale and pose_update is not None) or (
                controller_stale and controller_update is not None
            )
            if pose_stale:
                self._current_poses[hand] = None
                self._last_pose_update[hand] = None
            if controller_stale:
                self._controllers[hand] = None
                self._last_controller_update[hand] = None
            if pose_stale or controller_stale:
                self._is_engaged[hand] = False
                self._initial_poses[hand] = None
        if input_expired:
            self._publish_safe_command()

    def _publish_safe_command(self) -> None:
        """Publish any subclass-specific command needed to stop motion."""

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

    def get_status(self) -> WebXRTeleopStatus:
        with self._lock:
            left = self._controllers.get(Hand.LEFT)
            right = self._controllers.get(Hand.RIGHT)
            return WebXRTeleopStatus(
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
            self._last_pose_update[hand] = time.monotonic()

    def _on_joy_bytes(self, data: bytes) -> bool:
        """Decode LCM bytes into Joy, parse into WebXRControllerState."""
        msg = Joy.lcm_decode(data)
        hand = self._resolve_hand(msg.frame_id)
        try:
            controller = WebXRControllerState.from_joy(msg, is_left=(hand == Hand.LEFT))
        except ValueError:
            logger.warning(
                f"Malformed Joy for {hand.name}: axes={len(msg.axes or [])}, buttons={len(msg.buttons or [])}"
            )
            with self._lock:
                self._controllers[hand] = None
                self._last_controller_update[hand] = None
                self._is_engaged[hand] = False
                self._initial_poses[hand] = None
            return False
        with self._lock:
            self._controllers[hand] = controller
            self._last_controller_update[hand] = time.monotonic()
        return True

    def _start_server(self) -> None:
        """Start the embedded FastAPI server with HTTPS in a daemon thread."""
        if self._web_server_thread is not None and self._web_server_thread.is_alive():
            logger.warning("Web server already running")
            return

        if self._web_server is None:
            return

        self._web_server_thread = threading.Thread(
            target=self._web_server.run,
            kwargs={"ssl": True, "ssl_certs_dir": DIMOS_PROJECT_ROOT / "assets" / "teleop_certs"},
            daemon=True,
            name="WebXRTeleopWebServer",
        )
        self._web_server_thread.start()
        logger.info(f"WebXR teleop web server started on https://0.0.0.0:{self.config.server_port}")

    def _stop_server(self) -> None:
        """Shutdown the embedded web server."""
        if self._web_server is None:
            return
        self._web_server.shutdown()
        if self._web_server_thread is not None:
            self._web_server_thread.join(timeout=3)
            self._web_server_thread = None
        logger.info("WebXR teleop web server stopped")

    def _start_control_loop(self) -> None:
        """Start the control loop thread."""
        if self._control_loop_thread is not None and self._control_loop_thread.is_alive():
            return

        self._stop_event.clear()
        self._control_loop_thread = threading.Thread(
            target=self._control_loop,
            daemon=True,
            name="WebXRTeleopControlLoop",
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
                    self._expire_stale_state(time.monotonic())
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
            position=delta.position * self._translation_scale,
            orientation=delta.orientation,
            ts=current_pose.ts,
            frame_id=current_pose.frame_id,
        )

    def _set_translation_scale(self, translation_scale: float) -> None:
        """Set the positive multiplier applied to controller position deltas."""
        if not math.isfinite(translation_scale) or translation_scale <= 0.0:
            raise ValueError("translation_scale must be finite and positive")
        with self._lock:
            self._translation_scale = translation_scale

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
        left: WebXRControllerState | None,
        right: WebXRControllerState | None,
    ) -> None:
        """Publish button states for both controllers.

        Override to customize button output format (e.g., different bit layout,
        keep analog values, add extra streams).
        """
        buttons = Buttons.from_controllers(left, right)
        self.teleop_buttons.publish(buttons)
