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
Hosted Teleop Module — Cloudflare Realtime client.

Robot is a CLIENT (not a server). On start it:

  1. Builds an aiortc PeerConnection with one DataChannel ``cmd_unreliable``
     (unordered, maxRetransmits=0 — UDP-like, drop stale frames).
  2. POSTs the SDP offer to the broker microservice
     (``{broker_url}/api/v1/sessions``). Broker proxies to Cloudflare
     Realtime SFU, returns the SDP answer + a session_id.
  3. Establishes the DataChannel; operator commands (PoseStamped + Joy)
     start arriving as bytes on cmd_unreliable.
  4. Heartbeats the broker at 1 Hz to keep the session listed.

On stop, deregisters with broker and closes the PC.

Iteration 1 — operator → robot only. State channels (robot → operator)
to be added later.
"""

from __future__ import annotations

import asyncio
from enum import IntEnum
import threading
import time
from typing import Any

from aiortc import (
    RTCConfiguration,
    RTCIceServer,
    RTCPeerConnection,
    RTCSessionDescription,
)
from dimos_lcm.geometry_msgs import PoseStamped as LCMPoseStamped
from dimos_lcm.sensor_msgs import Joy as LCMJoy
import httpx

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.teleop.quest.quest_types import Buttons, QuestControllerState
from dimos.teleop.utils.teleop_transforms import webxr_to_robot
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class Hand(IntEnum):
    LEFT = 0
    RIGHT = 1


class HostedTeleopConfig(ModuleConfig):
    """Configuration for HostedTeleopModule."""

    control_loop_hz: float = 50.0

    # Broker — the microservice that mediates SDP exchange with Cloudflare
    # Realtime. Robots and operators only ever talk to this; Cloudflare
    # credentials live on the broker.
    broker_url: str = "https://teleop.dimensional-apps.com"
    broker_api_key: str = ""

    # Robot identity for the broker's session list.
    robot_id: str = ""
    robot_name: str = ""

    # ICE — STUN works in most cases; configure TURN for restrictive networks.
    stun_urls: list[str] = ["stun:stun.cloudflare.com:3478"]
    turn_urls: list[str] = []
    turn_username: str = ""
    turn_credential: str = ""

    heartbeat_hz: float = 1.0


class HostedTeleopModule(Module):
    """Cloudflare-Realtime-based teleop client.

    Outputs (operator → robot, published from received cmd_unreliable bytes):
      - ``left_controller_output: Out[PoseStamped]``
      - ``right_controller_output: Out[PoseStamped]``
      - ``buttons: Out[Buttons]``

    Subclass to override engagement, output formatting, or button packing —
    same hooks as ``QuestTeleopModule``.
    """

    config: HostedTeleopConfig

    left_controller_output: Out[PoseStamped]
    right_controller_output: Out[PoseStamped]
    buttons: Out[Buttons]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)

        # Per-hand state
        self._is_engaged: dict[Hand, bool] = {Hand.LEFT: False, Hand.RIGHT: False}
        self._initial_poses: dict[Hand, PoseStamped | None] = {
            Hand.LEFT: None,
            Hand.RIGHT: None,
        }
        self._current_poses: dict[Hand, PoseStamped | None] = {
            Hand.LEFT: None,
            Hand.RIGHT: None,
        }
        self._controllers: dict[Hand, QuestControllerState | None] = {
            Hand.LEFT: None,
            Hand.RIGHT: None,
        }
        self._lock = threading.RLock()

        # asyncio loop running in a dedicated thread (aiortc + httpx are async)
        self._loop: asyncio.AbstractEventLoop | None = None
        self._loop_thread: threading.Thread | None = None

        # WebRTC + broker state
        self._pc: RTCPeerConnection | None = None
        self._http: httpx.AsyncClient | None = None
        self._session_id: str | None = None

        # Background threads
        self._control_loop_thread: threading.Thread | None = None
        self._heartbeat_thread: threading.Thread | None = None
        self._stop_event = threading.Event()

        # Fingerprint dispatch — fired in asyncio context from DataChannel.on_message
        self._decoders: dict[bytes, Any] = {
            LCMPoseStamped._get_packed_fingerprint(): self._on_pose_bytes,
            LCMJoy._get_packed_fingerprint(): self._on_joy_bytes,
        }

    # ─── Lifecycle ───────────────────────────────────────────────────────────

    @rpc
    def start(self) -> None:
        super().start()
        self._start_event_loop()
        self._connect_blocking()
        self._start_heartbeat()
        self._start_control_loop()
        logger.info("HostedTeleopModule started")

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._control_loop_thread is not None:
            self._control_loop_thread.join(timeout=1.0)
            self._control_loop_thread = None
        if self._heartbeat_thread is not None:
            self._heartbeat_thread.join(timeout=2.0)
            self._heartbeat_thread = None
        if self._loop is not None and self._loop.is_running():
            try:
                asyncio.run_coroutine_threadsafe(self._disconnect(), self._loop).result(timeout=5.0)
            except Exception:
                logger.exception("Error during disconnect")
        self._stop_event_loop()
        super().stop()

    # ─── Event loop thread ───────────────────────────────────────────────────

    def _start_event_loop(self) -> None:
        ready = threading.Event()

        def runner() -> None:
            self._loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self._loop)
            ready.set()
            self._loop.run_forever()

        self._loop_thread = threading.Thread(target=runner, daemon=True, name="HostedTeleopLoop")
        self._loop_thread.start()
        ready.wait()

    def _stop_event_loop(self) -> None:
        if self._loop is not None and self._loop.is_running():
            self._loop.call_soon_threadsafe(self._loop.stop)
        if self._loop_thread is not None:
            self._loop_thread.join(timeout=2.0)
            self._loop_thread = None
        self._loop = None

    # ─── WebRTC + broker (async, runs on event loop thread) ──────────────────

    def _connect_blocking(self) -> None:
        """Sync wrapper: schedule _connect() and wait."""
        assert self._loop is not None, "_start_event_loop() must be called first"
        future = asyncio.run_coroutine_threadsafe(self._connect(), self._loop)
        future.result(timeout=20.0)

    async def _connect(self) -> None:
        """Open a WebRTC connection via the broker.

        Steps:
          1. Build PC + DataChannel('cmd_unreliable', unordered, no-retransmit)
          2. createOffer + setLocalDescription
          3. Wait for ICE gathering complete (non-trickle)
          4. POST {broker}/api/v1/sessions { sdp, type, robot_id, robot_name }
          5. Apply the returned SDP answer
        """
        self._http = httpx.AsyncClient(timeout=10.0)

        ice_servers = [RTCIceServer(urls=u) for u in self.config.stun_urls]
        for url in self.config.turn_urls or []:
            ice_servers.append(
                RTCIceServer(
                    urls=url,
                    username=self.config.turn_username or None,
                    credential=self.config.turn_credential or None,
                )
            )

        self._pc = RTCPeerConnection(RTCConfiguration(iceServers=ice_servers))

        # Create cmd_unreliable so the SDP advertises a data plane.
        # Operator publishes onto this same channel from the browser; the
        # SFU forwards. Bytes arrive here as on_message events.
        cmd_channel = self._pc.createDataChannel(
            "cmd_unreliable",
            ordered=False,
            maxRetransmits=0,
        )

        @cmd_channel.on("open")
        def _on_open() -> None:
            logger.info("cmd_unreliable channel open")

        @cmd_channel.on("message")
        def _on_message(data: Any) -> None:  # bytes from the operator
            if isinstance(data, bytes):
                self._dispatch_bytes(data)

        @self._pc.on("connectionstatechange")
        async def _on_state() -> None:
            assert self._pc is not None
            logger.info(f"PC state: {self._pc.connectionState}")

        offer = await self._pc.createOffer()
        await self._pc.setLocalDescription(offer)

        # Non-trickle ICE: wait for gathering before posting.
        if self._pc.iceGatheringState != "complete":
            done: asyncio.Future[None] = asyncio.get_event_loop().create_future()

            @self._pc.on("icegatheringstatechange")
            def _on_gathering() -> None:
                assert self._pc is not None
                if self._pc.iceGatheringState == "complete" and not done.done():
                    done.set_result(None)

            await done

        # Register with broker. Field names match dimensional-teleop's
        # POST /api/v1/sessions schema (sdp_offer in, sdp_answer out).
        url = f"{self.config.broker_url.rstrip('/')}/api/v1/sessions"
        headers = self._auth_headers()
        body = {
            "robot_id": self.config.robot_id,
            "robot_name": self.config.robot_name,
            "sdp_offer": self._pc.localDescription.sdp,
        }
        resp = await self._http.post(url, json=body, headers=headers)
        resp.raise_for_status()
        data = resp.json()
        self._session_id = data["session_id"]

        await self._pc.setRemoteDescription(
            RTCSessionDescription(sdp=data["sdp_answer"], type="answer")
        )
        logger.info(
            f"Registered with broker: session_id={self._session_id}, "
            f"cf_session_id={data.get('cf_session_id')}"
        )

    async def _disconnect(self) -> None:
        if self._http is not None and self._session_id is not None:
            try:
                url = f"{self.config.broker_url.rstrip('/')}/api/v1/sessions/{self._session_id}"
                await self._http.delete(url, headers=self._auth_headers())
            except Exception:
                logger.exception("Failed to deregister with broker")
        if self._pc is not None:
            await self._pc.close()
            self._pc = None
        if self._http is not None:
            await self._http.aclose()
            self._http = None
        self._session_id = None

    def _auth_headers(self) -> dict[str, str]:
        """Robot auth header — dimensional-teleop expects ``X-Robot-API-Key``.

        The configured key must be registered server-side and mapped to a
        ``robot_id`` matching ``config.robot_id``. The dev broker
        (``dev_broker.py``) ignores auth entirely.
        """
        if self.config.broker_api_key:
            return {"X-Robot-API-Key": self.config.broker_api_key}
        return {}

    # ─── Heartbeat ──────────────────────────────────────────────────────────

    def _start_heartbeat(self) -> None:
        def runner() -> None:
            interval = 1.0 / max(self.config.heartbeat_hz, 0.1)
            while not self._stop_event.is_set():
                if self._loop is not None and self._loop.is_running() and self._session_id:
                    try:
                        asyncio.run_coroutine_threadsafe(self._heartbeat(), self._loop).result(
                            timeout=2.0
                        )
                    except Exception:
                        logger.warning("Heartbeat failed (broker unreachable?)")
                self._stop_event.wait(interval)

        self._heartbeat_thread = threading.Thread(
            target=runner, daemon=True, name="HostedTeleopHeartbeat"
        )
        self._heartbeat_thread.start()

    async def _heartbeat(self) -> None:
        if self._http is None or self._session_id is None:
            return
        url = f"{self.config.broker_url.rstrip('/')}/api/v1/sessions/{self._session_id}/heartbeat"
        await self._http.post(url, headers=self._auth_headers())

    # ─── Bytes dispatch (operator → robot) ──────────────────────────────────

    def _dispatch_bytes(self, data: bytes) -> None:
        """Route raw LCM bytes to the right handler by 8-byte fingerprint."""
        decoder = self._decoders.get(data[:8])
        if decoder:
            decoder(data)
        else:
            logger.warning(f"Unknown message fingerprint: {data[:8].hex()}")

    def _on_pose_bytes(self, data: bytes) -> None:
        msg = PoseStamped.lcm_decode(data)
        try:
            hand = self._resolve_hand(msg.frame_id)
        except ValueError:
            return
        robot_pose = webxr_to_robot(msg, is_left_controller=(hand == Hand.LEFT))
        with self._lock:
            self._current_poses[hand] = robot_pose

    def _on_joy_bytes(self, data: bytes) -> None:
        msg = Joy.lcm_decode(data)
        try:
            hand = self._resolve_hand(msg.frame_id)
        except ValueError:
            return
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

    @staticmethod
    def _resolve_hand(frame_id: str) -> Hand:
        if frame_id == "left":
            return Hand.LEFT
        if frame_id == "right":
            return Hand.RIGHT
        raise ValueError(f"Unexpected frame_id: {frame_id!r}")

    # ─── Control loop (publishes Out streams) ───────────────────────────────

    def _start_control_loop(self) -> None:
        self._stop_event.clear()
        self._control_loop_thread = threading.Thread(
            target=self._control_loop,
            daemon=True,
            name="HostedTeleopControlLoop",
        )
        self._control_loop_thread.start()

    def _control_loop(self) -> None:
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
                    left = self._controllers.get(Hand.LEFT)
                    right = self._controllers.get(Hand.RIGHT)
                    self._publish_button_state(left, right)
            except Exception:
                logger.exception("Error in control loop")

            elapsed = time.perf_counter() - loop_start
            sleep_time = period - elapsed
            if sleep_time > 0:
                self._stop_event.wait(sleep_time)

    def _handle_engage(self) -> None:
        """Press-and-hold engage on each controller's primary button."""
        for hand in Hand:
            controller = self._controllers.get(hand)
            if controller is None:
                continue
            if controller.primary:
                if not self._is_engaged[hand]:
                    pose = self._current_poses.get(hand)
                    if pose is None:
                        logger.error(
                            f"Engage failed: {hand.name.lower()} controller has no pose data"
                        )
                        continue
                    self._initial_poses[hand] = pose
                    self._is_engaged[hand] = True
                    logger.info(f"{hand.name} engaged.")
            else:
                if self._is_engaged[hand]:
                    self._is_engaged[hand] = False
                    logger.info(f"{hand.name} disengaged.")

    def _should_publish(self, hand: Hand) -> bool:
        return self._is_engaged[hand]

    def _get_output_pose(self, hand: Hand) -> PoseStamped | None:
        """Default: delta from initial engaged pose."""
        current = self._current_poses.get(hand)
        initial = self._initial_poses.get(hand)
        if current is None or initial is None:
            return None
        delta = current - initial
        return PoseStamped(
            position=delta.position,
            orientation=delta.orientation,
            ts=current.ts,
            frame_id=current.frame_id,
        )

    def _publish_msg(self, hand: Hand, output_msg: PoseStamped) -> None:
        if hand == Hand.LEFT:
            self.left_controller_output.publish(output_msg)
        else:
            self.right_controller_output.publish(output_msg)

    def _publish_button_state(
        self,
        left: QuestControllerState | None,
        right: QuestControllerState | None,
    ) -> None:
        buttons = Buttons.from_controllers(left, right)
        self.buttons.publish(buttons)
