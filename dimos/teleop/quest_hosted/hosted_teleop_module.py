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

"""Hosted Teleop Module — Cloudflare Realtime SFU client.

Robot dials out to a broker (``dimensional-teleop``); operator commands
arrive on a negotiated ``cmd_unreliable`` DataChannel bound to an SCTP id
the broker hands us via heartbeat ack after an operator joins.
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
from dimos_lcm.geometry_msgs import PoseStamped as LCMPoseStamped, TwistStamped as LCMTwistStamped
from dimos_lcm.sensor_msgs import Joy as LCMJoy
import httpx

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.teleop.quest.quest_types import Buttons, QuestControllerState
from dimos.teleop.utils.teleop_transforms import webxr_to_robot
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class Hand(IntEnum):
    LEFT = 0
    RIGHT = 1


class HostedTeleopConfig(ModuleConfig):
    control_loop_hz: float = 50.0

    broker_url: str = "https://teleop.dimensionalos.com"
    broker_api_key: str = ""
    robot_id: str = ""
    robot_name: str = ""

    stun_urls: list[str] = ["stun:stun.cloudflare.com:3478"]
    turn_urls: list[str] = []
    turn_username: str = ""
    turn_credential: str = ""

    heartbeat_hz: float = 1.0


class HostedTeleopModule(Module):
    """Cloudflare-Realtime-based teleop client.

    Override hooks: ``_handle_engage``, ``_should_publish``,
    ``_get_output_pose``, ``_publish_msg``, ``_publish_button_state``.
    """

    config: HostedTeleopConfig

    left_controller_output: Out[PoseStamped]
    right_controller_output: Out[PoseStamped]
    buttons: Out[Buttons]
    cmd_vel: Out[Twist]
    cmd_vel_stamped: Out[TwistStamped]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)

        self._is_engaged: dict[Hand, bool] = {Hand.LEFT: False, Hand.RIGHT: False}
        self._initial_poses: dict[Hand, PoseStamped | None] = {Hand.LEFT: None, Hand.RIGHT: None}
        self._current_poses: dict[Hand, PoseStamped | None] = {Hand.LEFT: None, Hand.RIGHT: None}
        self._controllers: dict[Hand, QuestControllerState | None] = {
            Hand.LEFT: None,
            Hand.RIGHT: None,
        }
        self._lock = threading.RLock()

        # aiortc + httpx are async; run them on a dedicated event loop thread.
        self._loop: asyncio.AbstractEventLoop | None = None
        self._loop_thread: threading.Thread | None = None

        self._pc: RTCPeerConnection | None = None
        self._http: httpx.AsyncClient | None = None
        self._session_id: str | None = None

        # cmd_unreliable is opened lazily as a negotiated channel once the
        # broker reports an SCTP id via heartbeat ack (after an operator joins).
        self._cmd_channel = None
        self._cmd_channel_id: int | None = None

        self._control_loop_thread: threading.Thread | None = None
        self._heartbeat_thread: threading.Thread | None = None
        self._stop_event = threading.Event()

        self._decoders: dict[bytes, Any] = {
            LCMPoseStamped._get_packed_fingerprint(): self._on_pose_bytes,
            LCMJoy._get_packed_fingerprint(): self._on_joy_bytes,
            LCMTwistStamped._get_packed_fingerprint(): self._on_twist_bytes,
        }

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

    def _connect_blocking(self) -> None:
        assert self._loop is not None
        future = asyncio.run_coroutine_threadsafe(self._connect(), self._loop)
        future.result(timeout=20.0)

    async def _connect(self) -> None:
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

        # Throwaway DataChannel — forces an SCTP m-line into the offer so
        # the SFU has a transport to bind cmd_unreliable to later. Closed
        # as soon as the answer is applied.
        sctp_init = self._pc.createDataChannel("_sctp_init")

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

        url = f"{self.config.broker_url.rstrip('/')}/api/v1/sessions"
        body = {
            "robot_id": self.config.robot_id,
            "robot_name": self.config.robot_name,
            "sdp_offer": self._pc.localDescription.sdp,
        }
        resp = await self._http.post(url, json=body, headers=self._auth_headers())
        resp.raise_for_status()
        data = resp.json()
        self._session_id = data["session_id"]

        await self._pc.setRemoteDescription(
            RTCSessionDescription(sdp=data["sdp_answer"], type="answer")
        )

        try:
            sctp_init.close()
        except Exception:
            pass

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
        self._close_cmd_channel()
        self._cmd_channel_id = None
        if self._pc is not None:
            await self._pc.close()
            self._pc = None
        if self._http is not None:
            await self._http.aclose()
            self._http = None
        self._session_id = None

    def _auth_headers(self) -> dict[str, str]:
        if self.config.broker_api_key:
            return {"X-Robot-API-Key": self.config.broker_api_key}
        return {}

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
        try:
            # Broker's HeartbeatRequest requires a JSON body; {} hits the defaults.
            resp = await self._http.post(url, json={}, headers=self._auth_headers())
            resp.raise_for_status()
        except Exception as e:
            logger.warning(f"Heartbeat POST failed: {e}")
            return

        try:
            data = resp.json()
        except Exception:
            return

        sub_id = data.get("cmd_channel_subscriber_id")
        sub_id_int = int(sub_id) if sub_id is not None else None

        if sub_id_int != self._cmd_channel_id:
            self._close_cmd_channel()
            self._cmd_channel_id = sub_id_int
            if sub_id_int is not None:
                self._open_cmd_channel(sub_id_int)

    def _open_cmd_channel(self, sctp_id: int) -> None:
        if self._pc is None:
            return
        logger.info(f"Operator joined — opening negotiated cmd_unreliable on SCTP id {sctp_id}")
        channel = self._pc.createDataChannel(
            "cmd_unreliable",
            ordered=False,
            maxRetransmits=0,
            negotiated=True,
            id=sctp_id,
        )

        @channel.on("open")
        def _on_open() -> None:
            logger.info("cmd_unreliable channel OPEN")

        @channel.on("message")
        def _on_message(data: Any) -> None:
            if isinstance(data, bytes):
                self._dispatch_bytes(data)

        @channel.on("close")
        def _on_close() -> None:
            logger.info("cmd_unreliable channel closed")

        self._cmd_channel = channel

    def _close_cmd_channel(self) -> None:
        if self._cmd_channel is not None:
            try:
                self._cmd_channel.close()
            except Exception:
                pass
            self._cmd_channel = None

    def _dispatch_bytes(self, data: bytes) -> None:
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

    def _on_twist_bytes(self, data: bytes) -> None:
        # Keyboard mode — no engagement gating. Wire format is TwistStamped;
        # publish on both cmd_vel (plain, for GO2 / standard consumers) and
        # cmd_vel_stamped (with header, for recorders / latency analyzers).
        msg = TwistStamped.lcm_decode(data)
        self.cmd_vel.publish(Twist(linear=msg.linear, angular=msg.angular))
        self.cmd_vel_stamped.publish(msg)

    @staticmethod
    def _resolve_hand(frame_id: str) -> Hand:
        if frame_id == "left":
            return Hand.LEFT
        if frame_id == "right":
            return Hand.RIGHT
        raise ValueError(f"Unexpected frame_id: {frame_id!r}")

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
