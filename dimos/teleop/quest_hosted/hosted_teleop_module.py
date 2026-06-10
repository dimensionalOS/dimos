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

"""Hosted Teleop Module — Cloudflare Realtime SFU client."""

from __future__ import annotations

import asyncio
from enum import IntEnum
import json
import os
import threading
import time
from typing import Any

from aiortc import (
    RTCBundlePolicy,
    RTCConfiguration,
    RTCDataChannel,
    RTCIceServer,
    RTCPeerConnection,
    RTCSessionDescription,
)
from dimos_lcm.geometry_msgs import PoseStamped as LCMPoseStamped, TwistStamped as LCMTwistStamped
from dimos_lcm.sensor_msgs import Joy as LCMJoy
import httpx
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.teleop.quest.quest_types import Buttons, QuestControllerState
from dimos.teleop.quest_hosted.sdp import propagate_bundle_candidates
from dimos.teleop.quest_hosted.video_track import CameraVideoTrack
from dimos.teleop.utils.stream_stats import LiveStreamStats
from dimos.teleop.utils.teleop_transforms import webxr_to_robot
from dimos.teleop.utils.video_stats import VideoStats
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class Hand(IntEnum):
    LEFT = 0
    RIGHT = 1


class HostedTeleopConfig(ModuleConfig):
    control_loop_hz: float = 50.0

    broker_url: str = "https://teleop.dimensionalos.com"
    # Empty defaults; resolved from TELEOP_* env vars at start() if unset.
    broker_api_key: str = ""
    robot_id: str = ""
    robot_name: str = ""

    stun_urls: list[str] = ["stun:stun.cloudflare.com:3478"]
    turn_urls: list[str] = []
    turn_username: str = ""
    turn_credential: str = ""

    heartbeat_hz: float = 1.0
    telemetry_hz: float = 3.0  # robot → operator HUD command-plane stats


class HostedTeleopModule(Module):
    """Cloudflare-Realtime-based teleop client.

    Override hooks: ``_handle_engage``, ``_should_publish``,
    ``_get_output_pose``, ``_publish_msg``, ``_publish_button_state``.
    """

    config: HostedTeleopConfig

    left_controller_output: Out[PoseStamped]
    right_controller_output: Out[PoseStamped]
    buttons: Out[Buttons]
    # cmd_vel actuation port is on the mobile-base subclass; this stays generic.
    cmd_vel_stamped: Out[TwistStamped]
    # Operator-side video health, sampled in the browser via getStats() and
    # relayed over state_reliable. Recorders pick this up like any typed stream.
    video_stats: Out[VideoStats]
    color_image: In[Image]

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

        self._pc: RTCPeerConnection | None = None
        self._http: httpx.AsyncClient | None = None
        self._session_id: str | None = None

        # All three datachannels are negotiated; SCTP ids come from the broker
        # heartbeat ack (so they're None until an operator joins).
        self._cmd_channel: RTCDataChannel | None = None
        self._cmd_channel_id: int | None = None
        self._state_channel: RTCDataChannel | None = None
        self._state_channel_id: int | None = None
        self._state_back_channel: RTCDataChannel | None = None
        self._state_back_channel_id: int | None = None

        self._video_track = CameraVideoTrack()
        self._cmd_stats = LiveStreamStats()

        self._control_loop_thread: threading.Thread | None = None
        self._heartbeat_thread: threading.Thread | None = None
        self._telemetry_thread: threading.Thread | None = None
        self._stop_event = threading.Event()

        self._decoders: dict[bytes, Any] = {
            LCMPoseStamped._get_packed_fingerprint(): self._on_pose_bytes,
            LCMJoy._get_packed_fingerprint(): self._on_joy_bytes,
            LCMTwistStamped._get_packed_fingerprint(): self._on_twist_bytes,
        }

    @rpc
    def start(self) -> None:
        super().start()
        self._stop_event.clear()
        unsub = self.color_image.subscribe(self._video_track.set_latest)
        self.register_disposable(Disposable(unsub))
        self._connect_blocking()
        self._start_heartbeat()
        self._start_telemetry()
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
        if self._telemetry_thread is not None:
            self._telemetry_thread.join(timeout=2.0)
            self._telemetry_thread = None
        if self._loop is not None and self._loop.is_running():
            try:
                self.spawn(self._disconnect()).result(timeout=5.0)
            except Exception:
                logger.exception("Error during disconnect")
        # The base Module owns the event loop; it's torn down on module close.
        super().stop()

    def _connect_blocking(self) -> None:
        self.spawn(self._connect()).result(timeout=45.0)

    async def _connect(self) -> None:
        self._http = httpx.AsyncClient(timeout=30.0)

        ice_servers = [RTCIceServer(urls=u) for u in self.config.stun_urls]
        for url in self.config.turn_urls or []:
            ice_servers.append(
                RTCIceServer(
                    urls=url,
                    username=self.config.turn_username or None,
                    credential=self.config.turn_credential or None,
                )
            )

        # MAX_BUNDLE, addTrack-before-createDataChannel, and the id=0 throwaway
        # below are all CF/aiortc workarounds — see README before changing.
        self._pc = RTCPeerConnection(
            RTCConfiguration(
                iceServers=ice_servers,
                bundlePolicy=RTCBundlePolicy.MAX_BUNDLE,
            )
        )
        self._pc.addTrack(self._video_track)
        sctp_init = self._pc.createDataChannel("_sctp_init", negotiated=True, id=0)

        @self._pc.on("connectionstatechange")
        async def _on_state() -> None:
            if self._pc is None:
                return
            logger.info(f"PC state: {self._pc.connectionState}")
            if self._pc.connectionState == "connected":
                self._video_track.arm()

        offer = await self._pc.createOffer()
        await self._pc.setLocalDescription(offer)

        # Wait for ICE gathering before posting (non-trickle).
        if self._pc.iceGatheringState != "complete":
            done: asyncio.Future[None] = asyncio.get_event_loop().create_future()

            @self._pc.on("icegatheringstatechange")
            def _on_gathering() -> None:
                if self._pc is None:
                    return
                if self._pc.iceGatheringState == "complete" and not done.done():
                    done.set_result(None)

            await done

        url = f"{self.config.broker_url.rstrip('/')}/api/v1/sessions"
        body = {
            "robot_id": self.config.robot_id or os.getenv("TELEOP_ROBOT_ID", ""),
            "robot_name": self.config.robot_name or os.getenv("TELEOP_ROBOT_NAME", ""),
            "sdp_offer": self._pc.localDescription.sdp,
        }
        resp = await self._http.post(url, json=body, headers=self._auth_headers())
        if resp.status_code >= 400:
            # raise_for_status drops the body; surface the broker's JSON detail.
            logger.error(
                "Broker POST /sessions -> %s: %s",
                resp.status_code,
                resp.text[:1000],
            )
        resp.raise_for_status()
        data = resp.json()
        self._session_id = data["session_id"]

        answer_sdp = propagate_bundle_candidates(data["sdp_answer"])
        await self._pc.setRemoteDescription(RTCSessionDescription(sdp=answer_sdp, type="answer"))

        _ = sctp_init  # intentionally left open

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
        self._close_state_channel()
        self._state_channel_id = None
        self._close_state_back_channel()
        self._state_back_channel_id = None
        if self._pc is not None:
            await self._pc.close()
            self._pc = None
        if self._http is not None:
            await self._http.aclose()
            self._http = None
        self._session_id = None

    def _auth_headers(self) -> dict[str, str]:
        api_key = self.config.broker_api_key or os.getenv("TELEOP_API_KEY")
        if api_key:
            return {"X-Robot-API-Key": api_key}
        return {}

    def _start_heartbeat(self) -> None:
        def runner() -> None:
            interval = 1.0 / max(self.config.heartbeat_hz, 0.1)
            while not self._stop_event.is_set():
                if self._loop is not None and self._loop.is_running() and self._session_id:
                    try:
                        self.spawn(self._heartbeat()).result(timeout=2.0)
                    except Exception:
                        logger.exception("Heartbeat/channel-open failed")
                self._stop_event.wait(interval)

        self._heartbeat_thread = threading.Thread(
            target=runner, daemon=True, name="HostedTeleopHeartbeat"
        )
        self._heartbeat_thread.start()

    def _record_cmd_arrival(self, ts: float | None, seq: int | None) -> None:
        """Hook for the subclass cmd-decode path; feeds the HUD telemetry."""
        self._cmd_stats.record(ts, seq)

    def _start_telemetry(self) -> None:
        """Push command-plane health (latency/jitter/loss/rate) to the operator HUD."""

        def send_telemetry() -> None:
            stats = self._cmd_stats.snapshot()
            channel = self._state_back_channel
            if stats is None or channel is None or channel.readyState != "open":
                return
            try:
                channel.send(
                    json.dumps(
                        {
                            "type": "robot_telemetry",
                            "cmd": stats,
                            "robot_ts": time.time(),
                        }
                    )
                )
            except Exception:
                logger.debug("telemetry send failed", exc_info=True)

        def runner() -> None:
            interval = 1.0 / max(self.config.telemetry_hz, 0.1)
            while not self._stop_event.is_set():
                if self._loop is not None and self._loop.is_running():
                    self._loop.call_soon_threadsafe(send_telemetry)
                self._stop_event.wait(interval)

        self._telemetry_thread = threading.Thread(
            target=runner, daemon=True, name="HostedTeleopTelemetry"
        )
        self._telemetry_thread.start()

    async def _heartbeat(self) -> None:
        """POST a heartbeat; react to the SCTP ids the broker hands back by
        opening / re-opening / closing the three negotiated datachannels."""
        if self._http is None or self._session_id is None:
            return
        url = f"{self.config.broker_url.rstrip('/')}/api/v1/sessions/{self._session_id}/heartbeat"
        try:
            resp = await self._http.post(url, json={}, headers=self._auth_headers())
            resp.raise_for_status()
        except Exception as e:
            logger.warning(f"Heartbeat POST failed: {e}")
            return

        try:
            data = resp.json()
        except json.JSONDecodeError:
            logger.warning("Heartbeat response was not valid JSON")
            return

        sub_id = data.get("cmd_channel_subscriber_id")
        sub_id_int = int(sub_id) if sub_id is not None else None

        if sub_id_int != self._cmd_channel_id:
            self._close_cmd_channel()
            self._cmd_channel_id = sub_id_int
            if sub_id_int is not None:
                self._open_cmd_channel(sub_id_int)

        state_sub_id = data.get("state_channel_subscriber_id")
        state_sub_id_int = int(state_sub_id) if state_sub_id is not None else None

        if state_sub_id_int != self._state_channel_id:
            self._close_state_channel()
            self._state_channel_id = state_sub_id_int
            if state_sub_id_int is not None:
                self._open_state_channel(state_sub_id_int)

        state_back_pub_id = data.get("state_back_channel_publisher_id")
        state_back_pub_id_int = int(state_back_pub_id) if state_back_pub_id is not None else None

        if state_back_pub_id_int != self._state_back_channel_id:
            self._close_state_back_channel()
            self._state_back_channel_id = state_back_pub_id_int
            if state_back_pub_id_int is not None:
                self._open_state_back_channel(state_back_pub_id_int)

    def _open_cmd_channel(self, sctp_id: int) -> None:
        """Operator → robot, unreliable + unordered. Carries LCM-encoded commands."""
        if self._pc is None:
            return
        logger.info("Opening negotiated cmd_unreliable on SCTP id %d", sctp_id)
        channel = self._pc.createDataChannel(
            "cmd_unreliable",
            ordered=False,
            maxRetransmits=0,
            negotiated=True,
            id=sctp_id,
        )

        @channel.on("message")
        def _on_message(data: Any) -> None:
            if isinstance(data, bytes):
                self._dispatch_bytes(data)

        self._cmd_channel = channel

    def _close_cmd_channel(self) -> None:
        if self._cmd_channel is not None:
            try:
                self._cmd_channel.close()
            except Exception:
                logger.warning("Failed to close cmd channel", exc_info=True)
            self._cmd_channel = None

    def _open_state_channel(self, sctp_id: int) -> None:
        """Operator → robot, reliable + ordered. JSON control plane (ping, clock_report, video_stats)."""
        if self._pc is None:
            return
        logger.info("Opening negotiated state_reliable on SCTP id %d", sctp_id)
        channel = self._pc.createDataChannel(
            "state_reliable",
            ordered=True,
            negotiated=True,
            id=sctp_id,
        )

        @channel.on("message")
        def _on_message(data: Any) -> None:
            self._on_state_message(data)

        self._state_channel = channel

    def _close_state_channel(self) -> None:
        if self._state_channel is not None:
            try:
                self._state_channel.close()
            except Exception:
                logger.warning("Failed to close state channel", exc_info=True)
            self._state_channel = None

    def _open_state_back_channel(self, sctp_id: int) -> None:
        """Robot → operator (CF bridges one-way). Pong replies + robot_telemetry."""
        if self._pc is None:
            return
        logger.info("Opening negotiated state_reliable_back on SCTP id %d", sctp_id)
        channel = self._pc.createDataChannel(
            "state_reliable_back",
            ordered=True,
            negotiated=True,
            id=sctp_id,
        )
        self._state_back_channel = channel

    def _close_state_back_channel(self) -> None:
        if self._state_back_channel is not None:
            try:
                self._state_back_channel.close()
            except Exception:
                logger.warning("Failed to close state-back channel", exc_info=True)
            self._state_back_channel = None

    def _on_state_message(self, data: Any) -> None:
        """Dispatch a JSON message from state_reliable: ping → pong, plus clock_report / video_stats logging."""
        if isinstance(data, bytes):
            try:
                data = data.decode("utf-8")
            except UnicodeDecodeError:
                logger.warning("state_reliable: non-utf8 payload, dropping")
                return
        try:
            msg = json.loads(data)
        except Exception:
            logger.warning(f"state_reliable: malformed JSON: {data[:80]!r}")
            return

        kind = msg.get("type")
        if kind == "ping":
            client_ts = msg.get("client_ts")
            if client_ts is None:
                return
            pong = json.dumps({"type": "pong", "client_ts": client_ts, "robot_ts": time.time()})
            out = self._state_back_channel
            if out is not None and out.readyState == "open":
                out.send(pong)
            else:
                logger.warning(
                    "state_reliable: ping received but state_reliable_back not open — pong dropped"
                )
        elif kind == "clock_report":
            rtt = msg.get("rtt_ms")
            off = msg.get("offset_ms")
            logger.info(
                "clock-sync: operator rtt=%.1fms offset=%.1fms",
                float(rtt) if rtt is not None else float("nan"),
                float(off) if off is not None else float("nan"),
            )
        elif kind == "video_stats":
            stats = VideoStats.from_dict(msg)
            logger.info("video: %s", stats)
            self.video_stats.publish(stats)
        else:
            logger.debug(f"state_reliable: unknown message type {kind!r}")

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
        # Mobile-base subclass overrides this to actuate cmd_vel.
        msg = TwistStamped.lcm_decode(data)
        self.cmd_vel_stamped.publish(msg)

    @staticmethod
    def _resolve_hand(frame_id: str) -> Hand:
        if frame_id == "left":
            return Hand.LEFT
        if frame_id == "right":
            return Hand.RIGHT
        raise ValueError(f"Unexpected frame_id: {frame_id!r}")

    def _start_control_loop(self) -> None:
        # _stop_event already cleared in start() before any thread spawns.
        self._control_loop_thread = threading.Thread(
            target=self._control_loop,
            daemon=True,
            name="HostedTeleopControlLoop",
        )
        self._control_loop_thread.start()

    def _control_loop(self) -> None:
        """Fixed-rate cycle: engage-gating → per-hand publish → buttons publish.

        Drives the subclass-overridable hooks; the inbound cmd / pose / joy
        decode happens off this loop (in the datachannel callbacks).
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
