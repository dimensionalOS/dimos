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
import json
import os
import re
import threading
import time
from typing import Any

from aiortc import (
    RTCBundlePolicy,
    RTCConfiguration,
    RTCIceServer,
    RTCPeerConnection,
    RTCSessionDescription,
)
from aiortc.mediastreams import VIDEO_CLOCK_RATE, VIDEO_TIME_BASE, VideoStreamTrack
import av
from dimos_lcm.geometry_msgs import PoseStamped as LCMPoseStamped, TwistStamped as LCMTwistStamped
from dimos_lcm.sensor_msgs import Joy as LCMJoy
import httpx
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.teleop.quest.quest_types import Buttons, QuestControllerState
from dimos.teleop.utils.teleop_transforms import webxr_to_robot
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class Hand(IntEnum):
    LEFT = 0
    RIGHT = 1


def _propagate_bundle_candidates(sdp: str) -> str:
    """Copy ICE candidate lines into every bundled m-section that lacks them.

    Workaround for an aiortc MAX_BUNDLE bug: aiortc keys remote candidates by
    transport, and under one bundled transport the LAST m-section processed wins
    the dict. CF Realtime puts a=candidate / a=end-of-candidates only on the
    first (video) section; the empty SCTP section then overwrites it, so the one
    transport gets zero remote candidates and ICE stalls at 'checking'.
    Replicating the candidate block into the candidate-less sections makes the
    applied set correct no matter which section wins.
    """
    sections = re.split(r"(?m)^(?=m=)", sdp)
    cand_lines: list[str] = []
    for s in sections:
        if s.startswith("m="):
            found = re.findall(r"(?m)^(a=(?:candidate:|end-of-candidates).*)$", s)
            if found:
                cand_lines = found
                break
    if not cand_lines:
        return sdp  # nothing to propagate

    block = "\r\n".join(cand_lines) + "\r\n"
    out = []
    for s in sections:
        if s.startswith("m=") and not re.search(r"(?m)^a=candidate:", s):
            # Append the candidate block at the end of this section.
            out.append(s.rstrip("\r\n") + "\r\n" + block)
        else:
            out.append(s)
    return "".join(out)


# ImageFormat → PyAV pixel-format string. Anything not listed falls back to
# bgr24 — av will reformat as needed if the encoder requests something else.
_AV_FORMAT_MAP = {
    ImageFormat.BGR: "bgr24",
    ImageFormat.RGB: "rgb24",
    ImageFormat.BGRA: "bgra",
    ImageFormat.RGBA: "rgba",
    ImageFormat.GRAY: "gray",
}


class CameraVideoTrack(VideoStreamTrack):
    """aiortc video track sourced from the latest Image on the In port.

    Drain-mode: ``recv()`` returns only when a *new* Image has arrived since
    the last delivery, naturally throttling encode rate to the source's
    cadence. Avoids feeding the encoder duplicate frames at startup (which
    cause a warm-up burst that the browser plays in fast-forward).

    Wall-clock PTSs: timestamps reflect real elapsed time since the first
    delivered frame, not aiortc's idealized 30 fps schedule — so the browser
    paces playback at whatever the source actually produced.
    """

    def __init__(self) -> None:
        super().__init__()
        self._lock = threading.Lock()
        self._latest: Image | None = None
        self._frame_seq = 0  # bumped on each set_latest
        self._consumed_seq = 0  # last seq recv() returned
        self._armed = False  # gate: ignore everything until arm()
        self._first_wall: float | None = None

    def arm(self) -> None:
        """Discard any frames received so far and start delivering from now.

        Called by ``HostedTeleopModule`` once the PeerConnection is fully
        ``connected`` — guarantees the operator's video starts at "this
        instant", not at "whenever the robot booted".
        """
        with self._lock:
            self._consumed_seq = self._frame_seq
            self._armed = True

    def set_latest(self, img: Image) -> None:
        """Subscribe callback — overwrite the latest frame, bump seq."""
        with self._lock:
            self._latest = img
            self._frame_seq += 1

    async def recv(self) -> av.VideoFrame:
        # Block until armed AND a new (unconsumed) frame arrives.
        while True:
            with self._lock:
                if (
                    self._armed
                    and self._latest is not None
                    and self._frame_seq > self._consumed_seq
                ):
                    img = self._latest
                    self._consumed_seq = self._frame_seq
                    break
            await asyncio.sleep(0.005)

        now = time.time()
        if self._first_wall is None:
            self._first_wall = now
        pts = int((now - self._first_wall) * VIDEO_CLOCK_RATE)

        frame = av.VideoFrame.from_ndarray(img.data, format=_AV_FORMAT_MAP.get(img.format, "bgr24"))
        frame.pts = pts
        frame.time_base = VIDEO_TIME_BASE
        return frame


class HostedTeleopConfig(ModuleConfig):
    control_loop_hz: float = 50.0

    broker_url: str = os.getenv("TELEOP_BROKER_URL", "https://teleop.dimensionalos.com")
    broker_api_key: str = os.getenv("TELEOP_API_KEY", "")
    robot_id: str = os.getenv("TELEOP_ROBOT_ID", "")
    robot_name: str = os.getenv("TELEOP_ROBOT_NAME", "")

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

        # state_reliable mirrors cmd_unreliable but ordered+reliable, robot↔
        # operator. Phase 1.5: carries JSON ping/pong for clock sync; future
        # low-rate control-plane events (mode switch, etc.) ride here too.
        # CF Realtime bridges datachannels publisher→subscriber only, so we
        # use two channels: state_reliable (operator→robot inbound) and
        # state_reliable_back (robot→operator outbound for pongs/state).
        self._state_channel = None
        self._state_channel_id: int | None = None
        self._state_back_channel = None
        self._state_back_channel_id: int | None = None

        self._video_track = CameraVideoTrack()

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
        unsub = self.color_image.subscribe(self._video_track.set_latest)
        self.register_disposable(Disposable(unsub))
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
        # Must exceed the HTTP timeout below + ICE gathering. create_session
        # on the broker now does multiple CF round-trips (session + publisher
        # track add for video), so give it room.
        future.result(timeout=45.0)

    async def _connect(self) -> None:
        # 30s read timeout: the broker's create_session makes 2 CF calls when
        # video is enabled (session + add_tracks), and add_tracks itself has a
        # 30s CF-side timeout. 10s here would give up before the broker can.
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

        # Single bundled transport for video + datachannel. Two pieces are BOTH
        # required against aiortc 1.14 + CF:
        #  1. MAX_BUNDLE — make aiortc put video + SCTP on ONE ICE transport
        #     (default BALANCED gives two; the video one fails ICE against CF's
        #     single bundled transport).
        #  2. addTrack BEFORE createDataChannel — so a transceiver exists when
        #     the SCTP transport is created, which makes aiortc mark
        #     sctp._bundled=True up front. Without it, setRemoteDescription's
        #     bundle-collapse hits `not sctp._bundled`, discards the SHARED
        #     transport (nTransports 1→0), and ICE never starts. Verified:
        #     track-first + MAX_BUNDLE ⇒ sctp._bundled=True ⇒ discard skipped.
        self._pc = RTCPeerConnection(
            RTCConfiguration(
                iceServers=ice_servers,
                bundlePolicy=RTCBundlePolicy.MAX_BUNDLE,
            )
        )

        # Robot→operator camera. MUST be added before the datachannel (see above)
        # — adds the sendonly m=video the broker declares as a publisher track.
        self._pc.addTrack(self._video_track)

        # Throwaway DataChannel — forces an SCTP m-line into the offer so the SFU
        # has a transport to bind the real channels to later. Pinned to
        # negotiated id=0: a plain createDataChannel auto-grabs SCTP id 1 at
        # connect, which is the SAME id the broker assigns cmd_unreliable →
        # createDataChannel(id=1) then collides and throws. id=0 is reserved and
        # never handed out by the broker (it assigns 1,2,3…), so cmd/state/
        # state_back stay collision-free. Kept open: under MAX_BUNDLE it shares
        # the one transport with video; closing it would risk that transport.
        sctp_init = self._pc.createDataChannel("_sctp_init", negotiated=True, id=0)

        @self._pc.on("connectionstatechange")
        async def _on_state() -> None:
            if self._pc is None:  # fired during shutdown
                return
            logger.info(f"PC state: {self._pc.connectionState}")
            if self._pc.connectionState == "connected":
                self._video_track.arm()

        offer = await self._pc.createOffer()
        await self._pc.setLocalDescription(offer)

        # Non-trickle ICE: wait for gathering before posting.
        if self._pc.iceGatheringState != "complete":
            done: asyncio.Future[None] = asyncio.get_event_loop().create_future()

            @self._pc.on("icegatheringstatechange")
            def _on_gathering() -> None:
                if self._pc is None:  # fired during shutdown
                    return
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
        if resp.status_code >= 400:
            # Surface the broker's error body — raise_for_status() discards it.
            # A FastAPI HTTPException gives JSON {"detail": ...}; Caddy's own
            # 502 gives an HTML page (→ upstream crashed/unreachable).
            logger.error(
                "Broker POST /sessions -> %s: %s",
                resp.status_code,
                resp.text[:1000],
            )
        resp.raise_for_status()
        data = resp.json()
        self._session_id = data["session_id"]

        # Work around aiortc's MAX_BUNDLE candidate-dict overwrite: CF puts ICE
        # candidates only on the video section, but both bundled sections share
        # one transport and the empty SCTP section wins → remote=0. Replicate the
        # candidates into the candidate-less section(s) so they're applied.
        answer_sdp = _propagate_bundle_candidates(data["sdp_answer"])

        await self._pc.setRemoteDescription(
            RTCSessionDescription(sdp=answer_sdp, type="answer")
        )

        # NOTE: do NOT close sctp_init here. Under MAX_BUNDLE the SCTP shares
        # the single bundled ICE/DTLS transport with video; closing the only
        # datachannel would tear that transport down. The throwaway DC is
        # harmless to leave open — the real negotiated channels reuse the same
        # SCTP association by id later.
        _ = sctp_init  # keep a reference; intentionally left open

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
                        # Log the full exception: this path also opens the
                        # negotiated channels (_open_cmd_channel etc.), so a
                        # failure here can be a channel bug, not just an
                        # unreachable broker — don't swallow it as a bare warning.
                        logger.exception("Heartbeat/channel-open failed")
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

        logger.info(f"cmd_unreliable readyState immediately after create: {channel.readyState}")

        self._cmd_channel = channel

    def _close_cmd_channel(self) -> None:
        if self._cmd_channel is not None:
            try:
                self._cmd_channel.close()
            except Exception:
                pass
            self._cmd_channel = None

    def _open_state_channel(self, sctp_id: int) -> None:
        """Open the negotiated ``state_reliable`` channel on *sctp_id*.

        Reliable + ordered (opposite of ``cmd_unreliable``). Carries JSON
        messages — currently just the clock-sync ping/pong handshake; future
        low-rate control-plane events will ride here too.
        """
        if self._pc is None:
            return
        logger.info(f"Operator joined — opening negotiated state_reliable on SCTP id {sctp_id}")
        channel = self._pc.createDataChannel(
            "state_reliable",
            ordered=True,
            negotiated=True,
            id=sctp_id,
        )

        @channel.on("open")
        def _on_open() -> None:
            logger.info("state_reliable channel OPEN")

        @channel.on("message")
        def _on_message(data: Any) -> None:
            self._on_state_message(data)

        @channel.on("close")
        def _on_close() -> None:
            logger.info("state_reliable channel closed")

        logger.info(f"state_reliable readyState immediately after create: {channel.readyState}")

        self._state_channel = channel

    def _close_state_channel(self) -> None:
        if self._state_channel is not None:
            try:
                self._state_channel.close()
            except Exception:
                pass
            self._state_channel = None

    def _open_state_back_channel(self, sctp_id: int) -> None:
        """Open the negotiated ``state_reliable_back`` publisher channel.

        CF Realtime datachannels are unidirectional in their bridging — the
        existing ``state_reliable`` carries operator→robot only. This second
        channel carries the reverse direction (robot→operator) for clock-sync
        pong replies and any future robot-originated state updates.
        """
        if self._pc is None:
            return
        logger.info(
            f"Operator joined — opening negotiated state_reliable_back on SCTP id {sctp_id}"
        )
        channel = self._pc.createDataChannel(
            "state_reliable_back",
            ordered=True,
            negotiated=True,
            id=sctp_id,
        )

        @channel.on("open")
        def _on_open() -> None:
            logger.info("state_reliable_back channel OPEN")

        @channel.on("close")
        def _on_close() -> None:
            logger.info("state_reliable_back channel closed")

        logger.info(
            f"state_reliable_back readyState immediately after create: {channel.readyState}"
        )

        self._state_back_channel = channel

    def _close_state_back_channel(self) -> None:
        if self._state_back_channel is not None:
            try:
                self._state_back_channel.close()
            except Exception:
                pass
            self._state_back_channel = None

    def _on_state_message(self, data: Any) -> None:
        """Handle one JSON message from ``state_reliable``.

        Recognises ``{"type":"ping","client_ts":<seconds>}`` and echoes a
        ``{"type":"pong","client_ts":<same>,"robot_ts":<seconds>}``. Unknown
        types are logged and dropped — leaves room for future control-plane
        messages without breaking older clients.
        """
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
            # Prefer the reverse-direction channel (CF only bridges robot →
            # operator on this one). Fall back to state_reliable for older
            # brokers that don't provide a back channel — that path won't
            # actually route through CF, but at least we don't crash.
            out = self._state_back_channel or self._state_channel
            if out is not None and out.readyState == "open":
                out.send(pong)
                logger.info(
                    f"state_reliable: ping received (client_ts={client_ts}), pong sent on {out.label}"
                )
            else:
                logger.warning("state_reliable: ping received but no open channel for pong")
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
