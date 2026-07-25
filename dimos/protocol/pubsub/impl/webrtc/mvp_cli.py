# Copyright 2026 Dimensional Inc.
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

from __future__ import annotations

import asyncio
import contextlib
from dataclasses import dataclass
from itertools import pairwise
import json
import time
from typing import Any

import requests
import typer

from dimos.protocol.pubsub.impl.webrtc.mvp_metrics import (
    decode_frame_stamp,
    make_stamped_frame,
    percentile,
)
from dimos.protocol.pubsub.impl.webrtc.providers.broker import (
    DEFAULT_BROKER_URL,
    BrokerConfig,
)

webrtc_mvp_app = typer.Typer(
    help="Measure a robot-to-operator WebRTC video path through the hosted SFU",
    no_args_is_help=True,
)


@dataclass(frozen=True)
class _SyntheticImage:
    data: Any
    format: str = "bgr"


def _required(value: str | None, name: str) -> str:
    if value:
        return value
    raise typer.BadParameter(f"{name} is required")


def _operator_api(
    broker_url: str,
    operator_token: str,
    method: str,
    path: str,
    body: dict[str, Any] | None = None,
) -> dict[str, Any]:
    response = requests.request(
        method,
        f"{broker_url.rstrip('/')}/api/v1{path}",
        json=body,
        headers={
            "Authorization": f"Bearer {operator_token}",
            "Content-Type": "application/json",
        },
        timeout=30.0,
    )
    response.raise_for_status()
    data = response.json() if response.content else {}
    if not isinstance(data, dict):
        raise RuntimeError("Broker returned a non-object JSON response")
    return data


@webrtc_mvp_app.command("local")
def local(
    duration: float = typer.Option(5.0, "--duration", min=1.0),
    fps: float = typer.Option(15.0, "--fps", min=1.0, max=60.0),
    width: int = typer.Option(640, "--width", min=576),
    height: int = typer.Option(360, "--height", min=48),
) -> None:
    """Run a direct in-process H.264 WebRTC baseline without CDN credentials."""
    result = asyncio.run(
        _local_loopback(
            duration=duration,
            fps=fps,
            width=width,
            height=height,
        )
    )
    typer.echo(json.dumps(result, sort_keys=True))


@webrtc_mvp_app.command("publish")
def publish(
    api_key: str | None = typer.Option(
        None,
        "--api-key",
        envvar="TELEOP_API_KEY",
        help="Robot API key; prefer TELEOP_API_KEY",
        hide_input=True,
    ),
    broker_url: str = typer.Option(
        DEFAULT_BROKER_URL,
        "--broker-url",
        envvar="TELEOP_BROKER_URL",
    ),
    robot_name: str = typer.Option("webrtc-mvp", "--robot-name"),
    duration: float = typer.Option(
        0.0,
        "--duration",
        min=0.0,
        help="Seconds to publish; zero runs until Ctrl+C",
    ),
    fps: float = typer.Option(15.0, "--fps", min=1.0, max=60.0),
    width: int = typer.Option(640, "--width", min=576),
    height: int = typer.Option(360, "--height", min=48),
) -> None:
    """Publish timestamped synthetic H.264 video and print the session id."""
    key = _required(api_key, "TELEOP_API_KEY or --api-key")
    provider = BrokerConfig(
        broker_url=broker_url,
        api_key=key,
        robot_name=robot_name,
        robot_type="webrtc-mvp",
        video_codec="h264",
    ).provider()
    started = time.perf_counter()
    provider.start()
    session_id = getattr(provider, "session_id", None)
    if not isinstance(session_id, str):
        provider.stop()
        raise RuntimeError("Broker did not return a session id")
    typer.echo(
        json.dumps(
            {
                "event": "ready",
                "session_id": session_id,
                "broker_url": broker_url,
                "connect_ms": round((time.perf_counter() - started) * 1000, 2),
                "codec": "h264",
                "fps": fps,
            },
            sort_keys=True,
        )
    )

    period = 1.0 / fps
    deadline = time.monotonic()
    stop_at = deadline + duration if duration else None
    sequence = 0
    set_video_frame = getattr(provider, "set_video_frame", None)
    if set_video_frame is None:
        provider.stop()
        raise RuntimeError(f"{type(provider).__name__} does not support video")
    try:
        while stop_at is None or time.monotonic() < stop_at:
            data = make_stamped_frame(width=width, height=height, sequence=sequence)
            set_video_frame(_SyntheticImage(data=data))
            sequence += 1
            deadline += period
            time.sleep(max(0.0, deadline - time.monotonic()))
    except KeyboardInterrupt:
        pass
    finally:
        provider.stop()
    typer.echo(json.dumps({"event": "stopped", "frames_sent": sequence}, sort_keys=True))


@webrtc_mvp_app.command("subscribe")
def subscribe(
    session_id: str = typer.Argument(..., help="Session id printed by publish"),
    operator_token: str | None = typer.Option(
        None,
        "--operator-token",
        envvar="TELEOP_OPERATOR_TOKEN",
        help="Operator bearer token; prefer TELEOP_OPERATOR_TOKEN",
        hide_input=True,
    ),
    broker_url: str = typer.Option(
        DEFAULT_BROKER_URL,
        "--broker-url",
        envvar="TELEOP_BROKER_URL",
    ),
    duration: float = typer.Option(10.0, "--duration", min=1.0),
) -> None:
    """Receive video and report signaling, first-frame, and latency metrics."""
    token = _required(operator_token, "TELEOP_OPERATOR_TOKEN or --operator-token")
    result = asyncio.run(
        _subscribe(
            session_id=session_id,
            operator_token=token,
            broker_url=broker_url,
            duration=duration,
        )
    )
    typer.echo(json.dumps(result, sort_keys=True))


async def _subscribe(
    *,
    session_id: str,
    operator_token: str,
    broker_url: str,
    duration: float,
) -> dict[str, Any]:
    from aiortc import (
        RTCConfiguration,
        RTCIceServer,
        RTCPeerConnection,
        RTCSessionDescription,
    )

    from dimos.protocol.pubsub.impl.webrtc.providers.spec import wait_connected

    process_started = time.perf_counter()
    try:
        ice_data = await asyncio.to_thread(
            _operator_api,
            broker_url,
            operator_token,
            "GET",
            "/sessions/turn-credentials",
        )
    except requests.RequestException:
        ice_data = {}
    ice_servers = [
        RTCIceServer(
            urls=server["urls"],
            username=server.get("username"),
            credential=server.get("credential"),
        )
        for server in ice_data.get("ice_servers", [])
        if isinstance(server, dict) and server.get("urls")
    ]
    if not ice_servers:
        ice_servers = [RTCIceServer(urls=["stun:stun.cloudflare.com:3478"])]
    pc = RTCPeerConnection(RTCConfiguration(iceServers=ice_servers))
    pc.createDataChannel("_sctp_init", negotiated=True, id=0)
    latencies_ms: list[float] = []
    sequences: list[int] = []
    first_frame_at: float | None = None
    invalid_stamps = 0
    track_ready = asyncio.Event()
    consume_task: asyncio.Task[None] | None = None

    @pc.on("track")
    def _on_track(track: Any) -> None:
        nonlocal consume_task
        if track.kind != "video":
            return

        async def _consume() -> None:
            nonlocal first_frame_at, invalid_stamps
            track_ready.set()
            while True:
                frame = await track.recv()
                if first_frame_at is None:
                    first_frame_at = time.perf_counter()
                stamp = decode_frame_stamp(frame.to_ndarray(format="bgr24"))
                if stamp is None:
                    invalid_stamps += 1
                    continue
                now_ms = time.time_ns() // 1_000_000
                latencies_ms.append(float(now_ms - stamp.timestamp_ms))
                sequences.append(stamp.sequence)

        consume_task = asyncio.create_task(_consume())

    media_ready_at: float | None = None
    connected_at: float | None = None
    try:
        await pc.setLocalDescription(await pc.createOffer())
        while pc.iceGatheringState != "complete":
            await asyncio.sleep(0.05)
        join = await asyncio.to_thread(
            _operator_api,
            broker_url,
            operator_token,
            "POST",
            f"/sessions/{session_id}/join",
            {"role": "operator", "sdp_offer": pc.localDescription.sdp},
        )
        await pc.setRemoteDescription(RTCSessionDescription(sdp=join["sdp_answer"], type="answer"))
        await wait_connected(pc, timeout=20.0)
        connected_at = time.perf_counter()

        bridge = await asyncio.to_thread(
            _operator_api,
            broker_url,
            operator_token,
            "POST",
            f"/sessions/{session_id}/bridge-datachannel",
        )
        video_offer = bridge.get("video_offer")
        if not isinstance(video_offer, str):
            raise RuntimeError(f"Broker returned no video offer: {bridge.get('video_status')}")
        await pc.setRemoteDescription(RTCSessionDescription(sdp=video_offer, type="offer"))
        await pc.setLocalDescription(await pc.createAnswer())
        await asyncio.to_thread(
            _operator_api,
            broker_url,
            operator_token,
            "POST",
            f"/sessions/{session_id}/renegotiate-answer",
            {"sdp_answer": pc.localDescription.sdp},
        )
        media_ready_at = time.perf_counter()
        await asyncio.wait_for(track_ready.wait(), timeout=20.0)
        await asyncio.sleep(duration)
        stats = await pc.getStats()
    finally:
        with contextlib.suppress(requests.RequestException):
            await asyncio.to_thread(
                _operator_api,
                broker_url,
                operator_token,
                "POST",
                f"/sessions/{session_id}/leave",
                {"role": "operator"},
            )
        if consume_task is not None:
            consume_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await consume_task
        await pc.close()

    inbound = [
        value
        for key, value in stats.items()
        if "inbound-rtp" in key and getattr(value, "kind", None) == "video"
    ]
    return {
        "session_id": session_id,
        "broker_url": broker_url,
        "turn_configured": any(
            str(url).startswith("turn")
            for server in ice_servers
            for url in (server.urls if isinstance(server.urls, list) else [server.urls])
        ),
        "signaling_connected_ms": _elapsed_ms(process_started, connected_at),
        "media_ready_ms": _elapsed_ms(process_started, media_ready_at),
        "first_frame_ms": _elapsed_ms(process_started, first_frame_at),
        "media_to_first_frame_ms": _elapsed_ms(media_ready_at, first_frame_at),
        "frames_decoded": len(latencies_ms) + invalid_stamps,
        "valid_timestamps": len(latencies_ms),
        "invalid_timestamps": invalid_stamps,
        "latency_ms_p50": _rounded_percentile(latencies_ms, 0.50),
        "latency_ms_p95": _rounded_percentile(latencies_ms, 0.95),
        "latency_ms_max": round(max(latencies_ms), 2) if latencies_ms else None,
        "sequence_gaps": _sequence_gaps(sequences),
        "packets_received": sum(int(getattr(value, "packetsReceived", 0)) for value in inbound),
        "packets_lost": sum(int(getattr(value, "packetsLost", 0)) for value in inbound),
    }


async def _local_loopback(
    *,
    duration: float,
    fps: float,
    width: int,
    height: int,
) -> dict[str, Any]:
    from aiortc import RTCConfiguration, RTCPeerConnection, RTCRtpSender

    from dimos.protocol.pubsub.impl.webrtc.providers.spec import wait_connected
    from dimos.protocol.pubsub.impl.webrtc.providers.video_track import CameraVideoTrack

    process_started = time.perf_counter()
    loop = asyncio.get_running_loop()
    local_ice = RTCConfiguration(iceServers=[])
    sender = RTCPeerConnection(local_ice)
    receiver = RTCPeerConnection(local_ice)
    track = CameraVideoTrack(loop)
    sender.addTrack(track)
    video_transceiver = next(
        transceiver for transceiver in sender.getTransceivers() if transceiver.kind == "video"
    )
    h264_codecs = [
        codec
        for codec in RTCRtpSender.getCapabilities("video").codecs
        if codec.mimeType.lower() == "video/h264"
    ]
    if not h264_codecs:
        raise RuntimeError("No local H.264 encoder is available")
    video_transceiver.setCodecPreferences(h264_codecs)

    latencies_ms: list[float] = []
    sequences: list[int] = []
    first_frame_at: float | None = None
    invalid_stamps = 0
    first_frame = asyncio.Event()
    consume_task: asyncio.Task[None] | None = None

    @receiver.on("track")
    def _on_track(remote_track: Any) -> None:
        nonlocal consume_task

        async def _consume() -> None:
            nonlocal first_frame_at, invalid_stamps
            while True:
                frame = await remote_track.recv()
                if first_frame_at is None:
                    first_frame_at = time.perf_counter()
                    first_frame.set()
                stamp = decode_frame_stamp(frame.to_ndarray(format="bgr24"))
                if stamp is None:
                    invalid_stamps += 1
                    continue
                latencies_ms.append(float(time.time_ns() // 1_000_000 - stamp.timestamp_ms))
                sequences.append(stamp.sequence)

        consume_task = asyncio.create_task(_consume())

    offer_started = time.perf_counter()
    await sender.setLocalDescription(await sender.createOffer())
    offer_ready_at = time.perf_counter()
    await receiver.setRemoteDescription(sender.localDescription)
    await receiver.setLocalDescription(await receiver.createAnswer())
    answer_ready_at = time.perf_counter()
    await sender.setRemoteDescription(receiver.localDescription)
    ice_started = time.perf_counter()
    await asyncio.gather(
        wait_connected(sender, timeout=10.0),
        wait_connected(receiver, timeout=10.0),
    )
    media_ready_at = time.perf_counter()
    track.arm()

    feeding = True

    async def _feed() -> None:
        sequence = 0
        deadline = time.monotonic()
        period = 1.0 / fps
        while feeding:
            track.set_latest(
                _SyntheticImage(
                    data=make_stamped_frame(
                        width=width,
                        height=height,
                        sequence=sequence,
                    )
                )
            )
            sequence += 1
            deadline += period
            await asyncio.sleep(max(0.0, deadline - time.monotonic()))

    feed_task = asyncio.create_task(_feed())
    try:
        await asyncio.wait_for(first_frame.wait(), timeout=10.0)
        await asyncio.sleep(duration)
        stats = await receiver.getStats()
    finally:
        feeding = False
        feed_task.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await feed_task
        if consume_task is not None:
            consume_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await consume_task
        await sender.close()
        await receiver.close()

    inbound = [
        value
        for key, value in stats.items()
        if "inbound-rtp" in key and getattr(value, "kind", None) == "video"
    ]
    return {
        "mode": "local-loopback",
        "codec": "h264",
        "signaling_connected_ms": _elapsed_ms(process_started, media_ready_at),
        "offer_gather_ms": _elapsed_ms(offer_started, offer_ready_at),
        "answer_gather_ms": _elapsed_ms(offer_ready_at, answer_ready_at),
        "ice_connect_ms": _elapsed_ms(ice_started, media_ready_at),
        "media_to_first_frame_ms": _elapsed_ms(media_ready_at, first_frame_at),
        "frames_decoded": len(latencies_ms) + invalid_stamps,
        "valid_timestamps": len(latencies_ms),
        "invalid_timestamps": invalid_stamps,
        "latency_ms_p50": _rounded_percentile(latencies_ms, 0.50),
        "latency_ms_p95": _rounded_percentile(latencies_ms, 0.95),
        "latency_ms_max": round(max(latencies_ms), 2) if latencies_ms else None,
        "sequence_gaps": _sequence_gaps(sequences),
        "packets_received": sum(int(getattr(value, "packetsReceived", 0)) for value in inbound),
        "packets_lost": sum(int(getattr(value, "packetsLost", 0)) for value in inbound),
    }


def _elapsed_ms(start: float | None, end: float | None) -> float | None:
    if start is None or end is None:
        return None
    return round((end - start) * 1000, 2)


def _rounded_percentile(values: list[float], quantile: float) -> float | None:
    value = percentile(values, quantile)
    return round(value, 2) if value is not None else None


def _sequence_gaps(sequences: list[int]) -> int:
    return sum(
        max(0, (current - previous) % 65536 - 1) for previous, current in pairwise(sequences)
    )


def main() -> None:
    webrtc_mvp_app()


if __name__ == "__main__":
    main()
