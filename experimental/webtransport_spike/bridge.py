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

"""Throwaway WebTransport spike bridge.

Streams Go2 replay data (or synthetic data) to the Deno relay over an aioquic
WebTransport session, and receives teleop/ping datagrams back.

Run from the repo root:
    uv run --with aioquic python experimental/webtransport_spike/bridge.py [--synthetic]
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import ssl
import struct
import time

from aioquic.asyncio import QuicConnectionProtocol, connect
from aioquic.h3.connection import H3_ALPN, H3Connection
from aioquic.h3.events import (
    DatagramReceived,
    HeadersReceived,
    WebTransportStreamDataReceived,
)
from aioquic.quic.configuration import QuicConfiguration
from aioquic.quic.events import QuicEvent
import numpy as np

CHANNELS = ("video", "odom", "lidar")


class Bridge(QuicConnectionProtocol):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.h3 = H3Connection(self._quic, enable_webtransport=True)
        self.ready = asyncio.Event()
        self.session_id: int | None = None
        # teleop/pong bookkeeping (printed by the stats loop)
        self.teleop_count = 0
        self.teleop_gaps = 0
        self.pongs_sent = 0
        self._teleop_last_seq = -1

    def open_session(self, authority: str) -> None:
        self.session_id = self._quic.get_next_available_stream_id()
        self.h3.send_headers(
            self.session_id,
            [
                (b":method", b"CONNECT"),
                (b":protocol", b"webtransport"),
                (b":scheme", b"https"),
                (b":authority", authority.encode()),
                (b":path", b"/robot"),
            ],
            end_stream=False,
        )
        self.transmit()

    def quic_event_received(self, event: QuicEvent) -> None:
        for ev in self.h3.handle_event(event):
            if isinstance(ev, HeadersReceived) and ev.stream_id == self.session_id:
                print(f"[bridge] CONNECT response: {ev.headers}")
                if dict(ev.headers).get(b":status") == b"200":
                    self.ready.set()
                else:
                    print("[bridge] !!! WebTransport CONNECT rejected")
            elif isinstance(ev, DatagramReceived):
                self._on_datagram(ev.data)
            elif isinstance(ev, WebTransportStreamDataReceived):
                if ev.data:
                    # the relay never writes on our streams; if this fires,
                    # something is wrong (and aioquic may kill the session)
                    print(f"[bridge] unexpected WT stream data ({len(ev.data)} B)")

    def _on_datagram(self, data: bytes) -> None:
        try:
            msg = json.loads(data)
        except ValueError:
            return
        if msg.get("t") == "ping":
            msg["t"] = "pong"
            msg["echo"] = "bridge"
            self.h3.send_datagram(self.session_id, json.dumps(msg).encode())
            self.pongs_sent += 1
            self.transmit()
        elif msg.get("t") == "teleop":
            seq = msg.get("seq", 0)
            if 0 <= self._teleop_last_seq < seq - 1:
                self.teleop_gaps += seq - self._teleop_last_seq - 1
            self._teleop_last_seq = seq
            self.teleop_count += 1

    def send_message(self, ch: str, seq: int, payload: bytes) -> None:
        # One-shot BIDI stream per message. Not uni: Deno 2.6.10 never delivers
        # incoming WT uni-stream payloads (server-side bug). The relay never
        # writes back on these (aioquic would mis-parse the bytes as H3 frames)
        # and RESETs its send side, which aioquic's h3 layer ignores.
        # Framing carries an explicit payload length because Deno's
        # writer.close() does not send FIN promptly (empty Resource::close),
        # so receivers must not depend on EOF for message boundaries.
        header = json.dumps({"ch": ch, "seq": seq, "ts": time.time()}).encode()
        stream_id = self.h3.create_webtransport_stream(self.session_id, is_unidirectional=False)
        self._quic.send_stream_data(
            stream_id,
            struct.pack("<II", len(header), len(payload)) + header + payload,
            end_stream=True,
        )
        self.transmit()

    def send_hello(self, name: str) -> None:
        # hello goes over a datagram: a bidi control stream would die on the
        # relay's welcome reply (aioquic parses it as H3), see README.
        self.h3.send_datagram(
            self.session_id,
            json.dumps({"t": "hello", "v": 1, "role": "robot", "name": name}).encode(),
        )
        self.transmit()


def make_push(loop: asyncio.AbstractEventLoop, queues: dict[str, asyncio.Queue]):
    """Latest-wins handoff from producer threads (RxPY) into asyncio queues."""

    def push(ch: str, payload: bytes) -> None:
        def _put() -> None:
            q = queues[ch]
            if q.full():
                q.get_nowait()
            q.put_nowait(payload)

        loop.call_soon_threadsafe(_put)

    return push


async def synthetic_sources(push, channels: set[str]) -> None:
    """Fake data roughly matching the real recording's rates and sizes."""

    async def odom() -> None:
        while True:
            t = time.time()
            push(
                "odom",
                json.dumps(
                    {
                        "x": 3 * math.sin(t / 3),
                        "y": 2 * math.sin(t / 2),
                        "z": 0.0,
                        "yaw": (t / 2) % (2 * math.pi),
                        "ts": t,
                    }
                ).encode(),
            )
            await asyncio.sleep(1 / 20)

    async def video() -> None:
        blob = bytes(100_000)  # not a real JPEG; cockpit counts decode errors
        while True:
            push("video", blob)
            await asyncio.sleep(1 / 10)

    async def lidar() -> None:
        n = 20_000
        angles = np.linspace(0, 2 * np.pi, n, dtype=np.float32)
        while True:
            spin = time.time() % (2 * np.pi)
            r = 5 + 2 * np.sin(3 * angles + spin)
            pts = np.stack(
                [r * np.cos(angles), r * np.sin(angles), np.sin(angles * 5) * 0.5], axis=1
            ).astype(np.float32)
            push("lidar", pts.tobytes())
            await asyncio.sleep(1 / 8)

    pumps = {"odom": odom, "video": video, "lidar": lidar}
    await asyncio.gather(*(pumps[ch]() for ch in channels))


def real_sources(dataset: str, push):
    # Lazy import: pulls in the full dimos stack, only needed in replay mode.
    from dimos.robot.unitree.go2.connection import ReplayConnection

    conn = ReplayConnection(dataset=dataset, loop=True)
    conn.video_stream().subscribe(lambda img: push("video", img.to_jpeg_bytes(quality=75)))
    conn.odom_stream().subscribe(
        lambda p: push(
            "odom",
            json.dumps(
                # float(): the pose fields can be numpy scalars
                {
                    "x": float(p.x),
                    "y": float(p.y),
                    "z": float(p.z),
                    "yaw": float(p.yaw),
                    "ts": float(p.ts),
                }
            ).encode(),
        )
    )

    def lidar_payload(pc) -> bytes:
        # decimate: keeps each message under the browser's per-stream QUIC
        # flow-control window (a full 25k-point cloud is ~300 KB and stalls
        # mid-stream waiting for a window update)
        pts = pc.points_f32()
        if len(pts) > 10_000:
            pts = pts[:: math.ceil(len(pts) / 10_000)]
        return pts.tobytes()

    conn.lidar_stream().subscribe(lambda pc: push("lidar", lidar_payload(pc)))
    return conn  # caller keeps it alive


async def amain() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--synthetic", action="store_true", help="send fake data, skip dimos imports"
    )
    parser.add_argument("--dataset", default="go2_short")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=4433)
    parser.add_argument(
        "--channels", default="odom,video,lidar", help="synthetic mode: which pumps to run"
    )
    args = parser.parse_args()

    config = QuicConfiguration(
        is_client=True,
        alpn_protocols=H3_ALPN,
        max_datagram_frame_size=65536,  # REQUIRED for H3 datagrams (relay sends H3_DATAGRAM=1)
    )
    config.verify_mode = ssl.CERT_NONE  # ephemeral self-signed relay cert

    async with connect(
        args.host, args.port, configuration=config, create_protocol=Bridge
    ) as bridge:
        bridge.open_session(f"{args.host}:{args.port}")
        await asyncio.wait_for(bridge.ready.wait(), 5)
        print(f"[bridge] WT session established, session_id={bridge.session_id}")
        print(
            f"[bridge] h3 settings sent={bridge.h3.sent_settings} received={bridge.h3.received_settings}"
        )
        bridge.send_hello("go2-spike")

        queues: dict[str, asyncio.Queue] = {ch: asyncio.Queue(maxsize=1) for ch in CHANNELS}
        push = make_push(asyncio.get_running_loop(), queues)
        counters = {ch: [0, 0] for ch in CHANNELS}  # window [frames, bytes]

        async def pump(ch: str, q: asyncio.Queue) -> None:
            seq = 0
            while True:
                payload = await q.get()
                bridge.send_message(ch, seq, payload)
                counters[ch][0] += 1
                counters[ch][1] += len(payload)
                seq += 1

        async def stats() -> None:
            last_teleop = 0
            while True:
                await asyncio.sleep(2)
                tx = " ".join(
                    f"{ch}={c[0] / 2:.1f}Hz/{c[1] / 2048:.0f}KBs" for ch, c in counters.items()
                )
                teleop_hz = (bridge.teleop_count - last_teleop) / 2
                last_teleop = bridge.teleop_count
                print(
                    f"[bridge] tx {tx} | teleop rx {teleop_hz:.1f}Hz"
                    f" (total {bridge.teleop_count}, gaps {bridge.teleop_gaps})"
                    f" pongs {bridge.pongs_sent}"
                )
                for c in counters.values():
                    c[0] = c[1] = 0

        tasks = [pump(ch, q) for ch, q in queues.items()] + [stats()]
        if args.synthetic:
            tasks.append(synthetic_sources(push, set(args.channels.split(","))))
            print(f"[bridge] sending SYNTHETIC data ({args.channels})")
        else:
            conn = real_sources(args.dataset, push)  # noqa: F841  (keeps replay alive)
            print(f"[bridge] replaying dataset {args.dataset!r} (looped)")
        await asyncio.gather(*tasks)


if __name__ == "__main__":
    asyncio.run(amain())
