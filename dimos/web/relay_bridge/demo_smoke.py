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

"""Manual smoke demo for the relay chain.

Spawns the Deno relay (unless --url points at a running one), then drives a
robot client pushing synthetic color_image JPEGs as fast as they encode
(latest-wins) plus odom at 20 Hz (reliable), and a viewer client receiving
both. Open the printed debug URL in Chrome/Firefox to watch the same stream.

Run: uv run python -m dimos.web.relay_bridge.demo_smoke [--secs 20] [--url https://...]
"""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import json
import math
import time
from typing import Any
from urllib.parse import urlparse

import cv2
import numpy as np

from dimos.web.relay_bridge.protocol import (
    ChannelSpec,
    DataFrame,
    Manifest,
    RobotInfo,
    RobotManifest,
    Sub,
    Subs,
    Watch,
)
from dimos.web.relay_bridge.relay_process import RelayProcess
from dimos.web.relay_bridge.wt_client import RelayClient

WIDTH, HEIGHT = 640, 480

ROBOT = RobotInfo(id="smoke", name="Smoke Demo", model="synthetic")
MANIFEST = RobotManifest(
    channels=[
        ChannelSpec(ch="color_image", encoding="jpeg.v1", delivery="latest", maxHz=60.0),
        ChannelSpec(ch="odom", encoding="pose.json.v1", delivery="reliable", maxHz=20.0),
    ]
)


def make_jpeg(seq: int) -> bytes:
    """Synthetic camera frame: moving gradient + seq/timestamp overlay."""
    ramp = np.linspace(0, 255, WIDTH, dtype=np.uint8)
    gray = np.roll(np.tile(ramp, (HEIGHT, 1)), (seq * 7) % WIDTH, axis=1)
    image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    cv2.putText(
        image,
        f"seq {seq}  {time.strftime('%H:%M:%S')}",
        (20, 60),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.2,
        (0, 255, 0),
        2,
    )
    ok, encoded = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
    assert ok
    return encoded.tobytes()


class ViewerStats:
    def __init__(self) -> None:
        self.channels: dict[str, dict[str, Any]] = {}

    def on_frame(self, frame: DataFrame) -> None:
        ch = self.channels.setdefault(
            frame.header.ch,
            {"frames": 0, "bytes": 0, "seqs": set(), "last": -1, "ooo": 0, "lat_ms": 0.0},
        )
        ch["frames"] += 1
        ch["bytes"] += len(frame.payload)
        ch["seqs"].add(frame.header.seq)
        if frame.header.seq < ch["last"]:
            ch["ooo"] += 1
        ch["last"] = max(ch["last"], frame.header.seq)
        ch["lat_ms"] = (time.time() - frame.header.ts) * 1000

    def line(self) -> str:
        parts = []
        for name, ch in sorted(self.channels.items()):
            seqs = ch["seqs"]
            span_loss = (max(seqs) - min(seqs) + 1 - len(seqs)) if seqs else 0
            parts.append(
                f"{name}: {ch['frames']}f span_loss={span_loss} ooo={ch['ooo']} "
                f"lat={ch['lat_ms']:.1f}ms"
            )
        return " | ".join(parts) or "(nothing received yet)"


async def _attach_viewer(viewer: RelayClient) -> None:
    """watch the demo robot; the watch rides a lossy datagram, so it is
    resent until the manifest reply proves it landed. Subs are driven by
    _wait_subscribed, which resends them until the robot leg confirms."""
    await viewer.hello()
    deadline = time.monotonic() + 10
    while True:
        viewer.send_control(Watch(robotId=ROBOT.id))
        with contextlib.suppress(asyncio.TimeoutError):
            msg = await asyncio.wait_for(viewer._session.control_msgs.get(), 0.5)
            while not isinstance(msg, Manifest):
                msg = await asyncio.wait_for(viewer._session.control_msgs.get(), 0.5)
            break
        if time.monotonic() > deadline:
            raise TimeoutError("viewer could not watch the demo robot")


async def _wait_subscribed(robot: RelayClient, viewer: RelayClient, timeout: float = 10.0) -> None:
    """Resend subs until the relay reports both channels wanted.

    Subs ride lossy datagrams too, and the relay's subs snapshot to the robot
    is the only proof of delivery: a one-shot Sub lost on a remote path would
    never be healed. Polls the raw control queue - wait_for around the
    control_messages() generator would close it permanently on timeout.
    """
    wanted = {spec.ch for spec in MANIFEST.channels}
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for spec in MANIFEST.channels:
            viewer.send_control(Sub(ch=spec.ch))
        window = min(time.monotonic() + 0.5, deadline)
        while time.monotonic() < window:
            if robot.is_closed:
                raise RuntimeError("relay session closed before viewers subscribed")
            with contextlib.suppress(asyncio.TimeoutError):
                msg = await asyncio.wait_for(
                    robot._session.control_msgs.get(), window - time.monotonic()
                )
                if isinstance(msg, Subs) and wanted <= set(msg.chs):
                    return
    raise TimeoutError(
        f"relay never reported viewers subscribed to {sorted(wanted)} within {timeout} s"
    )


async def run(url: str, secs: float) -> None:
    stats = ViewerStats()
    async with (
        await RelayClient.connect(url, "robot") as robot,
        await RelayClient.connect(url, "viewer") as viewer,
    ):
        await robot.hello(robot=ROBOT, manifest=MANIFEST)
        await _attach_viewer(viewer)
        await _wait_subscribed(robot, viewer)
        rtt = await viewer.ping()
        print(f"connected; datagram RTT {rtt * 1000:.1f} ms")

        deadline = time.monotonic() + secs if secs > 0 else math.inf
        image_writer = robot.latest_writer("color_image")
        odom_sent = 0

        async def image_pump() -> None:
            seq = 0
            while time.monotonic() < deadline:
                image_writer.offer(make_jpeg(seq), meta={"w": WIDTH, "h": HEIGHT})
                seq += 1
                await asyncio.sleep(0)  # flat out: paced by encode + delivery

        async def odom_pump() -> None:
            nonlocal odom_sent
            while time.monotonic() < deadline:
                t = time.time()
                payload = json.dumps(
                    {"x": 3 * math.sin(t / 3), "y": 2 * math.sin(t / 2), "yaw": t % 6.28, "ts": t}
                ).encode()
                robot.send_frame("odom", payload, delivery="reliable")
                odom_sent += 1
                await asyncio.sleep(1 / 20)

        async def viewer_pump() -> None:
            async for frame in viewer.frames():
                stats.on_frame(frame)

        async def report() -> None:
            while time.monotonic() < deadline:
                await asyncio.sleep(2)
                print(
                    f"tx img {image_writer.sent} (dropped {image_writer.dropped}, "
                    f"resets {image_writer.resets}) odom {odom_sent} | rx {stats.line()}"
                )

        viewer_task = asyncio.create_task(viewer_pump())
        try:
            await asyncio.gather(image_pump(), odom_pump(), report())
        finally:
            await asyncio.sleep(0.3)  # let the tail of the stream arrive
            viewer_task.cancel()

        print("\nsummary:")
        print(
            f"  sent: color_image {image_writer.sent} (+{image_writer.dropped} shed "
            f"at source), odom {odom_sent}"
        )
        print(f"  received: {stats.line()}")
        odom = stats.channels.get("odom", {})
        odom_ok = odom and len(odom["seqs"]) == odom_sent and odom["last"] == odom_sent - 1
        img_frames = stats.channels.get("color_image", {}).get("frames", 0)
        print(
            f"  verdict: odom {'complete' if odom_ok else 'INCOMPLETE'}, "
            f"color_image {img_frames} frames delivered"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default=None, help="attach to a running relay (wtUrl)")
    parser.add_argument("--secs", type=float, default=0, help="run time; 0 = until Ctrl-C")
    args = parser.parse_args()

    if args.url is not None:
        # /api/info's wtUrl ends in /viewer; strip the path so each role picks its own.
        parsed = urlparse(args.url)
        url = f"{parsed.scheme}://{parsed.netloc}" if parsed.netloc else args.url
        asyncio.run(run(url, args.secs))
        return
    with RelayProcess() as info:
        print(f"relay up; open {info.debug_url} in Chrome/Firefox to watch")
        with contextlib.suppress(KeyboardInterrupt):
            asyncio.run(run(info.wt_url, args.secs))


if __name__ == "__main__":
    main()
