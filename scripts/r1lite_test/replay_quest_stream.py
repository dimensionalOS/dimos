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

"""Replay a recorded quest session against a running teleop blueprint.

Recordings are written by the quest module when its record_path config is set:
one JSON line per websocket frame with the receive timestamp and the raw LCM
payload. This script connects to the quest server exactly like the headset
does and resends every frame at the original relative timing, so a hardware
session drives the sim blueprint bit-exactly:

    dimos run r1lite-quest-teleop-sim                       # terminal 1
    python scripts/r1lite_test/replay_quest_stream.py \
        --file /tmp/quest_record.jsonl                      # terminal 2

The replay includes engage buttons, so the sim arms move as the operator's
did. Use --speed to time-stretch and --start/--end to replay a slice.
"""

from __future__ import annotations

import argparse
import asyncio
import base64
import json
from pathlib import Path
import ssl
import time

import websockets


def load_frames(path: Path, start: float, end: float) -> list[tuple[float, bytes]]:
    frames: list[tuple[float, bytes]] = []
    with path.open(encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            entry = json.loads(line)
            frames.append((float(entry["t"]), base64.b64decode(entry["data"])))
    if not frames:
        raise SystemExit(f"No frames in {path}")
    t0 = frames[0][0]
    frames = [(t - t0, data) for t, data in frames]
    return [(t, data) for t, data in frames if start <= t <= end]


async def replay(url: str, frames: list[tuple[float, bytes]], speed: float) -> None:
    ssl_ctx: ssl.SSLContext | None = None
    if url.startswith("wss"):
        ssl_ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
        ssl_ctx.check_hostname = False
        ssl_ctx.verify_mode = ssl.CERT_NONE

    async with websockets.connect(url, ssl=ssl_ctx) as ws:
        print(f"Connected to {url}, replaying {len(frames)} frames at {speed}x")
        base = frames[0][0]
        wall0 = time.monotonic()
        for t, data in frames:
            due = wall0 + (t - base) / speed
            delay = due - time.monotonic()
            if delay > 0:
                await asyncio.sleep(delay)
            await ws.send(data)
        print("Replay complete")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--file", type=Path, required=True, help="recording JSONL")
    parser.add_argument(
        "--url", default="wss://127.0.0.1:8443/ws", help="quest server websocket URL"
    )
    parser.add_argument("--speed", type=float, default=1.0, help="time-stretch factor")
    parser.add_argument("--start", type=float, default=0.0, help="skip frames before this second")
    parser.add_argument(
        "--end", type=float, default=float("inf"), help="skip frames after this second"
    )
    parser.add_argument("--loop", action="store_true", help="repeat forever")
    args = parser.parse_args()

    frames = load_frames(args.file, args.start, args.end)
    while True:
        asyncio.run(replay(args.url, frames, args.speed))
        if not args.loop:
            break


if __name__ == "__main__":
    main()
