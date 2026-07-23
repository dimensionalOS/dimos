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

"""Headless quest teleop driver: streams fake controller data over the websocket.

Connects to a running quest teleop blueprint exactly like the headset web
client does (binary LCM Joy and PoseStamped over /ws) and drives a repeating
pattern. Lets the whole teleop stack run on a bench with no headset:

    dimos run r1lite-quest-teleop-sim          # terminal 1
    python scripts/r1lite_test/fake_quest_stream.py --pattern wave   # terminal 2

Patterns:
    wave   engage both arms and move hands in slow circles, triggers pulsing
    drive  thumbstick square: forward, strafe, yaw, stop
    idle   connected controllers, no buttons, sticks centered
"""

from __future__ import annotations

import argparse
import asyncio
import math
import ssl
import time

import websockets

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Joy import Joy

RATE_HZ = 30.0

# Joy layout expected by QuestControllerState.from_joy:
# axes [thumbstick_x, thumbstick_y, trigger, grip]
# buttons [trigger, grip, touchpad, thumbstick, X/A, Y/B, menu]


def _joy(
    frame_id: str,
    *,
    stick_x: float = 0.0,
    stick_y: float = 0.0,
    trigger: float = 0.0,
    primary: bool = False,
) -> bytes:
    return Joy(
        ts=time.time(),
        frame_id=frame_id,
        axes=[stick_x, stick_y, trigger, 0.0],
        buttons=[int(trigger > 0.5), 0, 0, 0, int(primary), 0, 0],
    ).lcm_encode()


def _pose(frame_id: str, x: float, y: float, z: float) -> bytes:
    return PoseStamped(
        ts=time.time(),
        frame_id=frame_id,
        position=Vector3(x=x, y=y, z=z),
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
    ).lcm_encode()


async def _run(url: str, pattern: str, duration: float) -> None:
    ssl_ctx: ssl.SSLContext | None = None
    if url.startswith("wss"):
        ssl_ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
        ssl_ctx.check_hostname = False
        ssl_ctx.verify_mode = ssl.CERT_NONE  # self-signed teleop cert

    async with websockets.connect(url, ssl=ssl_ctx, max_size=None) as ws:
        print(f"connected to {url}, pattern={pattern}, duration={duration}s")
        t0 = time.monotonic()
        period = 1.0 / RATE_HZ
        while (t := time.monotonic() - t0) < duration:
            if pattern == "wave":
                engage = t > 1.0
                radius = 0.08
                dx = radius * math.cos(0.5 * math.tau * t / 4.0)
                dz = radius * math.sin(0.5 * math.tau * t / 4.0)
                trigger = 0.5 + 0.5 * math.sin(0.5 * math.tau * t / 6.0)
                await ws.send(_joy("left", trigger=trigger, primary=engage))
                await ws.send(_joy("right", trigger=1.0 - trigger, primary=engage))
                await ws.send(_pose("left", dx, 0.0, dz))
                await ws.send(_pose("right", dx, 0.0, -dz))
            elif pattern == "drive":
                phase = int(t) % 8
                stick = {
                    0: (0.0, -1.0, 0.0),  # forward
                    2: (1.0, 0.0, 0.0),  # strafe
                    4: (0.0, 0.0, 1.0),  # yaw
                }.get(phase, (0.0, 0.0, 0.0))
                await ws.send(_joy("left", stick_x=stick[0], stick_y=stick[1]))
                await ws.send(_joy("right", stick_x=stick[2]))
            else:
                await ws.send(_joy("left"))
                await ws.send(_joy("right"))
            await asyncio.sleep(period)
        print("done")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="wss://localhost:8443/ws")
    parser.add_argument("--pattern", choices=["wave", "drive", "idle"], default="wave")
    parser.add_argument("--duration", type=float, default=30.0)
    args = parser.parse_args()
    asyncio.run(_run(args.url, args.pattern, args.duration))


if __name__ == "__main__":
    main()
