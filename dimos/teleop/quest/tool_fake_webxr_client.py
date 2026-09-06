#!/usr/bin/env python3
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

"""Drive a running Quest teleop blueprint with no headset.

Sends the same three pose streams and Joy that the WebXR page sends, with both
primary buttons held, and sweeps the hands so a working IK chain produces
visible joint motion. Use it to tell "the operator link is broken" apart from
"the robot is not moving" without putting a headset on.

    dimos run teleop-quest-r1pro
    python -m dimos.teleop.quest.tool_fake_webxr_client

Then confirm the joints actually moved:

    dimos spy      # dimos/coordinator_joint_state should be live
"""

from __future__ import annotations

import argparse
import asyncio
import math
import ssl
import time

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Joy import Joy

# WebXR reference space: +x right, +y up, -z forward. Roughly a standing adult
# with both hands out front, which is a pose the arms can actually reach.
_HEAD = (0.0, 1.55, 0.0)
_LEFT = (-0.25, 1.10, -0.35)
_RIGHT = (0.25, 1.10, -0.35)
_PRIMARY_BUTTON = 4  # X on left, A on right
_BUTTON_COUNT = 7
_ENGAGE_AFTER_S = 1.0  # settle first, so engage is a clean rising edge


def _pose(frame_id: str, x: float, y: float, z: float) -> bytes:
    return PoseStamped(
        ts=time.time(),
        frame_id=frame_id,
        position=[x, y, z],
        orientation=[0.0, 0.0, 0.0, 1.0],
    ).lcm_encode()


def _joy(frame_id: str, primary: bool) -> bytes:
    buttons = [0] * _BUTTON_COUNT
    if primary:
        buttons[_PRIMARY_BUTTON] = 1
    return Joy(frame_id=frame_id, axes=[0.0, 0.0, 0.0, 0.0], buttons=buttons).lcm_encode()


async def _drive(url: str, seconds: float, hz: float, amplitude: float) -> None:
    import websockets

    # The teleop server serves a self-signed cert, same as it does to a headset.
    ctx = ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE

    async with websockets.connect(url, ssl=ctx, max_size=None) as ws:
        print(f"connected to {url}")
        start = time.time()
        sent = 0
        while (elapsed := time.time() - start) < seconds:
            sweep = elapsed * 0.8
            dx = amplitude * math.sin(sweep)
            dz = amplitude * math.cos(sweep)
            held = elapsed > _ENGAGE_AFTER_S

            await ws.send(_pose("head", *_HEAD))
            await ws.send(_pose("left", _LEFT[0] + dx, _LEFT[1], _LEFT[2] + dz))
            await ws.send(_pose("right", _RIGHT[0] + dx, _RIGHT[1], _RIGHT[2] + dz))
            await ws.send(_joy("left", held))
            await ws.send(_joy("right", held))
            sent += 5
            await asyncio.sleep(1.0 / hz)
        print(
            f"sent {sent} messages over {seconds:.0f}s, primaries held after {_ENGAGE_AFTER_S:.0f}s"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="wss://127.0.0.1:8443/ws")
    parser.add_argument("--seconds", type=float, default=15.0)
    parser.add_argument("--hz", type=float, default=30.0)
    parser.add_argument(
        "--amplitude",
        type=float,
        default=0.10,
        help="metres of hand sweep either side of the start pose",
    )
    args = parser.parse_args()
    asyncio.run(_drive(args.url, args.seconds, args.hz, args.amplitude))


if __name__ == "__main__":
    main()
