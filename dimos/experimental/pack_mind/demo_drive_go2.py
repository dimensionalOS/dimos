# Copyright 2025 Dimensional Inc.
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

"""Headless cmd_vel choreography for a live Go2 — no GUI / pygame required.

The keyboard teleop blueprint needs an X11/Cocoa window for pygame, which is
unavailable over SSH or in a headless session. This script drives the dog by
publishing Twist messages straight onto the /cmd_vel LCM topic at a fixed rate,
exactly like KeyboardTeleop does — the running blueprint's ControlCoordinator
relays them to the robot over WebRTC.

Run a relay blueprint in one terminal (it holds the WebRTC connection and the
ControlCoordinator; the pygame crash in its log is harmless):

    uv run dimos --robot-ip 192.168.12.1 run unitree-go2-webrtc-keyboard-teleop

Then, once it logs "ControlCoordinator started", in a second terminal on the
SAME machine (LCM is host-local multicast):

    uv run python dimos/experimental/pack_mind/demo_drive_go2.py
"""

from __future__ import annotations

import time

from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3

CMD_VEL_TOPIC = "/cmd_vel"
DEFAULT_RATE_HZ = 10.0


def drive(transport: LCMTransport[Twist], vx: float, wz: float, secs: float, hz: float = DEFAULT_RATE_HZ) -> None:
    """Hold a velocity for `secs` by republishing at `hz`.

    vx: forward m/s (positive forward, negative back).
    wz: yaw rad/s (positive left, negative right).
    Republishing is required: the connection layer auto-stops 0.2s after the
    last command, so a single publish only produces a brief nudge.
    """
    deadline = time.time() + secs
    period = 1.0 / hz
    while time.time() < deadline:
        transport.broadcast(
            None,
            Twist(
                linear=Vector3(x=vx, y=0.0, z=0.0),
                angular=Vector3(x=0.0, y=0.0, z=wz),
            ),
        )
        time.sleep(period)


def main() -> None:
    transport: LCMTransport[Twist] = LCMTransport(CMD_VEL_TOPIC, Twist)

    # Choreographed routine. Tune speeds/durations/order for your space.
    # Start conservative: clear ~2m, keep the e-stop within reach.
    drive(transport, vx=0.0, wz=0.0, secs=0.5)  # settle
    drive(transport, vx=0.3, wz=0.0, secs=2.0)  # forward
    drive(transport, vx=0.0, wz=0.5, secs=2.0)  # turn left in place
    drive(transport, vx=0.3, wz=0.0, secs=2.0)  # forward
    drive(transport, vx=0.0, wz=0.0, secs=0.5)  # stop


if __name__ == "__main__":
    main()
