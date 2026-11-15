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

import asyncio
import time

import pytest

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.web.relay_bridge import teleop
from dimos.web.relay_bridge.protocol import Stop as WireStop, Twist as WireTwist


def test_only_the_deadman_zero_is_logged_as_a_warning(monkeypatch: pytest.MonkeyPatch) -> None:
    """Key releases and e-stops are routine. Twist silence while driving means
    the chain to the cockpit broke, and only that is a warning."""
    levels: list[str] = []
    monkeypatch.setattr(teleop.logger, "info", lambda msg: levels.append("info"))
    monkeypatch.setattr(teleop.logger, "warning", lambda msg: levels.append("warning"))
    params = teleop.TeleopParams(max_linear=0.8, max_angular=1.0, boost=2.0, watchdog_s=0.0)
    published: list[Twist] = []
    state = teleop.Teleop(params, published.append)

    def twist(vx: float, seq: int) -> WireTwist:
        return WireTwist(vx=vx, vy=0.0, wz=0.0, seq=seq, ts=time.time(), gen=1)

    state.on_twist(twist(0.5, seq=1))
    state.on_twist(twist(0.0, seq=2))  # key release
    state.on_stop(WireStop(seq=3, ts=time.time(), gen=1))  # e-stop
    assert levels == ["info", "info"]

    async def until_the_deadman_zeroes() -> None:
        watchdog = asyncio.ensure_future(state.watchdog())
        while not published[-1].is_zero():
            await asyncio.sleep(0.01)
        watchdog.cancel()

    state.on_twist(twist(0.5, seq=4))
    asyncio.run(asyncio.wait_for(until_the_deadman_zeroes(), timeout=10))
    assert levels == ["info", "info", "warning"]
