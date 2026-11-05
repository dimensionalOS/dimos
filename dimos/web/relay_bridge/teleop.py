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

"""Cockpit teleop on the robot side: relay-stamped lease generations, the
per-lease sequence high-water mark, the release-edge zero and the deadman
watchdog (docs/web/bridge.md, "Teleop watchdog")."""

from __future__ import annotations

import asyncio
from collections.abc import Callable
from dataclasses import dataclass
import math
import time

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.utils.generic import finite_number
from dimos.utils.logging_config import setup_logger
from dimos.web.relay_bridge.protocol import (
    ChannelSpec,
    Stop as WireStop,
    TeleopStart as WireTeleopStart,
    TeleopStop as WireTeleopStop,
    Twist as WireTwist,
)

logger = setup_logger()

# Deadman poll granularity; small against the default 300 ms watchdog window
# so the zero lands close to the deadline.
_TELEOP_POLL_S = 0.05


@dataclass(frozen=True)
class TeleopParams:
    """Teleop tx channel params resolved at start (manifest, with defaults)."""

    max_linear: float
    max_angular: float
    boost: float
    watchdog_s: float


# Manifest param keys and their defaults (matching the Teleop panel's).
_TELEOP_PARAM_DEFAULTS = {"maxLinear": 0.8, "maxAngular": 1.0, "boost": 2.0, "watchdogMs": 300.0}


def resolve_teleop_params(spec: ChannelSpec) -> TeleopParams:
    values: dict[str, float] = {}
    for key, default in _TELEOP_PARAM_DEFAULTS.items():
        candidate = spec.params.get(key, default)
        value = finite_number(candidate, f"manifest channel {spec.ch!r} {key}")
        if value <= 0:
            raise RuntimeError(
                f"manifest channel {spec.ch!r} {key} must be a positive number, got {candidate!r}"
            )
        values[key] = value
    return TeleopParams(
        max_linear=values["maxLinear"],
        max_angular=values["maxAngular"],
        boost=values["boost"],
        watchdog_s=values["watchdogMs"] / 1000.0,
    )


def _clamp(value: float, bound: float) -> float:
    return max(-bound, min(bound, value))


class Teleop:
    """Teleop state for one bridge, all touched on the module loop only.

    `publish` is the bridge's tele_cmd_vel Out. `driving` implements the
    release-edge rule: publish a zero only after a nonzero (MovementManager
    cancels the nav goal on EVERY teleop message, so idle zeros must never
    repeat). The lease-generation floor is relay-stamped and monotonic in a
    session: anything below it is voided permanently, so a released holder's
    delayed datagrams cannot restart motion after a stop.
    """

    def __init__(self, params: TeleopParams, publish: Callable[[Twist], None]) -> None:
        self._params = params
        self._publish = publish
        self._driving = False
        self._gen: int | float = -math.inf
        self._last_seq = -math.inf
        self._last_rx = 0.0

    def on_twist(self, msg: WireTwist) -> None:
        params = self._params
        # Non-finite components cannot reach here: the wire decoders reject
        # NaN/Infinity (allow_inf_nan=False).
        gen = msg.gen
        if gen is None or gen < self._gen:
            return  # unstamped, or in flight from a lease already voided
        if gen == self._gen and msg.seq <= self._last_seq:
            # Within a generation the high-water mark is permanent: a paused
            # holder resumes with rising seq, while a delayed pre-stop twist
            # stays dead even after watchdog silence. A new lease (gen above
            # the floor) rebaselines instead.
            return
        self._gen = gen
        self._last_seq = float(msg.seq)
        self._last_rx = time.monotonic()
        bound_linear = params.max_linear * params.boost
        vx = _clamp(float(msg.vx), bound_linear)
        vy = _clamp(float(msg.vy), bound_linear)
        wz = _clamp(float(msg.wz), params.max_angular * params.boost)
        if vx == 0.0 and vy == 0.0 and wz == 0.0:
            self._zero("release")
            return
        self._driving = True
        self._publish(Twist(linear=Vector3(vx, vy, 0.0), angular=Vector3(0.0, 0.0, wz)))

    def on_stop(self, msg: WireStop) -> None:
        """E-stop: unconditional zero, even from idle - it must also cancel
        an autonomous nav goal (MovementManager cancels on any teleop msg)."""
        gen = msg.gen
        if gen is None or gen < self._gen:
            # A voided lease's in-flight e-stop must not blip the current
            # holder (the relay gate already blocks post-release sends).
            return
        self._zero("stop message (e-stop)", force=True)
        if gen > self._gen:
            self._gen = gen
            self._last_seq = float(msg.seq)
        else:
            # max(): a stale reordered e-stop must not lower the high-water
            # mark and let an already-superseded twist re-apply.
            self._last_seq = max(self._last_seq, float(msg.seq))
        self._last_rx = time.monotonic()

    def on_start(self, msg: WireTeleopStart) -> None:
        """A relay-granted lease: adopt its generation, voiding the previous
        one. Heals a lost teleop_stop (zeroing if it arrived mid-drive)."""
        gen = msg.gen
        if gen is None or gen <= self._gen:
            return  # a duplicated start must not reset the high-water mid-lease
        self._zero("new teleop lease")
        self._gen = gen
        self._last_seq = -math.inf
        self._last_rx = 0.0

    def on_end(self, msg: WireTeleopStop) -> None:
        """The lease ended (holder released, disconnected, or watched away)."""
        gen = msg.gen
        if gen is None or gen < self._gen:
            return  # a stale lease-end must not blip or reset the current holder
        self._zero("teleop lease ended")
        # The relay bumps by exactly 1 per grant, so this floor equals the
        # next lease's generation; the ended lease and everything below it
        # are voided permanently.
        self._gen = gen + 1
        self._last_seq = -math.inf
        self._last_rx = 0.0

    def session_ended(self) -> None:
        """A driving teleop stream cannot outlive its relay session (this also
        covers module stop, KeyboardTeleop.stop() parity). Idempotent: the
        edge gate makes the second call of a double-disconnect a no-op.
        Datagrams are QUIC-session-scoped and the relay's lease generation
        dies with the robot registration, so the floor resets too: nothing
        stale can leak into the next session."""
        self._zero("relay session ended")
        self._gen = -math.inf
        self._last_seq = -math.inf
        self._last_rx = 0.0

    async def watchdog(self) -> None:
        """Deadman: the cockpit repeats commands at publish_hz, so silence
        while driving means the chain broke (viewer gone, relay killed,
        datagrams lost) - zero within ~watchdog_s regardless of which hop
        failed."""
        watchdog_s = self._params.watchdog_s
        while True:
            await asyncio.sleep(_TELEOP_POLL_S)
            if self._driving and time.monotonic() - self._last_rx > watchdog_s:
                self._zero("watchdog: twist silence")

    def _zero(self, reason: str, *, force: bool = False) -> None:
        """Publish one zero twist; edge-gated unless `force` (e-stop)."""
        if not force and not self._driving:
            return
        self._driving = False
        logger.warning(f"relay bridge teleop: zero twist ({reason})")
        self._publish(Twist.zero())
