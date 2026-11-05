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

"""Decode ESP32 gesture-sensor BLE advertisements. No radio or dimOS imports.

The board advertises manufacturer id 0xFFFF followed by five bytes: state, sequence,
posture, distance_cm and a gesture counter. Bleak strips the two-byte id, so the
decoder sees the five bytes. These are sensor readings, not robot commands.
"""

from __future__ import annotations

from dataclasses import dataclass

MANUFACTURER_ID = 0xFFFF
PAYLOAD_LENGTH = 5
STALE_AFTER_S = 2.0

# Posture byte: 0 = reset/no gesture yet, 1 = SIT, 2 = STAND.
GESTURES: tuple[str, ...] = ("SIT", "STAND")


@dataclass(frozen=True)
class GesturePacket:
    state: int
    sequence: int
    posture: int
    distance_cm: int
    count: int

    @property
    def gesture(self) -> str | None:
        return GESTURES[self.posture - 1] if self.posture else None


def decode_packet(payload: bytes) -> GesturePacket:
    if len(payload) != PAYLOAD_LENGTH:
        raise ValueError(f"expected {PAYLOAD_LENGTH} manufacturer bytes, received {len(payload)}")
    packet = GesturePacket(*payload)
    if packet.state not in (0, 1) or packet.posture not in range(len(GESTURES) + 1):
        raise ValueError("invalid state or posture in manufacturer data")
    return packet


class GestureTracker:
    """Turn repeated advertisements into one event per gesture.

    A board re-advertises the same bytes many times, so a gesture is a change in the
    counter, not a received packet. The first packet, a packet after the board went
    stale, and a firmware reset (posture 0) set a baseline without emitting an event.
    """

    def __init__(self, stale_after_s: float = STALE_AFTER_S) -> None:
        self.stale_after_s = stale_after_s
        self.last_seen: float | None = None
        self.packet: GesturePacket | None = None

    def observe(self, packet: GesturePacket, now: float) -> tuple[bool, str | None]:
        """Return (is_new_gesture, warning)."""
        previous = self.packet
        baseline = previous is None or self.is_stale(now) or packet.posture == 0
        self.last_seen = now
        if previous is None or baseline:
            self.packet = packet
            return False, None
        delta = (packet.count - previous.count) % 256
        if delta == 0:
            if packet.posture != previous.posture:
                return False, "posture changed without a new gesture counter"
            self.packet = packet
            return False, None
        if delta >= 128:
            # An old or out-of-order advertisement must not become a new event.
            return False, "counter moved backwards; waiting for a fresh counter or baseline"
        self.packet = packet
        warning = f"missed {delta - 1} gesture counter(s)" if delta > 1 else None
        return True, warning

    def is_stale(self, now: float) -> bool:
        return self.last_seen is not None and now - self.last_seen > self.stale_after_s

    def state(self, now: float) -> str:
        if self.last_seen is None:
            return "waiting"
        return "stale" if self.is_stale(now) else "online"
