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
gesture_id, distance_cm and a gesture counter. Bleak strips the two-byte id, so the
decoder sees the five bytes. These are sensor readings, not robot commands.
"""

from __future__ import annotations

from dataclasses import dataclass

MANUFACTURER_ID = 0xFFFF
PAYLOAD_LENGTH = 5
STALE_AFTER_S = 2.0

# gesture_id byte: 0 = reset/no gesture yet, then the board alternates 1 = A, 2 = B, 1 = A...
# What A and B mean is up to the subscriber (a robot blueprint can map them to sit/stand).
GESTURES: tuple[str, ...] = ("A", "B")


@dataclass(frozen=True)
class GesturePacket:
    state: int
    sequence: int
    gesture_id: int
    distance_cm: int
    count: int

    @property
    def gesture(self) -> str | None:
        return GESTURES[self.gesture_id - 1] if self.gesture_id else None


def decode_packet(payload: bytes) -> GesturePacket:
    if len(payload) != PAYLOAD_LENGTH:
        raise ValueError(f"expected {PAYLOAD_LENGTH} manufacturer bytes, received {len(payload)}")
    packet = GesturePacket(*payload)
    if packet.state not in (0, 1) or packet.gesture_id not in range(len(GESTURES) + 1):
        raise ValueError("invalid state or gesture_id in manufacturer data")
    return packet


class GestureTracker:
    """Turn repeated advertisements into one event per gesture.

    A board re-advertises the same bytes many times, so a gesture is a change in the
    counter, not a received packet. The first packet, a packet after the board went
    stale, and a firmware reset (gesture_id 0) set a baseline without emitting an event.
    """

    def __init__(self, stale_after_s: float = STALE_AFTER_S) -> None:
        self.stale_after_s = stale_after_s
        self.last_seen: float | None = None
        self.packet: GesturePacket | None = None

    def observe(self, packet: GesturePacket, now: float) -> tuple[bool, str | None]:
        """Return (is_new_gesture, warning)."""
        previous = self.packet
        stale = self.is_stale(now)
        gap = None if self.last_seen is None else now - self.last_seen
        self.last_seen = now
        if previous is None or stale or packet.gesture_id == 0:
            self.packet = packet
            if previous is not None and stale and packet.gesture_id != 0:
                skipped = (packet.count - previous.count) % 256
                if skipped:
                    return False, (
                        f"{skipped} gesture(s) made during a {gap:.1f} s gap were not published"
                    )
            return False, None
        delta = (packet.count - previous.count) % 256
        if delta == 0:
            if packet.gesture_id != previous.gesture_id:
                return False, "gesture_id changed without a new gesture counter"
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
