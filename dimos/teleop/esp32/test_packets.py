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

import pytest

from dimos.teleop.esp32.packets import GesturePacket, GestureTracker, decode_packet


def packet(posture: int, count: int, distance_cm: int = 20) -> GesturePacket:
    return GesturePacket(state=0, sequence=0, posture=posture, distance_cm=distance_cm, count=count)


def test_decode_packet_reads_the_five_bytes() -> None:
    decoded = decode_packet(bytes([1, 7, 2, 18, 3]))
    assert decoded == GesturePacket(state=1, sequence=7, posture=2, distance_cm=18, count=3)
    assert decoded.gesture == "STAND"


@pytest.mark.parametrize(
    "payload", [bytes(4), bytes(6), bytes([2, 0, 1, 0, 0]), bytes([0, 0, 3, 0, 0])]
)
def test_decode_packet_rejects_bad_payloads(payload: bytes) -> None:
    with pytest.raises(ValueError):
        decode_packet(payload)


def test_first_packet_is_a_baseline_not_a_gesture() -> None:
    tracker = GestureTracker()
    assert tracker.observe(packet(1, count=5), now=0.0) == (False, None)


def test_new_counter_is_one_gesture_and_repeats_are_ignored() -> None:
    tracker = GestureTracker()
    tracker.observe(packet(1, count=5), now=0.0)
    assert tracker.observe(packet(2, count=6), now=0.1) == (True, None)
    assert tracker.observe(packet(2, count=6), now=0.2) == (False, None)


def test_counter_rolls_over_from_255_to_0() -> None:
    tracker = GestureTracker()
    tracker.observe(packet(1, count=255), now=0.0)
    assert tracker.observe(packet(2, count=0), now=0.1) == (True, None)


def test_skipped_counters_still_emit_with_a_warning() -> None:
    tracker = GestureTracker()
    tracker.observe(packet(1, count=1), now=0.0)
    assert tracker.observe(packet(2, count=4), now=0.1) == (True, "missed 2 gesture counter(s)")


def test_backwards_counter_is_ignored() -> None:
    tracker = GestureTracker()
    tracker.observe(packet(1, count=10), now=0.0)
    is_new, warning = tracker.observe(packet(2, count=9), now=0.1)
    assert not is_new and warning is not None


def test_stale_board_rebaselines_instead_of_emitting() -> None:
    tracker = GestureTracker(stale_after_s=2.0)
    tracker.observe(packet(1, count=1), now=0.0)
    assert tracker.state(now=3.0) == "stale"
    assert tracker.observe(packet(2, count=2), now=3.0) == (False, None)
    assert tracker.state(now=3.0) == "online"


def test_firmware_reset_rebaselines() -> None:
    tracker = GestureTracker()
    tracker.observe(packet(1, count=1), now=0.0)
    assert tracker.observe(packet(0, count=2), now=0.1) == (False, None)
