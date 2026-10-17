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

"""Telling which message is newer, and when a robot has gone quiet.

Messages arrive out of order and get repeated, so each carries a sequence
number that counts up. The higher number wins. ``ControlValues`` also carries
an ``epoch``; nothing here reads it.

Nothing here reads a clock. The current time is passed in, so a test can decide
a robot has gone quiet without waiting for it to happen.
"""

from __future__ import annotations

from dataclasses import dataclass


def is_newer(sequence: int, last_sequence: int | None) -> bool:
    """Whether a message is newer than the last one accepted.

    Its number has to be strictly higher, so a repeated or delayed message
    loses.

    Args:
        sequence: The message's number.
        last_sequence: The number of the last message accepted, or ``None`` if
            none yet.

    Returns:
        True if this message should be used.
    """
    return last_sequence is None or sequence > last_sequence


@dataclass(slots=True)
class Freshness:
    """When a robot was last heard from.

    Updated on every reading, up to 500 times a second, so unlike everything
    else here it is changed in place rather than replaced. It still holds no
    clock of its own; the time is passed in.
    """

    last_receipt: float | None = None

    def mark(self, now: float) -> None:
        """Note that a reading arrived, at time ``now`` in seconds."""
        self.last_receipt = now

    def is_stale(self, now: float, timeout_s: float) -> bool:
        """Whether the robot has gone quiet.

        A robot that has never reported counts as quiet. One that last reported
        exactly ``timeout_s`` ago does not, so a robot running at precisely its
        stated rate does not flicker in and out.

        Args:
            now: The current time, in seconds.
            timeout_s: How long without a reading counts as quiet.
        """
        if self.last_receipt is None:
            return True
        return (now - self.last_receipt) > timeout_s

    def age(self, now: float) -> float | None:
        """Seconds since the last reading, or ``None`` if there has been none."""
        if self.last_receipt is None:
            return None
        return now - self.last_receipt
