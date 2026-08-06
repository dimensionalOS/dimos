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

from __future__ import annotations

import time
from typing import Any, TypeAlias

from dimos_lcm.sensor_msgs import Joy as LCMJoy

from dimos.types.timestamped import Timestamped

# Types that can be converted to/from Joy
JoyConvertable: TypeAlias = (
    tuple[list[float], list[int]] | dict[str, list[float] | list[int]] | LCMJoy
)


def sec_nsec(ts):  # type: ignore[no-untyped-def]
    s = int(ts)
    return [s, int((ts - s) * 1_000_000_000)]


def _fields_of(source: Any) -> tuple[Any, ...]:
    """Pull the four Joy fields out of a dict, an (axes, buttons) pair, a Joy or an LCM message."""
    if isinstance(source, dict):
        return (
            source.get("ts"),
            source.get("frame_id", ""),
            source.get("axes"),
            source.get("buttons"),
        )
    if isinstance(source, tuple | list):
        return (None, "", list(source[0]), list(source[1]))
    if isinstance(source, Joy):
        return (source.ts, source.frame_id, list(source.axes), list(source.buttons))
    return (
        source.header.stamp.sec + (source.header.stamp.nsec / 1_000_000_000),
        source.header.frame_id,
        list(source.axes),
        list(source.buttons),
    )


class Joy(Timestamped):
    msg_name = "sensor_msgs.Joy"
    ts: float
    frame_id: str
    axes: list[float]
    buttons: list[int]

    def __init__(
        self,
        ts: float | JoyConvertable | Joy | None = None,
        frame_id: str = "",
        axes: list[float] | None = None,
        buttons: list[int] | None = None,
    ) -> None:
        """Initialize a Joy message.

        The first argument doubles as the source for the copy/dict/pair/LCM forms:
        ``Joy(other)``, ``Joy({...})``, ``Joy((axes, buttons))``, ``Joy(lcm_msg)``.

        Args:
            ts: Timestamp in seconds
            frame_id: Frame ID for the message
            axes: List of axis values (typically -1.0 to 1.0)
            buttons: List of button states (0 or 1)
        """
        stamp: Any = ts
        if isinstance(stamp, dict | tuple | list | Joy | LCMJoy):
            stamp, frame_id, axes, buttons = _fields_of(stamp)

        self.ts = time.time() if stamp is None else stamp
        self.frame_id = frame_id
        self.axes = axes if axes is not None else []
        self.buttons = buttons if buttons is not None else []

    def lcm_encode(self) -> bytes:
        lcm_msg = LCMJoy()
        [lcm_msg.header.stamp.sec, lcm_msg.header.stamp.nsec] = sec_nsec(self.ts)  # type: ignore[no-untyped-call]
        lcm_msg.header.frame_id = self.frame_id
        lcm_msg.axes_length = len(self.axes)
        lcm_msg.axes = self.axes
        lcm_msg.buttons_length = len(self.buttons)
        lcm_msg.buttons = self.buttons
        return lcm_msg.lcm_encode()  # type: ignore[no-any-return]

    @classmethod
    def lcm_decode(cls, data: bytes) -> Joy:
        lcm_msg = LCMJoy.lcm_decode(data)
        return cls(
            ts=lcm_msg.header.stamp.sec + (lcm_msg.header.stamp.nsec / 1_000_000_000),
            frame_id=lcm_msg.header.frame_id,
            axes=list(lcm_msg.axes) if lcm_msg.axes else [],
            buttons=list(lcm_msg.buttons) if lcm_msg.buttons else [],
        )

    def __str__(self) -> str:
        return (
            f"Joy(axes={len(self.axes)} values, buttons={len(self.buttons)} values, "
            f"frame_id='{self.frame_id}')"
        )

    def __repr__(self) -> str:
        return (
            f"Joy(ts={self.ts}, frame_id='{self.frame_id}', "
            f"axes={self.axes}, buttons={self.buttons})"
        )

    def __eq__(self, other) -> bool:  # type: ignore[no-untyped-def]
        """Check if two Joy messages are equal."""
        if not isinstance(other, Joy):
            return False
        return (
            self.axes == other.axes
            and self.buttons == other.buttons
            and self.frame_id == other.frame_id
        )
