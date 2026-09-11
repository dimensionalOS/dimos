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
from typing import Any, BinaryIO, TypeAlias

from dimos_lcm.geometry_msgs import TwistStamped as LCMTwistStamped

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import VectorConvertable
from dimos.types.timestamped import Timestamped

# Types that can be converted to/from TwistStamped
TwistConvertable: TypeAlias = (
    tuple[VectorConvertable, VectorConvertable] | LCMTwistStamped | dict[str, VectorConvertable]
)


def sec_nsec(ts):  # type: ignore[no-untyped-def]
    s = int(ts)
    return [s, int((ts - s) * 1_000_000_000)]


class TwistStamped(Twist, Timestamped):
    msg_name = "geometry_msgs.TwistStamped"
    ts: float
    frame_id: str

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        """Initialize a stamped twist.

        Takes ``(ts, frame_id)`` positionally, plus every Twist keyword. Any other
        positional form belongs to Twist, so ``TwistStamped(linear, angular)``
        works as it does on the base class.
        """
        ts: Any
        # A leading number or string is a timestamp; anything else is a Twist argument.
        if args and isinstance(args[0], int | float | str):
            ts = args[0]
            frame_id = args[1] if len(args) > 1 else kwargs.pop("frame_id", "")
            twist_args: tuple[Any, ...] = ()
        else:
            ts = kwargs.pop("ts", None)
            frame_id = kwargs.pop("frame_id", "")
            twist_args = args

        self.frame_id = frame_id
        self.ts = time.time() if ts is None else ts
        super().__init__(*twist_args, **kwargs)

    def lcm_encode(self) -> bytes:
        lcm_msg = LCMTwistStamped()
        lcm_msg.twist = self
        [lcm_msg.header.stamp.sec, lcm_msg.header.stamp.nsec] = sec_nsec(self.ts)  # type: ignore[no-untyped-call]
        lcm_msg.header.frame_id = self.frame_id
        return lcm_msg.lcm_encode()  # type: ignore[no-any-return]

    @classmethod
    def lcm_decode(cls, data: bytes | BinaryIO) -> TwistStamped:
        lcm_msg = LCMTwistStamped.lcm_decode(data)
        return cls(
            ts=lcm_msg.header.stamp.sec + (lcm_msg.header.stamp.nsec / 1_000_000_000),
            frame_id=lcm_msg.header.frame_id,
            linear=[lcm_msg.twist.linear.x, lcm_msg.twist.linear.y, lcm_msg.twist.linear.z],
            angular=[lcm_msg.twist.angular.x, lcm_msg.twist.angular.y, lcm_msg.twist.angular.z],
        )

    def __str__(self) -> str:
        return (
            f"TwistStamped(linear=[{self.linear.x:.3f}, {self.linear.y:.3f}, {self.linear.z:.3f}], "
            f"angular=[{self.angular.x:.3f}, {self.angular.y:.3f}, {self.angular.z:.3f}])"
        )
