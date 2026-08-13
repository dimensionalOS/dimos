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

from collections.abc import Sequence
import time
from typing import Any, BinaryIO

from dimos_lcm.geometry_msgs import WrenchStamped as LCMWrenchStamped
import numpy as np

from dimos.msgs.geometry_msgs.Wrench import Wrench
from dimos.types.timestamped import Timestamped


class WrenchStamped(Wrench, Timestamped):
    """A force/torque measurement with a timestamp and frame_id."""

    msg_name = "geometry_msgs.WrenchStamped"
    ts: float
    frame_id: str

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        """Initialize a stamped wrench.

        Takes ``(ts, frame_id)`` positionally, plus every Wrench keyword. Any other
        positional form belongs to Wrench, so ``WrenchStamped(force, torque)`` works
        as it does on the base class.
        """
        ts: Any
        # A leading number or string is a timestamp; anything else is a Wrench argument.
        if args and isinstance(args[0], int | float | str):
            ts = args[0]
            frame_id = args[1] if len(args) > 1 else kwargs.pop("frame_id", "")
            wrench_args: tuple[Any, ...] = ()
        else:
            ts = kwargs.pop("ts", None)
            frame_id = kwargs.pop("frame_id", "")
            wrench_args = args

        self.frame_id = frame_id
        self.ts = time.time() if ts is None else ts
        super().__init__(*wrench_args, **kwargs)

    @classmethod
    def from_array(  # type: ignore[override]
        cls,
        ft: Sequence[float] | np.ndarray,
        frame_id: str = "ft_sensor",
        ts: float | None = None,
    ) -> WrenchStamped:
        """Build from a 6-element ``[fx, fy, fz, tx, ty, tz]`` sensor reading."""
        if len(ft) != 6:
            raise ValueError(f"Expected 6 elements [fx, fy, fz, tx, ty, tz], got {len(ft)}")
        return cls(ts=ts, frame_id=frame_id, force=ft[:3], torque=ft[3:])

    def lcm_encode(self) -> bytes:
        lcm_msg = LCMWrenchStamped()
        lcm_msg.wrench = self
        [lcm_msg.header.stamp.sec, lcm_msg.header.stamp.nsec] = self.ros_timestamp()
        lcm_msg.header.frame_id = self.frame_id
        return lcm_msg.lcm_encode()  # type: ignore[no-any-return]

    @classmethod
    def lcm_decode(cls, data: bytes | BinaryIO) -> WrenchStamped:
        lcm_msg = LCMWrenchStamped.lcm_decode(data)
        return cls(
            ts=lcm_msg.header.stamp.sec + (lcm_msg.header.stamp.nsec / 1_000_000_000),
            frame_id=lcm_msg.header.frame_id,
            force=[lcm_msg.wrench.force.x, lcm_msg.wrench.force.y, lcm_msg.wrench.force.z],
            torque=[lcm_msg.wrench.torque.x, lcm_msg.wrench.torque.y, lcm_msg.wrench.torque.z],
        )

    def __repr__(self) -> str:
        return (
            f"WrenchStamped(force={self.force!r}, torque={self.torque!r}, "
            f"ts={self.ts}, frame_id={self.frame_id!r})"
        )

    def __str__(self) -> str:
        return (
            f"WrenchStamped(force=[{self.force.x:.3f}, {self.force.y:.3f}, {self.force.z:.3f}], "
            f"torque=[{self.torque.x:.3f}, {self.torque.y:.3f}, {self.torque.z:.3f}], "
            f"frame_id={self.frame_id!r})"
        )
