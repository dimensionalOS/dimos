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

"""DDS wire codecs: CDR bytes <-> message, keyed by DDS topic.

A :class:`DdsCodec` is the bytes<->payload pair for one DDS message type. The
same codec decodes a recorded mcap message and a live DDS sample (both are CDR),
and its ``encode`` half publishes back to the wire — so this is shared by the
reader, :class:`~dimos.robot.unitree.go2.dds.store.Go2McapStore`, and (later) a live
DDS bridge. It is distinct from memory2's storage codecs (pickle/lcm/jpeg);
they only coincide when an mcap is opened as a store.

``GO2_CODECS`` is the Go2 channel set — the default registry today.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, fields
import json
from typing import Any, Protocol, runtime_checkable

from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.go2.dds import cdr, ros
from dimos.robot.unitree.go2.dds.msgs.ControlEvent import ControlEvent
from dimos.robot.unitree.go2.dds.msgs.LowState import LowState
from dimos.robot.unitree.go2.dds.msgs.SportModeState import SportModeState
from dimos.robot.unitree.go2.dds.msgs.Telemetry import Telemetry


@runtime_checkable
class DdsCodec(Protocol):
    """Codec between DDS wire bytes (CDR) and a payload message."""

    @property
    def payload_type(self) -> type: ...

    def decode(self, data: bytes) -> Any: ...
    def encode(self, msg: Any) -> bytes: ...


@dataclass(frozen=True)
class CdrStructCodec:
    """Codec for a fixed CDR struct spec (e.g. the Unitree custom msgs)."""

    payload_type: type  # the spec dataclass; also the decoded payload type

    def decode(self, data: bytes) -> Any:
        msg, end = cdr.decode(data, self.payload_type)
        # Fixed-layout struct: leftover bytes mean the spec is wrong — fail loud.
        assert end == len(data), f"{self.payload_type.__name__}: {end} != {len(data)} bytes"
        return msg

    def encode(self, msg: Any) -> bytes:
        raise NotImplementedError("CDR struct encode not implemented yet")


@dataclass(frozen=True)
class FnCodec:
    """Codec wrapping a decode function (e.g. ROS wire -> dimos msg)."""

    payload_type: type
    decoder: Callable[[bytes], Any]

    def decode(self, data: bytes) -> Any:
        return self.decoder(data)

    def encode(self, msg: Any) -> bytes:
        raise NotImplementedError(f"encode not implemented for {self.payload_type.__name__}")


@dataclass(frozen=True)
class JsonCodec:
    """Codec for app-level JSON channels -> a dataclass.

    Keys absent from ``payload_type`` are dropped, so heterogeneous event logs
    (e.g. ``control_log``) and future fields decode without error.
    """

    payload_type: type

    def decode(self, data: bytes) -> Any:
        d = json.loads(data)
        names = {f.name for f in fields(self.payload_type)}
        return self.payload_type(**{k: v for k, v in d.items() if k in names})

    def encode(self, msg: Any) -> bytes:
        from dataclasses import asdict

        return json.dumps(asdict(msg)).encode()


# Go2 channel topic -> codec. The default registry (only platform we have today).
GO2_CODECS: dict[str, DdsCodec] = {
    "rt/utlidar/cloud": FnCodec(PointCloud2, ros.decode_pointcloud2),
    "rt/utlidar/imu": FnCodec(Imu, ros.decode_imu),
    "rt/utlidar/robot_odom": FnCodec(Odometry, ros.decode_odometry),
    "rt/frontvideo": FnCodec(Image, ros.decode_compressed_image),
    "rt/lowstate": CdrStructCodec(LowState),
    "rt/sportmodestate": CdrStructCodec(SportModeState),
    "telemetry": JsonCodec(Telemetry),
    "control_log": JsonCodec(ControlEvent),
}
