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

from __future__ import annotations

from dataclasses import dataclass, field
import time

from dimos_lcm.sensor_msgs.NavSatFix import NavSatFix as LCMNavSatFix
from dimos_lcm.sensor_msgs.NavSatStatus import NavSatStatus as LCMNavSatStatus

from dimos.types.timestamped import Timestamped


@dataclass
class NavSatFix(Timestamped):
    """Global position fix, ROS ``sensor_msgs/NavSatFix``. Degrees and metres (WGS-84).

    Enum values are the generated ``dimos_lcm`` ones (``NavSatStatus.STATUS_*``,
    ``NavSatFix.COVARIANCE_TYPE_*``).
    """

    msg_name = "sensor_msgs.NavSatFix"

    latitude: float = 0.0
    longitude: float = 0.0
    altitude: float = 0.0
    status: int = LCMNavSatStatus.STATUS_NO_FIX
    service: int = LCMNavSatStatus.SERVICE_GPS
    # Row-major 3x3 ENU covariance in m^2.
    position_covariance: list[float] = field(default_factory=lambda: [0.0] * 9)
    position_covariance_type: int = LCMNavSatFix.COVARIANCE_TYPE_UNKNOWN
    frame_id: str = ""
    ts: float = field(default_factory=time.time)

    def lcm_encode(self) -> bytes:
        msg = LCMNavSatFix()
        msg.header.seq = 0
        msg.header.frame_id = self.frame_id
        [msg.header.stamp.sec, msg.header.stamp.nsec] = self.ros_timestamp()
        msg.status.status = self.status
        msg.status.service = self.service
        msg.latitude = self.latitude
        msg.longitude = self.longitude
        msg.altitude = self.altitude
        msg.position_covariance = list(self.position_covariance)
        msg.position_covariance_type = self.position_covariance_type
        return msg.lcm_encode()  # type: ignore[no-any-return]

    @classmethod
    def lcm_decode(cls, data: bytes) -> NavSatFix:
        msg = LCMNavSatFix.lcm_decode(data)
        return cls(
            latitude=msg.latitude,
            longitude=msg.longitude,
            altitude=msg.altitude,
            status=msg.status.status,
            service=msg.status.service,
            position_covariance=list(msg.position_covariance),
            position_covariance_type=msg.position_covariance_type,
            frame_id=msg.header.frame_id,
            ts=msg.header.stamp.sec + msg.header.stamp.nsec / 1e9,
        )
