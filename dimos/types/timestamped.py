# Copyright 2025 Dimensional Inc.
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

from datetime import datetime, timezone

# any class that carries a timestamp should inherit from this
# this allows us to work with timeseries in consistent way, allign messages, replay etc
# aditional functionality will come to this class soon


class Timestamped:
    ts: float

    def __init__(self, ts: float):
        self.ts = ts

    def dt(self) -> datetime:
        return datetime.fromtimestamp(self.ts, tz=timezone.utc).astimezone()

    def ros_timestamp(self) -> dict[str, int]:
        """Convert timestamp to ROS-style dictionary."""
        sec = int(self.ts)
        nanosec = int((self.ts - sec) * 1_000_000_000)
        return [sec, nanosec]
