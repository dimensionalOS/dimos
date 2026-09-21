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

"""Inspect generated DimOS messages with explicit timestamp and duration helpers."""

from dimos_generated.dimos_msgs.msg import LineSegment3D, LineSegments3D, TrajectoryStatus
from dimos_generated.geometry_msgs.msg import Point

from dimos.msgs.time import (
    duration_from_seconds,
    time_from_nanoseconds,
    to_nanoseconds,
    to_seconds,
)


def main() -> None:
    message = LineSegments3D(segments=[LineSegment3D(start=Point(x=1), end=Point(y=2), weight=4)])
    message.header.stamp = time_from_nanoseconds(1_700_000_000_123_456_789)
    message.header.frame_id = "map"
    decoded = LineSegments3D.decode(message.encode())
    print(f"{decoded.msg_name}: frame={decoded.header.frame_id}")
    print(f"Exact source nanoseconds: {to_nanoseconds(decoded.header.stamp)}")
    segment = decoded.segments[0]
    print(f"Segment: start.x={segment.start.x}, end.y={segment.end.y}, weight={segment.weight}")
    status = TrajectoryStatus(
        state=TrajectoryStatus.EXECUTING, time_remaining=duration_from_seconds(1.25)
    )
    remaining = TrajectoryStatus.decode(status.encode()).time_remaining
    print(f"Trajectory remaining: sec={remaining.sec}, nanosec={remaining.nanosec}")
    print(f"Seconds for a controller API: {to_seconds(remaining)}")


if __name__ == "__main__":
    main()
