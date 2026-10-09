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

from pathlib import Path

from dimos_generated.builtin_interfaces.msg import Duration, Time
from dimos_generated.dimos_msgs.msg import LineSegment3D, LineSegments3D, TrajectoryStatus
from dimos_generated.geometry_msgs.msg import Point
from dimos_generated.sensor_msgs.msg import CameraInfo
from dimos_generated.std_msgs.msg import Header
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode

from dimos.msgs.camera_info import camera_info_from_yaml, intrinsic_matrix
from dimos.msgs.time import (
    duration_from_seconds,
    time_from_nanoseconds,
    to_nanoseconds,
    to_seconds,
)


def main() -> None:
    message = LineSegments3D(
        segments=[
            LineSegment3D(start=Point(x=1, y=0.0, z=0.0), end=Point(y=2, x=0.0, z=0.0), weight=4)
        ],
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
    )
    message.header.stamp = time_from_nanoseconds(1_700_000_000_123_456_789)
    message.header.frame_id = "map"
    decoded = cdr_decode(cdr_encode(message), LineSegments3D)
    print(f"{decoded.__msgtype__}: frame={decoded.header.frame_id}")
    print(f"Exact source nanoseconds: {to_nanoseconds(decoded.header.stamp)}")
    segment = decoded.segments[0]
    print(f"Segment: start.x={segment.start.x}, end.y={segment.end.y}, weight={segment.weight}")
    status = TrajectoryStatus(
        state=TrajectoryStatus.EXECUTING,
        time_remaining=duration_from_seconds(1.25),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
        progress=0.0,
        time_elapsed=Duration(sec=0, nanosec=0),
        error="",
    )
    remaining = cdr_decode(cdr_encode(status), TrajectoryStatus).time_remaining
    print(f"Trajectory remaining: sec={remaining.sec}, nanosec={remaining.nanosec}")
    print(f"Seconds for a controller API: {to_seconds(remaining)}")
    calibration = camera_info_from_yaml(
        Path(__file__).resolve().parents[2] / "dimos/robot/unitree/go2/front_camera_720.yaml",
        header=Header(frame_id="camera_optical", stamp=message.header.stamp),
    )
    restored = cdr_decode(cdr_encode(calibration), CameraInfo)
    print(f"Camera calibration: {restored.width}x{restored.height}, {restored.distortion_model}")
    print(f"Camera source nanoseconds: {to_nanoseconds(restored.header.stamp)}")
    print(f"Intrinsic matrix (independent NumPy copy):\n{intrinsic_matrix(restored)}")
    print(f"Distortion coefficients: {list(restored.d)}")


if __name__ == "__main__":
    main()
