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

"""Regenerate browser fixtures with: python -m dimos.web.relay_bridge.gen_cdr_fixtures."""

import base64
import json
from typing import Any

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.dimos_msgs.msg import LineSegment3D, LineSegments3D
from dimos_generated.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from dimos_generated.sensor_msgs.msg import Image, Imu, JointState
from dimos_generated.std_msgs.msg import Empty, Header, Int64, String

from dimos.message_codegen.definitions import Definitions
from dimos.web.cdr_codec import CDR_V1_SUFFIX, export_schema
from dimos.web.relay_bridge.locate import find_web_dir


def build_messages() -> list[tuple[str, Any]]:
    header = Header(stamp=Time(sec=1757400000, nanosec=500000123), frame_id="map")
    return [
        (
            "pose_stamped",
            PoseStamped(
                header=header,
                pose=Pose(
                    position=Point(x=1.5, y=-2.5, z=0.25), orientation=Quaternion(z=0.6, w=0.8)
                ),
            ),
        ),
        (
            "custom_segments",
            LineSegments3D(
                header=header, segments=[LineSegment3D(start=Point(x=1), end=Point(y=2), weight=4)]
            ),
        ),
        (
            "image",
            Image(
                header=header,
                width=2,
                height=1,
                encoding="rgb8",
                step=6,
                data=[0, 1, 2, 253, 254, 255],
            ),
        ),
        ("imu", Imu(header=header, orientation_covariance=range(9))),
        ("joints", JointState(header=header, name=["joint_a", "joint_b"], position=[1.5, -2.5])),
        ("int64", Int64(data=-9007199254740993)),
        ("empty", Empty()),
        ("string", String(data="robot 🦾 café")),
    ]


def build_vectors() -> list[dict[str, Any]]:
    definitions = {message.name: message for message in Definitions([]).resolve()}

    def walk(msg: Any) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for field in definitions[msg.msg_name].fields:
            value = getattr(msg, field.name)
            convert = (
                walk
                if field.type.nested
                else (str if field.type.name in ("int64", "uint64") else lambda x: x)
            )
            result[field.name] = (
                [convert(item) for item in value] if field.type.is_array else convert(value)
            )
        return result

    return [
        {
            "name": name,
            "encoding": msg.msg_name + CDR_V1_SUFFIX,
            "schema": export_schema(type(msg)),
            "value": walk(msg),
            "payload_b64": base64.b64encode(msg.encode()).decode(),
            "big_endian_b64": base64.b64encode(msg.encode(little_endian=False)).decode(),
        }
        for name, msg in build_messages()
    ]


def main() -> None:
    path = find_web_dir() / "shared" / "fixtures" / "cdr_frames.json"
    path.write_text(json.dumps({"vectors": build_vectors()}, indent=2, ensure_ascii=False) + "\n")
    print(path)


if __name__ == "__main__":
    main()
