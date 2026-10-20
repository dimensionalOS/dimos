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

"""Verify an installed message wheel from a clean environment outside the checkout."""

from importlib.metadata import distribution, entry_points
from pathlib import Path
import sys

from dimos_cdrgen.registry import decode, encode
import numpy as np

import dimos.messages
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.dimos_msgs.msg import LineSegment3D, LineSegments3D
from dimos.messages.geometry_msgs.msg import Point
from dimos.messages.sensor_msgs.msg import Image
from dimos.messages.std_msgs.msg import Header
from dimos.messages_schemas.provider import message_types, package_root


def main() -> None:
    assert sys.prefix != sys.base_prefix, "Run in the clean verification environment"
    assert dimos.messages.__file__ is not None
    installed = Path(dimos.messages.__file__).resolve()
    assert installed.is_relative_to(Path(sys.prefix).resolve()), installed
    owner = distribution("dimos")
    assert "dimos/messages/std_msgs/msg/__init__.py" in {str(p) for p in owner.files}
    assert len([ep for ep in entry_points(group="dimos.messages") if ep.name == "dimos"]) == 1
    print(f"Installed package: {installed}")
    print(f"Catalog: {len(message_types())} types")

    image = Image(
        header=Header(stamp=Time(sec=17, nanosec=23), frame_id="camera"),
        height=2,
        width=3,
        encoding="rgb8",
        is_bigendian=0,
        step=9,
        data=np.arange(18, dtype=np.uint8),
    )
    restored_image = decode(encode(image), Image)
    assert type(restored_image) is Image and type(restored_image.header) is Header
    assert restored_image.header == Header(Time(17, 23), "camera")
    assert (restored_image.height, restored_image.width, restored_image.step) == (2, 3, 9)
    assert restored_image.encoding == "rgb8" and restored_image.is_bigendian == 0
    np.testing.assert_array_equal(restored_image.data, np.arange(18, dtype=np.uint8))
    print("PASS: sensor_msgs/msg/Image pixels, encoding and source stamp")

    segments = LineSegments3D(
        header=Header(stamp=Time(sec=17, nanosec=23), frame_id="map"),
        segments=[LineSegment3D(start=Point(1.0, 2.0, 3.0), end=Point(4.0, 5.0, 6.0), weight=0.75)],
    )
    restored_segments = decode(encode(segments), LineSegments3D)
    assert restored_segments == segments
    assert type(restored_segments.header) is Header
    assert type(restored_segments.segments[0].start) is Point
    print("PASS: dimos_msgs/msg/LineSegments3D geometry, weight and nested type identity")

    resources = package_root()
    for relative in (
        "schemas/sensor_msgs/msg/Image.msg",
        "schemas/dimos_msgs/msg/LineSegments3D.msg",
        "cpp/CMakeLists.txt",
        "cpp/sensor_msgs/msg/Image.msg",
        "rust/Cargo.toml",
        "rust/build.rs",
        "rust/interfaces/dimos_msgs/msg/LineSegments3D.msg",
    ):
        assert (resources / relative).is_file(), relative
    assert not any(
        path.suffix in {".so", ".a", ".dylib", ".dll", ".rlib", ".pyd"}
        for path in resources.rglob("*")
    )
    print("PASS: packaged definitions and C++/Rust build inputs; no compiled message libraries")


if __name__ == "__main__":
    main()
