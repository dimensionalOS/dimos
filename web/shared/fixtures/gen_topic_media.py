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

"""Generate fixed-media golden vectors from actual generated DimOS LCM types."""

import json
from pathlib import Path
import struct
from typing import Any

from dimos_lcm.sensor_msgs import CompressedImage, Image, PointCloud2, PointField
from dimos_lcm.std_msgs import Header, Time

from dimos.web.relay_bridge.topic_media import encode_media

HEADER = Header(seq=9, stamp=Time(sec=123, nsec=456_000_000), frame_id="camera/β")


def image(
    encoding: str, data: bytes, width: int, height: int, step: int, big: bool = False
) -> Image:
    return Image(
        data_length=len(data),
        header=HEADER,
        height=height,
        width=width,
        encoding=encoding,
        is_bigendian=big,
        step=step,
        data=data,
    )


def cloud(big: bool = False) -> PointCloud2:
    fields = [
        PointField(name=name, offset=i * 4, datatype=7 if i < 4 else 6, count=1)
        for i, name in enumerate(["x", "y", "z", "intensity", "rgb"])
    ]
    rows = [
        [(1, 2, 3, 4, 0x112233), (10, -20, 30, 40, 0xFFEEDD)],
        [(float("nan"), 0, 0, 0, 0), (-5, 6, 7, 80, 0x123456)],
    ]
    data = b"".join(
        b"".join(struct.pack((">" if big else "<") + "ffffI", *point) for point in row) + b"PAD!"
        for row in rows
    )
    return PointCloud2(
        fields_length=len(fields),
        data_length=len(data),
        header=HEADER,
        height=2,
        width=2,
        fields=fields,
        is_bigendian=big,
        point_step=20,
        row_step=44,
        data=data,
        is_dense=False,
    )


def generate() -> list[dict[str, Any]]:
    depth16 = struct.pack("<6H", 0, 1, 65535, 1000, 12000, 42)
    depth32 = bytes.fromhex("00000000000000800000803f0000807f000080ff3412c07f")
    jpeg = encode_media(
        "image.jpeg.balanced.v1", image("rgb8", bytes([255, 0, 0]) * 128, 16, 8, 48).lcm_encode()
    ).payload
    cases = [
        (
            "color_compressed",
            "image.jpeg.balanced.v1",
            CompressedImage(
                data_length=len(jpeg),
                header=HEADER,
                format="bgr8; jpeg compressed bgr8",
                data=jpeg,
            ),
            "",
        ),
        ("depth16", "depth.lossless.v1", image("16UC1", depth16, 3, 2, 6), depth16.hex()),
        ("depth32", "depth.lossless.v1", image("32FC1", depth32, 3, 2, 12), depth32.hex()),
        (
            "depth16_big_padding",
            "depth.lossless.v1",
            image(
                "16UC1",
                struct.pack(">3H", 0, 1, 65535)
                + b"--"
                + struct.pack(">3H", 1000, 12000, 42)
                + b"--",
                3,
                2,
                8,
                True,
            ),
            depth16.hex(),
        ),
        (
            "color_rgb",
            "image.jpeg.balanced.v1",
            image("rgb8", bytes([255, 0, 0]) * 128, 16, 8, 48),
            "",
        ),
        (
            "color_bgr",
            "image.jpeg.balanced.v1",
            image("bgr8", bytes([0, 0, 255]) * 128, 16, 8, 48),
            "",
        ),
        ("cloud", "cloud.q16.full.v1", cloud(), ""),
        ("cloud_big", "cloud.q16.full.v1", cloud(True), ""),
        ("cloud_low", "cloud.q16.low.v1", cloud(), ""),
    ]
    vectors = []
    for name, profile, message, expected in cases:
        data = message.lcm_encode()
        result = encode_media(profile, data)
        vectors.append(
            {
                "name": name,
                "profile": profile,
                "lcm": data.hex(),
                "payload": result.payload.hex(),
                "encoding": result.encoding,
                "metadata": result.metadata,
                "expectedDepth": expected,
            }
        )
    return vectors


if __name__ == "__main__":
    Path(__file__).with_name("topic_media.json").write_text(json.dumps(generate(), indent=2) + "\n")
