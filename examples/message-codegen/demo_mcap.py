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

"""Record synthetic images, poses, transforms, and custom fields for both viewers."""

import argparse
from io import BytesIO
import math
from pathlib import Path

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    Quaternion,
    Transform,
    TransformStamped,
    Vector3,
)
from dimos_generated.sensor_msgs.msg import CompressedImage, Image
from dimos_generated.std_msgs.msg import Header
from dimos_generated.tf2_msgs.msg import TFMessage
from dimos_message_build.registry import encode, schema
import numpy as np
from PIL import Image as PillowImage
from story_messages.story_msgs.msg import DeviceReading

from dimos.protocol.cdr_mcap import CdrMcapWriter

START_NS = 1_700_000_000_000_000_000


def write_demo(path: Path, frames: int = 30) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with CdrMcapWriter(path) as writer:
        for index in range(frames):
            stamp_ns = START_NS + index * 100_000_000
            rows, columns = np.indices((192, 256), dtype=np.uint16)
            pixels = np.stack(
                (
                    (columns + index * 8) % 256,
                    (rows + index * 5) % 256,
                    np.full_like(rows, (index * 17) % 256),
                ),
                axis=-1,
            ).astype(np.uint8)
            image = Image(
                height=192,
                width=256,
                step=256 * 3,
                encoding="rgb8",
                data=np.asarray(pixels.reshape(-1), dtype=np.uint8),
                header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
                is_bigendian=0,
            )
            pose = PoseStamped(
                header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
                pose=Pose(
                    position=Point(x=0.0, y=0.0, z=0.0),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
            )
            pose.pose.position.x = math.cos(index / 5)
            pose.pose.position.y = math.sin(index / 5)
            pose.pose.orientation.w = 1.0
            pose.header.frame_id = "map"
            image.header.frame_id = "camera"
            for value in (image, pose):
                value.header.stamp.sec, value.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
            compressed_bytes = BytesIO()
            PillowImage.fromarray(pixels).save(compressed_bytes, format="PNG")
            compressed = CompressedImage(
                header=image.header,
                format="png",
                data=np.frombuffer(compressed_bytes.getvalue(), dtype=np.uint8),
            )
            telemetry = DeviceReading(
                header=pose.header,
                sequence=index,
                label="synthetic",
                value=20.0 + index / 10,
            )
            transform = TransformStamped(
                header=pose.header,
                child_frame_id="camera",
                transform=Transform(
                    translation=Vector3(x=0.0, y=0.0, z=0.0),
                    rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
            )
            transform.transform.translation.x = pose.pose.position.x
            transform.transform.translation.y = pose.pose.position.y
            transform.transform.rotation.w = 1.0
            transforms = TFMessage(transforms=[transform])
            for topic, value in (
                ("/camera/image", image),
                ("/camera/compressed", compressed),
                ("/robot/pose", pose),
                ("/telemetry", telemetry),
                ("/tf", transforms),
            ):
                writer.write(
                    topic,
                    encode(value),
                    schema_name=value.__msgtype__,
                    schema=schema(value.__msgtype__),
                    log_time_ns=stamp_ns + 1_000_000,
                    publish_time_ns=stamp_ns,
                    sequence=index,
                )
    print(f"Wrote {frames * 5} messages to {path} ({path.stat().st_size:,} bytes)")
    print("Five CDR channels; ROS2 schemas and all dependencies are embedded.")
    print(
        "Image: 256x192 RGB gradient; pose: circular path; telemetry: sequence 0..29 and value 20.0..22.9."
    )
    print("Open this same file in Foxglove and Rerun without installing story_messages.")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output", type=Path, default=Path("build/message-codegen/viewers/demo.mcap")
    )
    args = parser.parse_args()
    write_demo(args.output)


if __name__ == "__main__":
    main()
