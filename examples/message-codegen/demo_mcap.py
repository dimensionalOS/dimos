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

from dimos_generated.demo_msgs.msg import Telemetry
from dimos_generated.geometry_msgs.msg import PoseStamped, TransformStamped
from dimos_generated.sensor_msgs.msg import CompressedImage, Image
from dimos_generated.tf2_msgs.msg import TFMessage
import numpy as np
from PIL import Image as PillowImage

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
                height=192, width=256, step=256 * 3, encoding="rgb8", data=pixels.reshape(-1)
            )
            pose = PoseStamped()
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
                header=image.header, format="png", data=compressed_bytes.getvalue()
            )
            telemetry = Telemetry(
                header=pose.header,
                sequence=index,
                label="synthetic",
                position=pose.pose.position,
                hops=[1, 2, 3],
            )
            telemetry.reading.temperature = 20.0 + index / 10
            transform = TransformStamped(header=pose.header, child_frame_id="camera")
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
                    value.encode(),
                    schema_name=value.msg_name,
                    schema=value.schema,
                    log_time_ns=stamp_ns + 1_000_000,
                    publish_time_ns=stamp_ns,
                    sequence=index,
                )
    print(f"Wrote {frames * 5} messages to {path} ({path.stat().st_size:,} bytes)")
    print("Five CDR channels; ROS2 schemas and all dependencies are embedded.")
    print(
        "Image: 256x192 RGB gradient; pose: circular path; telemetry: sequence 0..29 and temperature 20.0..22.9."
    )
    print("Open this same file in Foxglove and Rerun without installing demo_msgs.")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output", type=Path, default=Path("build/message-codegen/viewers/demo.mcap")
    )
    args = parser.parse_args()
    write_demo(args.output)


if __name__ == "__main__":
    main()
