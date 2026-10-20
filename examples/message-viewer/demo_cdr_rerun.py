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

"""Record generated pose, image and RGBD cloud over loopback CDR into Rerun."""

from __future__ import annotations

import argparse
from contextlib import ExitStack
from functools import partial
from pathlib import Path
import threading
from typing import Any

import numpy as np
import rerun as rr
import rerun.blueprint as rrb

from dimos.core.cdr_transport import CdrTransport
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from dimos.messages.sensor_msgs.msg import CameraInfo
from dimos.messages.std_msgs.msg import Header
from dimos.msgs.camera_info import camera_info_from_intrinsics
from dimos.msgs.image import image_from_array
from dimos.msgs.pointcloud import pointcloud_from_rgbd
from dimos.visualization.rerun.cdr import camera_pinhole, generated_archetypes


def run(output: Path) -> None:
    # Reserve a new destination exclusively; never replace a user's recording.
    with output.open("xb"):
        pass
    message = PoseStamped(
        header=Header(stamp=Time(sec=1700000000, nanosec=123456789), frame_id="map"),
        pose=Pose(
            position=Point(x=1.5, y=-2.5, z=0.25), orientation=Quaternion(x=0, y=0, z=0.6, w=0.8)
        ),
    )
    image = image_from_array(
        np.array([[[255, 0, 0], [0, 255, 0]], [[0, 0, 255], [255, 255, 255]]], dtype=np.uint8),
        encoding="rgb8",
        header=message.header,
    )
    depth = image_from_array(
        np.full((2, 2), 1000, dtype=np.uint16), encoding="16UC1", header=message.header
    )
    calibration = camera_info_from_intrinsics(2, 2, 0.5, 0.5, 2, 2, header=message.header)
    cloud = pointcloud_from_rgbd(image, depth, calibration, depth_scale=0.001)
    messages: dict[str, Any] = {
        "pose": message,
        "image": image,
        "cloud": cloud,
        "camera_info": calibration,
    }
    config: dict[str, Any] = dict(
        mode="peer", listen=["tcp/127.0.0.1:0"], scouting=False, multicast=False, gossip=False
    )
    recording = rr.RecordingStream("dimos-cdr-pose")
    received = threading.Event()
    received_topics: set[str] = set()
    lock = threading.Lock()
    errors: list[Exception] = []

    def receive(value: Any, *, topic: str) -> None:
        try:
            stamp = value.header.stamp
            recording.set_time(
                "source", timestamp=np.datetime64(stamp.sec * 1_000_000_000 + stamp.nanosec, "ns")
            )
            if isinstance(value, CameraInfo):
                # A spatial image needs its calibration, not just pixel bytes.
                recording.log("world/image", camera_pinhole(value), strict=True)
            else:
                entries = generated_archetypes(value, "world/" + topic)
                assert entries is not None
                for entry in entries:
                    recording.log(entry[0], entry[1], strict=True)
        except Exception as error:
            errors.append(error)
        finally:
            with lock:
                received_topics.add(topic)
                if errors or received_topics == set(messages):
                    received.set()

    with ExitStack() as cleanup:
        cleanup.callback(recording.disconnect)
        recording.save(
            output,
            default_blueprint=rrb.Blueprint(
                rrb.Horizontal(
                    rrb.Spatial3DView(origin="world", name="Pose axes and RGBD cloud"),
                    rrb.Vertical(
                        rrb.TextDocumentView(
                            origin="world/pose/message", name="Exact message values"
                        ),
                        rrb.Spatial2DView(origin="world/image", name="Generated RGB pixels"),
                    ),
                )
            ),
        )
        # This demo explicitly identifies map with its viewer origin. Runtime
        # bridges use the producer's named TF frame; no coordinates are rewritten.
        recording.log("world", rr.Transform3D(child_frame="tf#/map"), static=True)
        publishers: dict[str, CdrTransport[Any]] = {}
        for topic, value in messages.items():
            subscriber = CdrTransport(topic, type(value), backend="zenoh", **config)
            cleanup.callback(subscriber.stop)
            cleanup.callback(subscriber.subscribe(partial(receive, topic=topic)))
            publisher = CdrTransport(topic, type(value), backend="zenoh", **config)
            cleanup.callback(publisher.stop)
            publishers[topic] = publisher
        for topic, value in messages.items():
            publishers[topic].broadcast(None, value)
        if not received.wait(timeout=10):
            raise TimeoutError(f"Missing loopback CDR topics: {set(messages) - received_topics}")
        if errors:
            raise errors[0]
        recording.flush(timeout_sec=10)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path, help="new .rrd file (must not exist)")
    args = parser.parse_args()
    run(args.output)
    print(f"Recorded actual loopback CDR; open with: rerun {args.output}")


if __name__ == "__main__":
    main()
