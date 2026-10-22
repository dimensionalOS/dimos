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

"""Demonstrate live Rerun bridge rendering and native CDR MCAP viewing without a robot."""

from __future__ import annotations

import argparse
from contextlib import ExitStack
from copy import deepcopy
from functools import partial
from pathlib import Path
import socket
import threading
from typing import Any

from dimos_cdrgen.registry import encode, schema
import imagecodecs
from mcap.writer import Writer
import numpy as np
import rerun as rr
import rerun.blueprint as rrb

from dimos.core.transport import ZenohTransport
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.dimos_msgs.msg import EntityMarker, EntityMarkers
from dimos.messages.geometry_msgs.msg import (
    Point,
    PointStamped,
    Pose,
    PoseStamped,
    PoseWithCovariance,
    Quaternion,
    Transform,
    TransformStamped,
    Twist,
    TwistWithCovariance,
    Vector3,
)
from dimos.messages.nav_msgs.msg import Odometry, Path as MessagePath
from dimos.messages.sensor_msgs.msg import (
    CameraInfo,
    CompressedImage,
    Image,
    PointCloud2,
    PointField,
    RegionOfInterest,
)
from dimos.messages.std_msgs.msg import Header
from dimos.messages.tf2_msgs.msg import TFMessage
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped as LegacyPoseStamped
from dimos.protocol.pubsub.impl.zenohpubsub import Topic
from dimos.protocol.pubsub.patterns import Glob
from dimos.protocol.rpc.zenohrpc import ZenohRPC
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.cdr import cloud_archetype


def scene_messages() -> dict[str, Any]:
    message = PoseStamped(
        header=Header(stamp=Time(sec=1700000000, nanosec=123456789), frame_id="map"),
        pose=Pose(
            position=Point(x=1.5, y=-2.5, z=0.25), orientation=Quaternion(x=0, y=0, z=0.6, w=0.8)
        ),
    )
    pixels = np.array([[[255, 0, 0], [0, 255, 0]], [[0, 0, 255], [255, 255, 255]]], dtype=np.uint8)
    image = Image(
        header=message.header,
        width=2,
        height=2,
        encoding="rgb8",
        is_bigendian=0,
        step=6,
        data=pixels.ravel(),
    )
    calibration = CameraInfo(
        distortion_model="plumb_bob",
        d=np.zeros(5, dtype=np.float64),
        r=np.eye(3, dtype=np.float64).ravel(),
        p=np.zeros(12, dtype=np.float64),
        binning_x=0,
        binning_y=0,
        roi=RegionOfInterest(x_offset=0, y_offset=0, height=0, width=0, do_rectify=False),
        header=message.header,
        width=2,
        height=2,
        k=np.array([2, 0, 0.5, 0, 2, 0.5, 0, 0, 1], dtype=np.float64),
    )
    points = np.array(
        [
            (-0.25, -0.25, 1, 0xFF0000),
            (0.25, -0.25, 1, 0x00FF00),
            (-0.25, 0.25, 1, 0x0000FF),
            (0.25, 0.25, 1, 0xFFFFFF),
        ],
        dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4")],
    )
    cloud = PointCloud2(
        header=message.header,
        height=1,
        width=4,
        point_step=16,
        row_step=64,
        is_bigendian=False,
        is_dense=True,
        fields=[
            PointField(name=name, offset=i * 4, datatype=7 if i < 3 else 6, count=1)
            for i, name in enumerate(points.dtype.names or ())
        ],
        data=np.frombuffer(points.tobytes(), dtype=np.uint8),
    )
    camera_header = Header(stamp=message.header.stamp, frame_id="camera")
    image.header = camera_header
    calibration.header = camera_header
    depth_pixels = np.array([[1, 256], [1000, 2000]], dtype=">u2")
    depth = Image(
        header=camera_header,
        width=2,
        height=2,
        encoding="16UC1",
        is_bigendian=1,
        step=4,
        data=np.frombuffer(depth_pixels.tobytes(), dtype=np.uint8),
    )
    compressed = CompressedImage(
        header=camera_header,
        format="png",
        data=np.frombuffer(imagecodecs.png_encode(pixels), dtype=np.uint8),
    )
    compressed_depth = CompressedImage(
        header=camera_header,
        format="16UC1; jxl",
        data=np.frombuffer(
            imagecodecs.jpegxl_encode(depth_pixels.astype(np.uint16), lossless=True), dtype=np.uint8
        ),
    )
    zero = Vector3(x=0, y=0, z=0)
    identity = Quaternion(x=0, y=0, z=0, w=1)
    tf = TFMessage(
        transforms=[
            TransformStamped(
                header=Header(stamp=message.header.stamp, frame_id="world"),
                child_frame_id="map",
                transform=Transform(translation=zero, rotation=identity),
            ),
            TransformStamped(
                header=message.header,
                child_frame_id="camera",
                transform=Transform(translation=Vector3(x=0, y=0, z=1), rotation=identity),
            ),
        ]
    )
    route = MessagePath(
        header=message.header,
        poses=[
            PoseStamped(
                header=message.header,
                pose=Pose(position=Point(x=x, y=y, z=0), orientation=identity),
            )
            for x, y in [(0, 0), (1, 0), (1, 1)]
        ],
    )
    odometry = Odometry(
        header=message.header,
        child_frame_id="base",
        pose=PoseWithCovariance(pose=message.pose, covariance=np.zeros(36, dtype=np.float64)),
        twist=TwistWithCovariance(
            twist=Twist(linear=zero, angular=zero), covariance=np.zeros(36, dtype=np.float64)
        ),
    )
    entities = EntityMarkers(
        header=message.header,
        markers=[
            EntityMarker(
                entity_id="E1",
                label="demo person",
                entity_type="person",
                position=Point(x=2, y=1, z=0.3),
            ),
            EntityMarker(
                entity_id="E2",
                label="demo object",
                entity_type="object",
                position=Point(x=2, y=2, z=0.3),
            ),
        ],
    )
    # Ordered publication makes both camera/image arrival orders reproducible.
    late_image = deepcopy(image)
    late_image.header = Header(stamp=message.header.stamp, frame_id="late_camera")
    late_calibration = deepcopy(calibration)
    late_calibration.header = late_image.header
    tf.transforms.append(
        TransformStamped(
            header=message.header,
            child_frame_id="late_camera",
            transform=Transform(translation=Vector3(x=0, y=1, z=1), rotation=identity),
        )
    )
    boxes = deepcopy(cloud)
    box_points = points.copy()
    box_points["x"] += 2
    boxes.data = np.frombuffer(box_points.tobytes(), dtype=np.uint8)
    heights = np.empty(4, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4")])
    heights["x"], heights["y"], heights["z"] = points["x"] - 2, points["y"], [0.5, 1, 1.5, 2]
    height_cloud = PointCloud2(
        header=message.header,
        height=1,
        width=4,
        point_step=12,
        row_step=48,
        is_bigendian=False,
        is_dense=True,
        fields=[
            PointField(name=name, offset=i * 4, datatype=7, count=1)
            for i, name in enumerate(("x", "y", "z"))
        ],
        data=np.frombuffer(heights.tobytes(), dtype=np.uint8),
    )
    return {
        "tf": tf,
        "pose": message,
        "point": PointStamped(header=message.header, point=Point(x=2, y=0, z=0.5)),
        "odometry": odometry,
        "path": route,
        "entities": entities,
        "camera_info": calibration,
        "image": image,
        "depth": depth,
        "compressed": compressed,
        "compressed_depth": compressed_depth,
        "late_image": late_image,
        "late_camera_info": late_calibration,
        "cloud": cloud,
        "cloud_boxes": boxes,
        "cloud_height": height_cloud,
    }


def viewer_layout() -> rrb.Blueprint:
    return rrb.Blueprint(
        rrb.Tabs(
            rrb.Spatial3DView(
                origin="world",
                name="Geometry, TF and colored cloud",
                contents=[
                    "world/pose/**",
                    "world/point/**",
                    "world/odometry/**",
                    "world/path/**",
                    "world/entities/**",
                    "world/cloud/**",
                    "world/cloud_boxes/**",
                    "world/cloud_height/**",
                    "world/legacy_pose/**",
                    "world/tf/**",
                    "tf#/**",
                ],
            ),
            rrb.Grid(
                rrb.Spatial2DView(origin="world/image", name="Generated RGB pixels"),
                rrb.Spatial2DView(origin="world/depth", name="Big endian depth"),
                rrb.Spatial2DView(origin="world/compressed", name="PNG pixels"),
                rrb.Spatial2DView(origin="world/compressed_depth", name="JXL depth"),
                rrb.Spatial2DView(origin="world/late_image", name="Calibration arriving last"),
                grid_columns=3,
            ),
            rrb.TextDocumentView(origin="world/pose/message", name="Exact message values"),
        )
    )


class DemoBridge(RerunBridgeModule):
    """A real bridge with a receipt signal so the demo can flush before exiting."""

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.received = threading.Event()
        self.errors: list[Exception] = []

    def _on_cdr_message(self, msg: Any, *, topic: str) -> None:
        try:
            super()._on_cdr_message(msg, topic=topic)
        except Exception as error:
            self.errors.append(error)
        finally:
            self.received.set()


class LoopbackRPC(ZenohRPC):
    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            mode="peer",
            listen=["tcp/127.0.0.1:0"],
            scouting=False,
            multicast=False,
            gossip=False,
            **kwargs,
        )


def run(output: Path) -> None:
    # Reserve destinations before starting sockets. Existing recordings are never overwritten.
    mcap = output.with_suffix(".mcap")
    with ExitStack() as cleanup:
        for destination in (output, mcap):
            cleanup.enter_context(destination.open("xb"))
        config: dict[str, Any] = dict(
            mode="peer", listen=["tcp/127.0.0.1:0"], scouting=False, multicast=False, gossip=False
        )
        messages = scene_messages()
        # Select a free local Rerun port instead of attaching to another user's viewer.
        with socket.socket() as reserve:
            reserve.bind(("127.0.0.1", 0))
            port = reserve.getsockname()[1]
        bridge = DemoBridge(
            pubsubs=[],
            cdr_transports={
                topic: ZenohTransport(Topic(topic, type(value)), **config)
                for topic, value in messages.items()
            },
            rpc_transport=LoopbackRPC,
            rerun_open="none",
            rerun_web=False,
            connect_url=f"rerun+http://127.0.0.1:{port}/proxy",
            tf_axes=0.25,
            blueprint=viewer_layout,
            visual_override={
                Glob("world/cloud_boxes"): partial(cloud_archetype, mode="boxes"),
                Glob("world/cloud_height"): partial(
                    cloud_archetype, mode="spheres", height_colors=True
                ),
            },
        )
        cleanup.callback(bridge.stop)
        bridge.start()
        rr.save(output)
        cleanup.callback(rr.disconnect)
        rr.send_blueprint(viewer_layout())
        rr.log("world", rr.Transform3D(child_frame="tf#/world"), static=True)
        with mcap.open("wb") as stream:
            writer = Writer(stream)
            writer.start(profile="ros2")
            try:
                for index, (topic, value) in enumerate(messages.items()):
                    with ExitStack() as sender:
                        transport = ZenohTransport(Topic(topic, type(value)), **config)
                        sender.callback(transport.stop)
                        bridge.received.clear()
                        transport.broadcast(None, value)
                        if not bridge.received.wait(timeout=10):
                            raise TimeoutError(f"Bridge did not receive {topic}")
                        if bridge.errors:
                            raise bridge.errors[0]
                    schema_id = writer.register_schema(
                        value.__msgtype__, "ros2msg", schema(value.__msgtype__).encode()
                    )
                    channel = writer.register_channel(topic, "cdr", schema_id)
                    timestamp = 1_700_000_000_123_456_789 + index * 1_000_000
                    writer.add_message(
                        channel, log_time=timestamp, publish_time=timestamp, data=encode(value)
                    )
                    print(f"Bridge rendered {topic}: {value.__msgtype__}")
            finally:
                writer.finish()
        # Existing messages still take the bridge's ordinary rendering path.
        bridge._on_message(LegacyPoseStamped(position=(0, -1, 0), frame_id="map"), "/legacy_pose")
        recording = rr.get_global_data_recording()
        if recording is not None:
            recording.flush()
    print(f"Live bridge: rerun {output}")
    print(f"Native MCAP view: rerun {mcap}")
    print(f"Reusable recording: {mcap}")
    print("Source timestamp: 1700000000.123456789; pose [1.5, -2.5, 0.25].")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "output",
        type=Path,
        help="new live .rrd filename; also creates a CDR .mcap for native Rerun viewing",
    )
    args = parser.parse_args()
    run(args.output)


if __name__ == "__main__":
    main()
