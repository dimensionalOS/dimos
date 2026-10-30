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

"""Convert an upstream camera observation to DimOS messages."""

from typing import Any

import numpy as np

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.sim2.sensors.camera.spec import Camera
from dimos.sim2.sensors.observations import ObservationFrame


def camera_messages(sensor: Camera, frame: ObservationFrame) -> dict[str, Any]:
    frame_id = f"{frame.robot_id or 'world'}/{sensor.name}_optical"
    values = frame.values
    changed = values["changed"]
    timestamps = values["timestamps"]
    messages: dict[str, Any] = {}
    transforms: dict[float, Transform] = {}
    color, depth = None, None
    if "image" in values:
        pixels = np.asarray(values["image"])
        if pixels.dtype != np.uint8:
            pixels = np.clip(pixels, 0, 255).astype(np.uint8)
        color = Image(
            data=pixels, format=ImageFormat.RGB, frame_id=frame_id, ts=timestamps["image"]
        )
    if "depth" in values:
        depth = Image(
            data=np.asarray(values["depth"], dtype=np.float32),
            format=ImageFormat.DEPTH,
            frame_id=frame_id,
            ts=timestamps["depth"],
        )
    for name in changed:
        metadata = values["depth_metadata" if name == "depth" else "metadata"]
        focal = float(metadata[8])
        ts = timestamps[name]
        info = CameraInfo.from_intrinsics(
            focal,
            focal,
            (sensor.width - 1) / 2,
            (sensor.height - 1) / 2,
            sensor.width,
            sensor.height,
            frame_id=frame_id,
        )
        info.ts = ts
        transforms[ts] = Transform(
            translation=Vector3(*metadata[1:4]),
            rotation=Quaternion(*metadata[4:8]),
            frame_id="world",
            child_frame_id=frame_id,
            ts=ts,
        )
        if name == "image":
            messages.update(color_image=color, camera_info=info)
        else:
            messages.update(depth_image=depth, depth_camera_info=info)
    # Upstream observables may have independently configured rates/delays.
    # Never pretend held depth and a newer color image are one RGB-D capture.
    if sensor.pointcloud and color is not None and depth is not None and color.ts == depth.ts:
        cloud = PointCloud2.from_rgbd(color, depth, info, depth_scale=1.0)
        messages["pointcloud"] = cloud.voxel_downsample(0.005)
    messages["tf"] = TFMessage(*(transforms[ts] for ts in sorted(transforms)))
    return messages
