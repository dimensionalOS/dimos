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

"""Rerun presentation helpers kept separate from generated wire types."""

from dimos_generated.sensor_msgs.msg import CameraInfo, CompressedImage, Image
from dimos_generated.tf2_msgs.msg import TFMessage
from dimos_generated.vision_msgs.msg import Detection3DArray
import matplotlib
import numpy as np
import rerun as rr

from dimos.msgs.image import image_to_jpeg, image_view


def tf_archetypes(message: TFMessage) -> list[tuple[str, rr.Transform3D]]:
    """Render named TF edges without embedding viewer behavior in messages."""
    result = []
    for edge in message.transforms:
        t, q = edge.transform.translation, edge.transform.rotation
        result.append(
            (
                f"tf_links/{rr.escape_entity_path_part(edge.child_frame_id)}",
                rr.Transform3D(
                    translation=[t.x, t.y, t.z],
                    quaternion=rr.Quaternion(xyzw=[q.x, q.y, q.z, q.w]),
                    parent_frame="tf#/" + edge.header.frame_id,
                    child_frame="tf#/" + edge.child_frame_id,
                ),
            )
        )
    return result


def register_colormap_annotation(name: str = "turbo") -> None:
    """Register 256 class colors for clouds carrying colormap indices."""
    colors = (matplotlib.colormaps[name](np.linspace(0, 1, 256))[:, :3] * 255).astype(np.uint8)
    rr.log(
        "/",
        rr.AnnotationContext(
            [
                rr.datatypes.ClassDescription(
                    info=rr.datatypes.AnnotationInfo(id=i, color=color.tolist())
                )
                for i, color in enumerate(colors)
            ]
        ),
        static=True,
    )


def camera_pinhole(info: CameraInfo) -> rr.Pinhole:
    """Calibration attached to the camera's optical frame."""
    return rr.Pinhole(
        focal_length=[info.k[0], info.k[4]],
        principal_point=[info.k[2], info.k[5]],
        width=info.width,
        height=info.height,
        image_plane_distance=1.0,
        parent_frame=f"tf#/{info.header.frame_id}" if info.header.frame_id else None,
    )


def image_archetype(message: Image | CompressedImage) -> rr.Image | rr.DepthImage | rr.EncodedImage:
    """Render ROS image encodings without adding methods to generated values."""
    if isinstance(message, CompressedImage):
        format_name = message.format.lower()
        if "jpeg" in format_name or "jpg" in format_name:
            media_type = "image/jpeg"
        elif "png" in format_name:
            media_type = "image/png"
        else:
            raise ValueError(f"unsupported compressed image format {message.format!r}")
        return rr.EncodedImage(contents=bytes(message.data), media_type=media_type)
    if message.encoding in ("16UC1", "32FC1"):
        return rr.DepthImage(
            image_view(message), meter=1000.0 if message.encoding == "16UC1" else 1.0
        )
    if message.encoding == "mono16":
        return rr.Image(image_view(message), color_model="L")
    return rr.EncodedImage(contents=image_to_jpeg(message), media_type="image/jpeg")


def detection_boxes(message: Detection3DArray) -> rr.Boxes3D:
    """Render detection geometry and labels from generated values."""
    centers = []
    half_sizes = []
    rotations = []
    labels = []
    for detection in message.detections:
        box = detection.bbox
        p, q, size = box.center.position, box.center.orientation, box.size
        centers.append((p.x, p.y, p.z))
        half_sizes.append((size.x / 2, size.y / 2, size.z / 2))
        rotations.append((q.x, q.y, q.z, q.w))
        identifier = detection.id.strip()
        label = next(
            (
                result.hypothesis.class_id.strip()
                for result in detection.results
                if result.hypothesis.class_id.strip()
            ),
            "",
        )
        labels.append(
            f"{label} id={identifier}"
            if label and identifier
            else label or (f"id={identifier}" if identifier else "")
        )
    return rr.Boxes3D(centers=centers, half_sizes=half_sizes, quaternions=rotations, labels=labels)
