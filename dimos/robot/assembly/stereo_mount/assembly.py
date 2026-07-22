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

"""Static tf for the stereo_mount rig (ZED + Mid-360), driven by its URDF.

``stereo_mount.urdf`` is the single source of truth for the mount geometry —
edit the joint origins there and both tf and any URDF consumer stay in sync.
:class:`StereoMountStaticTf` parses the URDF's fixed-joint tree at start and
republishes it onto tf on a fixed interval (see
:class:`~dimos.protocol.tf.static_tf_publisher.StaticTfPublisher` for why a
one-shot latched publish isn't enough).
"""

from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.protocol.tf.static_tf_publisher import StaticTfPublisher

STEREO_MOUNT_URDF = Path(__file__).parent / "stereo_mount.urdf"


def _parse_triple(value: str | None) -> tuple[float, float, float]:
    if not value:
        return (0.0, 0.0, 0.0)
    x, y, z = (float(part) for part in value.split())
    return (x, y, z)


def urdf_fixed_joint_transforms(urdf_path: Path | str = STEREO_MOUNT_URDF) -> list[Transform]:
    """One ``parent -> child`` Transform per fixed joint of a URDF.

    Only the joint tree is read (link geometry is ignored); URDF fixed-axis
    rpy matches :meth:`Quaternion.from_euler` directly.
    """
    root = ET.parse(urdf_path).getroot()
    transforms: list[Transform] = []
    for joint in root.findall("joint"):
        if joint.get("type") != "fixed":
            continue
        parent_elem = joint.find("parent")
        child_elem = joint.find("child")
        parent = parent_elem.get("link") if parent_elem is not None else None
        child = child_elem.get("link") if child_elem is not None else None
        if not parent or not child:
            continue
        origin = joint.find("origin")
        xyz = _parse_triple(origin.get("xyz") if origin is not None else None)
        rpy = _parse_triple(origin.get("rpy") if origin is not None else None)
        transforms.append(
            Transform(
                translation=Vector3(*xyz),
                rotation=Quaternion.from_euler(Vector3(*rpy)),
                frame_id=parent,
                child_frame_id=child,
            )
        )
    return transforms


class StereoMountStaticTf(StaticTfPublisher):
    """Publishes the stereo_mount URDF joint tree onto tf on a fixed interval."""

    def transforms(self) -> list[Transform]:
        return urdf_fixed_joint_transforms(STEREO_MOUNT_URDF)
