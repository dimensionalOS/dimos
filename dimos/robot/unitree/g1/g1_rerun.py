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

"""G1-specific Rerun visual helpers (robot dimensions, TF overrides)."""

from __future__ import annotations

from typing import Any


def g1_static_robot(rr: Any) -> list[Any]:
    """Static G1 humanoid wireframe box attached to the sensor TF frame.

    Half-sizes are ~50x40x120 cm (the G1 humanoid), and the box is
    centered 0.6m below the sensor (lidar mounted at head height).
    """
    return [
        rr.Boxes3D(
            half_sizes=[0.25, 0.20, 0.6],
            centers=[[0, 0, -0.6]],
            colors=[(0, 255, 127)],
            fill_mode="MajorWireframe",
        ),
        rr.Transform3D(parent_frame="tf#/sensor"),
    ]


def g1_odometry_tf_override(odom: Any) -> Any:
    """Publish odometry as a TF frame so sensor_scan/path/robot can reference it.

    The z is zeroed because point clouds already have the full init_pose
    transform applied (ground at z≈0). Using the raw odom.z (= mount height)
    would double-count the vertical offset.
    """
    import rerun as rr

    tf = rr.Transform3D(
        translation=[odom.x, odom.y, 0.0],
        rotation=rr.Quaternion(
            xyzw=[
                odom.orientation.x,
                odom.orientation.y,
                odom.orientation.z,
                odom.orientation.w,
            ]
        ),
        parent_frame="tf#/map",
        child_frame="tf#/sensor",
    )
    return [
        ("tf#/sensor", tf),
    ]
