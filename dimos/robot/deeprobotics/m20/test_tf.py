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

import rerun as rr

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.deeprobotics.m20.blueprints.basic import (
    _dynamic_tf_for_rerun,
    _m20_static_scene,
)
from dimos.robot.deeprobotics.m20.tf import (
    camera_mount_transforms,
    front_camera_info,
    rear_camera_info,
)


def test_camera_mount_transforms_are_fixed_relative_to_base() -> None:
    transforms = camera_mount_transforms(ts=123.0)

    assert [
        (transform.frame_id, transform.child_frame_id, transform.ts) for transform in transforms
    ] == [
        ("base_link", "camera_link", 123.0),
        ("camera_link", "camera_optical", 123.0),
        ("base_link", "rear_camera_link", 123.0),
        ("rear_camera_link", "rear_camera_optical", 123.0),
    ]
    assert transforms[0].translation.x == 0.3
    assert transforms[2].translation.x == -0.3

    base_to_rear_optical = transforms[2].rotation * transforms[3].rotation
    rear_optical_z_in_base = base_to_rear_optical.rotate_vector(Vector3(0.0, 0.0, 1.0))
    assert rear_optical_z_in_base == Vector3(-1.0, 0.0, 0.0)


def test_rerun_dynamic_tf_excludes_fixed_camera_mount() -> None:
    msg = TFMessage(
        Transform(frame_id="map", child_frame_id="base_link", ts=123.0),
        *camera_mount_transforms(ts=123.0),
    )

    rerun_data = _dynamic_tf_for_rerun(msg)

    assert [path for path, _ in rerun_data] == ["world/tf/base_link"]


def test_camera_info_matches_m20_image_resolution() -> None:
    for camera_info in (front_camera_info(), rear_camera_info()):
        assert (camera_info.width, camera_info.height) == (800, 600)
        assert (camera_info.K[2], camera_info.K[5]) == (400.0, 300.0)


def test_rerun_static_scene_contains_camera_rig_and_pinhole() -> None:
    scene = _m20_static_scene(rr)

    assert [path for path, _ in scene] == [
        "world/tf/base_link",
        "world/tf/camera_link",
        "world/tf/camera_optical",
        "world/tf/rear_camera_link",
        "world/tf/rear_camera_optical",
        "world/tf/rear_camera_optical",
        "/world/color_image",
        "/world/color_image_rear",
    ]
