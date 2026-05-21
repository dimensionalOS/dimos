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

from pathlib import Path

from dimos.core.coordination.blueprints import Blueprint
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.hardware.sensors.camera.webcam import Webcam
from dimos.perception.fiducial.blueprints.desk_marker_tf import (
    DESK_CAMERA_FRAME_ID,
    DESK_MARKER_ARUCO_DICTIONARY,
    DESK_MARKER_LENGTH_M,
    DESK_MARKER_NAMESPACE_PREFIX,
    DeskStaticTfModule,
    create_desk_webcam,
    desk_marker_tf,
)
from dimos.perception.fiducial.marker_tf_module import MarkerTfModule


def test_desk_marker_tf_blueprint_declares_static_tf_module() -> None:
    assert isinstance(desk_marker_tf, Blueprint)
    assert desk_marker_tf.blueprints[0].module is DeskStaticTfModule
    assert desk_marker_tf.blueprints[1].module is CameraModule
    assert desk_marker_tf.blueprints[1].kwargs["hardware"] is create_desk_webcam
    assert desk_marker_tf.blueprints[1].kwargs["transform"] is None
    assert desk_marker_tf.blueprints[2].module is MarkerTfModule
    assert desk_marker_tf.blueprints[2].kwargs["marker_length_m"] == DESK_MARKER_LENGTH_M
    assert desk_marker_tf.blueprints[2].kwargs["aruco_dictionary"] == DESK_MARKER_ARUCO_DICTIONARY
    assert (
        desk_marker_tf.blueprints[2].kwargs["marker_namespace_prefix"]
        == DESK_MARKER_NAMESPACE_PREFIX
    )


def test_create_desk_webcam_loads_camera_info_yaml(tmp_path: Path) -> None:
    camera_info_yaml = tmp_path / "camera_info.yaml"
    camera_info_yaml.write_text(
        """
image_width: 1920
image_height: 1080
camera_name: macbook_pro_14_2025_center_stage
distortion_model: plumb_bob
camera_matrix:
  rows: 3
  cols: 3
  data: [2236.0, 0.0, 990.0, 0.0, 2378.0, 568.0, 0.0, 0.0, 1.0]
distortion_coefficients:
  rows: 1
  cols: 5
  data: [1.7, -24.5, -0.03, -0.1, 212.1]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
projection_matrix:
  rows: 3
  cols: 4
  data: [2236.0, 0.0, 990.0, 0.0, 0.0, 2378.0, 568.0, 0.0, 0.0, 0.0, 1.0, 0.0]
""".lstrip()
    )

    camera = create_desk_webcam(camera_info_yaml, camera_index=1, fps=7.5)

    assert isinstance(camera, Webcam)
    assert camera.config.camera_index == 1
    assert camera.config.width == 1920
    assert camera.config.height == 1080
    assert camera.config.fps == 7.5
    assert camera.config.camera_info.frame_id == DESK_CAMERA_FRAME_ID


def test_desk_static_tf_module_publishes_world_to_camera_optical_chain() -> None:
    mod = DeskStaticTfModule(
        camera_translation_m=(0.3, 0.0, 0.2),
        camera_rotation_rpy_rad=(0.0, 0.0, 0.0),
    )
    try:
        mod.start()
        assert mod._last_publish_ts is not None

        world_camera = mod.tf.get("world", "camera_optical", mod._last_publish_ts, 1.0)
        assert world_camera is not None
        assert world_camera.frame_id == "world"
        assert world_camera.child_frame_id == "camera_optical"
        assert world_camera.translation.x == 0.3
        assert world_camera.translation.y == 0.0
        assert world_camera.translation.z == 0.2
    finally:
        mod.stop()
