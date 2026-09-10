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

import math
from pathlib import Path

import numpy as np
import pytest

from dimos.experimental.robot.bosdyn.spot.utils import (
    camera_mount_transforms,
    roll_optical_frame,
    rotate_camera_info,
    rotate_image,
    upright_roll,
)
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat


def test_camera_mount_transforms_uses_loaded_robot_topology(tmp_path: Path) -> None:
    urdf = tmp_path / "spot.urdf"
    urdf.write_text(
        """
        <robot name="spot">
          <link name="base"/>
          <link name="camera_mount"/>
          <link name="camera_optical"/>
          <joint name="mount" type="fixed">
            <origin xyz="1 0 0"/>
            <parent link="base"/>
            <child link="camera_mount"/>
          </joint>
          <joint name="optical" type="fixed">
            <origin xyz="0 2 0"/>
            <parent link="camera_mount"/>
            <child link="camera_optical"/>
          </joint>
        </robot>
        """
    )

    transforms = camera_mount_transforms(urdf, "body", ["camera_optical"])

    assert len(transforms) == 1
    assert transforms[0].frame_id == "body"
    assert transforms[0].child_frame_id == "camera_optical"
    assert transforms[0].translation.to_list() == [1.0, 2.0, 0.0]


def _mount(roll: float, pitch: float, yaw: float) -> Transform:
    """A base_link -> optical frame like the URDF's: body rpy, then the camera->optical turn."""
    camera = Transform(
        translation=Vector3(0.0, 0.0, 0.0),
        rotation=Quaternion.from_euler(Vector3(roll, pitch, yaw)),
        frame_id="base_link",
        child_frame_id="camera",
    )
    optical = Transform(
        translation=Vector3(0.0, 0.0, 0.0),
        rotation=Quaternion.from_euler(Vector3(-math.pi / 2, 0.0, -math.pi / 2)),
        frame_id="camera",
        child_frame_id="camera_optical",
    )
    return camera + optical


def test_upright_roll_is_zero_for_a_level_camera() -> None:
    assert upright_roll(_mount(0.0, 0.3, 1.2)) == pytest.approx(0.0, abs=1e-9)


def test_upright_roll_undoes_the_mount_roll() -> None:
    # Spot's frontleft mount: rolled 77.7 deg, pitched down, yawed right.
    mount = _mount(1.355558, 0.353917, -0.554196)
    roll = upright_roll(mount)
    assert roll == pytest.approx(-1.355558, abs=1e-6)
    assert upright_roll(roll_optical_frame(mount, roll)) == pytest.approx(0.0, abs=1e-9)


def test_upright_roll_measures_the_lean_left_after_a_quarter_turn() -> None:
    quarter_turned = roll_optical_frame(_mount(1.355558, 0.353917, -0.554196), -math.pi / 2)
    assert math.degrees(upright_roll(quarter_turned)) == pytest.approx(12.33, abs=0.01)


def _depth_image(width: int, height: int, hits: list[tuple[int, int, int]]) -> Image:
    """Zero depth everywhere except a 3x3 patch of `millimetres` centred on each hit."""
    data = np.zeros((height, width), dtype=np.uint16)
    for u, v, millimetres in hits:
        data[v - 1 : v + 2, u - 1 : u + 2] = millimetres
    return Image.from_numpy(data, format=ImageFormat.DEPTH16, frame_id="camera_optical", ts=1.0)


def _back_project(image: Image, info: CameraInfo, frame: Transform) -> np.ndarray:
    """Every non-zero depth pixel as a point in the frame's parent."""
    v, u = np.nonzero(image.data)
    z = image.data[v, u] / 1000.0
    rays = np.stack([(u - info.K[2]) / info.K[0] * z, (v - info.K[5]) / info.K[4] * z], axis=1)
    rays = np.column_stack([rays, z])
    rotation = frame.rotation.to_rotation_matrix()
    return rays @ rotation.T + np.array(frame.translation.to_list())


def _hausdorff(a: np.ndarray, b: np.ndarray) -> float:
    distances = np.linalg.norm(a[:, None, :] - b[None, :, :], axis=2)
    return max(distances.min(axis=1).max(), distances.min(axis=0).max())


@pytest.mark.parametrize("roll", [math.pi / 2, -math.pi / 2, math.pi, 0.2152, -0.2152, -1.355558])
def test_rotating_image_info_and_frame_together_keeps_the_3d_points(roll: float) -> None:
    mount = _mount(1.355558, 0.353917, -0.554196)
    info = CameraInfo.from_intrinsics(
        fx=214.5, fy=214.5, cx=205.3, cy=119.8, width=424, height=240, frame_id="camera_optical"
    )
    image = _depth_image(424, 240, [(20, 30, 1500), (400, 200, 2500), (212, 120, 800)])
    before = _back_project(image, info, mount)

    after = _back_project(
        rotate_image(image, roll), rotate_camera_info(info, roll), roll_optical_frame(mount, roll)
    )

    # Resampling shifts a patch's pixels by at most ~one pixel of ray (1.2 cm at 2.5 m).
    assert _hausdorff(before, after) < 0.02


def test_quarter_turn_rotation_is_lossless() -> None:
    image = _depth_image(424, 240, [(20, 30, 1500), (400, 200, 2500)])
    turned = rotate_image(image, -math.pi / 2)
    assert turned.data.shape == (424, 240)
    assert np.array_equal(turned.data, np.rot90(image.data, k=-1))


def test_rotating_by_a_lean_grows_the_canvas_and_moves_the_principal_point() -> None:
    info = CameraInfo.from_intrinsics(
        fx=214.5, fy=214.5, cx=119.8, cy=205.3, width=240, height=424, frame_id="camera_optical"
    )
    image = _depth_image(240, 424, [(120, 212, 1000)])
    roll = math.radians(12.33)

    rotated_info = rotate_camera_info(info, roll)
    rotated_image = rotate_image(image, roll)

    assert rotated_info.width > 240 and rotated_info.height > 424
    assert rotated_image.data.shape == (rotated_info.height, rotated_info.width)
    assert (rotated_info.K[0], rotated_info.K[4]) == (info.K[0], info.K[4])
    # The patch at the old centre lands on the new centre; the principal point
    # turns about it, keeping its 6.2 px distance.
    v, u = np.nonzero(rotated_image.data)
    centre = ((rotated_info.width - 1) / 2, (rotated_info.height - 1) / 2)
    assert (u.mean(), v.mean()) == pytest.approx(centre, abs=1.0)
    assert math.hypot(
        rotated_info.K[2] - centre[0], rotated_info.K[5] - centre[1]
    ) == pytest.approx(math.hypot(119.8 - 119.5, 205.3 - 211.5), abs=1e-6)
