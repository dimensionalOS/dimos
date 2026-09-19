# Copyright 2025-2026 Dimensional Inc.
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

"""The R1's head stereo blueprint: its calibration, its frame, its pose."""

import numpy as np

from dimos.hardware.sensors.camera.depth_cloud.module import StereoCloud
from dimos.robot.galaxea.r1pro.stereo import (
    HEAD_CAMERA_FRAME,
    HEAD_CAMERA_IN_BASE_RPY_RAD,
    HEAD_CAMERA_IN_BASE_XYZ_M,
    MAX_RANGE_M,
    r1pro_stereo_cloud,
)
from dimos.robot.galaxea.r1pro.stereo_calibration import R1PRO_HEAD_CALIBRATION


def _atom(blueprint, module):
    return next(atom for atom in blueprint.active_blueprints if atom.module is module)


def test_the_rig_calibration_reaches_the_matcher() -> None:
    kwargs = _atom(r1pro_stereo_cloud(R1PRO_HEAD_CALIBRATION), StereoCloud).kwargs
    assert kwargs["baseline_m"] == R1PRO_HEAD_CALIBRATION.baseline_m
    assert kwargs["right_yaw_rad"] == R1PRO_HEAD_CALIBRATION.right_yaw_rad
    assert kwargs["right_pitch_rad"] == R1PRO_HEAD_CALIBRATION.right_pitch_rad
    assert kwargs["right_roll_rad"] == R1PRO_HEAD_CALIBRATION.right_roll_rad
    assert kwargs["frame_id"] == HEAD_CAMERA_FRAME
    assert kwargs["max_range_m"] == MAX_RANGE_M
    assert kwargs["base_from_camera_xyz_m"] == HEAD_CAMERA_IN_BASE_XYZ_M
    assert kwargs["base_from_camera_rpy_rad"] == HEAD_CAMERA_IN_BASE_RPY_RAD
    # No gate unless a run asks for one.
    assert "min_height_m" not in kwargs
    assert "max_height_m" not in kwargs


def test_height_gate_and_denoise_are_passed_only_when_asked() -> None:
    blueprint = r1pro_stereo_cloud(
        R1PRO_HEAD_CALIBRATION, max_height_m=0.5, min_height_m=-0.2, denoise="none"
    )
    kwargs = _atom(blueprint, StereoCloud).kwargs
    assert kwargs["max_height_m"] == 0.5
    assert kwargs["min_height_m"] == -0.2
    assert kwargs["denoise"] == "none"


def test_the_eyes_and_the_outputs_are_remapped_onto_the_connections_streams() -> None:
    blueprint = r1pro_stereo_cloud(R1PRO_HEAD_CALIBRATION)
    key = blueprint._instance_key(StereoCloud)
    remaps = blueprint.remapping_map
    assert remaps[(key, "left")] == "head_left_color"
    assert remaps[(key, "right")] == "head_right_color"
    assert remaps[(key, "left_info")] == "head_left_info"
    assert remaps[(key, "right_info")] == "head_right_info"
    assert remaps[(key, "cloud")] == "head_cloud"
    assert remaps[(key, "depth")] == "head_depth"


def test_the_head_pose_constants_match_the_urdf() -> None:
    """The numbers the height cutoff is measured against, checked against FK.

    Loads the vendor URDF through pinocchio, exactly as the connection does.
    """
    from scipy.spatial.transform import Rotation

    from dimos.msgs.sensor_msgs.JointState import JointState
    from dimos.robot.galaxea.r1pro.config import R1PRO_MODEL
    from dimos.robot.galaxea.r1pro.connection import ArticulatedTf

    fk = ArticulatedTf(R1PRO_MODEL, [HEAD_CAMERA_FRAME])
    (transform,) = fk.transforms(JointState(name=[], position=[]))
    assert transform.frame_id == "base_link"
    xyz = (transform.translation.x, transform.translation.y, transform.translation.z)
    assert np.allclose(xyz, HEAD_CAMERA_IN_BASE_XYZ_M, atol=1e-3)
    q = transform.rotation
    rotation = Rotation.from_quat([q.x, q.y, q.z, q.w])
    expected = Rotation.from_euler("xyz", HEAD_CAMERA_IN_BASE_RPY_RAD)
    assert (rotation.inv() * expected).magnitude() < 1e-3
    # It is an optical frame: z points forward and down, y points down.
    forward = rotation.as_matrix()[:, 2]
    assert forward[0] > 0.9 and forward[2] < -0.3
