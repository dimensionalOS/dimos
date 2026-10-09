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

"""The native encodes with dimos_lcm; dimos decodes. These pin that contract.

server.py runs in LIBERO's Python 3.10 environment, but its encoders need only dimos_lcm,
so the wire format is testable here without LIBERO installed.
"""

import ast
import json
import math
from pathlib import Path

import numpy as np
import pytest

from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.String import String
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.simulation.libero import server

NATIVE_FILES = [Path(server.__file__)]


def test_color_image_round_trip() -> None:
    rgb = np.arange(8 * 12 * 3, dtype=np.uint8).reshape(8, 12, 3)
    out = Image.lcm_decode(server.image_msg(rgb, "rgb8", "wrist_camera_color_optical_frame", 1.5))
    np.testing.assert_array_equal(out.data, rgb)
    assert out.frame_id == "wrist_camera_color_optical_frame"


def test_camera_info_matches_mujoco_sim_intrinsics() -> None:
    out = CameraInfo.lcm_decode(server.camera_info_msg(640, 480, 60.0, "optical", 1.5))
    k = out.get_K_matrix()
    fy = 480 / (2 * math.tan(math.radians(60.0) / 2))
    np.testing.assert_allclose([k[0, 0], k[1, 1], k[0, 2], k[1, 2]], [fy, fy, 320.0, 240.0])


def test_joint_state_round_trip() -> None:
    names = [*server.ARM_JOINTS, server.GRIPPER_JOINT]
    pos = [0.1 * i for i in range(8)]
    out = JointState.lcm_decode(server.joint_state_msg(names, pos, [0.0] * 8, [0.0] * 8, 2.0))
    assert out.name == names
    np.testing.assert_allclose(out.position, pos)


def test_tf_round_trip() -> None:
    t = server.transform(
        "world", "bowl_1", np.array([1.0, 2.0, 3.0]), np.array([1.0, 0, 0, 0]), 2.0
    )
    (out,) = TFMessage.lcm_decode(server.tf_msg([t])).transforms
    assert (out.frame_id, out.child_frame_id) == ("world", "bowl_1")
    assert (out.translation.x, out.translation.y, out.translation.z) == (1.0, 2.0, 3.0)
    assert out.rotation.w == 1.0


def test_task_status_is_json() -> None:
    status = {"success": True, "predicates": [["on", "a", "b", True]]}
    assert json.loads(String.lcm_decode(server.string_msg(json.dumps(status))).data) == status


@pytest.mark.parametrize(
    ("opening", "fingers"),
    [(0.08, (0.04, -0.04)), (0.0, (0.0, 0.0)), (0.03, (0.015, -0.015)), (1.0, (0.04, -0.04))],
)
def test_gripper_opening_splits_across_the_fingers(
    opening: float, fingers: tuple[float, float]
) -> None:
    assert server.finger_targets(opening) == pytest.approx(fingers)


@pytest.mark.parametrize("path", NATIVE_FILES, ids=lambda p: p.name)
def test_native_files_stay_out_of_dimos_and_on_python_3_10(path: Path) -> None:
    tree = ast.parse(path.read_text(), feature_version=(3, 10))
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            modules = [alias.name for alias in node.names]
        elif isinstance(node, ast.ImportFrom):
            modules = [node.module or ""]
        else:
            continue
        assert not any(m == "dimos" or m.startswith("dimos.") for m in modules), modules
