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

"""alfred-manipulation-sim's scene: the bench where the stack expects it, seen from the D455's
pose on the URDF, and one rendered frame that shows every object."""

from __future__ import annotations

from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ElementTree

import mujoco
import numpy as np
from numpy.typing import NDArray
import pytest
from scipy.spatial.transform import Rotation

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.robot.diy.alfred.blueprints.alfred_manipulation_sim import (
    ALFRED_BENCH_SCENE,
    SIM_BASE_BODY,
    SIM_CAMERA,
)
from dimos.robot.diy.alfred.config import ALFRED_URDF
from dimos.robot.diy.alfred.mount_tf import mount_transforms
from dimos.robot.diy.alfred.tools.tool_render_bench import BENCH_OBJECTS

# The bench as the bench stack's numbers assume it.
BENCH_TOP_Z, BENCH_FRONT_X, BENCH_DEPTH, BENCH_WIDTH = 1.06, 0.45, 1.0, 1.6
# The picture's bottom row: 25 degrees of pitch plus half of the 58.8 degree vertical field.
CAMERA_SEES_BENCH_FROM_X = 0.61


def _model() -> tuple[mujoco.MjModel, mujoco.MjData]:
    model = mujoco.MjModel.from_xml_path(str(ALFRED_BENCH_SCENE))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def _matrix(transform: Transform) -> NDArray[np.float64]:
    matrix = np.eye(4)
    rotation = transform.rotation
    matrix[:3, :3] = Rotation.from_quat(
        [rotation.x, rotation.y, rotation.z, rotation.w]
    ).as_matrix()
    matrix[:3, 3] = [transform.translation.x, transform.translation.y, transform.translation.z]
    return matrix


def test_the_scene_is_the_bench_the_stack_expects() -> None:
    model, data = _model()
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, SIM_BASE_BODY)
    assert model.body_mocapid[body] >= 0, "base_link follows the base's odometry"
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, SIM_CAMERA) >= 0
    assert model.vis.global_.offwidth >= 848 and model.vis.global_.offheight >= 480
    top = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "bench_top")
    centre, half = data.geom_xpos[top], model.geom_size[top]
    assert centre[2] + half[2] == pytest.approx(BENCH_TOP_Z)
    assert centre[0] - half[0] == pytest.approx(BENCH_FRONT_X)
    assert (2 * half[0], 2 * half[1], centre[1]) == pytest.approx((BENCH_DEPTH, BENCH_WIDTH, 0.0))
    shoulder_y = 0.1545  # openarm_*_link1 off base_link's centre line (alfred_v1.urdf)
    for name in BENCH_OBJECTS:
        geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        lowest = data.geom_xpos[geom][2] - _half_height(model, geom)
        assert lowest == pytest.approx(BENCH_TOP_Z, abs=2e-3), f"{name} stands on the bench"
        x, y = data.xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)][:2]
        if name == "water_bottle":
            assert x == pytest.approx(0.80), "out of reach, for the pick's reach message"
            continue
        assert CAMERA_SEES_BENCH_FROM_X < x < 0.70, f"{name}: in view, as close as the view allows"
        assert abs(abs(y) - shoulder_y) < 0.2, f"{name}: along a shoulder's line"


def _half_height(model: mujoco.MjModel, geom: int) -> float:
    size = model.geom_size[geom]
    kind = model.geom_type[geom]
    if kind == mujoco.mjtGeom.mjGEOM_SPHERE:
        return float(size[0])
    if kind == mujoco.mjtGeom.mjGEOM_CAPSULE:
        return float(size[0])  # lying down
    if kind == mujoco.mjtGeom.mjGEOM_BOX:
        return float(size[2])
    return float(size[1])  # a standing cylinder


# The urdf is LFS data, so these run where it is pulled (like test_alfred.py).
@pytest.mark.self_hosted
def test_the_scene_camera_sits_where_the_urdf_puts_the_d455() -> None:
    """AlfredMountTf's base_link -> camera_front_link, then the URDF's colour optical frame
    turned a half turn about its axis (rotate_180): MuJoCo's camera is that frame, in its own
    axes (x right, y up, looking along -z)."""
    edges = {(t.frame_id, t.child_frame_id): t for t in mount_transforms("base_link")}
    parents = {child: parent for parent, child in edges}
    link, to_link = "camera_front_link", np.eye(4)
    while link in parents:
        to_link = _matrix(edges[(parents[link], link)]) @ to_link
        link = parents[link]
    assert link == "base_link"

    joints = {
        joint.find("child").attrib["link"]: joint.find("origin")  # type: ignore[union-attr]
        for joint in ElementTree.parse(ALFRED_URDF).getroot().findall("joint")
    }

    def fixed(child: str) -> NDArray[np.float64]:
        origin = joints[child]
        assert origin is not None
        matrix = np.eye(4)
        rpy = [float(v) for v in origin.attrib["rpy"].split()]
        matrix[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
        matrix[:3, 3] = [float(v) for v in origin.attrib["xyz"].split()]
        return matrix

    half_turn = np.eye(4)
    half_turn[:3, :3] = Rotation.from_euler("z", np.pi).as_matrix()
    optical = (
        to_link
        @ fixed("camera_front_color_frame")
        @ fixed("camera_front_color_optical_frame")
        @ half_turn
    )

    model, data = _model()
    camera = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, SIM_CAMERA)
    mounted = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{SIM_CAMERA}_link")
    assert np.allclose(data.xpos[mounted], to_link[:3, 3], atol=1e-6)
    assert np.allclose(data.xmat[mounted].reshape(3, 3), to_link[:3, :3], atol=1e-6)
    assert np.allclose(data.cam_xpos[camera], optical[:3, 3], atol=1e-6)
    mujoco_axes = optical[:3, :3] @ Rotation.from_euler("x", np.pi).as_matrix()
    assert np.allclose(data.cam_xmat[camera].reshape(3, 3), mujoco_axes, atol=1e-6)
    assert data.cam_xpos[camera] == pytest.approx([0.254, 0.0115, 1.5468], abs=1e-3)
    pitch = np.degrees(np.arcsin(-optical[2, 2]))
    assert pitch == pytest.approx(25.0, abs=0.01), "looking down 25 degrees"


@pytest.mark.self_hosted
def test_one_rendered_frame_shows_every_object(tmp_path: Path) -> None:
    """The standalone check, in a process of its own: mujoco binds its GL backend at import,
    and the tool picks a headless one first."""
    result = subprocess.run(
        [sys.executable, "-m", "dimos.robot.diy.alfred.tools.tool_render_bench", "--out", tmp_path],
        capture_output=True,
        text=True,
        timeout=120,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    for name in BENCH_OBJECTS:
        assert name in result.stdout
    assert (tmp_path / "color.png").stat().st_size > 10_000
    assert (tmp_path / "depth.png").is_file()
