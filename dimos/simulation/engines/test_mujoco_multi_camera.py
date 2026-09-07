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

from __future__ import annotations

from collections.abc import Iterator
from pathlib import Path
import time

import numpy as np
import pytest

pytest.importorskip("mujoco")

from dimos.msgs.sensor_msgs.Image import Image
from dimos.simulation.engines.mujoco_sim_module import (
    MujocoSimModule,
    SimCameraSpec,
    declare_sim_camera_module,
)

pytestmark = pytest.mark.mujoco


def _offscreen_gl_available() -> bool:
    """MuJoCo needs an offscreen GL platform; without one the sim thread dies."""
    import mujoco

    model = mujoco.MjModel.from_xml_string("<mujoco><worldbody/></mujoco>")
    try:
        mujoco.Renderer(model, height=8, width=8).close()
    except Exception:
        return False
    return True


requires_gl = pytest.mark.skipif(
    not _offscreen_gl_available(),
    reason="no offscreen GL platform; run with MUJOCO_GL=egl",
)

_SCENE = """
<mujoco model="two-camera">
  <compiler angle="radian"/>
  <option timestep="0.002"/>
  <worldbody>
    <light pos="0 0 2"/>
    <camera name="overhead" pos="0 0 1.5" quat="1 0 0 0" fovy="60"/>
    <body name="prop" pos="0.2 -0.1 0.3">
      <geom name="prop_box" type="box" size="0.05 0.04 0.03" mass="1"/>
      <camera name="side" pos="0.4 0 0" quat="0.5 0.5 0.5 0.5" fovy="60"/>
      <body name="prop_shell" pos="0 0 0.06">
        <geom name="prop_sphere" type="sphere" size="0.02" mass="0.1"/>
      </body>
    </body>
    <body name="arm" pos="0 0 0.1">
      <joint name="hinge_joint" type="hinge" axis="0 0 1" range="-2 2"/>
      <geom name="arm_capsule" type="capsule" fromto="0 0 0 0.2 0 0" size="0.02" mass="0.5"/>
      <body name="forearm" pos="0.2 0 0">
        <joint name="elbow_joint" type="hinge" axis="0 1 0" range="-2 2"/>
        <geom name="forearm_capsule" type="capsule" fromto="0 0 0 0.1 0 0" size="0.02" mass="0.2"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <position name="hinge_joint_act" joint="hinge_joint" kp="50" kv="5"/>
    <position name="elbow_joint_act" joint="elbow_joint" kp="50" kv="5"/>
  </actuator>
</mujoco>
"""

_EXTRA = (
    SimCameraSpec("overhead", "top_image", width=64, height=48, fps=30.0),
    SimCameraSpec("side", "left_wrist_image", width=64, height=48, fps=30.0),
)

TwoCameraSimModule = declare_sim_camera_module("TwoCameraSimModule", __name__, _EXTRA)


@pytest.fixture
def scene(tmp_path: Path) -> Path:
    path = tmp_path / "two_camera.xml"
    path.write_text(_SCENE.strip())
    return path


def _collect(module: MujocoSimModule, stream: str, count: int, timeout: float = 20.0) -> list:
    received: list = []
    getattr(module, stream).subscribe(received.append)
    deadline = time.monotonic() + timeout
    while len(received) < count and time.monotonic() < deadline:
        time.sleep(0.05)
    return received


@pytest.fixture
def running_module(scene: Path) -> Iterator[MujocoSimModule]:
    module = TwoCameraSimModule(
        address=scene,
        headless=True,
        dof=2,
        camera_name="unused",
        enable_color=False,
        enable_depth=False,
        enable_pointcloud=False,
        extra_cameras=list(_EXTRA),
    )
    module.start()
    yield module
    module.stop()


@requires_gl
def test_every_extra_camera_publishes_on_its_own_stream(
    running_module: MujocoSimModule,
) -> None:
    top = _collect(running_module, "top_image", 2)
    side = _collect(running_module, "left_wrist_image", 2)

    assert len(top) >= 2 and len(side) >= 2
    for frame in (*top, *side):
        assert isinstance(frame, Image)
        assert frame.data.shape == (48, 64, 3)
    assert top[0].frame_id == "overhead_color_optical_frame"
    assert side[0].frame_id == "side_color_optical_frame"
    # Distinct viewpoints, so the two streams must not be the same picture.
    assert not np.array_equal(top[-1].data, side[-1].data)


def test_body_poses_and_geoms_come_back_in_world_frame(
    running_module: MujocoSimModule,
) -> None:
    poses = running_module.get_body_poses(["prop", "arm", "nonexistent"])

    assert set(poses) == {"prop", "arm"}
    assert poses["prop"][:3] == pytest.approx([0.2, -0.1, 0.3], abs=1e-6)
    assert poses["prop"][3:] == pytest.approx([0.0, 0.0, 0.0, 1.0], abs=1e-6)

    geoms = running_module.get_body_geoms("prop")
    # prop_shell is welded to prop, so it belongs to the same rigid body;
    # the jointed forearm under arm does not.
    assert [g.name for g in geoms] == ["prop_box", "prop_sphere"]
    assert geoms[0].size == pytest.approx((0.05, 0.04, 0.03))
    assert geoms[0].position == pytest.approx((0.2, -0.1, 0.3), abs=1e-6)
    assert geoms[0].mesh is None
    assert geoms[1].position == pytest.approx((0.2, -0.1, 0.36), abs=1e-6)
    assert [g.name for g in running_module.get_body_geoms("arm")] == ["arm_capsule"]
    assert running_module.get_body_geoms("nonexistent") == []


def test_a_stream_the_class_does_not_declare_is_rejected(scene: Path) -> None:
    module = MujocoSimModule(
        address=scene,
        headless=True,
        dof=1,
        camera_name="overhead",
        extra_cameras=[SimCameraSpec("side", "left_wrist_image")],
    )
    try:
        with pytest.raises(ValueError, match="does not declare"):
            module.start()
    finally:
        module.stop()
