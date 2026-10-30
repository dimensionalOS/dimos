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

from dataclasses import asdict
from importlib.util import find_spec
import json

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dimos.robot.unitree.g1.connection import G1ConnectionConfig
from dimos.robot.unitree.g1.model import describe_simulated_robot
from dimos.sim2.backends.genesis import GenesisBackend
from dimos.sim2.messages import FrameMetadata
from dimos.sim2.physics import BodyWrench, JointDrive, KinematicTarget

pytestmark = [
    pytest.mark.self_hosted,
    pytest.mark.skipif(find_spec("genesis") is None, reason="requires the Genesis extra"),
]


@pytest.fixture(params=["stock", "motors"])
def g1_backend(request, tmp_path):
    scene = tmp_path / "ground.xml"
    scene.write_text(
        '<mujoco><compiler angle="radian"/><worldbody>'
        '<geom name="floor" type="plane" size="0 0 .1" friction=".6 .005 .0001"/>'
        "</worldbody></mujoco>"
    )
    config = G1ConnectionConfig(control_mode=request.param, sim_position=(0.0, 0.0, 1.0))
    robot = describe_simulated_robot(config)
    backend = GenesisBackend({"g1": robot}, scene=scene)
    try:
        backend.build()
        yield backend
    finally:
        backend.close()


def test_g1_named_state_serializes_and_body_wrench_changes_current_tick(g1_backend):
    backend = g1_backend
    initial = backend.read_state(FrameMetadata(0, 0, 0, 0.0))
    assert len(initial.robots["g1"].position) == 29
    assert "g1/left_knee_joint" in initial.joints
    assert initial.bodies["g1/pelvis"] == initial.robots["g1"].base
    assert json.loads(json.dumps(asdict(initial)))["robots"]["g1"]["base"]["position"][2] > 1.7
    visuals = backend.describe_visuals()
    assert visuals.geometries
    assert all(geometry.attachment.body in initial.bodies for geometry in visuals.geometries)
    assert all(frame.body in initial.bodies for frame in visuals.frames.values())
    backend.step(())
    baseline = backend.read_state(FrameMetadata(1, 0, 1, backend.sim_time))
    backend.reset()

    backend.step((BodyWrench("g1/pelvis", force=(300.0, 0.0, 0.0)),))
    pushed = backend.read_state(FrameMetadata(2, 1, 1, backend.sim_time))

    assert (
        pushed.bodies["g1/pelvis"].linear_velocity[0]
        > baseline.bodies["g1/pelvis"].linear_velocity[0] + 0.001
    )
    backend.reset()
    reset = backend.read_state(FrameMetadata(3, 2, 1, 0.0))
    assert reset.robots == initial.robots
    assert reset.bodies == initial.bodies
    assert not any(reset.robots["g1"].effort)
    if not backend.robots["g1"].position_only_joints:
        backend.step((JointDrive("g1/left_elbow_joint", effort=4.0),))
        driven = backend.read_state(FrameMetadata(4, 2, 2, backend.sim_time))
        assert driven.joints["g1/left_elbow_joint"].actuator_effort == (4.0,)
        backend.step(())
        released = backend.read_state(FrameMetadata(5, 2, 3, backend.sim_time))
        assert released.joints["g1/left_elbow_joint"].actuator_effort == (0.0,)


def test_g1_joint_drive_and_rejected_teleport_preserve_physical_contract(g1_backend):
    backend = g1_backend
    initial = backend.read_state(FrameMetadata(0, 0, 0, 0.0))
    name = "g1/left_elbow_joint"
    target = initial.joints[name].position[0] + 0.2
    position_only = bool(backend.robots["g1"].position_only_joints)
    drive = JointDrive(
        name, position=target, kp=0.0 if position_only else 30.0, kd=0.0 if position_only else 2.0
    )

    for _ in range(25):
        backend.step((drive,))
    result = backend.read_state(FrameMetadata(1, 0, 25, backend.sim_time))

    assert result.joints[name].position[0] > initial.joints[name].position[0] + 0.01
    assert np.isfinite(result.robots["g1"].position).all()
    before = result.bodies["g1/pelvis"]
    with pytest.raises(NotImplementedError, match="kinematic"):
        backend.step((KinematicTarget("g1/pelvis", (5.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),))
    assert (
        backend.read_state(FrameMetadata(2, 0, 25, backend.sim_time)).bodies["g1/pelvis"] == before
    )


def test_g1_body_wrenches_add_in_authored_body_frame(g1_backend):
    backend = g1_backend
    body = "g1/left_ankle_roll_link"
    initial = backend.read_state(FrameMetadata(0, 0, 0, 0.0))
    rotation = Rotation.from_quat(initial.bodies[body].orientation)
    force = tuple(rotation.apply((100.0, 0.0, 0.0)))
    backend.step((BodyWrench(body, force=force),))
    expected = backend.read_state(FrameMetadata(1, 0, 1, backend.sim_time))
    backend.reset()

    backend.step(
        (
            BodyWrench(body, force=(40.0, 0.0, 0.0), frame="body"),
            BodyWrench(body, force=(60.0, 0.0, 0.0), frame="body"),
        )
    )
    actual = backend.read_state(FrameMetadata(2, 1, 1, backend.sim_time))

    assert actual.robots["g1"].velocity == pytest.approx(expected.robots["g1"].velocity, abs=1e-6)
    assert actual.robots["g1"].base.linear_velocity == pytest.approx(
        expected.robots["g1"].base.linear_velocity, abs=1e-6
    )
    assert backend.read_state(actual.metadata) == actual
    with pytest.raises(ValueError, match="three components"):
        backend.validate((BodyWrench(body, force=(1.0, 2.0)),))


def test_scene_objects_run_without_robot_descriptions(scene_path):
    backend = GenesisBackend({}, scene=scene_path, physics_dt=0.01)
    try:
        backend.build()
        initial = backend.read_state(FrameMetadata(0, 0, 0, 0.0))
        backend.step((BodyWrench("box", force=(10.0, 0.0, 0.0)),))
        moved = backend.read_state(FrameMetadata(1, 0, 1, backend.sim_time))

        assert moved.robots == {}
        assert backend.sim_time == 0.01
        assert moved.bodies["box"].linear_velocity[0] > 0.05
        assert moved.bodies["box"].position[0] > initial.bodies["box"].position[0]
        backend.reset()
        assert backend.read_state(FrameMetadata(2, 1, 0, 0.0)).bodies == initial.bodies
    finally:
        backend.close()


def test_static_scene_can_step_without_any_joints(tmp_path):
    scene = tmp_path / "floor.xml"
    scene.write_text(
        '<mujoco><worldbody><geom name="floor" type="plane" size="1 1 .1"/></worldbody></mujoco>'
    )
    backend = GenesisBackend({}, scene=scene)
    try:
        backend.build()
        backend.step(())
        state = backend.read_state(FrameMetadata(0, 0, 1, backend.sim_time))

        assert state.robots == {}
        assert state.joints == {}
        assert state.bodies["world"].position == (0.0, 0.0, 0.0)
        assert backend.sim_time == 0.005
        backend.reset()
        assert backend.sim_time == 0.0
    finally:
        backend.close()
