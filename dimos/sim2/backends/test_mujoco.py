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

"""Real, small MuJoCo scenes verify the backend's public physics semantics."""

from dataclasses import asdict
from functools import partial
import json
import math

import numpy as np
import pytest
import robosuite

from dimos.sim2.backends.mujoco import MujocoBackend
from dimos.sim2.environment import MotorScene
from dimos.sim2.messages import FrameMetadata
from dimos.sim2.physics import BodyWrench, JointDrive, KinematicTarget


@pytest.fixture
def backend(world_config):
    with MujocoBackend(**world_config) as world:
        world.reset()
        yield world


def state(world):
    return world.read_state(FrameMetadata(1, 1, 1, world.sim_time))


def test_world_description_and_state_name_physical_bodies_and_joints(backend):
    observed = state(backend)

    assert backend.description.robot_roots == {"left": "left/base"}
    assert backend.description.robot_joints == {"left": ("left/hinge", "left/finger_joint")}
    assert backend.description.joints["door"].actuated is False
    assert observed.bodies["box"].position == pytest.approx((1.5, 0, 0.3))
    assert observed.bodies["left/base"].orientation == pytest.approx((0, 0, 0, 1))
    assert observed.joints["left/hinge"].position == pytest.approx((0.1,))
    assert observed.joints["left/finger_joint"].position == pytest.approx((0.0,))
    assert observed.robots["left"].position == pytest.approx((0.1, 850))
    assert observed.robots["left"].imu.linear_velocity == pytest.approx((0, 0, 0))


def test_world_state_contains_only_portable_python_values(backend):
    backend.step((JointDrive("left/hinge", effort=0.4),))
    payload = asdict(state(backend))
    pending = [payload]
    while pending:
        value = pending.pop()
        if isinstance(value, dict):
            pending.extend(value.values())
        elif isinstance(value, (tuple, list)):
            pending.extend(value)
        else:
            assert type(value) in (float, int, str, type(None))
    assert json.loads(json.dumps(payload, allow_nan=False))["joints"]["left/hinge"][
        "actuator_effort"
    ] == pytest.approx([0.4])


def test_joint_drive_preserves_effort_and_native_position_servo_semantics(backend):
    backend.step(
        (
            JointDrive("left/hinge", position=0.3, velocity=0.2, kp=2, kd=3, effort=0.4),
            JointDrive("left/finger_joint", position=0.02),
        )
    )
    observed = state(backend)

    assert observed.joints["left/hinge"].actuator_effort == pytest.approx((1.4,))
    assert observed.joints["left/hinge"].position[0] > 0.1
    for _ in range(300):
        backend.step(
            (
                JointDrive("left/hinge", position=0.1, kp=10, kd=1),
                JointDrive("left/finger_joint", position=0.02),
            )
        )
    settled = state(backend)
    assert settled.joints["left/finger_joint"].position == pytest.approx((0.02,), abs=0.001)
    assert settled.robots["left"].position[1] == pytest.approx(425, abs=25)
    with pytest.raises(ValueError, match="position targets only"):
        backend.validate((JointDrive("left/finger_joint", position=0.02, kp=1),))
    with pytest.raises(ValueError, match="no supported scalar drive"):
        backend.validate((JointDrive("door", effort=1),))


def test_motor_scene_trajectory_and_measurements_do_not_need_repeated_forward_solves(
    world_config, scene_path, robot_path
):
    scene_path.write_text(
        scene_path.read_text()
        .replace('gravity="0 0 0"', 'gravity="0 0 -9.81"')
        .replace(
            "</worldbody>",
            '<body name="prop" mocap="true" pos="0 3 1">'
            '<geom type="sphere" size=".1"/></body></worldbody>',
        )
    )
    robot_path.write_text(
        robot_path.read_text().replace(
            '<body name="base" pos="0 0 .3">',
            '<body name="base" pos="0 0 .3"><freejoint name="floating_base"/>',
        )
    )
    with MujocoBackend(**world_config) as optimized, MujocoBackend(**world_config) as repeated:
        optimized.reset()
        repeated.reset()
        for tick in range(80):
            commands = (
                JointDrive("left/hinge", position=0.1 + 0.03 * math.sin(tick / 10), kp=2, kd=1),
                JointDrive("left/finger_joint", position=0.02),
                BodyWrench("box", force=(0.2, 0, 0), point=(0, 0.1, 0), frame="body"),
                KinematicTarget("prop", (0.001 * tick, 3, 1), (0, 0, 0, 1)),
            )
            # The old bridge redundantly solved before applying its forces and
            # again after the environment had refreshed post-step measurements.
            repeated.data.xfrc_applied[:] = 0
            repeated.data.qfrc_applied[:] = 0
            repeated.environment.sim.forward()
            reference = repeated.step(commands)
            repeated.environment.sim.forward()
            measured = optimized.step(commands)

            np.testing.assert_allclose(optimized.data.qpos, repeated.data.qpos, atol=1e-12, rtol=0)
            np.testing.assert_allclose(optimized.data.qvel, repeated.data.qvel, atol=1e-12, rtol=0)
            np.testing.assert_allclose(
                optimized.data.sensordata, repeated.data.sensordata, atol=1e-12, rtol=0
            )
            assert state(optimized).robots["left"].effort == pytest.approx(
                state(repeated).robots["left"].effort, abs=1e-12, rel=0
            )
            np.testing.assert_allclose(
                measured.observations["left/imu/imu"],
                reference.observations["left/imu/imu"],
                atol=1e-12,
                rtol=0,
            )
            assert state(optimized).bodies["prop"].position == pytest.approx((0.001 * tick, 3, 1))


def test_body_force_is_integrated_for_one_step_and_not_held(backend):
    backend.step((BodyWrench("box", force=(2, 0, 0)),))
    pushed = state(backend).bodies["box"]
    backend.step(())
    released = state(backend).bodies["box"]

    assert pushed.linear_velocity == pytest.approx((2 * backend.step_seconds, 0, 0), abs=1e-7)
    assert pushed.position[0] > 1.5
    assert released.linear_velocity == pytest.approx(pushed.linear_velocity, abs=1e-7)
    assert released.position[0] > pushed.position[0]


def test_off_center_force_rotates_dynamic_body(backend):
    backend.step((BodyWrench("box", force=(1, 0, 0), point=(0, 1, 0), frame="body"),))
    moved = state(backend).bodies["box"]

    assert moved.linear_velocity[0] > 0
    assert moved.angular_velocity[2] < 0


def test_wrench_can_move_a_passive_fixture_through_its_joint(backend):
    backend.step((BodyWrench("door", torque=(0, 0, 1)),))
    observed = state(backend)

    assert observed.joints["door"].velocity[0] > 0
    assert observed.joints["door"].actuator_effort == pytest.approx((0,))


def test_invalid_command_batch_does_not_apply_earlier_valid_commands(backend):
    before = state(backend)
    with pytest.raises(ValueError, match="not an authored kinematic body"):
        backend.step(
            (BodyWrench("box", force=(100, 0, 0)), KinematicTarget("box", (3, 0, 0), (0, 0, 0, 1)))
        )

    assert state(backend) == before
    backend.step(())
    assert state(backend).bodies["box"].linear_velocity == pytest.approx((0, 0, 0))


def test_kinematic_targets_prescribe_pose_and_reset_restores_authored_pose(scene_path):
    scene_path.write_text(
        scene_path.read_text().replace(
            "</worldbody>",
            '<body name="prop" mocap="true" pos="0 3 1"><geom type="sphere" size=".1"/></body></worldbody>',
        )
    )
    with MujocoBackend(partial(MotorScene, scene_path, {})) as world:
        world.reset()
        world.step((KinematicTarget("prop", (1, 3, 2), (0, 0, math.sqrt(0.5), math.sqrt(0.5))),))
        moved = state(world)
        assert moved.bodies["prop"].position == pytest.approx((1, 3, 2))
        assert moved.bodies["prop"].orientation == pytest.approx(
            (0, 0, math.sqrt(0.5), math.sqrt(0.5))
        )
        with pytest.raises(ValueError, match="not dynamic"):
            world.validate((BodyWrench("prop", force=(1, 0, 0)),))
        world.reset()
        assert state(world).bodies["prop"].position == pytest.approx((0, 3, 1))
        assert state(world).scene_revision > moved.scene_revision


def test_scene_joint_actuators_use_the_shared_drive_semantics(scene_path):
    scene_path.write_text(
        scene_path.read_text().replace(
            "</mujoco>",
            '<actuator><motor name="door_motor" joint="door_hinge" ctrlrange="-2 2"/></actuator></mujoco>',
        )
    )
    with MujocoBackend(partial(MotorScene, scene_path, {})) as world:
        world.reset()
        world.step((JointDrive("door", effort=1),))
        observed = state(world)
        assert world.description.joints["door"].actuated
        assert observed.joints["door"].velocity[0] > 0
        assert observed.joints["door"].actuator_effort == pytest.approx((1,))


def test_reset_invalidates_checkpoint_model_and_clears_applied_forces(backend):
    metadata = FrameMetadata(1, 1, 1, backend.sim_time)
    previous = backend.checkpoint(metadata, 100.0)
    backend.step((BodyWrench("box", force=(1, 0, 0)),))
    backend.reset()
    backend.step(())
    current = backend.checkpoint(metadata, 101.0)

    assert state(backend).bodies["box"].linear_velocity == pytest.approx((0, 0, 0))
    assert current.model_path != previous.model_path
    assert current.values["wall_time"] == pytest.approx([101.0])
    assert backend.sensor_description()["backend"] == "mujoco"


def test_native_robosuite_task_preserves_actions_observations_and_horizon():
    factory = partial(
        robosuite.make,
        "Lift",
        robots="Panda",
        has_renderer=False,
        has_offscreen_renderer=False,
        use_camera_obs=False,
        control_freq=20,
        horizon=1,
        hard_reset=False,
    )
    with MujocoBackend(factory) as world:
        initial = world.reset()
        assert "robot0_joint_pos" in initial.observations
        world.set_task_action([0.0] * 7)
        result = world.step(())
        assert result.done
        assert world.sim_time == pytest.approx(0.05)
        assert np.isfinite(state(world).joints["robot0_joint1"].position).all()
        assert world.description.joints["robot0_joint1"].actuated
        assert state(world).joints["robot0_joint1"].actuator_effort is None
        with pytest.raises(RuntimeError, match="terminated"):
            world.step(())
        world.reset()
        assert not world.done
        assert world.sim_time == 0


def test_body_frame_force_is_rotated_into_world_coordinates(scene_path):
    scene_path.write_text(
        scene_path.read_text().replace(
            '<body name="box" pos="1.5 0 .3">',
            '<body name="box" pos="1.5 0 .3" quat=".7071067811865476 0 0 .7071067811865476">',
        )
    )
    with MujocoBackend(partial(MotorScene, scene_path, {})) as world:
        world.reset()
        world.step((BodyWrench("box", force=(2, 0, 0), frame="body"),))
        observed = state(world).bodies["box"]
        assert observed.linear_velocity == pytest.approx((0, 2 * world.step_seconds, 0), abs=1e-7)
        assert observed.position[1] > 0


def test_ball_joint_state_uses_xyzw_and_local_angular_velocity(scene_path):
    scene_path.write_text(
        scene_path.read_text().replace(
            "</worldbody>",
            '<body name="ball" pos="0 3 1"><joint name="ball_joint" type="ball"/><geom type="sphere" size=".1"/></body></worldbody>',
        )
    )
    with MujocoBackend(partial(MotorScene, scene_path, {})) as world:
        world.reset()
        observed = state(world)
        assert observed.joints["ball_joint"].position == pytest.approx((0, 0, 0, 1))
        assert observed.joints["ball_joint"].velocity == pytest.approx((0, 0, 0))
        assert world.description.joints["ball_joint"].kind == "spherical"
        with pytest.raises(ValueError, match="no supported scalar drive"):
            world.validate((JointDrive("ball_joint", effort=1),))


def test_visual_assets_attach_to_public_bodies_and_refresh_on_reset(backend):
    visuals = backend.describe_visuals()
    assert {geom.attachment.body for geom in visuals.geometries} <= state(backend).bodies.keys()
    assert visuals.cameras["left/view"].attachment.body == "left/base"
    backend.model.geom_rgba[0] = (0.2, 0.3, 0.4, 1.0)
    backend.reset()
    refreshed = backend.describe_visuals()
    assert refreshed.revision == state(backend).scene_revision > visuals.revision
    assert refreshed.geometries[0].rgba == pytest.approx((0.2, 0.3, 0.4, 1.0))
