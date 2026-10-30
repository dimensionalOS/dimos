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

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dimos.control.feedback import MotorControl
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.robot.urdf_manager import UrdfManager
from dimos.sim2.client import SimRobotClient
from dimos.sim2.mujoco import MujocoBackend
from dimos.sim2.runtime import Runtime
from dimos.sim2.sensors import MujocoSensors
from dimos.sim2.types import JointEfforts, JointTargets, Wrench


@pytest.fixture
def runtime(world):
    return Runtime(MujocoBackend(world))


def test_shared_world_moves_one_robot_and_pushes_box(runtime):
    runtime.submit(JointTargets(runtime.epoch, ("left/slide_x",), (0.65,), ttl=5.0))
    runtime.advance(1000)
    state = runtime.snapshot()
    assert state.position[0] == pytest.approx(0.65, abs=0.001)
    np.testing.assert_allclose(state.position[2:4], 0, atol=1e-7)
    box = runtime.backend.layout.bodies.index("box")
    assert state.poses[box, 0] > 0.15
    np.testing.assert_allclose(runtime.capture().imu[:, 2], 9.81, atol=0.001)


def test_force_moves_free_body_and_torque_rotates_passive_joint(runtime):
    runtime.submit(Wrench(runtime.epoch, "box", (5.0, 0.0, 0.0), duration=0.3))
    runtime.submit(
        Wrench(runtime.epoch, "door/panel", (0.0, 0.0, 0.0), torque=(0.0, 0.0, 0.2), duration=0.3)
    )
    runtime.advance(150)
    state = runtime.snapshot()
    assert state.poses[runtime.backend.layout.bodies.index("box"), 0] > 0.1
    assert state.position[-1] > 0.02


def test_reset_invalidates_pending_commands_and_expiry_holds(runtime):
    runtime.submit(JointTargets(runtime.epoch, ("left/slide_x",), (0.6,), ttl=0.02))
    runtime.advance(10)
    runtime.advance(600)
    assert runtime.snapshot().position[0] < 0.2  # expired; not still pursuing .6
    runtime.submit(JointTargets(runtime.epoch, ("right/slide_x",), (0.7,)))
    previous = runtime.epoch
    state = runtime.reset()
    assert state.epoch != previous and state.tick == 0
    runtime.advance(100)
    assert runtime.rejected == 1
    np.testing.assert_allclose(runtime.snapshot().position[:4], 0, atol=1e-7)


def test_invalid_batch_does_not_partially_apply_and_place_is_separate(runtime):
    runtime.submit(JointTargets(runtime.epoch, ("left/slide_x", "door/hinge"), (0.5, 0.5)))
    runtime.advance(100)
    assert runtime.rejected == 1
    assert runtime.snapshot().position[0] == pytest.approx(0.0)
    before = runtime.snapshot()
    with pytest.raises(ValueError, match="floating"):
        runtime.place("left/base_link", (1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0))
    assert runtime.epoch == before.epoch
    state = runtime.place("box", (0.2, 0.2, 0.4, 0.0, 0.0, 0.0, 1.0))
    assert state.epoch != before.epoch
    assert runtime.backend.data.time == state.sim_time == 0.0
    np.testing.assert_allclose(
        state.poses[runtime.backend.layout.bodies.index("box"), :3], [0.2, 0.2, 0.4]
    )


def test_urdf_manager_mount_chain_matches_native_sensor_pose(runtime, world):
    runtime.submit(
        JointTargets(runtime.epoch, ("left/slide_x", "left/slide_y"), (0.2, 0.1), ttl=3.0)
    )
    runtime.advance(600)
    state = runtime.snapshot()
    model = world.instances[0].model
    manager = UrdfManager(model=model)
    transforms = manager.joint_transforms(
        JointState(name=list(runtime.backend.layout.joints), position=state.position.tolist())
    )
    poses = {"left/base_link": np.eye(4)}
    for _ in transforms:
        for tf in transforms:
            if tf.frame_id in poses:
                poses[tf.child_frame_id] = poses[tf.frame_id] @ tf.to_matrix()
    manager.dispose()
    frame = "left/camera"
    index = runtime.backend.layout.bodies.index(frame)
    np.testing.assert_allclose(poses[frame][:3, 3], state.poses[index, :3], atol=1e-9)
    np.testing.assert_allclose(
        poses[frame][:3, :3],
        Rotation.from_quat(state.poses[index, 3:].copy()).as_matrix(),
        atol=1e-9,
    )
    ancestor, mount = model.load().resolve_frame(frame)
    assert ancestor == "left/tool"
    np.testing.assert_allclose(poses[ancestor] @ mount, poses[frame], atol=1e-9)


def test_sensor_history_and_image_follow_mounted_frame(runtime, world):
    reader = MujocoSensors(world, runtime.backend.layout)
    try:
        for _ in range(10):
            runtime.advance(10)
            reader.observe(runtime.snapshot())
        state = runtime.snapshot()
        rgb, depth = reader.camera("left/camera", state)
        assert rgb.shape == (240, 320, 3) and rgb.std() > 5
        assert depth.shape == (240, 320) and np.isfinite(depth).all()
        scan = reader.lidar("left/lidar")
        assert scan is not None and len(scan.points) > 30
        assert np.ptp(scan.times) >= 0.08
        assert np.min(np.abs(scan.points[:, 2])) < 0.005  # actual floor hits
        runtime.submit(JointTargets(runtime.epoch, ("left/slide_x",), (0.2,), ttl=2.0))
        runtime.advance(300)
        moved_rgb, _ = reader.camera("left/camera", runtime.snapshot())
        assert np.mean(np.abs(moved_rgb.astype(float) - rgb)) > 2
        reader.observe(runtime.reset())
        assert reader.lidar("left/lidar") is None  # no scan bridges a reset
    finally:
        reader.close()


def test_direct_effort_respects_limits_and_switches_back_to_position(runtime):
    runtime.submit(JointEfforts(runtime.epoch, ("left/slide_x",), (5.0,), ttl=0.1))
    runtime.advance(25)
    state = runtime.snapshot()
    assert state.velocity[0] > 0.1
    assert state.effort[0] == pytest.approx(5.0)
    runtime.submit(JointEfforts(runtime.epoch, ("left/slide_x",), (1000.0,)))
    runtime.advance()
    assert runtime.rejected == 1
    runtime.submit(JointTargets(runtime.epoch, ("left/slide_x",), (0.2,), ttl=2.0))
    runtime.advance(700)
    assert runtime.snapshot().position[0] == pytest.approx(0.2, abs=0.001)


def test_world_state_is_an_owned_sample_not_a_live_native_view(runtime):
    sample = runtime.snapshot()
    before = sample.poses.copy()
    runtime.submit(Wrench(runtime.epoch, "box", (10.0, 0.0, 0.0), duration=0.1))
    runtime.advance(100)
    np.testing.assert_array_equal(sample.poses, before)
    with pytest.raises(ValueError):
        sample.position[0] = 1.0


def test_lidar_rejects_a_missing_interval_instead_of_bridging_it(runtime, world):
    reader = MujocoSensors(world, runtime.backend.layout)
    try:
        reader.observe(runtime.snapshot())
        runtime.advance(100)
        reader.observe(runtime.snapshot())
        assert reader.lidar("left/lidar") is None
        assert reader.gaps == 1
    finally:
        reader.close()


def test_motor_servo_preserves_velocity_gains_and_feedforward(world):
    runtime = Runtime(MujocoBackend(world))
    command = JointTargets(
        runtime.epoch,
        ("left/slide_x", "left/slide_y"),
        (0.1, 0.0),
        velocities=(0.2, -0.1),
        kp=(40.0, 0.0),
        kd=(3.0, 2.0),
        feedforward=(1.5, -1.0),
    )
    client = SimRobotClient(command.joints, runtime.submit)
    client.observe(runtime.snapshot())
    assert client.write_motors(
        MotorControl(
            runtime.epoch,
            0,
            command.joints,
            MotorCommandArray(
                q=list(command.positions),
                dq=list(command.velocities),
                kp=list(command.kp),
                kd=list(command.kd),
                tau=list(command.feedforward),
            ),
        )
    )
    runtime.advance()
    state = runtime.snapshot()
    indices = [runtime.backend.layout.joints.index(n) for n in command.joints]
    q, dq = state.position[indices], state.velocity[indices]
    expected = np.array(command.kp) * (np.array(command.positions) - q)
    expected += np.array(command.kd) * (np.array(command.velocities) - dq)
    expected += command.feedforward
    assert state.effort[indices] == pytest.approx(expected)
    assert runtime.rejected == 0


@pytest.mark.parametrize(
    "fields",
    [
        {"kp": (1.0,)},
        {"kd": (1.0, -1.0)},
        {"velocities": (0.0, float("nan"))},
        {"feedforward": (0.0, float("inf"))},
    ],
)
def test_bad_motor_batch_does_not_partially_change_actuators(world, fields):
    backend = MujocoBackend(world)
    runtime = Runtime(backend, ("left/slide_x", "left/slide_y"))
    before_ctrl = backend.data.ctrl.copy()
    before_gain = backend.model.actuator_gainprm.copy()
    before_bias = backend.model.actuator_biasprm.copy()
    runtime.submit(
        JointTargets(runtime.epoch, ("left/slide_x", "left/slide_y"), (0.2, 0.1), tick=0, **fields)
    )
    runtime.prepare()
    assert runtime.rejected == 1
    assert not runtime.controlled(("left/slide_x", "left/slide_y"))
    assert backend.data.ctrl == pytest.approx(before_ctrl)
    assert backend.model.actuator_gainprm == pytest.approx(before_gain)
    assert backend.model.actuator_biasprm == pytest.approx(before_bias)
