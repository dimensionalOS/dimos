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

from dataclasses import replace
from uuid import uuid4

import mujoco
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.g1.sim2 import G1_GROOT
from dimos.sim2.ipc.abi import FrameField, FrameLayout, make_channel_descriptor
from dimos.sim2.ipc.channel import FrameMetadata, RobotChannel
from dimos.sim2.runtime import STATE
from dimos.sim2.sensors.lidar.models.mid360 import Mid360, _FiringPattern
from dimos.sim2.sensors.lidar.module import LidarModule
from dimos.sim2.sensors.lidar.raycast import Raycaster
from dimos.sim2.sensors.reader import WorldReader
from dimos.sim2.sensors.spec import Imu, Lidar
from dimos.sim2.spec import ControlInterface

pytestmark = pytest.mark.mujoco


@pytest.mark.parametrize("self_occlusion,expected", [(False, 4.9), (True, 0.9)])
def test_self_occlusion_uses_mount_geometry_without_hiding_other_robots(self_occlusion, expected):
    model = mujoco.MjModel.from_xml_string("""
        <mujoco><worldbody>
          <body name="robot"><geom type="box" pos="1 0 0" size="0.1 1 1"/></body>
          <body name="other"><geom type="box" pos="5 0 0" size="0.1 1 1"/></body>
        </worldbody></mujoco>
    """)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    query = Raycaster(model, model.body("robot").id, self_occlusion=self_occlusion)
    ranges, normals = query.hits(data, np.zeros(3), np.array([[1.0, 0, 0]]), 10.0)
    assert ranges.tolist() == pytest.approx([expected])
    assert normals[0] == pytest.approx([-1, 0, 0])


@pytest.fixture
def module():
    sensor = next(s for s in G1_GROOT.sensors if isinstance(s, Lidar))
    instance = LidarModule(
        robot_id="g1", root_body="pelvis", sensor=sensor, instance_name=f"test-lidar-{uuid4().hex}"
    )
    try:
        yield instance
    finally:
        instance.stop()


@pytest.mark.parametrize("pitch", [0.0, -0.35, 0.35])
def test_g1_scan_excludes_ceiling_after_mount_rotation(pitch, module, mocker):
    sensor = module.config.sensor
    model = mujoco.MjModel.from_xml_string(f"""
        <mujoco>
          <compiler angle="radian"/>
          <worldbody>
            <geom type="plane" size="10 10 0.1"/>
            <geom type="box" pos="0 0 3" size="10 10 0.1"/>
            <body name="g1/pelvis" pos="0 0 1" euler="0 {pitch} 0">
              <geom type="sphere" size="0.1"/>
              <site name="g1/mid360_link" pos="0.0002835 0.00003 0.41618"
                    quat="0 0.9997985784932998 0 -0.020069938783589012"/>
            </body>
          </worldbody>
        </mujoco>
    """)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    module.reader = mocker.Mock(spec=WorldReader, model=model, data=data, timestamp=123.0)
    publish = mocker.patch.object(module.pointcloud, "publish")
    module.open()

    module.capture()

    publish.assert_called_once()
    cloud = publish.call_args.args[0]
    points = cloud.points().numpy()
    assert len(points) > 1000
    assert points[:, 2] == pytest.approx(np.zeros(len(points)), abs=1e-6)
    assert cloud.frame_id == "world"
    assert cloud.ts == 123.0

    module.config.sensor = replace(sensor, maximum_world_elevation=None)
    module.capture()
    unfiltered = publish.call_args.args[0].points().numpy()
    assert np.count_nonzero(unfiltered[:, 2] > 2.8) > 100


def test_sensor_frame_preserves_ray_origin_and_cloud_timestamp(module, mocker):
    module.config.sensor = replace(module.config.sensor, output_frame="sensor")
    model = mujoco.MjModel.from_xml_string("""
        <mujoco><compiler angle="radian"/><worldbody>
          <geom type="plane" size="10 10 0.1"/>
          <body name="g1/pelvis" pos="2 3 1" euler="0 0.3 0.6">
            <geom type="sphere" size="0.1"/>
            <site name="g1/mid360_link" pos="0.2 0 0.1" euler="3.14159265359 0 0"/>
          </body>
        </worldbody></mujoco>
    """)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    module.reader = mocker.Mock(spec=WorldReader, model=model, data=data, timestamp=456.0)
    publish = mocker.patch.object(module.pointcloud, "publish")
    publish_tf = mocker.patch.object(module.tf, "publish")
    module.open()
    module.capture()
    cloud = publish.call_args.args[0]
    transform = publish_tf.call_args.args[0].transforms[0]
    assert transform.frame_id == "world"
    assert cloud.frame_id == transform.child_frame_id == "g1/lidar"
    assert cloud.ts == transform.ts == 456.0
    origin = np.array(transform.translation.to_tuple())
    assert origin == pytest.approx(data.site_xpos[0])
    rotation = Rotation.from_quat(transform.rotation.to_tuple())
    world_points = rotation.apply(cloud.points().numpy()) + origin
    assert len(world_points) > 1000
    assert world_points[:, 2] == pytest.approx(np.zeros(len(world_points)), abs=1e-5)


@pytest.fixture
def history(tmp_path):
    model = mujoco.MjModel.from_xml_string("""
    <mujoco><option timestep="0.005"/><worldbody>
      <body name="g1/pelvis" gravcomp="1"><freejoint/><geom size="0.1"/>
        <site name="g1/mid360_link"/>
      </body>
      <body name="wall" mocap="true" pos="5 0 0">
        <geom type="box" size="0.1 10 10"/>
      </body>
    </worldbody><sensor>
      <gyro name="g1/sensor/lidar_imu/gyro" site="g1/mid360_link"/>
      <accelerometer name="g1/sensor/lidar_imu/accel" site="g1/mid360_link"/>
    </sensor></mujoco>
    """)
    data = mujoco.MjData(model)
    nstate = mujoco.mj_stateSize(model, STATE)
    desc = replace(
        make_channel_descriptor(
            sim_id="history",
            robot_id="world",
            generation="test",
            shm_name=f"mid360-{uuid4().hex[:16]}",
            control_interface=ControlInterface.WHOLE_BODY,
            dof=1,
            physics_dt=0.005,
            control_decimation=1,
        ),
        observation_layout=FrameLayout(
            (
                FrameField("state", "<f8", (nstate,), 48),
                FrameField("wall_time", "<f8", (1,), 48 + nstate * 8),
            ),
            ((56 + nstate * 8 + 63) // 64) * 64,
        ),
        observation_slots=64,
    )
    path = tmp_path / "world.mjb"
    mujoco.mj_saveModel(model, str(path), None)
    with RobotChannel.create(desc) as writer:
        writer.set_lifecycle("ready")
        reader = WorldReader({"model": str(path), "snapshot": desc.to_dict()})

        def publish(t, x=0, wall_x=5, episode=1, quat=(1, 0, 0, 0), omega=0, wall_jitter=0):
            data.time = t
            data.qpos[:3] = (x, 0, 0)
            data.qpos[3:7] = quat
            data.qvel[5] = omega
            data.mocap_pos[0] = (wall_x, 0, 0)
            state = np.empty(nstate)
            mujoco.mj_getState(model, data, state, STATE)
            writer.set_episode(episode)
            writer.publish_observation(
                {"state": state, "wall_time": [1000 + t + wall_jitter]},
                FrameMetadata(0, episode, round(t / 0.005), 0, t),
            )

        try:
            yield reader, writer, publish
        finally:
            reader.close()
            writer.unlink()


@pytest.fixture
def scanner(history, mocker):
    reader, _, _ = history
    coefs = np.zeros((4, 1, 3))
    coefs[:, 0, 0] = 1
    mocker.patch(
        "dimos.sim2.sensors.lidar.models.mid360._pattern",
        return_value=_FiringPattern(0, 0, 0, 0, coefs),
    )
    module = LidarModule(
        robot_id="g1",
        root_body="pelvis",
        instance_name=f"rolling-{uuid4().hex}",
        sensor=Lidar(
            "lidar",
            "mid360_link",
            Mid360,
            model_kwargs={"downsample": 1000, "noise": False, "dropout": False},
        ),
    )
    module.reader = reader
    raw = mocker.patch.object(module.raw_pointcloud, "publish")
    corrected = mocker.patch.object(module.pointcloud, "publish")
    mocker.patch.object(module.tf, "publish")
    module.open()
    try:
        yield module, raw, corrected
    finally:
        module.stop()


def test_bounded_history_retains_order_after_wrap(history):
    reader, writer, publish = history
    for tick in range(100):
        publish(tick * 0.005, x=tick)
    frames = reader.channel.read_observations()
    assert len(frames) == 63
    assert [f.metadata.sequence for f in frames] == list(range(38, 101))
    assert frames[0].metadata.sim_time == pytest.approx(37 * 0.005)
    assert frames[-1].metadata.sim_time == pytest.approx(99 * 0.005)
    assert writer.read_observation().metadata.sequence == 100


def test_lidar_imu_preserves_acquisition_clock_units_and_backlog(history, scanner, mocker):
    reader, _, publish = history
    module, raw, _ = scanner
    module.config.sensor = replace(module.config.sensor, imu=Imu("lidar_imu", "mid360_link"))
    module.open()
    imu_output = mocker.patch.object(module.imu_raw, "publish")
    publish(0, omega=0.2, quat=(0, 1, 0, 0))
    assert reader.update()
    module.capture()
    for tick in range(1, 21):
        publish(tick * 0.005, omega=0.2, quat=(0, 1, 0, 0), wall_jitter=0.02)
    assert reader.update()
    module.capture()
    samples = [call.args[0] for call in imu_output.call_args_list]
    assert len(samples) == 21
    assert np.diff([sample.ts for sample in samples]) == pytest.approx(np.full(20, 0.005))
    assert samples[-1].linear_acceleration.to_tuple() == pytest.approx((0, 0, -9.81))
    assert samples[-1].angular_velocity.to_tuple() == pytest.approx((0, 0, 0.2))
    assert samples[-1].orientation_covariance[0] == -1
    assert samples[-1].frame_id == "g1/lidar_imu"
    assert raw.call_args.args[0].ts == samples[0].ts
    assert module.sensor_status()["dropped_imu_samples"] == 0


def test_estimator_device_does_not_publish_corrected_cloud_or_truth_tf(history, scanner, mocker):
    reader, _, publish = history
    module, raw, corrected = scanner
    module.config.sensor = replace(module.config.sensor, truth_outputs=False)
    truth_tf = mocker.patch.object(module.tf, "publish")
    for tick in range(21):
        publish(tick * 0.005)
    assert reader.update()
    module.capture()
    raw.assert_called_once()
    corrected.assert_not_called()
    truth_tf.assert_not_called()


def test_history_interpolates_freejoint_rotation_and_moving_scene(history):
    reader, _, publish = history
    publish(0, x=0, wall_x=5)
    publish(0.005, x=2, wall_x=6, quat=(0, 0, 0, 1))
    assert reader.update() and reader.history(0, 0.005)
    reader.restore_at(0.0025)
    assert reader.data.qpos[:3] == pytest.approx([1, 0, 0])
    assert np.abs(reader.data.qpos[3:7]) == pytest.approx([np.sqrt(0.5), 0, 0, np.sqrt(0.5)])
    assert reader.data.mocap_pos[0] == pytest.approx([5.5, 0, 0])
    assert reader.timestamp == pytest.approx(1000.0025)


def test_rolling_scan_keeps_motion_in_raw_cloud_and_deskews_mapping_cloud(history, scanner):
    reader, _, publish = history
    module, raw, corrected = scanner
    for tick in range(21):
        publish(tick * 0.005, x=2 * tick * 0.005)
    assert reader.update()
    module.capture()
    cloud = raw.call_args.args[0]
    mapped = corrected.call_args.args[0]
    assert np.ptp(cloud.points().numpy()[:, 0]) > 0.15
    assert mapped.points().numpy()[:, 0] == pytest.approx(4.9, abs=1e-6)
    assert cloud.ts == pytest.approx(1000)
    assert mapped.ts == pytest.approx(1000.1)
    assert cloud.frame_id == "g1/lidar" and mapped.frame_id == "world"
    decoded = PointCloud2.lcm_decode(cloud.lcm_encode())
    np.testing.assert_array_equal(decoded.offset_times_u32(), cloud.offset_times_u32())
    np.testing.assert_array_equal(decoded.lines_u8(), np.tile(np.arange(4), 5))
    assert decoded.offset_times_u32()[-1] == 80_015_000
    assert module.sensor_status()["scans"] == 1


def test_moving_geometry_is_raycast_at_capture_time(history, scanner):
    reader, _, publish = history
    module, _, corrected = scanner
    for tick in range(21):
        publish(tick * 0.005, wall_x=5 + tick * 0.005)
    assert reader.update()
    module.capture()
    points = corrected.call_args.args[0].points().numpy()
    assert np.ptp(points[:, 0]) > 0.079
    assert points[:, 0].max() < 5.0


def test_reset_does_not_mix_history_or_repeat_previous_scan(history, scanner):
    reader, _, publish = history
    module, raw, _ = scanner
    for tick in range(21):
        publish(tick * 0.005)
    assert reader.update()
    module.capture()
    first = raw.call_args.args[0]
    publish(0.005, episode=2)
    assert reader.update()
    module.capture()
    assert raw.call_count == 1
    for tick in range(1, 23):
        publish(tick * 0.005, episode=2)
    assert reader.update()
    module.capture()
    assert raw.call_count == 1  # No initial state at zero in the new episode.
    assert module.sensor_status()["history_pending"]
    for tick in range(23, 41):
        publish(tick * 0.005, episode=2)
    assert reader.update()
    module.capture()
    assert raw.call_count == 2
    np.testing.assert_array_equal(
        first.offset_times_u32(), raw.call_args.args[0].offset_times_u32()
    )
    assert module.sensor_status()["episode"] == 2


def test_history_gap_is_not_replaced_with_instantaneous_scan(history, scanner):
    reader, _, publish = history
    module, raw, _ = scanner
    publish(0)
    publish(0.1)
    assert reader.update()
    module.capture()
    raw.assert_not_called()
    assert module.sensor_status()["history_pending"]
