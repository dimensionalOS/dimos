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

from contextlib import ExitStack
from dataclasses import replace
import json
import os
from pathlib import Path
import threading
import time
from uuid import uuid4

import mujoco
import numpy as np
import pytest

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.transport import ZenohTransport
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu as ImuMessage
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.sim2.blueprint import simulation
from dimos.sim2.client import SimRobotClient
from dimos.sim2.messages import FrameMetadata, StreamDescriptor
from dimos.sim2.module import SimulationModule
from dimos.sim2.sensors.camera.spec import Camera
from dimos.sim2.sensors.imu.spec import Imu
from dimos.sim2.sensors.lidar.device import LidarDevice
from dimos.sim2.sensors.lidar.models.mid360.model import Mid360
from dimos.sim2.sensors.lidar.models.mid360.pattern import Mid360Pattern
from dimos.sim2.sensors.lidar.module import LidarModule
from dimos.sim2.sensors.lidar.spec import Lidar
from dimos.sim2.sensors.reader import WorldReader
from dimos.sim2.spec import Joint, RobotConfig, RobotInstance, WorldConfig
from dimos.sim2.streams import SimStream
from dimos.sim2.world.runtime import STATE


def test_return_noise_is_seeded_per_scan_and_rejects_grazing_misses():
    model = Mid360(seed=7)
    ranges = np.full(1000, 10.0)
    normal = np.ones(1000)
    noisy = model.measure(ranges, normal, 0.2)
    assert np.std(noisy - ranges) > 0.003
    np.testing.assert_array_equal(noisy, model.measure(ranges, normal, 0.2))
    assert not np.array_equal(noisy, model.measure(ranges, normal, 0.3))
    assert np.all(model.measure(ranges, np.zeros(1000), 0.2) == -1)
    assert model.measure(np.array([-1.0, 0.1, 41.0]), np.ones(3), 0).tolist() == [-1, -1, -1]


def test_noise_free_diagnostics_keep_the_same_valid_ranges():
    model = Mid360(noise=False, dropout=False)
    assert model.measure(np.array([-1.0, 0.2, 10]), np.ones(3), 0).tolist() == [-1, 0.2, 10]


def test_fourier_evaluation_matches_real_sine_cosine_series_across_channels():
    coefs = np.random.default_rng(7).normal(size=(4, 9, 3))
    pattern = Mid360Pattern(3.25, 1.1, 1, 1, coefs)
    indices = np.array([0, 1, 2, 3, 41, 802, 800_003], dtype=np.int64)
    expected = []
    for index in indices:
        fast, slow = 2 * np.pi * (index // 4 * 4 / 200_000) * np.array([3.25, 1.1])
        phases = np.array([slow, fast - slow, fast, fast + slow])
        basis = np.r_[1, np.column_stack((np.cos(phases), np.sin(phases))).ravel()]
        vector = basis @ coefs[index % 4]
        expected.append(vector / np.linalg.norm(vector))
    np.testing.assert_allclose(
        np.concatenate([pattern.directions(int(index), 1) for index in indices]),
        expected,
        atol=1e-12,
    )


@pytest.fixture
def pattern(mocker):
    coefs = np.zeros((4, 3, 3))
    coefs[:, 1] = [[1, 0, 0], [0, 1, 0], [-1, 0, 0], [0, -1, 0]]
    coefs[:, 2] = [[0, 1, 0], [-1, 0, 0], [0, -1, 0], [1, 0, 0]]
    mocker.patch(
        "dimos.sim2.sensors.lidar.models.mid360.model._pattern",
        return_value=Mid360Pattern(2.3, 0, 1, 0, coefs),
    )
    return Mid360()


def test_scan_preserves_channels_timing_and_continuous_absolute_phase(pattern):
    first = pattern.scan(0, 0.1)
    second = pattern.scan(0.1, 0.1)
    assert len(first.offsets) == 20_000
    assert first.offsets[[0, 1, -1]] == pytest.approx([0, 0.000005, 0.099995])
    assert first.lines[:8].tolist() == [0, 1, 2, 3, 0, 1, 2, 3]
    np.testing.assert_allclose(
        first.directions[:4], [[1, 0, 0], [0, 1, 0], [-1, 0, 0], [0, -1, 0]], atol=1e-12
    )
    np.testing.assert_allclose(
        second.directions[0], [np.cos(2 * np.pi * 0.23), np.sin(2 * np.pi * 0.23), 0], atol=1e-12
    )
    np.testing.assert_allclose(
        pattern.scan(0, 0.2).directions, np.concatenate([first.directions, second.directions])
    )
    assert not np.allclose(pattern.scan(4, 0.1).directions, first.directions)
    np.testing.assert_allclose(np.linalg.norm(first.directions, axis=1), 1)


def test_downsampling_keeps_four_lasers_and_original_time_offsets(pattern):
    rays = Mid360(downsample=8).scan(0, 0.1)
    assert len(rays.offsets) == 2500
    assert rays.lines[:8].tolist() == [0, 1, 2, 3, 0, 1, 2, 3]
    assert rays.offsets[4] == 32 / 200_000
    assert rays.offsets[-1] > 0.099
    np.testing.assert_allclose(
        rays.directions, pattern.scan(0, 0.1).directions.reshape(-1, 4, 3)[::8].reshape(-1, 3)
    )


@pytest.mark.parametrize("duration", [0, -0.1, 0.100001, 0.000005, float("nan")])
def test_scan_rejects_partial_channel_groups(pattern, duration):
    with pytest.raises(ValueError, match="four-laser"):
        pattern.scan(0, duration)


@pytest.mark.parametrize("start", [-1, float("inf"), float("nan")])
def test_scan_rejects_invalid_start(pattern, start):
    with pytest.raises(ValueError, match="start"):
        pattern.scan(start, 0.1)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"point_rate_hz": 100_000},
        {"downsample": 1.5},
        {"downsample": 0},
        {"motion_sample_rate_hz": 0},
        {"max_range": 0},
        {"min_range": float("nan")},
    ],
)
def test_invalid_sensor_parameters_are_rejected(kwargs):
    with pytest.raises(ValueError):
        Mid360(**kwargs)


@pytest.fixture
def history(tmp_path, request):
    timestep = getattr(request, "param", 0.005)
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
    model.opt.timestep = timestep
    data = mujoco.MjData(model)
    nstate = mujoco.mj_stateSize(model, STATE)
    desc = StreamDescriptor(
        sim_id="history",
        robot_id="world",
        generation=uuid4().hex,
        physics_dt=timestep,
        observation_slots=64,
    )
    path = tmp_path / "world.mjb"
    mujoco.mj_saveModel(model, str(path), None)
    with ExitStack() as stack:
        writer = SimStream.create(desc)
        stack.callback(writer.close)
        writer.set_lifecycle("ready")
        reader = WorldReader({"model": str(path), "snapshot": desc.to_dict()})
        stack.callback(reader.close)

        def publish(t, x=0, wall_x=5, episode=1, quat=(1, 0, 0, 0), omega=0, wall_jitter=0):
            data.time = t
            data.qpos[:3] = (x, 0, 0)
            data.qpos[3:7] = quat
            data.qvel[5] = omega
            data.mocap_pos[0] = (wall_x, 0, 0)
            state = np.empty(nstate)
            mujoco.mj_getState(model, data, state, STATE)
            if writer.episode_id != episode:
                writer.set_episode(episode)
            writer.publish_observation(
                {"state": state, "wall_time": [1000 + t + wall_jitter]},
                FrameMetadata(0, episode, round(t / timestep), 0, t),
            )

        yield reader, writer, publish


@pytest.fixture
def scanner(history, mocker):
    reader, _, _ = history
    coefs = np.zeros((4, 1, 3))
    coefs[:, 0, 0] = 1
    mocker.patch(
        "dimos.sim2.sensors.lidar.models.mid360.model._pattern",
        return_value=Mid360Pattern(0, 0, 0, 0, coefs),
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
    assert len(frames) == 64
    assert [f.metadata.sequence for f in frames] == list(range(37, 101))
    assert frames[0].metadata.sim_time == pytest.approx(36 * 0.005)
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
        publish(tick * 0.005, omega=0.2, quat=(0, 1, 0, 0))
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
    module.open()
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


def test_rolling_device_returns_intervals_and_delay_without_a_module(history, scanner):
    reader, _, publish = history
    module, _, _ = scanner
    device = LidarDevice(reader, replace(module.config.sensor, latency=0.025), "g1", "pelvis")
    try:
        for tick in range(21):
            publish(tick * 0.005)
        assert device.poll() == []
        for tick in range(21, 26):
            publish(tick * 0.005)
        results = device.poll()
        raw = next(value for value in results if value.port == "raw_pointcloud")
        assert (raw.acquisition_start, raw.acquisition_end, raw.available_at) == pytest.approx(
            (0, 0.1, 0.125)
        )
        assert raw.value.ts == pytest.approx(1000)
        assert raw.value.offset_times_u32()[-1] == 80_015_000
    finally:
        device.close()


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


@pytest.mark.parametrize("rate", [50, 100, 200])
def test_imu_decimation_restarts_after_reset_without_old_samples(history, scanner, mocker, rate):
    reader, _, publish = history
    module, _, _ = scanner
    module.config.sensor = replace(
        module.config.sensor, imu=Imu("lidar_imu", "mid360_link", rate_hz=rate)
    )
    module.open()
    output = mocker.patch.object(module.imu_raw, "publish")
    for episode in (1, 2):
        output.reset_mock()
        for tick in range(21):
            publish(tick * 0.005, episode=episode, wall_jitter=10 * episode)
            assert reader.update()
            module.capture()
        samples = [call.args[0] for call in output.call_args_list]
        assert len(samples) == round(0.1 * rate) + 1
        assert [s.ts for s in samples] == pytest.approx(
            1000 + 10 * episode + np.arange(len(samples)) / rate
        )
        assert module.sensor_status()["dropped_imu_samples"] == 0


@pytest.mark.parametrize("history", [0.002], indirect=True)
def test_200hz_imu_samples_500hz_physics_without_losing_every_other_sample(
    history,
    scanner,
    mocker,
):
    reader, _, publish = history
    module, _, _ = scanner
    module.config.sensor = replace(module.config.sensor, imu=Imu("lidar_imu", "mid360_link"))
    module.open()
    output = mocker.patch.object(module.imu_raw, "publish")
    for tick in range(51):
        publish(tick * 0.002, omega=0.2)
        assert reader.update()
        module.capture()
    samples = [call.args[0] for call in output.call_args_list]
    assert len(samples) == 21
    assert [s.ts for s in samples] == pytest.approx(1000 + np.arange(21) * 0.005)
    assert [s.angular_velocity.z for s in samples] == pytest.approx([0.2] * 21)
    assert module.sensor_status()["dropped_imu_samples"] == 0


@pytest.mark.self_hosted
@pytest.mark.parametrize("calibration", ["g1", "go2"])
def test_installed_calibration_is_continuous_and_covers_full_sensor_field(calibration):
    pattern = Mid360(noise=False, dropout=False, calibration=calibration)
    first = pattern.scan(0, 0.1)
    second = pattern.scan(0.1, 0.1)
    entire = pattern.scan(0, 0.2)
    np.testing.assert_allclose(
        entire.directions, np.concatenate([first.directions, second.directions]), atol=1e-12
    )
    np.testing.assert_allclose(np.linalg.norm(entire.directions, axis=1), 1)
    elevations = np.degrees(np.arcsin(entire.directions[:, 2]))
    assert elevations.min() < -5 and elevations.max() > 50
    azimuths = np.arctan2(entire.directions[:, 1], entire.directions[:, 0])
    assert np.ptp(azimuths) > 6.2


def test_resuming_physics_reanchors_rolling_acquisition_clock(
    worlds, world_config, tmp_path, pattern, mocker
):
    clock = mocker.patch("dimos.sim2.world.runtime.time.time", return_value=1000.0)
    sensor = Lidar(
        "lidar",
        "scan",
        Mid360,
        imu=Imu("lidar_imu", "scan"),
        model_kwargs={"downsample": 1000, "noise": False, "dropout": False},
    )
    instance = world_config.robots["left"]
    robot = replace(instance.config, sensors=(instance.config.sensors[0], sensor))
    world = worlds(config=replace(world_config, robots={"left": replace(instance, config=robot)}))
    model_path = tmp_path / "pause.mjb"
    mujoco.mj_saveModel(world.model, str(model_path), None)
    with ExitStack() as stack:
        reader = WorldReader(
            {"model": str(model_path), "snapshot": world.snapshot_descriptor.to_dict()}
        )
        stack.callback(reader.close)
        module = LidarModule(robot_id="left", root_body="base", sensor=sensor)
        stack.callback(module.stop)
        module.reader = reader
        imu_output = mocker.patch.object(module.imu_raw, "publish")
        raw_output = mocker.patch.object(module.raw_pointcloud, "publish")
        mocker.patch.object(module.pointcloud, "publish")
        mocker.patch.object(module.tf, "publish")
        module.open()
        assert reader.update()
        module.capture()
        for tick in range(1, 21):
            clock.return_value = 1000 + tick * 0.005
            world.step()
        assert reader.update()
        module.capture()
        assert imu_output.call_args.args[0].ts == pytest.approx(1000.1)
        assert raw_output.call_count == 1
        world.set_paused(True)
        clock.return_value = 1010.1

        world.set_paused(False)
        assert reader.update()
        module.capture()

        assert imu_output.call_args.args[0].ts == pytest.approx(1010.1)
        assert raw_output.call_count == 1
        assert module.sensor_status()["history_pending"]
        assert module.sensor_status()["dropped_imu_samples"] == 0


@pytest.mark.native_e2e
@pytest.mark.parametrize("with_camera", [False, True])
def test_pointlio_tracks_a_driven_stage_using_only_raw_lidar_and_imu(
    tmp_path, record_property, with_camera
):
    binary = Path(
        os.environ.get(
            "SIM2_POINTLIO_BINARY",
            str(DIMOS_PROJECT_ROOT / "target/release/pointlio_native"),
        )
    ).resolve()
    assert binary.is_file(), "build current main's pointlio_native or set SIM2_POINTLIO_BINARY"
    scene = tmp_path / "room.xml"
    scene.write_text("""
    <mujoco><worldbody>
      <geom type="plane" size="8 8 .1"/>
      <geom type="box" pos="3 0 2" size=".1 3 2"/>
      <geom type="box" pos="-3 0 2" size=".1 3 2"/>
      <geom type="box" pos="0 3 2" size="3 .1 2"/>
      <geom type="box" pos="0 -3 2" size="3 .1 2"/>
      <geom type="box" pos="0 0 4" size="3 3 .1"/>
      <geom type="box" pos="1.5 1 .6" size=".3 .5 .6" euler="0 0 30"/>
    </worldbody></mujoco>
    """)
    model = tmp_path / "stage.xml"
    model.write_text("""
    <mujoco><worldbody><body name="base"><body name="stage">
      <joint name="travel" type="slide" axis="1 0 0" damping="1"/>
      <geom type="box" size=".05 .05 .05" mass="1"/>
      <site name="laser" pos="0 0 .1"/>
      <camera name="view" pos="1 -1 1" xyaxes=".707 .707 0 -.408 .408 .816"/>
    </body></body></worldbody>
    <actuator><motor name="drive" joint="travel" ctrlrange="-100 100"/></actuator>
    </mujoco>
    """)
    robot = RobotConfig(
        model=model,
        root_body="base",
        joints=(Joint("travel", "travel", "drive", kp=500, kd=40),),
        sensors=(
            Lidar(
                "lidar",
                "laser",
                Mid360,
                imu=Imu("lidar_imu", "laser"),
                truth_outputs=False,
            ),
        ),
    )
    if with_camera:
        robot = robot.with_sensor(Camera("camera", "view", width=640, height=480, rate_hz=30))
    estimates, imu_times, raw_times = [], [], []
    camera_ages = []
    ready = threading.Event()
    tick_ready = threading.Event()

    def on_odom(sample):
        estimates.append(sample)
        if len(estimates) >= 5:
            ready.set()

    def on_imu(sample):
        imu_times.append(sample.ts)
        tick_ready.set()

    with ExitStack() as stack:
        transports = {}
        for topic, message, callback in (
            ("odometry", Odometry, on_odom),
            ("imu_raw", ImuMessage, on_imu),
            ("lidar_raw", PointCloud2, lambda sample: raw_times.append(sample.ts)),
            ("color_image", Image, lambda sample: camera_ages.append(time.time() - sample.ts)),
        ):
            transport = ZenohTransport(f"mid360-test/{uuid4().hex}/{topic}", message)
            stack.callback(transport.stop)
            stack.callback(transport.subscribe(callback))
            transports[topic, message] = transport
        blueprint = autoconnect(
            simulation(world=WorldConfig(scene, {"stage": RobotInstance(robot, xyz=(0, 0, 1))})),
            PointLio.blueprint(
                source_dir=None,
                build_command=None,
                executable=str(binary),
                sensor_frame_id="stage/lidar",
                extrinsic_t=[0.0, 0.0, 0.0],
                extrinsic_r=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            ),
        ).remappings(
            [
                ("stage_lidar", "raw_pointcloud", "lidar_raw"),
                ("stage_lidar", "imu_raw", "imu_raw"),
                ("stage_camera", "color_image", "color_image"),
            ]
        )
        started = time.monotonic()
        coordinator = ModuleCoordinator.build(
            blueprint.transports(transports).global_config(
                n_workers=1, viewer="none", transport="zenoh"
            )
        )
        stack.callback(coordinator.stop)
        startup = time.monotonic() - started
        sim = coordinator.get_instance(SimulationModule)
        lidar = coordinator.get_instance("stage_lidar")
        assert ready.wait(30), f"Point-LIO did not initialize: {lidar.sensor_status()}"
        descriptor = StreamDescriptor.from_dict(sim.describe()["robots"]["stage"])
        client = stack.enter_context(SimRobotClient(descriptor, writable=True))
        before = sim.status()
        first_imu, first_scan, first_image = len(imu_times), len(raw_times), len(camera_ages)
        started = time.monotonic()
        truth = []
        last_write = -1.0
        while time.monotonic() - started < 10:
            assert tick_ready.wait(2), "simulated IMU stopped"
            tick_ready.clear()
            observed = client.read()
            if observed.metadata.sim_time - last_write < 0.02:
                continue
            last_write = observed.metadata.sim_time
            elapsed = time.monotonic() - started
            target = min(0.6, max(0.0, elapsed - 1) * 0.1)
            client.write_positions([target], observed=observed)
            status = lidar.sensor_status()
            pose = status.get("truth_pose")
            if pose is not None and (not truth or pose["timestamp"] > truth[-1][0]):
                truth.append((pose["timestamp"], np.asarray(pose["position"])))
        elapsed = time.monotonic() - started
        after = sim.status()
        status = lidar.sensor_status()
        samples = tuple(estimates)
        errors = []
        offset = None
        for stamp, position in truth:
            sample = min(samples, key=lambda value: abs(value.ts - stamp))
            if abs(sample.ts - stamp) > 0.035:
                continue
            estimate = np.asarray(sample.position.to_tuple())
            if offset is None:
                offset = position - estimate
            errors.append(float(np.linalg.norm(estimate + offset - position)))
        evidence = {
            "startup_seconds": startup,
            "seconds": elapsed,
            "realtime_factor": (after["sim_time"] - before["sim_time"]) / elapsed,
            "scan_hz": (len(raw_times) - first_scan) / elapsed,
            "imu_hz": (len(imu_times) - first_imu) / elapsed,
            "camera_hz": (len(camera_ages) - first_image) / elapsed,
            "camera_age_p99_ms": float(np.percentile(camera_ages[first_image:], 99) * 1000)
            if len(camera_ages) > first_image
            else None,
            "matched_poses": len(errors),
            "position_rms_m": float(np.sqrt(np.mean(np.square(errors)))) if errors else None,
            "position_max_m": max(errors) if errors else None,
            "displacement_m": float(np.linalg.norm(truth[-1][1] - truth[0][1])) if truth else 0,
            "sensor": status,
        }
        record_property("sim2_mid360_evidence", json.dumps(evidence))
        assert status["error"] is None and len(errors) > 50, evidence
        assert evidence["scan_hz"] > 7 and evidence["imu_hz"] > 150, evidence
        assert evidence["displacement_m"] > 0.5, evidence
        assert max(errors) < 0.1, evidence
        assert status["dropped_scans"] == status["dropped_imu_samples"] == 0, evidence
        if with_camera:
            assert evidence["camera_hz"] > 25, evidence
