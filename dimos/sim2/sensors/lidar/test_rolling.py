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

"""A rolling lidar integrates the snapshot history: motion stays in the raw scan."""

from contextlib import closing
from dataclasses import replace
from uuid import uuid4

import mujoco
import numpy as np
import pytest

from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.sim2.backends.mujoco import STATE
from dimos.sim2.messages import FrameMetadata, StateFrame
from dimos.sim2.sensors.imu.spec import Imu
from dimos.sim2.sensors.lidar.device import LidarDevice
from dimos.sim2.sensors.lidar.models.mid360.model import Mid360
from dimos.sim2.sensors.lidar.models.mid360.pattern import Mid360Pattern
from dimos.sim2.sensors.lidar.module import LidarModule
from dimos.sim2.sensors.lidar.spec import Lidar
from dimos.sim2.sensors.reader import WorldReader


@pytest.fixture
def history(tmp_path):
    """A hand-stepped world: a sensor body sliding toward a wall, published as snapshots."""
    timestep = 0.005
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
    path = tmp_path / "world.mjb"
    mujoco.mj_saveModel(model, str(path), None)
    reader = WorldReader(
        {"model": str(path), "physics_dt": timestep, "snapshot_stride": 1, "snapshot_slots": 64}
    )
    sequence = [0]

    def publish(t, x=0, wall_x=5, episode=1, quat=(1, 0, 0, 0), omega=0):
        data.time = t
        data.qpos[:3] = (x, 0, 0)
        data.qpos[3:7] = quat
        data.qvel[5] = omega
        data.mocap_pos[0] = (wall_x, 0, 0)
        state = np.empty(nstate)
        mujoco.mj_getState(model, data, state, STATE)
        sequence[0] += 1
        reader.push(
            StateFrame(
                FrameMetadata(sequence[0], episode, round(t / timestep), t),
                {"state": state, "wall_time": np.array([1000 + t])},
            )
        )

    return reader, publish


@pytest.fixture
def scanner(history, mocker):
    reader, _ = history
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
    raw = mocker.patch.object(module.lidar, "publish")
    corrected = mocker.patch.object(module.pointcloud, "publish")
    mocker.patch.object(module.tf, "publish")
    module.open()
    try:
        yield module, raw, corrected
    finally:
        module.stop()


def test_rolling_scan_keeps_motion_in_the_raw_scan_and_deskews_the_truth_cloud(history, scanner):
    reader, publish = history
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


def test_rolling_device_reports_the_scan_interval_and_delivery_delay(history, scanner):
    reader, publish = history
    module, _, _ = scanner
    device = LidarDevice(reader, replace(module.config.sensor, latency=0.025), "g1", "pelvis")
    try:
        for tick in range(21):
            publish(tick * 0.005)
        assert device.poll() == []
        for tick in range(21, 26):
            publish(tick * 0.005)
        results = device.poll()
        raw = next(value for value in results if value.port == "lidar")
        assert (raw.acquisition_start, raw.acquisition_end, raw.available_at) == pytest.approx(
            (0, 0.1, 0.125)
        )
        assert raw.value.ts == pytest.approx(1000)
        assert raw.value.offset_times_u32()[-1] == 80_015_000
    finally:
        device.close()


def test_driver_frames_survive_the_robot_and_sensor_binding_names(history, scanner):
    reader, publish = history
    module, _, _ = scanner
    sensor = replace(
        module.config.sensor,
        frame_id="mid360_link",
        imu_frame_id="imu_link",
        imu=Imu("lidar_imu", "mid360_link"),
        truth_outputs=False,
    )
    with closing(LidarDevice(reader, sensor, "g1", "pelvis")) as device:
        for tick in range(21):
            publish(tick * 0.005)
        measurements = device.poll()

    cloud = next(value.value for value in measurements if value.port == "lidar")
    imu = [value.value for value in measurements if value.port == "imu"]
    assert cloud.frame_id == "mid360_link"
    assert cloud.points().numpy()[:, 0] == pytest.approx(4.9, abs=1e-6)
    assert imu and {sample.frame_id for sample in imu} == {"imu_link"}
    assert {measurement.port for measurement in measurements} == {"lidar", "imu"}


def test_history_gap_drops_the_scan_instead_of_faking_an_instant_one(history, scanner):
    reader, publish = history
    module, raw, _ = scanner
    for tick in list(range(10)) + list(range(15, 21)):
        publish(tick * 0.005)
    assert reader.update()
    module.capture()
    raw.assert_not_called()
    status = module.sensor_status()
    assert status["history_pending"] and status["scans"] == 0
