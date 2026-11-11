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

from dimos_lcm.common_msgs import SampleHeader
from dimos_lcm.geometry_msgs import Quaternion
import pytest
from scipy.spatial.transform import Rotation

from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.sim2.sensors.imu.module import ImuConfig, SimImuModule


@pytest.fixture
def imu(world, mocker):
    module = SimImuModule(
        sensor=ImuConfig("left/imu", gyro_bias=(0.1, 0.2, 0.3), accel_std=0.02, seed=4)
    )
    client = mocker.patch("dimos.sim2.sensors.imu.module.RPCClient.remote").return_value
    client.simulation_config.return_value = {"sim_gravity_world": (0, 0, -9.81)}
    module.build()
    yield module
    module.dispose()


def test_noise_is_acquired_once_and_timestamp_survives_codec(poses, imu):
    truth = poses()
    a = imu.measure(truth)
    b = imu.measure(truth)
    assert a is b
    assert a.angular_velocity.x == pytest.approx(0.1)
    restored = Imu.lcm_decode(a.lcm_encode())
    assert restored.ts == 0
    assert restored.frame_id == "left/imu"
    assert restored.linear_acceleration.z == a.linear_acceleration.z
    assert restored.orientation_covariance[0] == -1
    truth.header = SampleHeader(clock_id="test", epoch=1, frame_id="left/imu")
    c = imu.measure(truth)
    assert c.ts == 0
    assert c.linear_acceleration.z != a.linear_acceleration.z


def test_wrong_frame_cannot_feed_imu(poses, imu):
    truth = poses()
    next(b for b in truth.bodies if b.body_id == "left/imu").body_id = "other/imu"
    with pytest.raises(ValueError, match="lacks configured IMU body"):
        imu.measure(truth)


@pytest.fixture
def ideal_imu(world, mocker):
    module = SimImuModule(sensor=ImuConfig(frame_id="left/imu", orientation="ideal"))
    client = mocker.patch("dimos.sim2.sensors.imu.module.RPCClient.remote").return_value
    client.simulation_config.return_value = {"sim_gravity_world": (0, 0, -9.81)}
    module.build()
    yield module
    module.dispose()


def test_stationary_motion_is_kinematic_and_imu_adds_specific_force(backend, poses, ideal_imu):
    body = next(b for b in backend.read_bodies() if b.body_id == "left/imu")
    a = body.acceleration.linear
    assert [a.x, a.y, a.z] == pytest.approx([0, 0, 0], abs=1e-9)
    sample = ideal_imu.measure(poses())
    assert sample is not None
    assert sample.linear_acceleration.z == pytest.approx(9.81)
    assert backend.model.nsensor == 0


@pytest.mark.parametrize("gravity", [(0.0, 0.0, -1.62), (0.0, 0.0, 0.0)])
def test_configured_gravity_is_rotated_into_sensor_axes(poses, ideal_imu, gravity, mocker):
    client = mocker.patch("dimos.sim2.sensors.imu.module.RPCClient.remote").return_value
    client.simulation_config.return_value = {"sim_gravity_world": gravity}
    ideal_imu.build()
    truth = poses()
    body = next(b for b in truth.bodies if b.body_id == "left/imu")
    body.pose.orientation = Quaternion(*Rotation.from_euler("y", 90, degrees=True).as_quat())
    measured = ideal_imu.measure(truth)
    assert measured is not None
    a = measured.linear_acceleration
    assert [a.x, a.y, a.z] == pytest.approx([gravity[2], 0, 0], abs=1e-9)


@pytest.mark.parametrize("gravity", [(0, 0), (0, 0, float("nan"))])
def test_invalid_gravity_fails_build(ideal_imu, gravity, mocker):
    client = mocker.patch("dimos.sim2.sensors.imu.module.RPCClient.remote").return_value
    client.simulation_config.return_value = {"sim_gravity_world": gravity}
    with pytest.raises(ValueError, match="finite 3-vector"):
        ideal_imu.build()
    client.stop_rpc_client.assert_called_once()
