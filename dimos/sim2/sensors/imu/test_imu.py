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
import pytest

from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.sim2.sensors.imu.module import ImuConfig, SimImuModule


@pytest.fixture
def imu():
    module = SimImuModule(
        sensor=ImuConfig("left/imu", gyro_bias=(0.1, 0.2, 0.3), accel_std=0.02, seed=4)
    )
    yield module
    module.dispose()


def test_noise_is_acquired_once_and_metadata_survives_codec(poses, imu):
    truth = poses()
    a = imu.measure(truth)
    b = imu.measure(truth)
    assert a is b
    assert a.angular_velocity.x == pytest.approx(0.1)
    restored = Imu.lcm_decode(a.lcm_encode())
    assert restored.sample_header.clock_id == "test"
    assert restored.sample_header.time_ns == 0
    assert restored.linear_acceleration.z == a.linear_acceleration.z
    assert restored.orientation_covariance[0] == -1
    truth.header = SampleHeader(clock_id="test", epoch=1, frame_id="left/imu")
    c = imu.measure(truth)
    assert c.sample_header.epoch == 1
    assert c.linear_acceleration.z != a.linear_acceleration.z


def test_wrong_frame_cannot_feed_imu(poses, imu):
    truth = poses()
    truth.body_motion[0].frame_id = "right/imu"
    with pytest.raises(ValueError, match="lacks configured IMU motion"):
        imu.measure(truth)


@pytest.fixture
def ideal_imu():
    module = SimImuModule(sensor=ImuConfig(frame_id="left/imu", orientation="ideal"))
    yield module
    module.dispose()


def test_stationary_motion_is_kinematic_and_imu_adds_specific_force(backend, poses, ideal_imu):
    motion = backend.read_motion("left/imu")
    assert motion.linear_acceleration_world == pytest.approx([0, 0, 0], abs=1e-9)
    sample = ideal_imu.measure(poses())
    assert sample is not None
    assert sample.linear_acceleration.z == pytest.approx(9.81)
    assert backend.model.nsensor == 0
