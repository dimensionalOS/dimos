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


def test_noise_is_acquired_once_and_metadata_survives_codec(backend, imu):
    truth = backend.read_motion("left/imu")
    truth.header = SampleHeader(clock_id="test", frame_id="left/imu")
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


def test_wrong_frame_cannot_feed_imu(backend, imu):
    truth = backend.read_motion("right/imu")
    truth.header = SampleHeader(clock_id="test", frame_id="right/imu")
    assert imu.measure(truth) is None
