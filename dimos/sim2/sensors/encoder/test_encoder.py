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

import pytest

from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.sim2.sensors.encoder.module import EncoderConfig, SimJointEncoderModule

pytestmark = pytest.mark.self_hosted  # Joint truth is acquired from a real USD physics scene.


@pytest.fixture
def encoder():
    module = SimJointEncoderModule(
        sensor=EncoderConfig(("left/slide_x",), offset=0.013, resolution=0.01)
    )
    yield module
    module.dispose()


def test_encoder_calibration_is_used_and_effort_is_not_fabricated(poses, encoder):
    truth = poses()
    measured = encoder.measure(truth)
    assert measured.position == pytest.approx([0.01])
    assert measured.effort == []
    restored = JointState.lcm_decode(measured.lcm_encode())
    assert restored.ts == 0
    assert restored.position == measured.position
    assert encoder.measure(truth) is measured


async def test_rate_warning_counts_encoder_outputs_rather_than_input_observations(
    encoder, poses, publication_clock, mocker
):
    publish = mocker.patch.object(encoder.joint_state, "publish")
    warning = mocker.patch("dimos.sim2.publication.logger.warning")
    for tick in range(20):
        await encoder.handle_observations(poses(time_ns=tick * 1_000_000))
    assert publish.call_count == 10
    publication_clock.advance_to(5)
    assert warning.call_count == 1
    assert warning.call_args.kwargs["output"] == "joint_state"
    assert warning.call_args.kwargs["requested_hz"] == 500
    assert warning.call_args.kwargs["actual_hz"] == 2
    assert warning.call_args.kwargs["published"] == 10
