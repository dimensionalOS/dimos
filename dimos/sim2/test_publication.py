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

from dimos.core.module import Module
from dimos.sim2.publication import PublicationRate


@pytest.fixture
def owner():
    module = Module(instance_name="sensor")
    yield module
    module.dispose()


@pytest.fixture
def warning(mocker):
    return mocker.patch("dimos.sim2.publication.logger.warning")


def test_slow_output_reports_rate_and_shortfall_after_loading(owner, publication_clock, warning):
    rate = PublicationRate(owner, "camera", 50)
    publication_clock.advance_to(120)
    warning.assert_not_called()
    rate.begin()
    for _ in range(125):
        rate.begin()  # Further inputs must not reset the window.
        rate.published()
    publication_clock.advance_to(125)
    warning.assert_called_once_with(
        "Sim2 output is below its requested publication rate",
        module="sensor",
        output="camera",
        requested_hz=50,
        actual_hz=25,
        published=125,
        publication_shortfall=125,
        window_s=5,
    )


@pytest.mark.parametrize("count", [225, 250, 300])
def test_tolerable_jitter_and_faster_output_do_not_warn(owner, publication_clock, warning, count):
    rate = PublicationRate(owner, "observations", 50)
    rate.begin()
    for _ in range(count):
        rate.published()
    publication_clock.advance_to(5)
    warning.assert_not_called()


def test_total_stall_warns_without_more_input_and_is_throttled(owner, publication_clock, warning):
    rate = PublicationRate(owner, "imu", 200)
    rate.begin()
    publication_clock.advance_to(4.99)
    warning.assert_not_called()
    publication_clock.advance_to(5)
    assert warning.call_count == 1
    assert warning.call_args.kwargs["actual_hz"] == 0
    assert warning.call_args.kwargs["publication_shortfall"] == 1000
    publication_clock.advance_to(9.99)
    assert warning.call_count == 1
    publication_clock.advance_to(10)
    assert warning.call_count == 2


def test_recovery_is_measured_independently_of_previous_window(owner, publication_clock, warning):
    rate = PublicationRate(owner, "lidar", 10)
    rate.begin()
    publication_clock.advance_to(5)
    assert warning.call_count == 1
    for _ in range(50):
        rate.published()
    publication_clock.advance_to(10)
    assert warning.call_count == 1


def test_low_frequency_output_has_enough_samples_before_warning(owner, publication_clock, warning):
    rate = PublicationRate(owner, "slow_camera", 0.2)
    rate.begin()
    publication_clock.advance_to(49)
    warning.assert_not_called()
    for _ in range(9):
        rate.published()
    publication_clock.advance_to(50)
    warning.assert_not_called()
    publication_clock.advance_to(100)
    assert warning.call_args.kwargs["publication_shortfall"] == 10
    assert warning.call_args.kwargs["window_s"] == 50


def test_each_camera_is_checked_against_its_own_rate(owner, publication_clock, warning):
    fast = PublicationRate(owner, "camera_a", 50)
    slow = PublicationRate(owner, "camera_b", 15)
    fast.begin()
    slow.begin()
    for _ in range(250):
        fast.published()
    for _ in range(25):
        slow.published()
    publication_clock.advance_to(5)
    assert warning.call_count == 1
    assert warning.call_args.kwargs["output"] == "camera_b"
    assert warning.call_args.kwargs["requested_hz"] == 15
    assert warning.call_args.kwargs["actual_hz"] == 5


def test_module_disposal_stops_periodic_warnings(owner, publication_clock, warning):
    rate = PublicationRate(owner, "imu", 200)
    rate.begin()
    owner.dispose()
    publication_clock.advance_to(20)
    warning.assert_not_called()
