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

import math

import pytest

from dimos.robot.galaxea.r1pro.joylo_module import R1ProJoylo


@pytest.fixture
def module(mocker):
    mocker.patch("dimos.robot.galaxea.r1pro.joylo_module.time.time", return_value=100)
    instance = R1ProJoylo()
    yield instance
    instance.dispose()


@pytest.mark.parametrize("stamp", [99.7, 100.1, math.nan, math.inf])
def test_queued_old_future_or_nonfinite_samples_do_not_renew_input_authority(module, stamp):
    assert not module._fresh("leader", stamp)
    assert module._fresh("leader", 100)


def test_duplicates_and_reordering_are_rejected_per_input_source(module):
    assert module._fresh("leader", 99.9)
    assert not module._fresh("leader", 99.9)
    assert not module._fresh("leader", 99.8)
    assert module._fresh("joycon", 99.8)
    assert module._fresh("leader", 100)
