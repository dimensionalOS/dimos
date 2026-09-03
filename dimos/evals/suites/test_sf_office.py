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

from types import SimpleNamespace

from dimos.evals.environments.dataset import Dataset
from dimos.evals.environments.occupancy_dataset import OccupancyDataset
from dimos.evals.suites.sf_office import SUITE as OCCUPANCY_SUITE
from dimos.evals.suites.sf_office_pose import SUITE as POSE_SUITE


def test_occupancy_suite_uses_only_agent_encoded_costmaps() -> None:
    assert len(OCCUPANCY_SUITE) == 6
    for case in OCCUPANCY_SUITE:
        assert isinstance(case.environment, OccupancyDataset)
        assert case.timeout_s == 120.0
        assert "global_costmap" in case.inputs
        assert "obs.data.agent_encode()" in case.inputs
        assert "do not inspect the raw grid array" in case.inputs


def test_pose_suite_uses_every_agent_encoded_odom_pose() -> None:
    odom = object()
    store = SimpleNamespace(streams=SimpleNamespace(odom=odom))

    assert len(POSE_SUITE) == 7
    for case in POSE_SUITE:
        assert isinstance(case.environment, Dataset)
        assert case.environment.name == "go2_short"
        assert len(case.environment.select) == 1
        assert case.environment.select[0](store) is odom
        assert case.timeout_s == 120.0
        assert "every observation" in case.inputs
        assert "obs.data.agent_encode()" in case.inputs
        assert "do not inspect raw pose fields" in case.inputs.lower()
        assert "construct a Path" in case.inputs
        assert "create or inspect images" in case.inputs
