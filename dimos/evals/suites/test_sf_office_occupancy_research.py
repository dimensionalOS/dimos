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

import json
from pathlib import Path
from types import SimpleNamespace

from dimos.evals.environments.dataset import Dataset
from dimos.evals.suites.sf_office_occupancy_research import SUITE


def test_research_suite_defines_fifteen_strict_agent_encode_contracts() -> None:
    costmaps = object()
    store = SimpleNamespace(streams=SimpleNamespace(global_costmap=costmaps))

    assert len(SUITE) == 15
    assert len({case.id for case in SUITE}) == len(SUITE)
    for case in SUITE:
        assert isinstance(case.environment, Dataset)
        assert case.environment.config.name.endswith("recording_go2.db")
        assert len(case.environment.config.select) == 1
        assert case.environment.config.select[0](store) is costmaps
        assert case.timeout_s == 120.0
        assert "global_costmap" in case.inputs
        assert "obs.data.agent_encode()" in case.inputs
        assert "sole spatial evidence" in case.inputs
        assert "Do not inspect raw message attributes" in case.inputs
        assert "Only known-free space is traversable" in case.inputs
        assert "Return only JSON" in case.inputs
        assert "research" in case.tags
        assert "<" not in case.inputs


def test_research_suite_marks_temporal_cases() -> None:
    temporal = {case.id for case in SUITE if "temporal" in case.tags}

    assert temporal == {
        "sf_office_occupancy_first_reachable_time",
        "sf_office_occupancy_movement_square",
        "sf_office_occupancy_possible_person_motion",
    }


def test_research_answers_cover_every_case() -> None:
    path = Path(__file__).with_name("sf_office_occupancy_answers.json")
    manifest = json.loads(path.read_text())

    assert set(manifest["answers"]) == {case.id for case in SUITE}
    assert manifest["latest_grid"]["sha256"] == (
        "e47342fa8a98df18cd5ee32572a45600f7493c40af55664b8ff1263fd8ffe49c"
    )
