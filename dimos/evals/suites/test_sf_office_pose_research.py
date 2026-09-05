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

from dimos.evals.suites.sf_office_pose_research import SUITE


def test_pose_research_suite_matches_reference_manifest() -> None:
    manifest = json.loads(Path(__file__).with_name("sf_office_pose_answers.json").read_text())[
        "answers"
    ]

    assert len(SUITE) == 9
    assert {case.id for case in SUITE} == set(manifest)
    assert len({case.id for case in SUITE}) == len(SUITE)


def test_pose_research_cases_enforce_encoded_odom_evidence() -> None:
    for case in SUITE:
        assert "odom PoseStamped" in case.inputs
        assert "obs.data.agent_encode()" in case.inputs
        assert "sole evidence" in case.inputs
        assert case.timeout_s == 180.0
        assert "autoresearch" in case.tags
