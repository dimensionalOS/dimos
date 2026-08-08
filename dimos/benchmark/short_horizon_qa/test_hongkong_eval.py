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

"""Self-hosted preparation gate over the real Hong Kong office recording."""

from pathlib import Path

import pytest

from dimos.benchmark.agent_eval.models import EvalCase
from dimos.benchmark.short_horizon_qa.models import MapperSettings
from dimos.benchmark.short_horizon_qa.prepare import prepare_bundle
from dimos.utils.data import get_data


@pytest.mark.self_hosted
def test_real_hongkong_recording_prepares_direct_demo_case(tmp_path: Path) -> None:
    case_path = (
        Path(__file__).parent / "cases" / "demo_go2_hongkong_office-room-count-smoke" / "case.json"
    )
    case = EvalCase.model_validate_json(case_path.read_bytes())
    map_progress: list[tuple[int, int]] = []
    manifest = prepare_bundle(
        get_data("go2_hongkong_office.db"),
        [],
        tmp_path / "bundle",
        progress=[case.source.progress],
        mapper=MapperSettings(device="CPU:0"),
        map_progress=lambda current, total: map_progress.append((current, total)),
    )
    assert case.case_id == "demo-go2-hongkong-office-room-count-smoke"
    assert manifest.cutoffs[0].normalized_progress == 1.0
    assert manifest.cutoffs[0].map_frame_count == 4235
    assert map_progress[-1] == (4235, 4235)
