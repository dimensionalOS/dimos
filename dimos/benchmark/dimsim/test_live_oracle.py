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

from pathlib import Path

import pytest

from dimos.benchmark.dimsim.apartment_profile import APARTMENT_COLLISION_SOURCE_COUNT
from dimos.benchmark.dimsim.bundle import generate_smoke_release, load_full_release
from dimos.benchmark.dimsim.config import CATEGORY_ORDER, PUBLIC_TEMPLATES
from dimos.benchmark.dimsim.models import (
    EntityChoiceOutcome,
    EnumOutcome,
    IntegerOutcome,
    TerminalOutcome,
)
from dimos.benchmark.dimsim.oracle import SceneClientOracleProvider
from dimos.simulation.dimsim.scene_client import SceneClient


@pytest.mark.tool
def test_live_apartment_oracle_generates_each_smoke_question(tmp_path: Path) -> None:
    """Contract test for an already-running pinned apartment scene."""

    client = SceneClient()
    client.start()
    try:
        view = SceneClientOracleProvider(client).get_scene_oracle_view()
        generate_smoke_release(view, tmp_path / "release")
    finally:
        client.stop()

    manifest, public, contracts, outcomes = load_full_release(tmp_path / "release")
    assert manifest.complete is True
    assert tuple(task.category for task in public) == CATEGORY_ORDER
    assert tuple(task.text for task in public) == tuple(
        PUBLIC_TEMPLATES[category] for category in CATEGORY_ORDER
    )
    assert len(contracts) == len(outcomes) == 4
    assert view.navigation.collision_source_count == APARTMENT_COLLISION_SOURCE_COUNT
    assert view.navigation.clearance_radius_m >= (view.embodiment.footprint_radius_m + 0.05)
    assert isinstance(outcomes[0].expected, TerminalOutcome)
    assert outcomes[1].expected == EnumOutcome(value="OFF")
    assert outcomes[2].expected == IntegerOutcome(value=4)
    assert isinstance(outcomes[3].expected, EntityChoiceOutcome)
    assert outcomes[3].expected.entity_id == "6eaff768b565c-19c7335c107"
