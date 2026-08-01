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

from pydantic import ValidationError
import pytest

from dimos.benchmark.dimsim.bundle import json_schemas
from dimos.benchmark.dimsim.fixture import apartment_oracle_fixture
from dimos.benchmark.dimsim.generation import compile_smoke_tasks
from dimos.benchmark.dimsim.models import EnumOutcome, ExpectedOutcome, SceneOracleView
from dimos.benchmark.dimsim.utilities import oracle_view_bytes, oracle_view_digest
from dimos.benchmark.spatial.utilities import canonical_json


def test_oracle_canonical_bytes_survive_json_key_reordering() -> None:
    view = apartment_oracle_fixture()
    payload = json.loads(oracle_view_bytes(view))
    reordered = dict(reversed(tuple(payload.items())))

    reparsed = SceneOracleView.model_validate_json(canonical_json(reordered))

    assert oracle_view_bytes(reparsed) == oracle_view_bytes(view)


def test_truth_bearing_state_changes_oracle_digest() -> None:
    off = apartment_oracle_fixture(television_power="OFF")
    on = apartment_oracle_fixture(television_power="ON")

    assert oracle_view_digest(off) != oracle_view_digest(on)


def test_oracle_rejects_unknown_fields_and_unsupported_schema() -> None:
    payload = apartment_oracle_fixture().model_dump(mode="json")
    payload["unexpected"] = True
    with pytest.raises(ValidationError, match="unexpected"):
        SceneOracleView.model_validate(payload)

    payload.pop("unexpected")
    payload["schema_version"] = "2.0"
    with pytest.raises(ValidationError, match="schema_version"):
        SceneOracleView.model_validate(payload)


def test_oracle_rejects_malformed_region_reference() -> None:
    view = apartment_oracle_fixture()
    first = view.entities[0].model_copy(update={"region_ids": ("missing-region",)})

    with pytest.raises(ValidationError, match="unknown region references"):
        SceneOracleView.model_validate_json(
            oracle_view_bytes(view.model_copy(update={"entities": (first, *view.entities[1:])}))
        )


def test_expected_outcome_rejects_invalid_answer_type() -> None:
    task = compile_smoke_tasks(apartment_oracle_fixture())[1]
    payload = task.outcome.model_dump(mode="json")
    payload["expected"] = {"kind": "enum", "value": 4}

    with pytest.raises(ValidationError, match="expected.enum.value"):
        ExpectedOutcome.model_validate(payload)


def test_every_persisted_record_exposes_json_schema() -> None:
    schemas = json_schemas()

    assert set(schemas) == {
        "Manifest",
        "PublicTask",
        "TaskContract",
        "ExpectedOutcome",
        "GenerationReport",
    }
    assert schemas["ExpectedOutcome"]["additionalProperties"] is False
    assert EnumOutcome.model_json_schema()["additionalProperties"] is False
