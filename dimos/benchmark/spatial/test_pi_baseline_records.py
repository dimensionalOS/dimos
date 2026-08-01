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

# Copyright 2026 Dimensional Inc.
from datetime import datetime, timezone

from pydantic import ValidationError
import pytest

from dimos.benchmark.spatial.models import AnswerType
from dimos.benchmark.spatial.pi_baseline.records import (
    BaselineConfig,
    Prediction,
    Provenance,
    RunRecord,
    StagingRecord,
)


def test_records_are_strict_and_immutable() -> None:
    run = RunRecord(
        run_id="run-1",
        started_at=datetime.now(timezone.utc),
        release_id="release-id",
        release_version="v1.0.0",
    )
    with pytest.raises(ValidationError):
        RunRecord.model_validate({**run.model_dump(), "unexpected": True})
    with pytest.raises(ValidationError):
        run.run_id = "changed"  # type: ignore[misc]


def test_staging_paths_hash_and_typed_prediction() -> None:
    staging = StagingRecord(
        case_path="cases/case.v1.json",
        map_path="maps/map.lcm",
        map_sha256="a" * 64,
        schema_sha256="b" * 64,
    )
    assert staging.schema_version == "1.0"
    with pytest.raises(ValidationError):
        StagingRecord(case_path="../case.json", map_path="map.lcm", map_sha256="a" * 64)
    with pytest.raises(ValueError):
        Prediction.typed("instance", AnswerType.BOOLEAN, 1)
    assert Prediction.typed("instance", AnswerType.INTEGER, 2).value == 2
    Provenance(
        release_id="release", release_version="v1.0.0", case_sha256="b" * 64, map_sha256="c" * 64
    )
    assert BaselineConfig().resolver_version == "public-exact-v1"
