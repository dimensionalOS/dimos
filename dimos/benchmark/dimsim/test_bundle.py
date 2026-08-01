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

import pytest

from dimos.benchmark.dimsim.bundle import (
    generate_smoke_release,
    load_full_release,
    load_public_tasks,
)
from dimos.benchmark.dimsim.fixture import apartment_oracle_fixture
from dimos.benchmark.dimsim.generation import GenerationError


def test_release_writes_canonical_public_and_private_layout(tmp_path: Path) -> None:
    root = tmp_path / "release"

    manifest = generate_smoke_release(apartment_oracle_fixture(), root)

    assert manifest.complete is True
    assert manifest.task_count == 4
    assert {path.relative_to(root).as_posix() for path in root.rglob("*") if path.is_file()} == {
        "manifest.json",
        "public/tasks.jsonl",
        "oracle/task_contracts.jsonl",
        "oracle/expected_outcomes.jsonl",
        "oracle/generation_report.json",
    }
    report = json.loads((root / "oracle" / "generation_report.json").read_text())
    assert report["complete"] is True
    assert all(check["passed"] for check in report["checks"])
    assert {check["name"] for check in report["checks"]} == {
        "schema-validity",
        "category-cardinality",
        "entity-resolution",
        "answer-typing",
        "destination-reachability",
        "comparison-distance-stability",
        "stable-reference-integrity",
        "public-oracle-leakage",
        "canonical-regeneration",
        "source-provenance",
    }


def test_same_fixture_regenerates_byte_equivalent_release(tmp_path: Path) -> None:
    first = tmp_path / "first"
    second = tmp_path / "second"

    generate_smoke_release(apartment_oracle_fixture(), first)
    generate_smoke_release(apartment_oracle_fixture(), second)

    assert {
        path.relative_to(first): path.read_bytes() for path in first.rglob("*") if path.is_file()
    } == {
        path.relative_to(second): path.read_bytes() for path in second.rglob("*") if path.is_file()
    }


def test_public_root_loads_without_oracle_and_contains_no_private_fields(tmp_path: Path) -> None:
    root = tmp_path / "release"
    generate_smoke_release(apartment_oracle_fixture(), root)

    tasks = load_public_tasks(root / "public")
    serialized = (root / "public" / "tasks.jsonl").read_text()

    assert len(tasks) == 4
    assert "oracle_view_digest" not in serialized
    assert "entity_id" not in serialized
    assert '"expected"' not in serialized


def test_full_release_joins_only_by_opaque_task_id(tmp_path: Path) -> None:
    root = tmp_path / "release"
    generate_smoke_release(apartment_oracle_fixture(), root)

    manifest, public, contracts, outcomes = load_full_release(root)

    assert manifest.task_count == 4
    assert {item.task_id for item in public} == {item.task_id for item in contracts}
    assert {item.task_id for item in public} == {item.task_id for item in outcomes}


def test_failed_generation_writes_only_private_diagnostics(tmp_path: Path) -> None:
    root = tmp_path / "failed"
    view = apartment_oracle_fixture()
    changed = view.model_copy(
        update={
            "entities": tuple(
                entity for entity in view.entities if entity.semantic_class != "television"
            )
        }
    )

    with pytest.raises(GenerationError, match="television"):
        generate_smoke_release(changed, root)

    assert not (root / "manifest.json").exists()
    assert not (root / "public").exists()
    report = json.loads((root / "oracle" / "generation_report.json").read_text())
    assert report["complete"] is False


def test_full_release_rejects_incomplete_or_count_mismatched_manifest(
    tmp_path: Path,
) -> None:
    root = tmp_path / "release"
    generate_smoke_release(apartment_oracle_fixture(), root)
    manifest_path = root / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    manifest["complete"] = False
    manifest["task_count"] = 3
    manifest_path.write_text(json.dumps(manifest))

    with pytest.raises(ValueError, match="complete"):
        load_full_release(root)


def test_full_release_rejects_duplicate_task_records(tmp_path: Path) -> None:
    root = tmp_path / "release"
    generate_smoke_release(apartment_oracle_fixture(), root)
    public_path = root / "public" / "tasks.jsonl"
    lines = public_path.read_text().splitlines()
    public_path.write_text("\n".join((lines[0], lines[0], *lines[2:])) + "\n")

    with pytest.raises(ValueError, match="duplicate"):
        load_full_release(root)


def test_full_release_rejects_incompatible_joined_record_shapes(tmp_path: Path) -> None:
    root = tmp_path / "release"
    generate_smoke_release(apartment_oracle_fixture(), root)
    public_path = root / "public" / "tasks.jsonl"
    records = [json.loads(line) for line in public_path.read_text().splitlines()]
    records[0]["response_type"] = "integer"
    public_path.write_text("".join(json.dumps(record) + "\n" for record in records))

    with pytest.raises(ValueError, match="incompatible"):
        load_full_release(root)


def test_full_release_rejects_identity_payload_mismatch(tmp_path: Path) -> None:
    root = tmp_path / "release"
    generate_smoke_release(apartment_oracle_fixture(), root)
    contract_path = root / "oracle" / "task_contracts.jsonl"
    records = [json.loads(line) for line in contract_path.read_text().splitlines()]
    records[0]["identity_payload"]["threshold_m"] = 99.0
    contract_path.write_text("".join(json.dumps(record) + "\n" for record in records))

    with pytest.raises(ValueError, match="task ID reconstruction"):
        load_full_release(root)


def test_full_release_rejects_source_digest_mismatch(tmp_path: Path) -> None:
    root = tmp_path / "release"
    generate_smoke_release(apartment_oracle_fixture(), root)
    outcome_path = root / "oracle" / "expected_outcomes.jsonl"
    records = [json.loads(line) for line in outcome_path.read_text().splitlines()]
    records[0]["oracle_view_digest"] = "0" * 64
    outcome_path.write_text("".join(json.dumps(record) + "\n" for record in records))

    with pytest.raises(ValueError, match="source digest mismatch"):
        load_full_release(root)


def test_full_release_rejects_unsupported_generator_revision(tmp_path: Path) -> None:
    root = tmp_path / "release"
    generate_smoke_release(apartment_oracle_fixture(), root)
    manifest_path = root / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    manifest["generator_revision"] = "future-generator-v99"
    manifest_path.write_text(json.dumps(manifest))

    with pytest.raises(ValueError, match="unsupported generator revision"):
        load_full_release(root)


def test_full_release_rejects_unsupported_profile_revision(tmp_path: Path) -> None:
    root = tmp_path / "release"
    generate_smoke_release(apartment_oracle_fixture(), root)
    contract_path = root / "oracle" / "task_contracts.jsonl"
    records = [json.loads(line) for line in contract_path.read_text().splitlines()]
    for record in records:
        record["source"]["profile_revision"] = "future-apartment-profile-v99"
    contract_path.write_text("".join(json.dumps(record) + "\n" for record in records))

    with pytest.raises(ValueError, match="unsupported source revision"):
        load_full_release(root)
