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

"""Smoke generation gate regressions for spatial pilot generation."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from dimos.benchmark.spatial.cli import main
from dimos.benchmark.spatial.models import Predicate
import dimos.benchmark.spatial.smoke as smoke_module
from dimos.benchmark.spatial.smoke import (
    REQUIRED_SMOKE_CHECKS,
    SMOKE_REPORT_NAME,
    PilotSourceError,
    SmokeGateError,
    generate_smoke_corpus,
    require_smoke_gate,
    run_pilot_generation,
    validate_smoke_corpus,
)


def test_generate_smoke_writes_all_predicates_variants_and_passing_report(tmp_path: Path) -> None:
    root = tmp_path / "smoke"

    report = generate_smoke_corpus(root)

    assert report.complete
    assert set(report.covered_predicates) == set(Predicate)
    assert report.missing_predicates == ()
    assert report.failed_checks == ()
    assert set(report.passed_checks) == set(REQUIRED_SMOKE_CHECKS)
    payload = json.loads((root / SMOKE_REPORT_NAME).read_text(encoding="utf-8"))
    assert payload["complete"] is True
    assert set(payload["covered_predicates"]) == {predicate.value for predicate in Predicate}
    readme = (root / "README.md").read_text(encoding="utf-8")
    assert "10 development and 20 held-out" in readme
    assert "1,170" in readme
    assert "VoxelGridMapper" in readme
    assert "static_spatial_benchmark_data_terms.md" in readme
    for variant in ("clean", "noisy-01", "noisy-02"):
        assert list(root.glob(f"public/scenes/*/trajectories/*/variants/{variant}/instances.jsonl"))


def test_validate_smoke_reports_missing_predicate_and_variant(tmp_path: Path) -> None:
    root = tmp_path / "smoke"
    generate_smoke_corpus(root)
    questions_path = next(root.glob("public/scenes/*/trajectories/*/questions.jsonl"))
    retained = [
        line
        for line in questions_path.read_text(encoding="utf-8").splitlines()
        if '"predicate":"pose-occupancy"' not in line
    ]
    questions_path.write_text("\n".join(retained) + "\n", encoding="utf-8")

    report = validate_smoke_corpus(root)

    assert not report.complete
    assert Predicate.POSE_OCCUPANCY in report.missing_predicates
    assert "pairing" in report.failed_checks


def test_pilot_gate_blocks_failed_smoke_with_actionable_error(tmp_path: Path) -> None:
    smoke_root = tmp_path / "smoke"
    generate_smoke_corpus(smoke_root)
    clean_instances = next(
        smoke_root.glob("public/scenes/*/trajectories/*/variants/clean/instances.jsonl")
    )
    clean_instances.write_text("", encoding="utf-8")

    with pytest.raises(SmokeGateError, match="failed checks"):
        require_smoke_gate(smoke_root)


def test_smoke_report_maps_hash_and_decode_failures_to_artifact_check(tmp_path: Path) -> None:
    root = tmp_path / "smoke"
    generate_smoke_corpus(root)
    map_path = next(root.glob("public/scenes/*/trajectories/*/variants/clean/global_map.pc2.lcm"))
    map_path.write_bytes(b"not a pointcloud2 lcm payload")

    report = validate_smoke_corpus(root)

    assert not report.complete
    assert "artifact-decode-hash" in report.failed_checks
    assert "artifact-decode-hash" not in report.passed_checks


def test_pilot_gate_reports_missing_smoke_root_as_gate_failure(tmp_path: Path) -> None:
    missing_smoke_root = tmp_path / "missing-smoke"

    with pytest.raises(SmokeGateError, match="failed checks"):
        require_smoke_gate(missing_smoke_root)

    assert (
        main(
            [
                "generate-pilot",
                "--output",
                str(tmp_path / "pilot"),
                "--smoke-root",
                str(missing_smoke_root),
            ]
        )
        == 2
    )


def test_pilot_gate_persists_success_report_before_pilot_output(tmp_path: Path) -> None:
    smoke_root = tmp_path / "smoke"
    pilot_root = tmp_path / "pilot"
    generate_smoke_corpus(smoke_root)

    with pytest.raises(PilotSourceError, match="source root"):
        run_pilot_generation(pilot_root, smoke_root)

    assert (pilot_root / SMOKE_REPORT_NAME).is_file()
    assert (pilot_root / "README.md").is_file()
    report_payload = json.loads(
        (pilot_root / "pilot_generation_report.json").read_text(encoding="utf-8")
    )
    assert report_payload["blocked"] is True
    assert report_payload["development_scene_count"] == 10
    assert report_payload["held_out_scene_count"] == 20


def test_cli_smoke_and_pilot_commands(tmp_path: Path) -> None:
    smoke_root = tmp_path / "smoke"
    pilot_root = tmp_path / "pilot"

    assert main(["generate-smoke", "--output", str(smoke_root)]) == 0
    assert (
        main(["generate-pilot", "--output", str(pilot_root), "--smoke-root", str(smoke_root)]) == 3
    )
    assert (pilot_root / SMOKE_REPORT_NAME).is_file()
    assert main(["validate-release", "--root", str(smoke_root)]) == 0
    validation_payload = json.loads(
        (smoke_root / "release_validation_report.json").read_text(encoding="utf-8")
    )
    assert validation_payload["complete"] is True


def test_pilot_reports_insufficient_structured3d_source(tmp_path: Path) -> None:
    smoke_root = tmp_path / "smoke"
    pilot_root = tmp_path / "pilot"
    source_root = tmp_path / "Structured3D"
    (source_root / "scene_00000").mkdir(parents=True)
    (source_root / "scene_00000" / "annotation_3d.json").write_text("{}", encoding="utf-8")
    generate_smoke_corpus(smoke_root)

    with pytest.raises(PilotSourceError, match="need 30"):
        run_pilot_generation(pilot_root, smoke_root, source_root)

    payload = json.loads((pilot_root / "pilot_generation_report.json").read_text(encoding="utf-8"))
    assert payload["blocker_code"] == "insufficient-structured3d-scenes"
    assert payload["expected_instance_count_approx"] == 1170


def test_pilot_cli_passes_worker_count_after_smoke_gate(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    smoke_root = tmp_path / "smoke"
    pilot_root = tmp_path / "pilot"
    source_root = tmp_path / "Structured3D"
    for index in range(30):
        scene = source_root / f"scene_{index:05d}"
        scene.mkdir(parents=True)
        (scene / "annotation_3d.json").write_text("{}", encoding="utf-8")
    generate_smoke_corpus(smoke_root)
    observed: dict[str, int] = {}

    def fake_generate(
        root: Path, annotations: tuple[Path, ...], *, workers: int
    ) -> smoke_module._PilotGenerationResult:
        observed["workers"] = workers
        observed["annotations"] = len(annotations)
        return smoke_module._PilotGenerationResult(retained_count=0, rejected_scenes=())

    monkeypatch.setattr(smoke_module, "_generate_source_backed_pilot", fake_generate)

    assert (
        main(
            [
                "generate-pilot",
                "--output",
                str(pilot_root),
                "--smoke-root",
                str(smoke_root),
                "--source-root",
                str(source_root),
                "--workers",
                "2",
            ]
        )
        == 3
    )
    assert observed == {"workers": 2, "annotations": 30}


def test_pilot_candidate_scan_reports_progress(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    annotation = tmp_path / "scene_00000" / "annotation_3d.json"
    annotation.parent.mkdir()
    annotation.write_text("{}", encoding="utf-8")

    def fake_discover(path: Path) -> smoke_module._CandidateScanResult:
        return smoke_module._CandidateScanResult(
            candidate=None,
            rejected_scene={"source_scene_key": path.parent.name, "reason": "fixture rejection"},
        )

    monkeypatch.setattr(smoke_module, "_discover_pilot_candidate", fake_discover)

    result = smoke_module._generate_source_backed_pilot(
        tmp_path / "pilot", (annotation,), workers=1
    )

    assert result.retained_count == 0
    assert result.rejected_scenes == (
        {"source_scene_key": "scene_00000", "reason": "fixture rejection"},
    )
    captured = capsys.readouterr()
    assert "pilot candidate scan start" in captured.err
    assert "pilot candidate scan progress scanned=1/1" in captured.err


def test_pilot_documentation_present() -> None:
    doc = Path("docs/development/static_spatial_benchmark_pilot.md")

    text = doc.read_text(encoding="utf-8")

    assert "review_overrides.jsonl" in text
    assert "Viser" in text
    assert "agent-facing" in text
