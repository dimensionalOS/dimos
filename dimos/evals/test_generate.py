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

"""Offline tests for generated VQA schemas, quality gates, and eval wiring."""

from __future__ import annotations

from dataclasses import replace
import json
from pathlib import Path
from types import SimpleNamespace
from typing import Any

from langchain_core.language_models.fake_chat_models import FakeListChatModel
import numpy as np
import pytest
from typer.testing import CliRunner

from dimos.evals.cli import app
from dimos.evals.generate import (
    GenerationOutput,
    VqaGenerationSpec,
    generate_from_spec,
    load_generation_spec,
    load_vqa_manifest,
    materialize_vqa_dataset,
)
from dimos.evals.runner import EvalRunner
from dimos.evals.vqa import SceneEntity, SceneSnapshot, generate_questions


def _snapshot(*, seed: int = 100) -> SceneSnapshot:
    return SceneSnapshot(
        image=np.zeros((100, 100, 3), dtype=np.uint8),
        entities=(
            SceneEntity(
                id="cup_0",
                category="cup",
                pixel_area=400,
                centroid=(20.0, 30.0),
                bbox=(10, 20, 30, 40),
            ),
            SceneEntity(
                id="bottled_drink_0",
                category="bottled drink",
                pixel_area=500,
                centroid=(80.0, 30.0),
                bbox=(70, 20, 90, 40),
            ),
            SceneEntity(id="plate_0", category="plate", pixel_area=0),
        ),
        provenance={
            "source": "robocasa",
            "source_version": "test",
            "task": "BeverageOrganization",
            "seed": seed,
            "layout_id": 1,
            "style_id": 1,
            "camera": "robot0_agentview_center",
        },
    )


def _spec(output: Path) -> VqaGenerationSpec:
    return VqaGenerationSpec(
        source="robocasa",
        use_case="beverage inventory",
        task="BeverageOrganization",
        episodes=1,
        seed=100,
        camera="robot0_agentview_center",
        image_size=(100, 100),
        question_families=("semantic_presence", "spatial_left_right"),
        targets=("cup", "bottled_drink", "plate"),
        output=output,
        min_visible_pixels=50,
        min_horizontal_separation=0.1,
        max_bbox_overlap=0.25,
    )


def test_question_families_use_visibility_and_image_coordinates() -> None:
    batch = generate_questions(
        _snapshot(),
        targets=("cup", "bottled_drink", "plate"),
        families=("semantic_presence", "spatial_left_right"),
        min_visible_pixels=50,
        min_horizontal_separation=0.1,
        max_bbox_overlap=0.25,
        max_spatial_pairs=8,
    )

    presence = [question for question in batch.questions if question.family == "semantic_presence"]
    spatial = [question for question in batch.questions if question.family == "spatial_left_right"]
    assert [question.reference_outputs for question in presence] == ["yes", "yes", "no"]
    assert len(spatial) == 1
    assert spatial[0].reference_outputs == "right"
    assert spatial[0].oracle == {
        "family": "spatial_left_right",
        "subject": "bottled_drink_0",
        "reference": "cup_0",
    }


def test_question_generation_rejects_small_and_ambiguous_entities() -> None:
    snapshot = SceneSnapshot(
        image=np.zeros((100, 100, 3), dtype=np.uint8),
        entities=(
            SceneEntity(
                id="cup_0",
                category="cup",
                pixel_area=10,
                centroid=(10.0, 10.0),
                bbox=(8, 8, 12, 12),
            ),
            SceneEntity(
                id="bottle_0",
                category="bottle",
                pixel_area=200,
                centroid=(40.0, 20.0),
                bbox=(30, 10, 50, 30),
            ),
            SceneEntity(
                id="bottle_1",
                category="bottle",
                pixel_area=200,
                centroid=(80.0, 20.0),
                bbox=(70, 10, 90, 30),
            ),
        ),
        provenance={"seed": 1},
    )
    batch = generate_questions(
        snapshot,
        targets=("cup", "bottle"),
        families=("semantic_presence", "spatial_left_right"),
        min_visible_pixels=50,
        min_horizontal_separation=0.1,
        max_bbox_overlap=0.25,
        max_spatial_pairs=8,
    )

    assert batch.rejected == {
        "presence_too_small": 1,
        "spatial_ambiguous_label": 2,
    }
    assert not [question for question in batch.questions if question.family == "spatial_left_right"]


def test_scene_entity_rejects_inconsistent_visibility() -> None:
    with pytest.raises(ValueError, match="invisible entities"):
        SceneEntity(
            id="cup",
            category="cup",
            pixel_area=0,
            centroid=(1.0, 1.0),
            bbox=(0, 0, 2, 2),
        )


def test_load_generation_spec_resolves_output_and_rejects_unknown_fields(
    tmp_path: Path,
) -> None:
    path = tmp_path / "use_case.yaml"
    path.write_text(
        """source: robocasa
use_case: beverage_inventory
task: BeverageOrganization
episodes: 2
seed: 10
camera: robot0_agentview_center
image_size: [128, 96]
question_families: [semantic_presence]
targets: [cup, bottled_drink]
output: generated/run
"""
    )
    spec = load_generation_spec(path)
    assert spec.output == (tmp_path / "generated/run").resolve()
    assert spec.image_size == (128, 96)
    assert spec.split == "target"

    path.write_text(path.read_text() + "surprise: true\n")
    with pytest.raises(ValueError, match="unknown VQA generation spec fields"):
        load_generation_spec(path)


def test_generate_from_spec_uses_source_adapter(tmp_path: Path, monkeypatch: Any) -> None:
    path = tmp_path / "use_case.yaml"
    path.write_text(
        """source: robocasa
use_case: beverage_inventory
task: BeverageOrganization
episodes: 1
seed: 100
camera: robot0_agentview_center
image_size: [100, 100]
question_families: [semantic_presence, spatial_left_right]
targets: [cup, bottled_drink, plate]
output: generated
min_visible_pixels: 50
min_horizontal_separation: 0.1
max_bbox_overlap: 0.25
"""
    )
    seen: dict[str, object] = {}

    def export(spec: VqaGenerationSpec, *, source_python: str):
        seen["spec"] = spec
        seen["source_python"] = source_python
        return [_snapshot()]

    monkeypatch.setattr("dimos.evals.generate.export_robocasa_snapshots", export)

    output = generate_from_spec(path, source_python="/external/python")

    assert output.accepted == 3
    assert seen["source_python"] == "/external/python"
    assert isinstance(seen["spec"], VqaGenerationSpec)


def test_materialize_load_and_run_generated_cases(tmp_path: Path) -> None:
    output = materialize_vqa_dataset(_spec(tmp_path / "generated"), [_snapshot()])
    rows = json.loads(output.manifest.read_text())
    assert output.accepted == 3
    assert output.accepted_by_family == {
        "semantic_presence": 2,
        "spatial_left_right": 1,
    }
    assert output.rejected == {"presence_balance": 1}
    assert [row["reference_outputs"] for row in rows] == ["yes", "no", "right"]
    assert all(row["dataset"] == "observations.db" for row in rows)
    assert all(row["provenance"]["seed"] == 100 for row in rows)
    assert all("oracle" in row for row in rows)
    summary = json.loads(output.summary.read_text())
    assert summary["accepted_by_answer"] == {"no": 1, "right": 1, "yes": 1}
    assert summary["observed_categories"] == ["bottled_drink", "cup", "plate"]

    cases = load_vqa_manifest(output.manifest)
    assert len(cases) == 3

    class TextOnlyRunner(EvalRunner):
        def encode(self, stream: Any) -> list[dict[str, Any]]:
            observations = list(stream)
            assert len(observations) == 1
            assert observations[0].data.shape == (100, 100, 3)
            return [{"type": "text", "text": "one generated RGB observation"}]

    runner = TextOnlyRunner(
        chat_model=FakeListChatModel(responses=["yes", "no", "right"]),
        out_dir=tmp_path / "eval-results",
    )
    results = runner.run(cases)
    assert all(result.passed for result in results)
    assert json.loads((runner.run_dir / "summary.json").read_text())["pass_rate"] == 1.0


def test_generation_is_deterministic_and_refuses_overwrite(tmp_path: Path) -> None:
    first = materialize_vqa_dataset(_spec(tmp_path / "first"), [_snapshot()])
    second = materialize_vqa_dataset(_spec(tmp_path / "second"), [_snapshot()])
    assert json.loads(first.manifest.read_text()) == json.loads(second.manifest.read_text())

    with pytest.raises(FileExistsError, match="output already exists"):
        materialize_vqa_dataset(_spec(tmp_path / "first"), [_snapshot()])


def test_generation_requires_every_requested_family(tmp_path: Path) -> None:
    spec = replace(
        _spec(tmp_path / "generated"),
        targets=("cup", "bottled_drink"),
        question_families=("semantic_presence",),
    )
    with pytest.raises(ValueError, match="observed_categories=.*bottled_drink"):
        materialize_vqa_dataset(spec, [_snapshot()])


def test_manifest_rejects_unknown_answer_type(tmp_path: Path) -> None:
    manifest = tmp_path / "cases.json"
    manifest.write_text(
        json.dumps(
            [
                {
                    "schema_version": 1,
                    "id": "bad",
                    "inputs": "?",
                    "reference_outputs": "x",
                    "answer_type": "free_form",
                    "dataset": "observations.db",
                    "stream": "color_image",
                    "frame_ts": 0,
                    "tags": [],
                }
            ]
        )
    )
    with pytest.raises(ValueError, match="unsupported answer_type"):
        load_vqa_manifest(manifest)


def test_cli_generate_reports_materialized_outputs(tmp_path: Path, monkeypatch: Any) -> None:
    spec = tmp_path / "use_case.yaml"
    spec.write_text("source: robocasa\n")
    generated = tmp_path / "generated"
    output = GenerationOutput(
        output_dir=generated,
        manifest=generated / "cases.json",
        dataset=generated / "observations.db",
        summary=generated / "generation_summary.json",
        accepted=3,
        accepted_by_family={"semantic_presence": 2, "spatial_left_right": 1},
        rejected={"spatial_too_close": 1},
    )
    monkeypatch.setattr("dimos.evals.generate.generate_from_spec", lambda *args, **kwargs: output)

    result = CliRunner().invoke(
        app,
        ["generate", str(spec), "--source-python", "/external/python"],
    )

    assert result.exit_code == 0
    assert "Generated 3 VQA cases" in result.stdout
    assert "semantic_presence: 2" in result.stdout
    assert str(output.manifest) in result.stdout


def test_cli_run_routes_json_manifests_to_existing_runner(tmp_path: Path, monkeypatch: Any) -> None:
    manifest = tmp_path / "cases.json"
    manifest.write_text("[]")
    cases = [object()]
    seen: dict[str, object] = {}

    def load(path: str) -> list[object]:
        seen["manifest"] = path
        return cases

    class FakeRunner:
        run_dir = tmp_path / "run"

        def __init__(self, **kwargs: object) -> None:
            seen["runner_kwargs"] = kwargs

        def run(self, values: object, **kwargs: object) -> list[object]:
            seen["cases"] = values
            seen["run_kwargs"] = kwargs
            return []

    monkeypatch.setattr("dimos.evals.generate.load_vqa_manifest", load)
    monkeypatch.setattr("dimos.evals.runner.EvalRunner", FakeRunner)
    monkeypatch.setattr(
        "dimos.evals.runner.summarize",
        lambda results: SimpleNamespace(
            n=0, mean_score=0.0, pass_rate=0.0, errors=0, duration_s=0.0
        ),
    )

    result = CliRunner().invoke(app, ["run", str(manifest), "--limit", "2"])

    assert result.exit_code == 0
    assert seen["manifest"] == str(manifest)
    assert seen["cases"] is cases
    assert seen["run_kwargs"] == {"tags": frozenset(), "limit": 2}
