# Copyright 2026 Dimensional Inc.

import json
from pathlib import Path

from click import unstyle
from typer.testing import CliRunner

from dimos.benchmark.vqa.generation.specification import VqaGenerationSpecification
from dimos.cli import vqa


def test_vqa_generation_cli_has_no_explicit_query_or_model_options() -> None:
    result = CliRunner().invoke(vqa.app, ["generate", "--help"])
    assert result.exit_code == 0
    output = unstyle(result.output)

    assert "--query" not in output
    assert "--propose-questions" not in output
    assert "--question-model" not in output
    assert "--oracle-model" not in output
    assert "--spec" in output


def test_generation_spec_resolves_the_same_options_as_the_cli(tmp_path: Path) -> None:
    spec = tmp_path / "generation.json"
    spec.write_text(
        json.dumps(
            {
                "recording": "go2_bigoffice.db",
                "start_index": 10,
                "stop_index": 40,
                "stride": 5,
                "question_mode": "agentic",
                "grounding": {"min_mask_area_px": 256, "min_foreground_points": 4},
                "output": "/tmp/vqa",
            }
        )
    )

    generation = vqa._resolve_generation_spec(spec, None, None, None, None, None, None, None, None)

    assert generation.recording == "go2_bigoffice.db"
    assert generation.question_mode == "agentic"
    assert generation.grounding.min_mask_area_px == 256
    assert generation.output == "/tmp/vqa"


def test_generation_spec_rejects_mixed_cli_options(tmp_path: Path) -> None:
    spec = tmp_path / "generation.json"
    spec.write_text('{"recording":"go2.db","stop_index":10}')

    result = CliRunner().invoke(
        vqa.app,
        ["generate", "--spec", str(spec), "--recording", "other.db"],
    )

    assert result.exit_code != 0
    assert "cannot be combined" in result.output


def test_generation_run_records_resolved_request(tmp_path: Path) -> None:
    vqa._write_generation_run(
        tmp_path,
        VqaGenerationSpecification(recording="go2.db", stop_index=10),
        {"frame_count": 2, "accepted_question_count": 3, "rejected_question_count": 1},
    )

    payload = json.loads((tmp_path / "audit" / "run.json").read_text())

    assert payload["generation"]["recording"] == "go2.db"
    assert payload["generation"]["output"] == str(tmp_path)
    assert payload["summary"]["accepted_question_count"] == 3


def test_completed_frame_must_match_generation_settings(tmp_path: Path) -> None:
    frame = tmp_path / "audit" / "frame-000001"
    frame.mkdir(parents=True)
    (frame / "frame.json").write_text(
        json.dumps(
            {
                "recording": "go2.db",
                "frame_index": 1,
                "question_source": "openai_image_agent",
                "question_model": vqa.QUESTION_MODEL,
                "oracle_model": None,
                "grounding": {"min_mask_area_px": 128, "min_foreground_points": 3},
            }
        )
    )

    vqa._validate_completed_frame(frame, "go2.db", 1, "constrained", 128, 3)

    try:
        vqa._validate_completed_frame(frame, "other.db", 1, "constrained", 128, 3)
    except Exception as exc:
        assert "different settings" in str(exc)
    else:
        raise AssertionError("mismatched completed frame was accepted")
