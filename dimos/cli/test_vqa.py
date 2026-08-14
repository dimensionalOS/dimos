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

import json
from pathlib import Path
from types import SimpleNamespace
from typing import Any

from click import unstyle
from typer.testing import CliRunner

from dimos.benchmark.vqa.generation.config import GenerationConfig
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


def test_generate_delegates_resolved_specification(monkeypatch: Any) -> None:
    captured: dict[str, object] = {}

    def execute(generation: GenerationConfig, **kwargs: object) -> object:
        captured["generation"] = generation
        captured.update(kwargs)
        return SimpleNamespace(summary={"frame_count": 1})

    monkeypatch.setattr(vqa, "execute_generation", execute)

    result = CliRunner().invoke(
        vqa.app,
        ["generate", "--recording", "go2.db", "--stop-index", "1"],
    )

    assert result.exit_code == 0
    assert captured["generation"] == GenerationConfig(recording="go2.db", stop_index=1)
    assert captured["progress"] is not None
