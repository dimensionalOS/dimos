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

import pytest

from dimos.benchmark.vqa.generation.config import QUESTION_MODEL, GenerationConfig
from dimos.benchmark.vqa.generation.dataset import (
    _validate_completed_frame,
    _write_generation_run,
)


def test_generation_run_records_resolved_request(tmp_path: Path) -> None:
    _write_generation_run(
        tmp_path,
        GenerationConfig(recording="go2.db", stop_index=10),
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
                "question_model": QUESTION_MODEL,
                "oracle_model": None,
                "grounding": {"min_mask_area_px": 128, "min_foreground_points": 3},
            }
        )
    )
    generation = GenerationConfig(recording="go2.db", stop_index=10)

    _validate_completed_frame(frame, generation, 1)

    with pytest.raises(ValueError, match="different settings"):
        _validate_completed_frame(
            frame,
            GenerationConfig(recording="other.db", stop_index=10),
            1,
        )
