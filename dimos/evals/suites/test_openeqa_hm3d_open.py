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

from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np

from dimos.evals.media.frames_to_store import dir_to_store
from dimos.evals.suites.openeqa_hm3d_open import build_suite


def test_build_suite_loads_prepared_cases(tmp_path: Path) -> None:
    frames = tmp_path / "frames"
    frames.mkdir()
    bgr = np.zeros((8, 12, 3), dtype=np.uint8)
    assert cv2.imwrite(str(frames / "00000-rgb.png"), bgr)
    store = tmp_path / "stores" / "hm3d-v0" / "000-hm3d-demo.db"
    assert dir_to_store(frames, store) == 1

    cases = tmp_path / "cases.jsonl"
    cases.write_text(
        json.dumps(
            {
                "id": "q-demo",
                "question": "What color is the wall?",
                "answer": "white",
                "category": "object recognition",
                "episode_history": "hm3d-v0/000-hm3d-demo",
                "source": "stores/hm3d-v0/000-hm3d-demo.db",
            }
        )
        + "\n",
        encoding="utf-8",
    )

    suite = build_suite(tmp_path)
    assert len(suite) == 1
    assert suite[0].id == "q-demo"
    assert "openeqa" in suite[0].tags
    assert suite[0].threshold == 0.75


def test_build_suite_missing_cases_is_empty(tmp_path: Path) -> None:
    assert build_suite(tmp_path) == []
