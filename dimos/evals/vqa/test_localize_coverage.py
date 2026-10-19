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

import numpy as np
import pytest
from pydantic import ValidationError

from dimos.evals.vqa.generate import (
    GenerationRequest,
    PublicCase,
    PrivateLabel,
    _GeneratedFrame,
    _apply_localize_coverage_budget,
    _effective_localize_coverage_k,
)
from dimos.evals.vqa.localize_coverage import (
    DEFAULT_LOCALIZE_COVERAGE_K,
    case_family_from_id,
    collect_coverage_candidates_from_audit,
    coverage_cell,
    select_localize_coverage_ids,
)
from dimos.msgs.sensor_msgs.Image import Image


def test_coverage_cell_maps_selectors() -> None:
    assert coverage_cell("ego_any_fore_aft", ("chair",)) == ("ego_any_fore_aft", "any_ahead")
    assert coverage_cell("height_order", ("chair", "table")) == ("height_order", "unique")
    assert coverage_cell("ego_fore_aft", ("nearest chair",)) == ("ego_fore_aft", "nearest")
    assert coverage_cell("ego_fore_aft", ("left chair",)) == ("ego_fore_aft", "left")
    assert coverage_cell("ego_fore_aft", ("chair next to table",)) == ("ego_fore_aft", "next_to")
    assert coverage_cell("ego_fore_aft", ("chair",)) == ("ego_fore_aft", "unique")


def test_select_localize_coverage_ids_prefers_distinct_poses() -> None:
    candidates = [
        ("a", ("ego_fore_aft", "unique"), (0, 0, 0)),
        ("b", ("ego_fore_aft", "unique"), (0, 0, 0)),
        ("c", ("ego_fore_aft", "unique"), (1, 0, 0)),
        ("d", ("ego_fore_aft", "unique"), (2, 0, 0)),
        ("e", ("ego_fore_aft", "unique"), (3, 0, 0)),
        ("f", ("ego_fore_aft", "unique"), (4, 0, 0)),
        ("g", ("ego_fore_aft", "unique"), (5, 0, 0)),
        ("h", ("ego_any_fore_aft", "any_ahead"), (0, 0, 0)),
        ("i", ("ego_any_fore_aft", "any_ahead"), (1, 0, 0)),
    ]
    kept = select_localize_coverage_ids(candidates, k=5)
    assert kept == {"a", "c", "d", "e", "f", "h", "i"}
    assert "b" not in kept
    assert "g" not in kept


def test_select_fills_when_poses_missing() -> None:
    candidates = [
        ("a", ("ego_turn", "unique"), None),
        ("b", ("ego_turn", "unique"), None),
        ("c", ("ego_turn", "unique"), None),
    ]
    assert select_localize_coverage_ids(candidates, k=2) == {"a", "b"}


def test_case_family_from_id() -> None:
    assert case_family_from_id("frame-000010-chair-ego_fore_aft") == "ego_fore_aft"
    assert case_family_from_id("frame-000010-chair-ego_fore_aft-2") == "ego_fore_aft"


def test_collect_coverage_candidates_from_audit() -> None:
    rows = [
        {
            "status": "answered",
            "proposal": {"object_names": ["nearest chair"]},
            "answer": {
                "question": "Is the nearest chair ahead of you?",
                "evidence": {"robot_position_xyz": [1.2, 3.4, 0.0], "robot_yaw_rad": 0.1},
            },
        }
    ]
    candidate = collect_coverage_candidates_from_audit(
        case_id="frame-1-nearest-chair-ego_fore_aft",
        family="ego_fore_aft",
        question="Is the nearest chair ahead of you?",
        audit_rows=rows,
    )
    assert candidate is not None
    assert candidate[0] == "frame-1-nearest-chair-ego_fore_aft"
    assert candidate[1] == ("ego_fore_aft", "nearest")
    assert candidate[2] == (1, 3, 0)


def test_effective_coverage_k_defaults_and_disable() -> None:
    plain = GenerationRequest(dataset="go2_short", image_index=0)
    assert _effective_localize_coverage_k(plain) is None

    localized = GenerationRequest(dataset="go2_short", image_index=0, with_localize=True)
    assert _effective_localize_coverage_k(localized) == DEFAULT_LOCALIZE_COVERAGE_K

    disabled = GenerationRequest(
        dataset="go2_short",
        image_index=0,
        with_localize=True,
        localize_coverage_k=0,
    )
    assert _effective_localize_coverage_k(disabled) is None

    custom = GenerationRequest(
        dataset="go2_short",
        image_index=0,
        with_localize=True,
        localize_coverage_k=3,
    )
    assert _effective_localize_coverage_k(custom) == 3


def test_generation_request_rejects_coverage_k_without_localize() -> None:
    with pytest.raises(ValidationError, match="localize_coverage_k"):
        GenerationRequest(dataset="go2_short", image_index=0, localize_coverage_k=5)


def test_apply_localize_coverage_budget_keeps_image_path_cases() -> None:
    image = Image(data=np.zeros((4, 4, 3), dtype=np.uint8), ts=1.0)
    localize_cases = tuple(
        PublicCase(
            id=f"frame-000001-chair-ego_fore_aft-{i}" if i else "frame-000001-chair-ego_fore_aft",
            image="assets/frame-000001.png",
            question=f"Is the chair ahead? {i}",
            choices=("yes", "no"),
        )
        for i in range(6)
    )
    image_case = PublicCase(
        id="frame-000001-lamp-object_distance",
        image="assets/frame-000001.png",
        question="How far is the lamp?",
        choices=("1m", "2m"),
    )
    cases = (*localize_cases, image_case)
    labels = tuple(PrivateLabel(id=case.id, answer=case.choices[0]) for case in cases)
    audit_rows = tuple(
        {
            "status": "answered",
            "proposal": {"object_names": ["chair"]},
            "answer": {
                "question": case.question,
                "evidence": {
                    "robot_position_xyz": [float(i), 0.0, 0.0],
                    "robot_yaw_rad": 0.0,
                },
            },
        }
        for i, case in enumerate(localize_cases)
    )
    frame = _GeneratedFrame(
        index=1,
        image=image,
        cases=cases,
        labels=labels,
        audit_rows=audit_rows,
        families=(),
    )
    filtered_cases, filtered_labels, stats = _apply_localize_coverage_budget(
        [frame], list(cases), list(labels), k=5
    )
    ids = {case.id for case in filtered_cases}
    assert image_case.id in ids
    assert len(ids) == 6
    assert stats["localize_before"] == 6
    assert stats["localize_after"] == 5
    assert stats["non_localize_kept"] == 1
    assert {label.id for label in filtered_labels} == ids
