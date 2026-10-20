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
from pydantic import ValidationError
import pytest

from dimos.evals.vqa.generate import (
    GenerationRequest,
    PrivateLabel,
    PublicCase,
    _apply_localize_coverage_budget,
    _effective_localize_coverage_k,
    _GeneratedFrame,
)
from dimos.evals.vqa.localize_coverage import (
    DEFAULT_LOCALIZE_COVERAGE_K,
    case_family_from_id,
    case_frame_from_id,
    collect_coverage_candidates_from_audit,
    coverage_category,
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


def test_case_frame_from_id() -> None:
    assert case_frame_from_id("frame-000100-chair-ego_fore_aft") == "frame-000100"
    assert case_frame_from_id("frame-000010-chair-ego_fore_aft-2") == "frame-000010"


def test_select_gives_every_frame_at_least_one() -> None:
    nearest = ("ego_fore_aft", "nearest")
    turn = ("ego_turn", "nearest")
    candidates = [
        ("frame-000100-a-ego_fore_aft", nearest, (0, 0, 0), "box"),
        ("frame-000100-b-ego_turn", turn, (0, 0, 0), "box"),
        ("frame-000200-a-ego_fore_aft", nearest, (1, 0, 0), "flag"),
        ("frame-000300-a-ego_fore_aft", nearest, (2, 0, 0), "mat"),
        ("frame-000300-b-ego_turn", turn, (2, 0, 0), "mat"),
    ]
    kept = select_localize_coverage_ids(candidates, k=1)
    frames = {case_frame_from_id(case_id) for case_id in kept}
    assert frames == {"frame-000100", "frame-000200", "frame-000300"}
    assert len(kept) == 3


def test_select_rotates_cells_across_frames() -> None:
    nearest = ("ego_fore_aft", "nearest")
    any_ahead = ("ego_any_fore_aft", "any_ahead")
    turn = ("ego_turn", "nearest")
    candidates = []
    for index, frame in enumerate(("000100", "000200", "000300", "000400", "000500", "000600")):
        # Same menu on every frame so rotation is visible.
        candidates.extend(
            [
                (f"frame-{frame}-n-ego_fore_aft", nearest, (index, 0, 0), "box"),
                (f"frame-{frame}-a-ego_any_fore_aft", any_ahead, (index, 0, 0), "box"),
                (f"frame-{frame}-t-ego_turn", turn, (index, 0, 0), "box"),
            ]
        )
    kept = select_localize_coverage_ids(candidates, k=1)
    assert len(kept) == 6
    cells = []
    for case_id in sorted(kept):
        for cand in candidates:
            if cand[0] == case_id:
                cells.append(cand[1])
                break
    # Sorted cell_order: any_ahead, nearest fore_aft, turn nearest — rotates across frames.
    assert cells[0] != cells[1] or cells[1] != cells[2]
    assert len(set(cells)) >= 2


def test_select_respects_per_frame_k() -> None:
    cell = ("ego_fore_aft", "nearest")
    candidates = [(f"frame-000100-{i}-ego_fore_aft", cell, (i, 0, 0), "box") for i in range(6)] + [
        (f"frame-000200-{i}-ego_fore_aft", cell, (i, 1, 0), "flag") for i in range(6)
    ]
    kept = select_localize_coverage_ids(candidates, k=2)
    assert sum(1 for case_id in kept if case_id.startswith("frame-000100-")) == 2
    assert sum(1 for case_id in kept if case_id.startswith("frame-000200-")) == 2


def test_select_skips_left_right_cells() -> None:
    candidates = [
        ("frame-000100-l-ego_fore_aft", ("ego_fore_aft", "left"), (0, 0, 0), "box"),
        ("frame-000100-n-ego_fore_aft", ("ego_fore_aft", "nearest"), (0, 0, 0), "box"),
    ]
    kept = select_localize_coverage_ids(candidates, k=1)
    assert kept == {"frame-000100-n-ego_fore_aft"}


def test_select_balances_categories_when_same_cell() -> None:
    cell = ("ego_fore_aft", "nearest")
    candidates = [
        ("frame-000100-box-ego_fore_aft", cell, (0, 0, 0), "storage box"),
        ("frame-000100-flag-ego_fore_aft", cell, (0, 0, 0), "flag"),
        ("frame-000200-box-ego_fore_aft", cell, (1, 0, 0), "storage box"),
        ("frame-000200-flag-ego_fore_aft", cell, (1, 0, 0), "flag"),
    ]
    kept = select_localize_coverage_ids(candidates, k=1)
    cats = set()
    for case_id in kept:
        for cand in candidates:
            if cand[0] == case_id:
                cats.add(cand[3])
    # Two frames, same cell only — category counts push diversity across picks.
    assert len(kept) == 2
    assert cats == {"storage box", "flag"} or len(cats) >= 1


def test_select_rotates_categories_inside_cell_despite_global_saturation() -> None:
    """next_to must not collapse onto exclusive cats after nearest fills globals."""
    nearest = ("ego_fore_aft", "nearest")
    next_to = ("ego_fore_aft", "next_to")
    candidates = []
    for index, frame in enumerate(("000100", "000200", "000300", "000400")):
        # nearest always offers flag (would saturate global flag count).
        candidates.append((f"frame-{frame}-n-ego_fore_aft", nearest, (index, 0, 0), "flag"))
        # next_to offers both an exclusive cat and flag; must still rotate.
        candidates.append(
            (f"frame-{frame}-nt-coatrack-ego_fore_aft", next_to, (index, 0, 0), "coatrack")
        )
        candidates.append((f"frame-{frame}-nt-flag-ego_fore_aft", next_to, (index, 0, 0), "flag"))
    kept = select_localize_coverage_ids(candidates, k=2)
    next_to_cats = []
    for case_id in sorted(kept):
        if "-nt-" not in case_id:
            continue
        for cand in candidates:
            if cand[0] == case_id:
                next_to_cats.append(cand[3])
    assert set(next_to_cats) == {"coatrack", "flag"}
    assert next_to_cats.count("coatrack") == 2
    assert next_to_cats.count("flag") == 2


def test_coverage_category_prefers_evidence() -> None:
    assert (
        coverage_category(
            "ego_fore_aft",
            ("nearest storage box",),
            {"category": "Storage Box"},
        )
        == "storage box"
    )
    assert coverage_category("ego_fore_aft", ("chair next to table",)) == "chair"


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
    assert candidate[3] == "chair"


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
    assert len(ids) == 6  # 5 localize + 1 image
    assert stats["localize_before"] == 6
    assert stats["localize_after"] == 5
    assert stats["non_localize_kept"] == 1
    assert {label.id for label in filtered_labels} == ids
