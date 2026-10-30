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

"""Standalone VQA dataset adapter for the shared evaluation runner.

A new kind of input is a new environment (:class:`ImageFile`), not a new case
class: each row becomes a plain :class:`EvalCase` graded on the choice the
model names."""

from __future__ import annotations

from collections.abc import Callable, Iterator, Sequence
import hashlib
import json
from pathlib import Path, PurePosixPath
from typing import Any

import jsonlines

from dimos.evals.environments.image_file import ImageFile
from dimos.evals.scorers import choice, exact
from dimos.evals.types import EvalCase, Outcome, Suite
from dimos.evals.vqa.generate import PrivateLabel, PublicCase


def source_record(dataset: Path) -> dict[str, Any]:
    """Resolved VQA source and hashes of the files used by the generated layout."""
    root = dataset.expanduser().resolve()
    files = [root / "cases.jsonl", root / "labels.jsonl", *sorted((root / "assets").rglob("*"))]
    return {
        "kind": "vqa_dataset",
        "path": str(root),
        "fingerprints": {
            str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in files
            if path.is_file()
        },
    }


def grade_choice(choices: Sequence[str], answer: str) -> Callable[[Outcome], float]:
    """Exact match on the last choice the reply names; naming none scores 0."""
    parse = choice(choices)

    def grade(o: Outcome) -> float:
        try:
            got = parse(o.trajectory.final_answer)
        except ValueError:
            return 0.0
        return exact(answer.lower(), got)

    return grade


_SPATIAL_CHOICE_HINTS = frozenset(
    {
        "ahead",
        "behind",
        "left",
        "right",
        "center",
        "turn around",
        "turn left",
        "turn right",
    }
)


def _is_spatial_case(case: PublicCase) -> bool:
    """True when choices are ego-spatial labels (ahead/behind/left/right/turn)."""
    return any(choice.casefold() in _SPATIAL_CHOICE_HINTS for choice in case.choices)


def format_case_inputs(case: PublicCase) -> str:
    """Question + Choices + answering rules (extra body-frame rules for spatial)."""
    choice_list = ", ".join(json.dumps(c) for c in case.choices)
    lines = [
        case.question,
        f"Choices: {json.dumps(case.choices)}",
        (
            "Reply with exactly one token from Choices, copied verbatim "
            f"(one of: {choice_list}). "
            'If the question says "turn left" / "turn right", the matching Choice tokens are '
            'still "left" / "right" — never invent spellings like "turn left". '
            "Do not refuse or say you lack information."
        ),
    ]
    if _is_spatial_case(case):
        lines += [
            "",
            "Spatial answering rules:",
            "- Images are the robot forward camera in time order; each [t=…] stamp is seconds from the first shown frame.",
            "- Earlier frames are history to help you understand where the referent is relative to the robot; they are not the answer moment.",
            "- The LAST image is question time (current robot pose / heading). Judge ahead, behind, left, right, and turn around in the robot body frame at that last frame only.",
            "- Prefer left / right / ahead when the referent's bearing at question time is clearly to that side or already in front.",
            "- Choose turn around only when the referent lies roughly behind the robot at question time (about a 180° yaw). Do not pick it merely because the referent appeared earlier in the clip.",
            "- An object can be behind at question time even if it is no longer in the final forward-camera image.",
        ]
    return "\n".join(lines)


def load_suite(dataset: Path) -> Suite:
    """Load public cases, private labels, and images from a generated dataset."""
    root = dataset.resolve()
    case_ids: set[str] = set()
    duplicate_cases = False
    for row in _read_jsonl(root / "cases.jsonl"):
        case = PublicCase.model_validate(row)
        duplicate_cases |= case.id in case_ids
        case_ids.add(case.id)

    label_by_id: dict[str, PrivateLabel] = {}
    duplicate_labels = False
    for row in _read_jsonl(root / "labels.jsonl"):
        parsed_label = PrivateLabel.model_validate(row)
        duplicate_labels |= parsed_label.id in label_by_id
        label_by_id[parsed_label.id] = parsed_label

    if not case_ids:
        raise ValueError(f"VQA dataset contains no cases: {root}")
    if duplicate_cases:
        raise ValueError("VQA dataset contains duplicate case IDs")
    if duplicate_labels:
        raise ValueError("VQA dataset contains duplicate label IDs")
    missing_labels = sorted(case_ids - label_by_id.keys())
    missing_cases = sorted(label_by_id.keys() - case_ids)
    if missing_labels or missing_cases:
        raise ValueError(
            f"VQA case/label IDs do not match: missing_labels={missing_labels}, "
            f"missing_cases={missing_cases}"
        )

    case_ids.clear()
    suite: list[EvalCase] = []
    for row in _read_jsonl(root / "cases.jsonl"):
        case = PublicCase.model_validate(row)
        label = label_by_id.pop(case.id)
        if label.answer not in case.choices:
            raise ValueError(f"VQA label for {case.id!r} is not one of its choices")
        suite.append(
            EvalCase(
                id=case.id,
                inputs=format_case_inputs(case),
                environment=ImageFile(_resolve_image(root, case.image)),
                grade=grade_choice(case.choices, label.answer),
                tags=frozenset({"vqa"}),
            )
        )
    return suite


def _read_jsonl(path: Path) -> Iterator[Any]:
    try:
        with jsonlines.open(path) as reader:
            yield from reader.iter(skip_empty=True)
    except (OSError, jsonlines.Error) as exc:
        raise ValueError(f"invalid VQA dataset file: {path}") from exc


def _resolve_image(root: Path, relative: str) -> Path:
    path = PurePosixPath(relative)
    if path.is_absolute() or not path.parts or ".." in path.parts:
        raise ValueError(f"VQA image path must be dataset-relative: {relative!r}")
    resolved = root.joinpath(*path.parts).resolve()
    if not resolved.is_relative_to(root):
        raise ValueError(f"VQA image path escapes the dataset: {relative!r}")
    if not resolved.is_file():
        raise FileNotFoundError(f"VQA image does not exist: {resolved}")
    return resolved
