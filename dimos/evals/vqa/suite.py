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

"""Standalone VQA dataset adapter for the shared evaluation runner."""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass
import json
from pathlib import Path, PurePosixPath
from typing import Any, cast

import jsonlines
from pydantic import BaseModel, ConfigDict, Field

from dimos.evals.scorers import exact
from dimos.evals.types import EvalCase, EvalResult, EvalRig, Suite
from dimos.evals.vqa.generate import PrivateLabel, PublicCase
from dimos.msgs.sensor_msgs.Image import Image


class VqaResponse(BaseModel):
    """Structured answer returned by the model under evaluation."""

    model_config = ConfigDict(extra="forbid")

    answer: str = Field(description="Exactly one of the choices provided in the question.")


@dataclass(frozen=True, kw_only=True)
class VqaEvalCase(EvalCase):
    """One standalone image question evaluated by the shared runner."""

    image_path: Path
    choices: tuple[str, ...]
    expected: str

    def evaluate(self, rig: EvalRig) -> EvalResult:
        image = Image.from_file(self.image_path)
        context = [] if rig.blind else cast("list[dict[str, Any]]", image.agent_encode())
        prompt = (
            f"{self.inputs}\nChoices: {json.dumps(self.choices)}\nAnswer with exactly one choice."
        )
        response = rig.ask_structured(context, prompt, VqaResponse)
        outputs = response.model_dump_json()
        score = exact(self.expected, response.answer) if response.answer in self.choices else 0.0
        return EvalResult(case_id=self.id, outputs=outputs, score=score)

    def preflight(self, rig: EvalRig) -> None:
        if not self.image_path.is_file():
            raise FileNotFoundError(f"VQA image does not exist: {self.image_path}")


def load_suite(dataset: Path) -> Suite:
    """Load public cases, private labels, and images from a generated dataset."""
    root = dataset.resolve()
    cases: list[PublicCase] = []
    case_ids: set[str] = set()
    duplicate_cases = False
    for row in _read_jsonl(root / "cases.jsonl"):
        case = PublicCase.model_validate(row)
        duplicate_cases |= case.id in case_ids
        case_ids.add(case.id)
        cases.append(case)

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

    suite: list[VqaEvalCase] = []
    for case in cases:
        label = label_by_id.pop(case.id)
        if label.answer not in case.choices:
            raise ValueError(f"VQA label for {case.id!r} is not one of its choices")
        suite.append(
            VqaEvalCase(
                id=case.id,
                inputs=case.question,
                image_path=_resolve_image(root, case.image),
                choices=case.choices,
                expected=label.answer,
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
