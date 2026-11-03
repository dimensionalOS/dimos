#!/usr/bin/env python3
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

"""One-shot: rewrite relative clip spans in cases.jsonl to absolute recording times.

Uses each clip's on-disk ``manifest.json`` (``t_start`` / ``t_end``). Does not
change generate / load_suite. Example::

    python scripts/migrate_vqa_clip_spans_to_absolute.py /path/to/vqa-dataset
    python scripts/migrate_vqa_clip_spans_to_absolute.py /path/to/vqa-dataset --dry-run
"""

from __future__ import annotations

import argparse
from pathlib import Path, PurePosixPath
import re
import shutil
import sys

import jsonlines

from dimos.evals.vqa.clip import format_clip_span, load_clip_manifest
from dimos.evals.vqa.generate import PublicCase

_CLIP_SPAN_IN_TEXT = re.compile(
    r"t=-?[0-9]+(?:\.[0-9]+)?s to t=-?[0-9]+(?:\.[0-9]+)?s"
    r"(?: \(absolute recording time\))?"
)


def _resolve_dir(root: Path, relative: str) -> Path:
    path = PurePosixPath(relative)
    if path.is_absolute() or not path.parts or ".." in path.parts:
        raise ValueError(f"VQA path must be dataset-relative: {relative!r}")
    resolved = root.joinpath(*path.parts).resolve()
    if not resolved.is_relative_to(root):
        raise ValueError(f"VQA path escapes the dataset: {relative!r}")
    if not resolved.is_dir():
        raise FileNotFoundError(f"VQA clip directory does not exist: {resolved}")
    return resolved


def _rewrite_spans(question: str, spans: tuple[str, ...]) -> str:
    matches = list(_CLIP_SPAN_IN_TEXT.finditer(question))
    if not matches or not spans:
        return question
    parts: list[str] = []
    cursor = 0
    for index, match in enumerate(matches):
        parts.append(question[cursor : match.start()])
        parts.append(spans[index] if index < len(spans) else match.group(0))
        cursor = match.end()
    parts.append(question[cursor:])
    return "".join(parts)


def _spans_for_case(root: Path, case: PublicCase) -> tuple[str, ...]:
    if case.clip is not None:
        manifest = load_clip_manifest(_resolve_dir(root, case.clip))
        return (format_clip_span(manifest.t_start, manifest.t_end),)
    if case.clips is not None:
        spans: list[str] = []
        for relative in case.clips:
            manifest = load_clip_manifest(_resolve_dir(root, relative))
            spans.append(format_clip_span(manifest.t_start, manifest.t_end))
        return tuple(spans)
    return ()


def migrate_dataset(root: Path, *, dry_run: bool) -> int:
    cases_path = root / "cases.jsonl"
    if not cases_path.is_file():
        raise FileNotFoundError(f"missing {cases_path}")

    rows: list[dict[str, object]] = []
    changed = 0
    with jsonlines.open(cases_path) as reader:
        for row in reader.iter(skip_empty=True):
            case = PublicCase.model_validate(row)
            spans = _spans_for_case(root, case)
            if not spans:
                rows.append(case.model_dump(mode="json", exclude_none=True))
                continue
            new_question = _rewrite_spans(case.question, spans)
            if new_question != case.question:
                changed += 1
                case = case.model_copy(update={"question": new_question})
            rows.append(case.model_dump(mode="json", exclude_none=True))

    print(f"{root}: {changed} case(s) would be updated" if dry_run else f"{root}: updated {changed} case(s)")
    if dry_run or changed == 0:
        return changed

    backup = cases_path.with_suffix(".jsonl.bak")
    shutil.copy2(cases_path, backup)
    tmp = cases_path.with_suffix(".jsonl.tmp")
    with jsonlines.open(tmp, mode="w") as writer:
        writer.write_all(rows)
    tmp.replace(cases_path)
    print(f"backup: {backup}")
    return changed


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "datasets",
        nargs="+",
        type=Path,
        help="Generated VQA dataset directories (contain cases.jsonl + assets/clips)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print how many cases would change; do not write",
    )
    args = parser.parse_args(argv)
    total = 0
    for dataset in args.datasets:
        total += migrate_dataset(dataset.expanduser().resolve(), dry_run=args.dry_run)
    return 0 if total >= 0 else 1


if __name__ == "__main__":
    sys.exit(main())
