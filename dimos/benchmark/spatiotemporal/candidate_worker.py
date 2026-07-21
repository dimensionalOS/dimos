# Copyright 2025-2026 Dimensional Inc.
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

"""Subprocess entry point for the filesystem-isolated public candidate."""

import argparse
import json
from pathlib import Path


def _candidate_child(root: Path, path: Path, *, must_exist: bool) -> Path:
    resolved_root = root.resolve(strict=True)
    resolved = path.resolve(strict=must_exist)
    if not resolved.is_relative_to(resolved_root):
        raise ValueError(f"candidate path escapes temporary root: {path}")
    if path.is_symlink():
        raise ValueError(f"candidate path must not be a symlink: {path}")
    return resolved


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--video", type=Path, required=True)
    parser.add_argument("--questions", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--duration-s", type=int, required=True)
    parser.add_argument("--result", type=Path, required=True)
    args = parser.parse_args()

    output_root = args.output_root.resolve(strict=True)
    video = _candidate_child(output_root, args.video, must_exist=True)
    questions_path = _candidate_child(output_root, args.questions, must_exist=True)
    result_path = _candidate_child(output_root, args.result, must_exist=False)

    from dimos.benchmark.spatiotemporal.demo import (
        _run_temporal_memory_candidate_in_process,
    )
    from dimos.benchmark.spatiotemporal.models import Question

    questions = tuple(
        Question.model_validate_json(line)
        for line in questions_path.read_text(encoding="utf-8").splitlines()
        if line
    )
    answers, runtime = _run_temporal_memory_candidate_in_process(
        video,
        questions,
        output_root,
        args.duration_s,
    )
    result_path.write_text(
        json.dumps(
            {"answers": answers, "runtime": runtime},
            sort_keys=True,
            separators=(",", ":"),
        )
        + "\n",
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
