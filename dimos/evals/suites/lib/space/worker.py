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

"""Isolated replay into SPACE's unmodified aggregate evaluator, including spawn."""

from __future__ import annotations

import argparse
from collections.abc import Sequence
from dataclasses import dataclass
from functools import partial
import importlib
import json
from pathlib import Path
from typing import Any

from dimos.evals.suites.lib.space.upstream import OfficialSpace, RawReplyAgent, question_digest

MODEL_NAME = "dimos_saved_responses"


@dataclass(frozen=True)
class ReplayConfig:
    responses: dict[str, str]
    agent_name: str = RawReplyAgent.__name__
    use_vllm: bool = False


def load_replies(questions: Path, replies: Path) -> dict[str, str]:
    """Require one recorded reply for each distinct, unchanged text question."""
    rows = json.loads(questions.read_text())
    responses = json.loads(replies.read_text())
    if not isinstance(rows, list) or not rows:
        raise ValueError("Official SPACE scoring requires a nonempty QA list")
    keys: list[str] = []
    for row in rows:
        if not isinstance(row, dict) or not isinstance(row.get("question"), str):
            raise ValueError("SPACE QA rows must contain string questions")
        if "answer" not in row:
            raise ValueError("SPACE QA row is missing its official answer")
        keys.append(question_digest(row["question"]))
    if len(keys) != len(set(keys)):
        raise ValueError("Duplicate SPACE questions make the replay mapping ambiguous")
    if not isinstance(responses, dict) or any(not isinstance(v, str) for v in responses.values()):
        raise ValueError("Saved SPACE responses must map question hashes to raw strings")
    if set(responses) != set(keys):
        raise ValueError("Saved SPACE response keys must exactly match the scored questions")
    return responses


def run_aggregate(questions: Path, replies: Path, output: Path) -> None:
    """Use native SPACE aggregation after prepare() has registered the adapter."""
    responses = load_replies(questions, replies)
    if output.exists() and any(output.iterdir()):
        raise ValueError("Official SPACE output directory must be empty for an unambiguous result")
    output.mkdir(parents=True, exist_ok=True)
    # Optional external modules have already been pinned and prepared by main().
    registry = importlib.import_module("space.registry")
    evaluator = importlib.import_module("space.evaluate_qas")
    registry.register_config(MODEL_NAME)(partial(ReplayConfig, responses=responses))
    evaluator.main(
        model_name=MODEL_NAME,
        data_path=str(questions.resolve()),
        save_dir=str(output.resolve()),
        n_workers=1,
    )
    results = list(output.glob(f"{MODEL_NAME}/*/results.json"))
    if len(results) != 1:
        raise RuntimeError("SPACE did not produce exactly one official results.json")
    record: dict[str, Any] = json.loads(results[0].read_text())
    if len(record["all_metrics"]) != len(responses) or len(record["all_predictions"]) != len(
        responses
    ):
        raise RuntimeError("SPACE result count does not match the replayed subset")


def main(argv: Sequence[str] | None = None, *, worker_only: bool = False) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--questions", type=Path, required=True)
    parser.add_argument("--replies", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    scorer = OfficialSpace(args.source)
    scorer.prepare()
    if not worker_only:
        run_aggregate(args.questions, args.replies, args.output)


if __name__ in {"__main__", "__mp_main__"}:
    # Spawn re-executes this module as __mp_main__. Register the adapter there,
    # but run the aggregate only in the initiating process. Ordinary imports do
    # not prepare SPACE, download data, or create worker processes.
    main(worker_only=__name__ == "__mp_main__")
