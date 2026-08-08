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

"""Run one SPACE text task end to end and keep the two records of it in agreement.

The order here is the design. Everything a run is judged on — which questions,
which release, which scorer, which build — is written to disk before a single
worker starts, and the run only succeeds if SPACE's own ``results.json`` and
the per-question records this side wrote say the same thing about every answer.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from datetime import datetime, timezone
import importlib
import json
import multiprocessing as mp
import os
from pathlib import Path
import sys
from typing import Any

from dimos.benchmark.space_qa.adapter import BenchmarkItem, SubsetSpec
from dimos.benchmark.space_qa.data import release_sha256, resolve_space_data
from dimos.benchmark.space_qa.manifest import (
    MANIFEST_NAME,
    RECORD_NAME,
    RUN_DIR_ENV,
    RunManifest,
    build_manifest,
    case_dir,
    qa_dir_name,
    subset_path,
    write_manifest,
)
from dimos.benchmark.space_qa.source import SPACE_REVISION, ensure_space_source, space_cache_root
from dimos.benchmark.space_qa.suite import SpaceQAAdapter
from dimos.benchmark.space_qa.tasks import space_text_task

AGENT_MODULE = "dimos.benchmark.space_qa.agent"
CONFIG_NAME = "dimos_qa"
DEFAULT_WORKERS = 2
RECORDS_NAME = "cases.jsonl"
SPACE_OUTPUT_DIR_NAME = "space"


@dataclass(frozen=True, kw_only=True)
class SpaceRunSummary:
    """Where a finished run put its evidence, and the one number SPACE reports."""

    task: str
    seed: int
    groups: int
    questions: int
    # SPACE reports accuracy as a percentage over the questions it scored.
    mean_accuracy: float
    run_dir: Path
    manifest_path: Path
    results_path: Path
    records_path: Path


def run_space_task(
    *,
    task_name: str,
    groups: int,
    seed: int,
    workers: int = DEFAULT_WORKERS,
    output: Path | None = None,
) -> SpaceRunSummary:
    """Sample a subset, let SPACE score it through the DimOS path, and cross-check."""
    task = space_text_task(task_name)
    if workers < 1:
        raise ValueError(f"workers must be at least 1, got {workers}")
    run_dir = (output or default_run_dir(task.name)).expanduser().resolve()
    if (run_dir / MANIFEST_NAME).is_file() or (run_dir / SPACE_OUTPUT_DIR_NAME).exists():
        raise FileExistsError(f"{run_dir} already holds a run; pass a fresh --output directory")
    # libobjc reads this flag when it loads, which happened at interpreter
    # start, so setting it here cannot change this process or its forked
    # workers. It is for the processes those workers exec (the Pi CLI): they
    # inherit the environment and their fresh libobjc honours it. If the fork
    # itself hits an Objective-C abort, export the flag in the shell instead.
    os.environ.setdefault("OBJC_DISABLE_INITIALIZE_FORK_SAFETY", "YES")

    source = ensure_space_source()
    data_root = resolve_space_data()
    adapter = SpaceQAAdapter.from_data_root(task, data_root)
    items = adapter.iter_items(SubsetSpec(seed=seed, groups=groups))

    run_dir.mkdir(parents=True, exist_ok=True)
    manifest = prepare_run(
        run_dir,
        adapter,
        items,
        task_name=task.name,
        seed=seed,
        groups=groups,
        data_sha256=release_sha256(data_root),
    )

    # Fork, not spawn: the SPACE registries live only in this process's memory,
    # and a spawned child would come up with neither the agent nor the config.
    mp.set_start_method("fork", force=True)
    if str(source) not in sys.path:
        sys.path.insert(0, str(source))
    importlib.import_module(AGENT_MODULE)
    os.environ[RUN_DIR_ENV] = str(run_dir)

    from space.evaluate_qas import main as evaluate_qas_main  # type: ignore[import-not-found]

    space_output = run_dir / SPACE_OUTPUT_DIR_NAME
    evaluate_qas_main(
        model_name=CONFIG_NAME,
        data_path=str(subset_path(run_dir)),
        save_dir=str(space_output),
        n_workers=workers,
    )

    results_path = locate_results(space_output)
    results = json.loads(results_path.read_text(encoding="utf-8"))
    records = collect_records(results_path.parent, manifest)
    cross_check_predictions(records, results)
    records_path = run_dir / RECORDS_NAME
    records_path.write_text(
        "".join(
            json.dumps(record, ensure_ascii=False, sort_keys=True) + "\n" for record in records
        ),
        encoding="utf-8",
    )
    return SpaceRunSummary(
        task=task.name,
        seed=seed,
        groups=groups,
        questions=len(records),
        mean_accuracy=float(results["mean_metrics"]["accuracy"]),
        run_dir=run_dir,
        manifest_path=run_dir / MANIFEST_NAME,
        results_path=results_path,
        records_path=records_path,
    )


def default_run_dir(task_name: str) -> Path:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    return space_cache_root() / "runs" / task_name / stamp


def prepare_run(
    run_dir: Path,
    adapter: SpaceQAAdapter,
    items: Sequence[BenchmarkItem],
    *,
    task_name: str,
    seed: int,
    groups: int,
    data_sha256: str | None,
) -> RunManifest:
    """Write the subset, one case per question, and the manifest that ties them together."""
    if not items:
        raise ValueError("a run needs at least one question")
    subset = subset_path(run_dir)
    subset.parent.mkdir(parents=True, exist_ok=True)
    # ensure_ascii on purpose: evaluate_qas opens this file with a bare open(),
    # so a non-ASCII subset would be read through the ambient locale encoding.
    subset.write_text(json.dumps(adapter.upstream_rows(items)), encoding="utf-8")

    for subset_index, item in enumerate(items):
        directory = case_dir(run_dir, subset_index)
        directory.mkdir(parents=True, exist_ok=True)
        (directory / "case.json").write_text(
            adapter.to_case(item).model_dump_json(indent=2) + "\n", encoding="utf-8"
        )

    manifest = build_manifest(
        benchmark=adapter.name,
        task=task_name,
        seed=seed,
        groups=groups,
        space_revision=SPACE_REVISION,
        data_sha256=data_sha256,
        items=items,
    )
    write_manifest(run_dir, manifest)
    return manifest


def locate_results(space_output: Path) -> Path:
    """Find the ``results.json`` SPACE wrote under the timestamp it chose for itself."""
    found = sorted(space_output.glob(f"{CONFIG_NAME}/*/results.json"))
    if len(found) != 1:
        raise FileNotFoundError(
            f"expected exactly one results.json under {space_output}, found {len(found)}"
        )
    return found[0]


def collect_records(results_dir: Path, manifest: RunManifest) -> list[dict[str, Any]]:
    """Read back the per-question record each worker wrote, in subset order."""
    records = []
    for row in sorted(manifest.rows, key=lambda selected: selected.subset_index):
        path = results_dir / qa_dir_name(row.subset_index) / RECORD_NAME
        if not path.is_file():
            raise FileNotFoundError(f"question {row.subset_index} left no record at {path}")
        record = json.loads(path.read_text(encoding="utf-8"))
        if record.get("question_sha256") != row.question_sha256:
            raise ValueError(
                f"question {row.subset_index} was answered as "
                f"{record.get('question_sha256')}, but the manifest selected "
                f"{row.question_sha256}"
            )
        records.append(record)
    return records


def cross_check_predictions(
    records: Sequence[Mapping[str, Any]], results: Mapping[str, Any]
) -> None:
    """Refuse a run whose two records of the same answers disagree.

    SPACE's ``results.json`` is the score. These records are how a reader gets
    from that number back to a question, a transcript and a failure reason, and
    they are only worth reading if they describe the same run.
    """
    predictions = results.get("all_predictions")
    if not isinstance(predictions, list):
        raise ValueError("the official results.json carries no all_predictions list")
    if len(predictions) != len(records):
        raise ValueError(
            f"the official results.json scored {len(predictions)} questions, "
            f"but {len(records)} were recorded"
        )
    for index, (record, prediction) in enumerate(zip(records, predictions, strict=True)):
        if record.get("pred") != prediction:
            raise ValueError(
                f"question {index} was recorded as {record.get('pred')!r} but scored "
                f"as {prediction!r}"
            )
