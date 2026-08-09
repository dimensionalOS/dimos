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

"""What a run selected, written before it starts and read back by its workers.

A SPACE run reports one accuracy number. The manifest is what makes that
number mean something later: which rows were drawn, from which release, under
which scorer, against which build of this repository. It also carries the run
directory to the worker processes, which SPACE constructs itself and hands
nothing but a config of defaults.
"""

from __future__ import annotations

from collections.abc import Iterable
import json
import os
from pathlib import Path
import subprocess

from pydantic import Field

from dimos.benchmark.space_qa.adapter import BenchmarkItem, BenchmarkModel

# Set by ``run_space_task`` before the pool forks; read by each worker.
RUN_DIR_ENV = "DIMOS_SPACE_RUN_DIR"

MANIFEST_NAME = "manifest.json"
CASES_DIR_NAME = "cases"
SUBSET_DIR_NAME = "subset"
RECORD_NAME = "dimos_case.json"
# Registered with SPACE by the agent and asked for by name by the run; SPACE
# connects the two only by equality.
CONFIG_NAME = "dimos_qa"

SHA256_LENGTH = 64


class SelectedRow(BenchmarkModel):
    """One drawn question: where SPACE sees it, where it lives upstream, and what it says."""

    subset_index: int = Field(ge=0)
    ordinal: int = Field(ge=0)
    question_sha256: str = Field(min_length=SHA256_LENGTH, max_length=SHA256_LENGTH)


class RunManifest(BenchmarkModel):
    benchmark: str = Field(min_length=1)
    task: str = Field(min_length=1)
    seed: int
    groups: int = Field(ge=1)
    space_revision: str = Field(min_length=1)
    # Digest of the archive the release was unpacked from, on the word of the
    # provenance record beside it. None when the release was reached through an
    # override and carries no such record.
    data_sha256: str | None = None
    # Digest of the bytes of the task's qas.json this run actually read, taken
    # while reading it. Where data_sha256 repeats a claim about the release,
    # this one is evidence about the file the questions came out of.
    qas_sha256: str | None = None
    # None when this is not running from a git checkout.
    dimos_revision: str | None = None
    rows: tuple[SelectedRow, ...]

    def row_for(self, subset_index: int) -> SelectedRow:
        for row in self.rows:
            if row.subset_index == subset_index:
                return row
        raise KeyError(f"the run selected no question at subset index {subset_index}")


def build_manifest(
    *,
    benchmark: str,
    task: str,
    seed: int,
    groups: int,
    space_revision: str,
    data_sha256: str | None,
    qas_sha256: str | None,
    items: Iterable[BenchmarkItem],
) -> RunManifest:
    return RunManifest(
        benchmark=benchmark,
        task=task,
        seed=seed,
        groups=groups,
        space_revision=space_revision,
        data_sha256=data_sha256,
        qas_sha256=qas_sha256,
        dimos_revision=dimos_revision(),
        rows=tuple(
            SelectedRow(
                subset_index=subset_index,
                ordinal=item.ordinal,
                question_sha256=item.question_sha256,
            )
            for subset_index, item in enumerate(items)
        ),
    )


def write_manifest(run_dir: Path, manifest: RunManifest) -> Path:
    path = run_dir / MANIFEST_NAME
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(manifest.model_dump(mode="json"), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return path


def read_manifest(run_dir: Path) -> RunManifest:
    return RunManifest.model_validate_json((run_dir / MANIFEST_NAME).read_bytes())


def qa_dir_name(subset_index: int) -> str:
    """The directory name ``evaluate_qas`` gives a question, by its place in the subset."""
    return f"qa_{subset_index:05d}"


def case_dir(run_dir: Path, subset_index: int) -> Path:
    """Mirrors the ``qa_XXXXX`` directory SPACE gives the worker for the same row."""
    return run_dir / CASES_DIR_NAME / qa_dir_name(subset_index)


def subset_path(run_dir: Path) -> Path:
    return run_dir / SUBSET_DIR_NAME / "qas.json"


def run_dir_from_env() -> Path:
    location = os.environ.get(RUN_DIR_ENV)
    if not location:
        raise RuntimeError(f"{RUN_DIR_ENV} is unset; this agent only runs under run_space_task")
    run_dir = Path(location)
    if not (run_dir / MANIFEST_NAME).is_file():
        raise FileNotFoundError(f"{RUN_DIR_ENV}={location} holds no {MANIFEST_NAME}")
    return run_dir


def dimos_revision() -> str | None:
    """The commit a score was produced on, or None outside a checkout."""
    repository = Path(__file__).resolve().parents[3]
    try:
        completed = subprocess.run(
            ["git", "-C", str(repository), "rev-parse", "HEAD"],
            capture_output=True,
            text=True,
            timeout=30,
            check=False,
        )
    except OSError:
        return None
    return completed.stdout.strip() or None if completed.returncode == 0 else None
