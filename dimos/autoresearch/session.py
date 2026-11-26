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

"""A durable search over immutable candidate artifacts and repeatable trials."""

from __future__ import annotations

from collections.abc import Callable, Iterator
from contextlib import contextmanager
import json
from pathlib import Path
import re
from statistics import fmean
import tempfile
from typing import Literal

from filelock import FileLock, Timeout
from pydantic import AliasChoices, BaseModel, Field, FiniteFloat, JsonValue

from dimos.autoresearch.storage import artifact_path, read_manifest, read_trials, write_text

Split = Literal["dev", "heldout"]
Direction = Literal["maximize", "minimize"]
SessionStatus = Literal["idle", "running", "frozen", "scored"]


class Budget(BaseModel):
    max_candidates: int = Field(default=15, gt=0)
    dev_trials: int = Field(default=5, gt=0)
    heldout_trials: int = Field(default=10, gt=0)


class SessionInfo(BaseModel):
    task: str
    budget: Budget
    experiment: str
    metric: str
    direction: Direction = "maximize"
    settings: dict[str, JsonValue] = Field(default_factory=dict)
    model: str = ""
    status: SessionStatus = "idle"
    frozen_candidate: int | None = None
    heldout_score: FiniteFloat | None = Field(
        default=None, validation_alias=AliasChoices("heldout_score", "heldout_rate")
    )


class Trial(BaseModel):
    """Everything an evaluator receives. Paths are absolute; outputs belong in directory."""

    task: str
    artifact: Path
    directory: Path
    split: Split
    index: int


class Evaluation(BaseModel):
    """One measurement in the session's metric and any evidence explaining it."""

    score: FiniteFloat
    note: str = ""
    evidence: dict[str, JsonValue] = Field(default_factory=dict)


class TrialRecord(Evaluation):
    directory: Path | None = None
    split: Split
    index: int


class CandidateScore(BaseModel):
    candidate: int
    dev_trials: int
    dev_score: float | None


class Session:
    def __init__(self, directory: Path) -> None:
        self.dir = directory.resolve()
        if not (self.dir / "session.json").is_file():
            raise FileNotFoundError(f"{directory} holds no session.json")

    @classmethod
    def create(cls, directory: Path, info: SessionInfo) -> Session:
        directory.mkdir(parents=True, exist_ok=True)
        with FileLock(directory / ".session.lock"):
            manifest = directory / "session.json"
            if manifest.exists():
                raise FileExistsError(f"{directory} already holds a session")
            (directory / "candidates").mkdir(exist_ok=True)
            (directory / "library").mkdir(exist_ok=True)
            write_text(manifest, info.model_dump_json(indent=2))
        return cls(directory)

    @property
    def name(self) -> str:
        return self.dir.name

    def info(self) -> SessionInfo:
        return SessionInfo.model_validate(read_manifest(self.dir))

    def _save(self, info: SessionInfo) -> None:
        write_text(self.dir / "session.json", info.model_dump_json(indent=2))

    def set_model(self, model: str) -> None:
        with FileLock(self.dir / ".session.lock"):
            info = self.info()
            info.model = model
            self._save(info)

    def candidates(self) -> list[int]:
        return sorted(
            int(p.name) for p in (self.dir / "candidates").iterdir() if p.name.isdecimal()
        )

    def candidate_dir(self, candidate: int) -> Path:
        path = self.dir / "candidates" / f"{candidate:02d}"
        if not path.is_dir():
            raise KeyError(f"session {self.name} has no candidate {candidate}")
        return path

    def artifact(self, candidate: int) -> Path:
        return artifact_path(self.candidate_dir(candidate))

    def hypothesis(self, candidate: int) -> str:
        return (self.candidate_dir(candidate) / "hypothesis.md").read_text()

    def submit(self, filename: str, content: bytes, hypothesis: str) -> int:
        """Store an opaque file and the reason for trying it."""
        if not filename or filename in (".", "..") or Path(filename).name != filename:
            raise ValueError("candidate filename must be a single file name")
        with FileLock(self.dir / ".session.lock"):
            info = self.info()
            if info.frozen_candidate is not None:
                raise RuntimeError(f"cannot submit: session {self.name} is {info.status}")
            number = len(self.candidates()) + 1
            if number > info.budget.max_candidates:
                raise RuntimeError(
                    f"the budget of {info.budget.max_candidates} candidates is spent"
                )
            candidates = self.dir / "candidates"
            with tempfile.TemporaryDirectory(dir=candidates, prefix=".submit-") as temp:
                path = Path(temp) / "candidate"
                (path / "artifact").mkdir(parents=True)
                artifact = path / "artifact" / filename
                (path / "artifact-name").write_text(filename)
                artifact.write_bytes(content)
                artifact.chmod(0o444)
                (path / "hypothesis.md").write_text(hypothesis)
                (path / "results.json").write_text("[]")
                path.rename(candidates / f"{number:02d}")
        return number

    def results(self, candidate: int) -> list[TrialRecord]:
        records = read_trials(self.candidate_dir(candidate))
        return [TrialRecord.model_validate(record) for record in records]

    def leaderboard(self) -> list[CandidateScore]:
        board = []
        for candidate in self.candidates():
            scores = [r.score for r in self.results(candidate) if r.split == "dev"]
            board.append(
                CandidateScore(
                    candidate=candidate,
                    dev_trials=len(scores),
                    dev_score=fmean(scores) if scores else None,
                )
            )
        sign = -1 if self.info().direction == "maximize" else 1
        return sorted(
            board,
            key=lambda row: (row.dev_score is None, sign * (row.dev_score or 0.0), row.candidate),
        )

    def pending(self, candidate: int, split: Split) -> list[int]:
        info = self.info()
        if split == "dev" and info.frozen_candidate is not None:
            raise RuntimeError(
                f"the session is frozen on candidate {info.frozen_candidate}; dev runs are over"
            )
        if split == "heldout" and info.frozen_candidate != candidate:
            raise RuntimeError("held-out trials run on the frozen candidate: call freeze")
        done = {r.index for r in self.results(candidate) if r.split == split}
        count = info.budget.dev_trials if split == "dev" else info.budget.heldout_trials
        return [i for i in range(count) if i not in done]

    @contextmanager
    def _exclusive_run(self) -> Iterator[None]:
        lock = FileLock(self.dir / ".run.lock")
        try:
            lock.acquire(timeout=0)
        except Timeout as exc:
            raise RuntimeError("a trial run is already in progress") from exc
        try:
            yield
        finally:
            lock.release()

    def freeze(self, candidate: int) -> None:
        """Commit to one candidate. Repeating the same choice permits resuming evaluation."""
        with self._exclusive_run(), FileLock(self.dir / ".session.lock"):
            self.candidate_dir(candidate)
            info = self.info()
            if info.frozen_candidate == candidate:
                return
            if info.frozen_candidate is not None:
                raise RuntimeError(f"session is frozen on candidate {info.frozen_candidate}")
            info.frozen_candidate = candidate
            info.status = "frozen"
            self._save(info)

    def evaluate(
        self,
        candidate: int,
        split: Split,
        evaluator: Callable[[Trial], Evaluation],
        *,
        on_trial: Callable[[int], None] | None = None,
    ) -> None:
        """Measure pending trials, checkpoint each result, and score a completed held-out split.

        Evaluator exceptions leave the current index pending. A fresh attempt gets its own
        output directory so evidence from an interrupted trial cannot contaminate the retry.
        """
        with self._exclusive_run():
            pending = self.pending(candidate, split)
            info = self.info()
            base = self.candidate_dir(candidate)
            if split == "dev":
                self._set_phase("running")
            try:
                records = self.results(candidate)
                for index in pending:
                    if on_trial is not None:
                        on_trial(index)
                    directory = base / "trials" / f"{split}-{index:02d}"
                    directory.parent.mkdir(exist_ok=True)
                    if directory.exists():
                        directory = Path(
                            tempfile.mkdtemp(dir=directory.parent, prefix=f"{directory.name}-")
                        )
                    else:
                        directory.mkdir()
                    trial = Trial(
                        task=info.task,
                        artifact=self.artifact(candidate),
                        directory=directory,
                        split=split,
                        index=index,
                    )
                    measurement = evaluator(trial)
                    records.append(
                        TrialRecord(
                            directory=directory,
                            split=split,
                            index=index,
                            **measurement.model_dump(),
                        )
                    )
                    write_text(
                        base / "results.json",
                        json.dumps([r.model_dump(mode="json") for r in records], indent=2),
                    )
                if split == "heldout":
                    self._set_phase("scored", fmean(r.score for r in records if r.split == split))
            finally:
                if split == "dev":
                    self._set_phase("idle")

    def _set_phase(self, phase: SessionStatus, score: float | None = None) -> None:
        with FileLock(self.dir / ".session.lock"):
            info = self.info()
            info.status = phase
            info.heldout_score = score
            self._save(info)

    def note(self, title: str, text: str) -> Path:
        slug = re.sub(r"[^a-z0-9]+", "-", title.lower()).strip("-")
        if not slug:
            raise ValueError(f"title {title!r} has no letters or digits to name a file by")
        path = self.dir / "library" / f"{slug}.md"
        write_text(path, f"# {title}\n\n{text.strip()}\n")
        return path

    def library(self) -> list[Path]:
        return sorted((self.dir / "library").glob("*.md"))
