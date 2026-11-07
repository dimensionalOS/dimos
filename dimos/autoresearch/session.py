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

"""One autoresearch session on disk: the task and budget, the immutable
candidate programs, their trial results, the library notes and the frozen
choice. Everything is plain JSON and text under one directory, so any number
of ``Session`` objects can look at the same session."""

from __future__ import annotations

import json
from pathlib import Path
import re
from typing import Literal

from pydantic import BaseModel, Field

from dimos.autoresearch.program import ProgramStatus, validate_program
from dimos.constants import STATE_DIR

SESSIONS_DIR = STATE_DIR / "autoresearch"

Split = Literal["dev", "heldout"]


class Budget(BaseModel):
    """How much searching a session may do and the dev pass rate that ends
    it early."""

    iterations: int = Field(gt=0)
    candidates_per_round: int = Field(gt=0)
    dev_trials: int = Field(gt=0)
    heldout_trials: int = Field(gt=0)
    threshold: float = Field(ge=0.0, le=1.0)


class SessionInfo(BaseModel):
    """The contents of ``session.json``."""

    task: str
    budget: Budget
    model: str
    status: Literal["open", "frozen"] = "open"


class TrialRecord(BaseModel):
    """One trial of one candidate, as stored in its ``results.json``.

    ``stage`` and ``note`` come from the judge, ``status`` from the program
    run (completed, raised or timeout), ``trace`` is the absolute path of the
    call log and ``episode`` the index of the saved recorder episode, if any.
    """

    split: Split
    index: int
    success: bool
    stage: str
    note: str
    status: ProgramStatus
    duration_s: float
    trace: Path
    episode: int | None


class CandidateScore(BaseModel):
    """One candidate's tallies on each split."""

    candidate: int
    dev_passed: int
    dev_trials: int
    heldout_passed: int
    heldout_trials: int

    @property
    def dev_rate(self) -> float:
        return self.dev_passed / self.dev_trials if self.dev_trials else 0.0


class Session:
    """A session directory. Every method reads or writes the files under it,
    so a second ``Session`` on the same directory sees the same state."""

    def __init__(self, directory: Path) -> None:
        if not (directory / "session.json").is_file():
            raise FileNotFoundError(f"{directory} holds no session.json")
        self.dir = directory

    @classmethod
    def create(cls, directory: Path, task: str, budget: Budget, model: str) -> Session:
        """Start a new session in ``directory``, which must not exist yet."""
        if directory.exists():
            raise FileExistsError(f"{directory} already exists")
        directory.mkdir(parents=True)
        (directory / "candidates").mkdir()
        (directory / "library").mkdir()
        info = SessionInfo(task=task, budget=budget, model=model)
        (directory / "session.json").write_text(info.model_dump_json(indent=2))
        return cls(directory)

    @property
    def name(self) -> str:
        return self.dir.name

    def info(self) -> SessionInfo:
        return SessionInfo.model_validate_json((self.dir / "session.json").read_text())

    def candidates(self) -> list[int]:
        """Candidate numbers submitted so far, in order."""
        return sorted(int(path.name) for path in (self.dir / "candidates").iterdir())

    def candidate_dir(self, candidate: int) -> Path:
        path = self.dir / "candidates" / f"{candidate:02d}"
        if not path.is_dir():
            raise KeyError(f"session {self.name} has no candidate {candidate}")
        return path

    def program_path(self, candidate: int) -> Path:
        return self.candidate_dir(candidate) / "program.py"

    def hypothesis(self, candidate: int) -> str:
        return (self.candidate_dir(candidate) / "hypothesis.md").read_text()

    def submit(self, program_text: str, hypothesis: str) -> int:
        """Store a new candidate and return its number. The program must meet
        the contract in ``validate_program``; a frozen session refuses."""
        self._require_open("submit")
        validate_program(program_text)
        number = len(self.candidates()) + 1
        path = self.dir / "candidates" / f"{number:02d}"
        path.mkdir()
        for name, text in (("program.py", program_text), ("hypothesis.md", hypothesis)):
            (path / name).write_text(text)
            (path / name).chmod(0o444)
        (path / "results.json").write_text("[]")
        return number

    def record_trial(self, candidate: int, record: TrialRecord) -> None:
        path = self.candidate_dir(candidate) / "results.json"
        records = [*self.results(candidate), record]
        path.write_text(json.dumps([item.model_dump(mode="json") for item in records], indent=2))

    def results(self, candidate: int) -> list[TrialRecord]:
        path = self.candidate_dir(candidate) / "results.json"
        return [TrialRecord.model_validate(item) for item in json.loads(path.read_text())]

    def trial_dir(self, candidate: int, split: Split, index: int) -> Path:
        """Make the directory for one trial's trace and keyframes. A trial
        runs once per candidate, so an existing directory is refused."""
        path = self.candidate_dir(candidate) / "trials" / f"{split}-{index:02d}"
        if path.exists():
            raise FileExistsError(f"candidate {candidate} already ran trial {split}-{index}")
        path.mkdir(parents=True)
        return path

    def leaderboard(self) -> list[CandidateScore]:
        """Every candidate's tallies, best dev pass rate first."""
        scores = []
        for candidate in self.candidates():
            records = self.results(candidate)
            dev = [record for record in records if record.split == "dev"]
            heldout = [record for record in records if record.split == "heldout"]
            scores.append(
                CandidateScore(
                    candidate=candidate,
                    dev_passed=sum(record.success for record in dev),
                    dev_trials=len(dev),
                    heldout_passed=sum(record.success for record in heldout),
                    heldout_trials=len(heldout),
                )
            )
        return sorted(scores, key=lambda score: (-score.dev_rate, score.candidate))

    def freeze(self, candidate: int) -> None:
        """Name the final candidate. After this nothing can be submitted."""
        self._require_open("freeze")
        self.candidate_dir(candidate)
        (self.dir / "frozen").write_text(f"{candidate}\n")
        info = self.info().model_copy(update={"status": "frozen"})
        (self.dir / "session.json").write_text(info.model_dump_json(indent=2))

    def frozen(self) -> int | None:
        """The frozen candidate's number, or None while the session is open."""
        path = self.dir / "frozen"
        return int(path.read_text()) if path.is_file() else None

    def note(self, title: str, text: str) -> Path:
        """Write a library entry (``library/<slug>.md``) and return its path."""
        slug = re.sub(r"[^a-z0-9]+", "-", title.lower()).strip("-")
        if not slug:
            raise ValueError(f"title {title!r} has no letters or digits to name a file by")
        path = self.dir / "library" / f"{slug}.md"
        path.write_text(f"# {title}\n\n{text.strip()}\n")
        return path

    def library(self) -> list[Path]:
        """The library entries, in name order."""
        return sorted((self.dir / "library").glob("*.md"))

    def _require_open(self, action: str) -> None:
        if self.info().status != "open":
            raise RuntimeError(f"cannot {action}: session {self.name} is frozen")
