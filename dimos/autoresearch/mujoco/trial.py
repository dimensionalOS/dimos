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

"""An arm experiment evaluates a program through scene reset, motion, judging and recording."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
import threading
from typing import Protocol

from pydantic import BaseModel

from dimos.autoresearch.mujoco.judge import Judge, Snapshot
from dimos.autoresearch.session import Evaluation, Split, Trial, TrialRecord
from dimos.imitation.collection.episode import EpisodeStatus
from dimos.imitation.collection.episode_monitor import EpisodeCommand, EpisodeControlSpec
from dimos.manipulation.code_policy.api import TRACE_NAME
from dimos.manipulation.code_policy.runner import ProgramStatus, run_program

HELDOUT_SEED_BASE = 100_000


class World(Protocol):
    def reset(self, seed: int) -> None: ...
    def snapshot(self) -> Snapshot: ...


class NoEpisodes:
    """A trial without recording has no saved episode index."""

    def get_status(self) -> EpisodeStatus:
        return EpisodeStatus(ts=0.0, state="idle", episodes_saved=0, episodes_discarded=0)

    def command(self, event: EpisodeCommand) -> EpisodeStatus:
        return self.get_status()


class ArmEvidence(BaseModel):
    stage: str
    status: ProgramStatus
    duration_s: float
    trace: Path
    start_ts: float
    episode: int | None
    backend: str


class ArmTrialRecord(ArmEvidence):
    """The arm-facing view of a stored measurement, also used by the skill exporter."""

    split: Split
    index: int
    success: bool
    note: str

    @classmethod
    def from_record(cls, record: TrialRecord) -> ArmTrialRecord:
        return cls.model_validate(
            dict(
                record.evidence,
                split=record.split,
                index=record.index,
                success=bool(record.score),
                note=record.note,
            )
        )


@dataclass
class ArmEvaluator:
    world: World
    judge: Judge
    episodes: EpisodeControlSpec
    backend_factory: str
    timeout_s: float
    env: Mapping[str, str] | None = None
    stop: threading.Event | None = None

    def __call__(self, trial: Trial) -> Evaluation:
        seed = trial.index if trial.split == "dev" else HELDOUT_SEED_BASE + trial.index
        self.world.reset(seed)
        before = self.world.snapshot()
        start = self.episodes.command("start")
        try:
            outcome = run_program(
                trial.artifact,
                trial.directory,
                trial.task,
                self.timeout_s,
                self.backend_factory,
                self.env,
                self.stop,
            )
            verdict = self.judge.verdict(before, self.world.snapshot())
        except BaseException:
            self.episodes.command("discard")
            raise
        episode = None
        if verdict.success:
            saved = self.episodes.command("save").episodes_saved
            episode = saved - 1 if saved else None
        else:
            self.episodes.command("discard")
        note = verdict.note
        if outcome.status != "completed":
            note = f"{note}; program {outcome.status}: {outcome.error.strip().splitlines()[-1]}"
        evidence = ArmEvidence(
            stage=verdict.stage,
            status=outcome.status,
            duration_s=outcome.duration_s,
            trace=trial.directory / TRACE_NAME,
            start_ts=start.ts,
            episode=episode,
            backend=self.backend_factory,
        )
        return Evaluation(
            score=float(verdict.success), note=note, evidence=evidence.model_dump(mode="json")
        )
