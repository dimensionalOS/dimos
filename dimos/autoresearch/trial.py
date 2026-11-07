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

"""Running one candidate on one trial: reset the world, record an episode,
run the program, judge the result, store the record."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Protocol

from dimos.autoresearch.episodes import trial_episode
from dimos.autoresearch.judge import Judge, Snapshot
from dimos.autoresearch.program import run_program
from dimos.autoresearch.session import Session, Split, TrialRecord
from dimos.imitation.collection.episode_monitor import EpisodeControlSpec

# Held-out seeds start here so no held-out scene repeats a dev scene.
HELDOUT_SEED_BASE = 100_000


class World(Protocol):
    """The scene the program works in: ``reset(seed)`` lays it out the same
    way for the same seed, ``snapshot()`` reports where everything is."""

    def reset(self, seed: int) -> None: ...

    def snapshot(self) -> Snapshot: ...


class FakeWorld:
    """A scripted world for tests: ``reset`` records the seed, ``snapshot``
    plays back the given snapshots in order."""

    def __init__(self, snapshots: Sequence[Snapshot]) -> None:
        self._snapshots = iter(snapshots)
        self.seeds: list[int] = []

    def reset(self, seed: int) -> None:
        self.seeds.append(seed)

    def snapshot(self) -> Snapshot:
        return next(self._snapshots)


def trial_seed(split: Split, index: int) -> int:
    """The scene seed of trial ``index`` on ``split``; fixed, so every candidate sees the same scenes."""
    return index if split == "dev" else HELDOUT_SEED_BASE + index


def run_trial(
    session: Session,
    candidate: int,
    split: Split,
    index: int,
    world: World,
    judge: Judge,
    episodes: EpisodeControlSpec,
    backend_factory: str,
    timeout_s: float,
) -> TrialRecord:
    """Run ``candidate`` on trial ``index`` of ``split`` and store the record.

    ``backend_factory`` and ``timeout_s`` go to ``run_program``. The recorder
    episode is saved only when the judge says success. The program's own
    failure (raised, timeout) is noted on the record; the judge still decides
    success from the world.
    """
    trial_dir = session.trial_dir(candidate, split, index)
    world.reset(trial_seed(split, index))
    before = world.snapshot()
    with trial_episode(episodes) as episode:
        outcome = run_program(
            session.program_path(candidate),
            trial_dir,
            session.info().task,
            timeout_s,
            backend_factory,
        )
        verdict = judge.verdict(before, world.snapshot())
        episode.success = verdict.success
    note = verdict.note
    if outcome.status != "completed":
        last_line = outcome.error.strip().splitlines()[-1]
        note = f"{note}; program {outcome.status}: {last_line}"
    record = TrialRecord(
        split=split,
        index=index,
        success=verdict.success,
        stage=verdict.stage,
        note=note,
        status=outcome.status,
        duration_s=outcome.duration_s,
        trace=outcome.trace,
        episode=episode.index,
    )
    session.record_trial(candidate, record)
    return record
