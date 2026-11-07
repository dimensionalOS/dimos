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

"""Running one candidate on one trial: reset the world, start a recorder
episode, run the program, judge the result, save or discard the episode,
store the record."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Protocol

from dimos.autoresearch.judge import Judge, Snapshot
from dimos.autoresearch.program import TRACE_NAME, run_program
from dimos.autoresearch.session import Session, Split, TrialRecord
from dimos.imitation.collection.episode import EpisodeStatus
from dimos.imitation.collection.episode_monitor import EpisodeCommand, EpisodeControlSpec

# Held-out seeds start here so no held-out scene repeats a dev scene.
HELDOUT_SEED_BASE = 100_000


class World(Protocol):
    """The scene the program works in: ``reset(seed)`` lays it out the same
    way for the same seed, ``snapshot()`` reports where everything is."""

    def reset(self, seed: int) -> None: ...

    def snapshot(self) -> Snapshot: ...


class NoEpisodes:
    """Stands in for the recorder on a stack without one: commands are
    accepted, nothing is recorded and no trial gets an episode index."""

    def get_status(self) -> EpisodeStatus:
        return _nothing_recorded()

    def command(self, event: EpisodeCommand) -> EpisodeStatus:
        return _nothing_recorded()


def _nothing_recorded() -> EpisodeStatus:
    return EpisodeStatus(ts=0.0, state="idle", episodes_saved=0, episodes_discarded=0)


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
    env: Mapping[str, str] | None = None,
) -> TrialRecord:
    """Run ``candidate`` on trial ``index`` of ``split`` and store the record.
    ``timeout_s`` (seconds), ``backend_factory`` and ``env`` go to ``run_program``.
    The recorder episode is saved only when the judge says success; the judge
    decides from the world even when the program raised or timed out, and the
    record's note then says so."""
    trial_dir = session.trial_dir(candidate, split, index)
    world.reset(trial_seed(split, index))
    before = world.snapshot()
    start = episodes.command("start")
    try:
        outcome = run_program(
            session.program_path(candidate),
            trial_dir,
            session.info().task,
            timeout_s,
            backend_factory,
            env,
        )
        verdict = judge.verdict(before, world.snapshot())
    except BaseException:
        episodes.command("discard")
        raise
    episode: int | None = None
    if verdict.success:
        saved = episodes.command("save").episodes_saved
        episode = saved - 1 if saved else None
    else:
        episodes.command("discard")
    note = verdict.note
    if outcome.status != "completed":
        note = f"{note}; program {outcome.status}: {outcome.error.strip().splitlines()[-1]}"
    record = TrialRecord(
        split=split,
        index=index,
        success=verdict.success,
        stage=verdict.stage,
        note=note,
        status=outcome.status,
        duration_s=outcome.duration_s,
        trace=trial_dir / TRACE_NAME,
        start_ts=start.ts,
        episode=episode,
        backend=backend_factory,
    )
    session.record_trial(candidate, record)
    return record
