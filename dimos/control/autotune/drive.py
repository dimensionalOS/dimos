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

"""Drive the excitation battery through the coordinator, segmenting episodes.

The sequencer is the autotune-side piece that meets the live system. It plays
each :class:`ExcitationRun` as ONE episode:

    episode start  ->  play signal at tick rate for the run duration  ->  zero
    command  ->  episode save

Episode transitions are driven PROGRAMMATICALLY (the excitation sequencer knows
exactly when each run begins and ends), not by teleop keypresses - but the
recorded ``EpisodeStatus`` stream is the same one ``EpisodeMonitor`` emits, so
the learning recorder segments autotune runs exactly as it segments teleop
episodes. We do not fork ``EpisodeMonitor`` semantics; we just emit the same
events on a known schedule.

Everything here is defined against small Protocols so the sequencer is fully
unit-testable against mocks. The live adapters (publish a Twist on the
coordinator's ``twist_command``, publish ``EpisodeStatus``, real wall clock) are
thin wrappers - they cannot be validated without a running coordinator, but the
ordering/segmentation logic they depend on is pinned by the mock tests.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import Protocol, runtime_checkable

from dimos.control.autotune.excitation import ExcitationRun


@runtime_checkable
class CommandSink(Protocol):
    """Accepts a per-channel command and turns it into a robot command (e.g. a
    Twist on the coordinator). ``stop`` commands all channels to zero."""

    def send(self, channel_values: dict[str, float]) -> None: ...
    def stop(self) -> None: ...


@runtime_checkable
class EpisodeSink(Protocol):
    """Programmatic episode segmentation: emit start/save on the same
    ``EpisodeStatus`` timeline the recorder reads."""

    def start_episode(self, label: str) -> None: ...
    def save_episode(self) -> None: ...


@runtime_checkable
class Clock(Protocol):
    """Monotonic clock + sleep, injectable so tests run in virtual time."""

    def now(self) -> float: ...
    def sleep(self, seconds: float) -> None: ...


def play_run(
    run: ExcitationRun,
    sink: CommandSink,
    episodes: EpisodeSink,
    clock: Clock,
    *,
    tick_hz: float,
) -> int:
    """Play one excitation run as one episode. Returns the number of command
    ticks emitted.

    The excited channel is commanded by ``run.signal(t)``; all other channels
    are implicitly zero (the sink only receives the excited channel). On exit the
    sink is stopped (zeroed) and the episode is saved, even if the body raised -
    we never leave the base commanded."""
    if tick_hz <= 0:
        raise ValueError("tick_hz must be > 0")
    dt = 1.0 / tick_hz
    n_ticks = max(1, round(run.duration_s * tick_hz))
    episodes.start_episode(run.label)
    emitted = 0
    try:
        t0 = clock.now()
        for i in range(n_ticks):
            t = i * dt
            sink.send({run.channel: run.signal(t)})
            emitted += 1
            # Pace to the tick boundary relative to the run start.
            target = t0 + (i + 1) * dt
            remaining = target - clock.now()
            if remaining > 0:
                clock.sleep(remaining)
    finally:
        sink.stop()
        episodes.save_episode()
    return emitted


def run_battery(
    runs: Sequence[ExcitationRun],
    sink: CommandSink,
    episodes: EpisodeSink,
    clock: Clock,
    *,
    tick_hz: float,
    settle_s: float = 1.0,
) -> int:
    """Play a whole battery: each run is its own episode, with a settle dwell
    (zeroed command) between runs so the base returns to rest before the next
    excitation. Returns the number of episodes played.

    The base is NOT repositioned automatically - the operator does that between
    runs in the existing playbook (teleop during the dwell). ``settle_s`` only
    guarantees a commanded-zero rest, not a returned-to-origin pose."""
    played = 0
    for run in runs:
        play_run(run, sink, episodes, clock, tick_hz=tick_hz)
        played += 1
        # Inter-run settle: hold zero so transients die before the next episode.
        if settle_s > 0:
            sink.stop()
            clock.sleep(settle_s)
    return played
