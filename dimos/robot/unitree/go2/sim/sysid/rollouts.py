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

"""Segment rollouts across processes. The physics is a pure function; use that.

The rule for parallelising the fit: PARALLELISE WHERE THE COMPUTATION IS A
PURE FUNCTION; KEEP THE SAMPLER SEQUENTIAL. A segment rollout is a pure
function of (knob values, plan) — the plan is decided before any physics — so
segments fan out across processes and reassemble in input order, which speeds
up every trial, inner and outer, and every Jacobian. Trials within one study
must NOT be parallelised: CMA-ES updates its distribution from the history it
has seen, so parallel trials change what each trial sees and the result stops
being reproducible from a seed — and seeded reproducibility is what the whole
restart-and-median argument rests on.

Serial and parallel are BIT-IDENTICAL by construction: both paths run
:func:`_eval`, byte for byte — a worker differs only in which process the pure
function runs in. Workers receive the CONFIGURED BACKEND itself, pickled
(a seam requirement — see :class:`~dimos.robot.unitree.go2.sim.backend
.Backend`), so no engine is named here and a different simulator
parallelises by construction. MuJoCo is not thread-safe on shared
``MjData``, hence processes, each compiling its own model (which
:class:`~dimos.robot.unitree.go2.sim.engines.mujoco.MujocoBackend` does per rollout
anyway).
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
import concurrent.futures
from dataclasses import dataclass
import multiprocessing
from pathlib import Path
import threading

import numpy as np

from dimos.robot.unitree.go2.sim.backend import Backend
from dimos.robot.unitree.go2.sim.sysid.ingest import GO2_READER
from dimos.robot.unitree.go2.sim.sysid.recording import (
    RecordingReader,
    Streams,
    read_recording,
)
from dimos.robot.unitree.go2.sim.sysid.replay import ReplayResult, replay


@dataclass(frozen=True)
class RolloutSpec:
    """Everything one rollout needs, decided before any physics runs.

    The spec carries the FULL schedule inputs — window, seed, protect — so a
    task is self-contained and two specs that should share a clip schedule do
    so because the caller gave them the same fields, visibly, not because of
    executor state.
    """

    values: Mapping[str, float]  # knob values for backend.apply
    t0: float
    duration: float
    window: float | tuple[float, float] | None
    seed: int
    suspended: bool = False
    protect: np.ndarray | None = None


def _eval(backend: Backend, st: Streams, spec: RolloutSpec) -> ReplayResult:
    """THE evaluation, serial and parallel alike."""
    backend.apply(spec.values)
    return replay(
        st,
        spec.t0,
        spec.duration,
        backend,
        window=spec.window,
        seed=spec.seed,
        protect=spec.protect,
        suspended=spec.suspended,
    )


# Worker-process state, set once by the initializer. One backend and one
# Streams per process; the backend arrives pickled from the parent, so
# serial and worker rollouts run identically configured engines, and every
# task recompiles its model, so nothing leaks between tasks.
_WORKER: dict[str, object] = {}


def _init_worker(recording: str, backend: Backend, reader: RecordingReader) -> None:
    _WORKER["st"] = read_recording(recording, reader)  # cache-warm: the parent read it first
    _WORKER["backend"] = backend


def _eval_in_worker(spec: RolloutSpec) -> ReplayResult:
    backend = _WORKER["backend"]
    st = _WORKER["st"]
    assert isinstance(st, Streams)
    return _eval(backend, st, spec)  # type: ignore[arg-type]


class Rollouts:
    """Evaluates rollout specs against one recording, serially or across processes.

    ``backend`` is the configured engine — an envelope-carrying
    :class:`~dimos.robot.unitree.go2.sim.engines.mujoco.MujocoBackend`, an Isaac
    backend, anything behind the seam. ``reader`` is the recording format
    (default: the Go2 DDS reader), pickled to workers exactly like the
    backend. ``workers <= 1`` runs in-process;
    more spawns that many worker processes (spawned, not forked — a forked
    MuJoCo is a bug that looks like a speedup), each receiving the SAME
    backend by pickle. Results always come back in input order.
    """

    def __init__(
        self,
        recording: str | Path,
        backend: Backend,
        *,
        workers: int = 1,
        reader: RecordingReader | None = None,
    ) -> None:
        self.recording = Path(recording)
        self.workers = workers
        self._reader = GO2_READER if reader is None else reader
        self.streams = read_recording(self.recording, self._reader)  # warms the worker cache
        self._backend = backend
        self._pool: concurrent.futures.ProcessPoolExecutor | None = None
        self._lock = threading.Lock()  # run() is called from study threads

    def _ensure_pool(self) -> concurrent.futures.ProcessPoolExecutor:
        with self._lock:
            if self._pool is None:
                self._pool = concurrent.futures.ProcessPoolExecutor(
                    max_workers=self.workers,
                    mp_context=multiprocessing.get_context("spawn"),
                    initializer=_init_worker,
                    initargs=(str(self.recording), self._backend, self._reader),
                )
        return self._pool

    def run(self, specs: Sequence[RolloutSpec]) -> list[ReplayResult]:
        if self.workers <= 1:
            # One in-process backend; the lock keeps concurrent study threads
            # from interleaving apply() and rollout() on it.
            with self._lock:
                return [_eval(self._backend, self.streams, s) for s in specs]
        pool = self._ensure_pool()
        futures = [pool.submit(_eval_in_worker, s) for s in specs]
        return [f.result() for f in futures]

    def close(self) -> None:
        if self._pool is not None:
            self._pool.shutdown()
            self._pool = None

    def __enter__(self) -> Rollouts:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()
