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

"""DataPrepModule — wraps the dataprep pipeline as a Module with RPC surface.

All dataset-shape types and pure helpers live in `dataprep.py`. This file
just adds the Module lifecycle + thread + status tracking.
"""

from __future__ import annotations

import json
import threading
import traceback
from collections.abc import Iterator
from pathlib import Path
from typing import Any

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.learning.dataprep import (
    EpisodeExtractor,
    OutputConfig,
    Sample,
    StreamField,
    SyncConfig,
    extract_episodes,
    get_writer,
    iter_episode_samples,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class DataPrepModuleConfig(ModuleConfig):
    # Fields are defaulted so partial CLI overrides (e.g. just `source=...`)
    # pass blueprint validation; blueprint atoms supply real values.
    source:      str = ""
    episodes:    EpisodeExtractor = EpisodeExtractor()
    observation: dict[str, StreamField] = {}
    action:      dict[str, StreamField] = {}
    sync:        SyncConfig = SyncConfig(anchor="image", rate_hz=30.0, tolerance_ms=50.0)
    output:      OutputConfig = OutputConfig(format="lerobot", path="data/datasets/default")
    auto_run:    bool = False


class DataPrepModule(Module):
    """Wraps a long-running dataset build job."""

    config: DataPrepModuleConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._status: dict[str, Any] = {
            "state":         "idle",   # idle | running | succeeded | failed
            "current_phase": None,     # scan_episodes | write | done
            "progress_pct":  0.0,
            "dataset_path":  None,
            "error":         None,
            "episodes_seen": 0,
            "samples_seen":  0,
        }

    # ── lifecycle ────────────────────────────────────────────────────────────

    @rpc
    def start(self) -> None:
        super().start()
        if self.config.auto_run:
            self.build()

    @rpc
    def stop(self) -> None:
        # Build thread is daemon: dies with the process. No mid-iteration interrupt.
        super().stop()

    @rpc
    def build(self) -> None:
        """Spawn a daemon thread running the build pipeline. Returns immediately."""
        with self._lock:
            if self._status["state"] == "running":
                return
            self._status.update(
                state="running",
                current_phase=None,
                progress_pct=0.0,
                dataset_path=None,
                error=None,
                episodes_seen=0,
                samples_seen=0,
            )
        self._thread = threading.Thread(target=self._run_build, daemon=True)
        self._thread.start()

    @rpc
    def get_status(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._status)

    @rpc
    def inspect(self) -> dict[str, Any]:
        """Read-only summary: episode count, drop rates, joint names, stats presence."""
        from dimos.memory2.store.sqlite import SqliteStore

        store = SqliteStore(path=self.config.source, must_exist=True)
        try:
            episodes = extract_episodes(store, self.config.episodes)
            saved = sum(1 for e in episodes if e.success)
            dropped = sum(1 for e in episodes if not e.success)
            durations = [e.duration for e in episodes if e.success]
            return {
                "source":         self.config.source,
                "streams":        store.list_streams(),
                "episodes_saved": saved,
                "episodes_dropped": dropped,
                "duration_s":     {
                    "min":  min(durations) if durations else 0.0,
                    "max":  max(durations) if durations else 0.0,
                    "mean": (sum(durations) / len(durations)) if durations else 0.0,
                },
            }
        finally:
            store.stop()

    # ── internals ────────────────────────────────────────────────────────────

    def _run_build(self) -> None:
        """Thread target. Opens session.db, walks samples episode-by-episode,
        drives the format writer, snapshots config to <output.path>/dimos_meta.json.
        Updates _status under _lock.
        """
        try:
            logger.info(
                "[dataprep] starting build  source=%s  extractor=%s  output=%s",
                self.config.source,
                self.config.episodes.extractor,
                self.config.output.path,
            )
            self._update_status(current_phase="scan_episodes")

            from dimos.memory2.store.sqlite import SqliteStore

            store = SqliteStore(path=self.config.source, must_exist=True)
            try:
                logger.info("[dataprep] streams in source: %s", store.list_streams())
                all_eps = extract_episodes(store, self.config.episodes)
                episodes = [e for e in all_eps if e.success]
                logger.info(
                    "[dataprep] episodes extracted: %d total / %d successful",
                    len(all_eps), len(episodes),
                )
                self._update_status(episodes_seen=len(episodes))

                if not episodes:
                    raise RuntimeError(
                        f"No successful episodes extracted from {self.config.source!r} "
                        f"using extractor={self.config.episodes.extractor!r}. "
                        f"Available streams: {store.list_streams()}. "
                        f"For a single-demo .db with no episode_status stream, use "
                        f"extractor='whole_session' or 'ranges'."
                    )

                streams = {**self.config.observation, **self.config.action}
                obs_keys = set(self.config.observation)
                action_keys = set(self.config.action)
                logger.info(
                    "[dataprep] obs streams=%s  action streams=%s  sync=%s",
                    sorted(obs_keys), sorted(action_keys),
                    self.config.sync.model_dump(),
                )

                writer = get_writer(self.config.output.format)

                self._update_status(current_phase="write")
                logger.info(
                    "[dataprep] writing %s dataset to %s",
                    self.config.output.format, self.config.output.path,
                )

                samples_seen = 0
                episodes_done = 0
                total = len(episodes)

                def _all_samples() -> Iterator[Sample]:
                    nonlocal samples_seen, episodes_done
                    for ep in episodes:
                        for sample in iter_episode_samples(
                            store=store,
                            episode=ep,
                            streams=streams,
                            sync=self.config.sync,
                            obs_keys=obs_keys,
                            action_keys=action_keys,
                        ):
                            samples_seen += 1
                            if samples_seen % 50 == 0:
                                self._update_status(
                                    samples_seen=samples_seen,
                                    progress_pct=100.0 * episodes_done / total,
                                )
                                logger.info(
                                    "[dataprep] %.1f%%  samples=%d  ep %d/%d",
                                    100.0 * episodes_done / total,
                                    samples_seen, episodes_done, total,
                                )
                            yield sample
                        episodes_done += 1
                        self._update_status(
                            samples_seen=samples_seen,
                            progress_pct=100.0 * episodes_done / total,
                        )

                dataset_path = writer(_all_samples(), self.config.output)

                self._write_dimos_meta(Path(dataset_path), episodes)

                self._update_status(
                    state="succeeded",
                    current_phase="done",
                    progress_pct=100.0,
                    dataset_path=str(dataset_path),
                )
                logger.info(
                    "[dataprep] succeeded — wrote %d samples across %d episodes to %s",
                    samples_seen, total, dataset_path,
                )
            finally:
                store.stop()
        except Exception as e:
            err = f"{type(e).__name__}: {e}\n{traceback.format_exc()}"
            self._update_status(state="failed", error=err)
            logger.error("[dataprep] FAILED: %s", err)

    def _write_dimos_meta(self, dataset_path: Path, episodes: list[Any]) -> None:
        """Sidecar describing how this dataset was built, recording the
        obs/action schema alongside the dataset."""
        meta = {
            "source":      self.config.source,
            "observation": {k: v.model_dump() for k, v in self.config.observation.items()},
            "action":      {k: v.model_dump() for k, v in self.config.action.items()},
            "sync":        self.config.sync.model_dump(),
            "episodes":    [
                {"id": e.id, "start_ts": e.start_ts, "end_ts": e.end_ts,
                 "task_label": e.task_label, "success": e.success}
                for e in episodes
            ],
            "format":      self.config.output.format,
            "metadata":    self.config.output.metadata,
        }
        with open(dataset_path / "dimos_meta.json", "w") as f:
            json.dump(meta, f, indent=2, default=str)

    def _update_status(self, **kwargs: Any) -> None:
        with self._lock:
            self._status.update(kwargs)


__all__ = ["DataPrepModule", "DataPrepModuleConfig"]
