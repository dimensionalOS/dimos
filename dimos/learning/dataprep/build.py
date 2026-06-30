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

"""DataPrep build orchestration — the impure layer over `core.py`.

`run_dataprep` (build) and `inspect_dataset` (read-back) own the I/O and side
effects — open/close the store, drive the writer/reader, emit logs, write
files; they compose the pure helpers in `core.py` and the per-format
readers/writers. Exposed by the `dimos dataprep` subcommand.
"""

from __future__ import annotations

from collections.abc import Iterator
import json
from pathlib import Path
from typing import Any

from dimos.learning.dataprep.core import (
    DataPrepConfig,
    Episode,
    Sample,
    extract_episodes,
    get_inspector,
    get_writer,
    iter_episode_samples,
)
from dimos.memory2.store.sqlite import SqliteStore
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def _write_dimos_meta(dataset_path: Path, config: DataPrepConfig, episodes: list[Episode]) -> None:
    """Sidecar describing how this dataset was built, recording the obs/action
    schema alongside the dataset."""
    meta = {
        "source": config.source,
        "observation": {k: v.model_dump() for k, v in config.observation.items()},
        "action": {k: v.model_dump() for k, v in config.action.items()},
        "sync": config.sync.model_dump(),
        "episodes": [
            {
                "id": e.id,
                "start_ts": e.start_ts,
                "end_ts": e.end_ts,
                "task_label": e.task_label,
                "success": e.success,
            }
            for e in episodes
        ],
        "format": config.output.format,
        "metadata": config.output.metadata,
    }
    # Writers return a directory (lerobot) or a file (hdf5). Put the sidecar
    # *inside* a directory, or *beside* a file (`<name>.dimos_meta.json`).
    if dataset_path.is_dir():
        meta_path = dataset_path / "dimos_meta.json"
    else:
        meta_path = dataset_path.with_name(f"{dataset_path.stem}.dimos_meta.json")
    with open(meta_path, "w") as f:
        json.dump(meta, f, indent=2, default=str)


def run_dataprep(config: DataPrepConfig) -> Path:
    """Build a dataset from a recording and return the dataset path.

    Opens the source store, extracts episodes, streams samples through the
    configured format writer, and writes `dimos_meta.json`. Synchronous —
    raises on failure so the caller owns the exit code.
    """
    shared = set(config.observation) & set(config.action)
    if shared:
        raise ValueError(
            f"observation and action share feature name(s) {sorted(shared)}; "
            f"give each a distinct key (they may still map to the same stream)."
        )

    logger.info(
        "[dataprep] starting build  source=%s  extractor=%s  output=%s",
        config.source,
        config.episodes.extractor,
        config.output.path,
    )
    store = SqliteStore(path=config.source, must_exist=True)
    try:
        logger.info("[dataprep] streams in source: %s", store.list_streams())
        all_eps = extract_episodes(store, config.episodes)
        successful = [e for e in all_eps if e.success]
        logger.info(
            "[dataprep] episodes extracted: %d total / %d successful",
            len(all_eps),
            len(successful),
        )

        if not successful:
            raise RuntimeError(
                f"No successful episodes extracted from {config.source!r} "
                f"using extractor={config.episodes.extractor!r}. "
                f"Available streams: {store.list_streams()}. "
                f"For a recording with no episode_status stream, set "
                f"extractor='ranges' with explicit (start, end) tuples."
            )

        obs_keys = set(config.observation)
        action_keys = set(config.action)
        streams = {**config.observation, **config.action}
        logger.info(
            "[dataprep] obs streams=%s  action streams=%s  sync=%s",
            sorted(obs_keys),
            sorted(action_keys),
            config.sync.model_dump(),
        )
        writer = get_writer(config.output.format)
        # fps drives written timestamps + video rate, so tie it to the resample
        # rate; an explicit metadata.fps still wins.
        output = config.output
        if config.sync.rate_hz > 0 and "fps" not in output.metadata:
            output = output.model_copy(
                update={"metadata": {**output.metadata, "fps": config.sync.rate_hz}}
            )
        logger.info("[dataprep] writing %s dataset to %s", config.output.format, output.path)

        samples_seen = 0
        episodes_done = 0
        total = len(successful)
        produced: list[Episode] = []  # episodes that yielded ≥1 sample

        def _all_samples() -> Iterator[Sample]:
            nonlocal samples_seen, episodes_done
            for ep in successful:
                before = samples_seen
                for sample in iter_episode_samples(
                    store=store,
                    episode=ep,
                    streams=streams,
                    sync=config.sync,
                    obs_keys=obs_keys,
                    action_keys=action_keys,
                ):
                    samples_seen += 1
                    if samples_seen % 50 == 0:
                        logger.info(
                            "[dataprep] %.1f%%  samples=%d  ep %d/%d",
                            100.0 * episodes_done / total,
                            samples_seen,
                            episodes_done,
                            total,
                        )
                    yield sample
                if samples_seen > before:
                    produced.append(ep)
                episodes_done += 1

        dataset_path = Path(writer(_all_samples(), output))
        written = [e.model_copy(update={"id": f"ep_{i:06d}"}) for i, e in enumerate(produced)]
        _write_dimos_meta(dataset_path, config, written)
        logger.info(
            "[dataprep] succeeded — wrote %d samples across %d episodes to %s",
            samples_seen,
            len(written),
            dataset_path,
        )
        return dataset_path
    finally:
        store.stop()


def inspect_dataset(path: Path | str, fmt: str | None = None) -> dict[str, Any]:
    """Summarize a built dataset: observation/action features (shape + dtype),
    episode/frame counts, and whether shapes/lengths are uniform.

    `fmt` is auto-detected when omitted: a `.hdf5`/`.h5` file → hdf5; a
    directory containing `meta/info.json` → lerobot.
    """
    p = Path(path)
    if fmt is None:
        if p.suffix in (".h5", ".hdf5"):
            fmt = "hdf5"
        elif (p / "meta" / "info.json").exists():
            fmt = "lerobot"
        else:
            raise ValueError(
                f"Cannot detect dataset format at {p}: expected a .hdf5 file or a "
                f"lerobot directory with meta/info.json. Pass --format explicitly."
            )
    return get_inspector(fmt)(p)
