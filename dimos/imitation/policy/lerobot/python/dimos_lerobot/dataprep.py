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

"""Native LeRobot dataset writer and isolated command entry point."""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import redirect_stdout, suppress
from pathlib import Path
import sys
from typing import Any, Protocol, cast

from lerobot.datasets import LeRobotDatasetMetadata
from lerobot.datasets.lerobot_dataset import LeRobotDataset
import numpy as np
from numpy.typing import NDArray

from dimos.imitation.dataprep._lerobot_protocol import (
    REQUEST_ADAPTER,
    BuildRequest,
    BuildResult,
    InspectRequest,
    InspectResult,
)
from dimos.imitation.dataprep.build import run_dataprep
from dimos.imitation.dataprep.core import OutputConfig, Sample, summarize_lengths


class _WritableDataset(Protocol):
    root: Path

    def add_frame(self, frame: dict[str, Any]) -> None: ...

    def save_episode(self, *, parallel_encoding: bool = True) -> None: ...

    def clear_episode_buffer(self) -> None: ...

    def finalize(self) -> None: ...


def _task(sample: Sample) -> str:
    value = sample.task_label
    if not isinstance(value, str) or not value.strip():
        raise ValueError("every LeRobot frame requires an episode task label")
    return value


def _sample_features(sample: Sample) -> dict[str, NDArray[Any]]:
    return {
        **sample.observation,
        **sample.action,
        **{f"complementary_info.{key}": value for key, value in sample.complementary_info.items()},
    }


def write(samples: Iterator[Sample], output: OutputConfig) -> Path:
    """Write synchronized DimOS samples through LeRobot's native dataset API."""
    repo_id = output.metadata.get("repo_id")
    if not isinstance(repo_id, str) or not repo_id.strip():
        raise ValueError("LeRobot output.metadata.repo_id is required")
    fps_value = output.metadata.get("fps")
    if (
        not isinstance(fps_value, (int, float))
        or isinstance(fps_value, bool)
        or fps_value <= 0
        or not float(fps_value).is_integer()
    ):
        raise ValueError("LeRobot output.metadata.fps must be a positive integer")
    fps = int(fps_value)

    iterator = iter(samples)
    try:
        first = next(iterator)
    except StopIteration as error:
        raise ValueError("cannot create a LeRobot dataset without samples") from error

    raw_schema = output.metadata.get("feature_schema")
    if not isinstance(raw_schema, dict) or not raw_schema:
        raise ValueError("LeRobot output.metadata.feature_schema is required")
    features: dict[str, dict[str, Any]] = {}
    for key, raw in raw_schema.items():
        if not isinstance(key, str) or not isinstance(raw, dict):
            raise ValueError("feature_schema must map feature names to definitions")
        dtype = raw.get("dtype")
        shape = raw.get("shape")
        names = raw.get("names")
        if not isinstance(dtype, str):
            raise ValueError(f"feature {key!r} requires a dtype string")
        if not isinstance(shape, (list, tuple)) or not all(
            isinstance(value, int) and value > 0 for value in shape
        ):
            raise ValueError(f"feature {key!r} requires a positive integer shape")
        if not isinstance(names, list) or not all(isinstance(name, str) for name in names):
            raise ValueError(f"feature {key!r} requires string axis names")
        features[key] = {"dtype": dtype, "shape": tuple(shape), "names": names}

    first_values = _sample_features(first)
    if set(first_values) != set(features):
        raise ValueError(
            f"sample features {sorted(first_values)} do not match schema {sorted(features)}"
        )
    dataset = cast(
        "_WritableDataset",
        LeRobotDataset.create(
            repo_id=repo_id,
            fps=fps,
            features=features,
            root=output.path,
            robot_type=output.metadata.get("robot_type"),
            use_videos=True,
        ),
    )
    current_episode: str | None = None
    finished: set[str] = set()

    def add(sample: Sample) -> None:
        nonlocal current_episode
        if current_episode is not None and sample.episode_id != current_episode:
            dataset.save_episode(parallel_encoding=False)
            finished.add(current_episode)
        if sample.episode_id in finished:
            raise ValueError(f"episode {sample.episode_id!r} is not contiguous")
        current_episode = sample.episode_id
        values = _sample_features(sample)
        if set(values) != set(features):
            raise ValueError(f"sample feature keys changed in episode {sample.episode_id}")
        frame: dict[str, Any] = {"task": _task(sample)}
        for key, definition in features.items():
            value = np.asarray(values[key])
            shape = tuple(definition["shape"])
            if value.shape != shape:
                raise ValueError(f"{key} shape changed from {shape} to {value.shape}")
            dtype = definition["dtype"]
            frame[key] = (
                value.astype(np.uint8, copy=False)
                if dtype == "video"
                else value.astype(np.dtype(dtype), copy=False)
            )
        dataset.add_frame(frame)

    try:
        add(first)
        for sample in iterator:
            add(sample)
        dataset.save_episode(parallel_encoding=False)
        dataset.finalize()
    except BaseException:
        with suppress(Exception):
            dataset.clear_episode_buffer()
        with suppress(Exception):
            dataset.finalize()
        raise
    return Path(dataset.root)


def inspect_dataset(path: Path) -> dict[str, Any]:
    """Summarize a local dataset through LeRobot's public metadata interface."""
    metadata = LeRobotDatasetMetadata(repo_id="local/dataset", root=path)
    observation: dict[str, Any] = {}
    action: dict[str, Any] = {}
    metadata_columns = {"timestamp", "frame_index", "episode_index", "index", "task_index"}
    for name, feature in metadata.features.items():
        if name in metadata_columns:
            continue
        entry = {"shape": feature.get("shape"), "dtype": feature.get("dtype")}
        if name.startswith("observation"):
            observation[name] = entry
        elif name.startswith("action"):
            action[name] = entry
    lengths = list(metadata.episodes["length"]) if metadata.episodes is not None else []
    return {
        "format": "lerobot",
        "version": metadata.info.codebase_version,
        "path": str(metadata.root),
        "episodes": metadata.total_episodes,
        "frames": metadata.total_frames,
        "fps": metadata.fps,
        "robot": metadata.robot_type,
        "observation": observation,
        "action": action,
        "episode_lengths": summarize_lengths(lengths),
        "shapes_uniform": True,
        "has_stats": metadata.stats is not None,
    }


def main(argv: list[str] | None = None) -> None:
    """Execute one typed build or inspect request from stdin."""
    args = list(sys.argv[1:] if argv is None else argv)
    if args:
        raise SystemExit("usage: python -m dimos_lerobot.dataprep")
    request = REQUEST_ADAPTER.validate_json(sys.stdin.read())
    protocol_stdout = sys.stdout
    result: BuildResult | InspectResult
    with redirect_stdout(sys.stderr):
        if isinstance(request, BuildRequest):
            result = BuildResult(path=run_dataprep(request.config, writer=write))
        elif isinstance(request, InspectRequest):
            result = InspectResult(info=inspect_dataset(request.path))
        else:
            raise TypeError(f"unsupported request {type(request).__name__}")
    protocol_stdout.write(result.model_dump_json())


if __name__ == "__main__":
    main()
