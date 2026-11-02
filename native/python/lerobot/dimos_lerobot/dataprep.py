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
import json
from pathlib import Path
import sys
from typing import Any

from lerobot.datasets import LeRobotDatasetMetadata
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from numpy.typing import NDArray

from dimos.imitation.dataprep.build import run_dataprep
from dimos.imitation.dataprep.core import summarize_lengths
from dimos.imitation.dataprep.formats.lerobot.features import canonical_feature_names
from dimos.imitation.dataprep.formats.lerobot.protocol import (
    REQUEST_ADAPTER,
    BuildRequest,
    BuildResult,
    InspectRequest,
    InspectResult,
)
from dimos.imitation.dataprep.formats.lerobot.publication import staged_dataset
from dimos.imitation.dataprep.schema import OutputConfig, Sample


def _task(sample: Sample, default_task_label: Any) -> str:
    value = sample.task_label or default_task_label
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
    names = canonical_feature_names(
        {key: raw_schema[key]["dtype"] for key in first.observation},
        first.action.keys(),
    )
    renamed = [names.get(key, key) for key in raw_schema]
    if len(set(renamed)) != len(renamed):
        raise ValueError("LeRobot feature rename collision in output schema")
    features = {
        names.get(key, key): {**definition, "shape": tuple(definition["shape"])}
        for key, definition in raw_schema.items()
    }
    default_task_label = output.metadata.get("default_task_label", "task")
    destination = output.path
    with staged_dataset(destination) as staged_path:
        dataset = LeRobotDataset.create(
            repo_id=repo_id,
            fps=fps,
            features=features,
            root=staged_path,
            robot_type=output.metadata.get("robot_type", output.metadata.get("robot", "unknown")),
            use_videos=True,
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
            dataset.add_frame(
                {
                    "task": _task(sample, default_task_label),
                    **{
                        names.get(key, key): value
                        for key, value in _sample_features(sample).items()
                    },
                }
            )

        try:
            add(first)
            for sample in iterator:
                add(sample)
            dataset.save_episode(parallel_encoding=False)
            dataset.finalize()  # type: ignore[no-untyped-call]
        except BaseException:
            with suppress(Exception):
                dataset.clear_episode_buffer()
            with suppress(Exception):
                dataset.finalize()  # type: ignore[no-untyped-call]
            raise
    return destination


def inspect_dataset(path: Path) -> dict[str, Any]:
    """Summarize a local dataset through LeRobot's public metadata interface."""
    metadata = LeRobotDatasetMetadata(repo_id="local/dataset", root=path)
    observation: dict[str, Any] = {}
    action: dict[str, Any] = {}
    other: dict[str, Any] = {}
    sidecar_path = path / "dimos_meta.json"
    sidecar = json.loads(sidecar_path.read_text()) if sidecar_path.is_file() else {}
    names = sidecar.get("feature_names", {})
    observation_names = {names.get(key, key) for key in sidecar.get("observation", {})}
    action_names = {names.get(key, key) for key in sidecar.get("action", {})}
    metadata_columns = {"timestamp", "frame_index", "episode_index", "index", "task_index"}
    for name, feature in metadata.features.items():
        if name in metadata_columns:
            continue
        entry = {"shape": list(feature["shape"]), "dtype": feature.get("dtype")}
        if name in observation_names:
            observation[name] = entry
        elif name in action_names:
            action[name] = entry
        elif name.startswith("observation"):
            observation[name] = entry
        elif name.startswith("action"):
            action[name] = entry
        else:
            other[name] = entry
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
        "features": other,
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
