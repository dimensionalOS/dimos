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

import argparse
from collections.abc import Iterator
from contextlib import suppress
from pathlib import Path
from typing import Any, Protocol, cast

from lerobot.datasets.lerobot_dataset import LeRobotDataset
import numpy as np
from numpy.typing import NDArray

from dimos.imitation.dataprep.build import run_dataprep
from dimos.imitation.dataprep.core import DataPrepConfig, OutputConfig, Sample, is_image_array

_IMAGE_FEATURE = "observation.images.wrist"
_STATE_FEATURE = "observation.state"
_ACTION_FEATURE = "action"


class _WritableDataset(Protocol):
    root: Path

    def add_frame(self, frame: dict[str, Any]) -> None: ...

    def save_episode(self, *, parallel_encoding: bool = True) -> None: ...

    def clear_episode_buffer(self) -> None: ...

    def finalize(self) -> None: ...


def _one_feature(
    values: dict[str, NDArray[Any]], *, image: bool, kind: str
) -> tuple[str, NDArray[Any]]:
    matches = [(name, value) for name, value in values.items() if is_image_array(value) is image]
    if len(matches) != 1:
        expected = "image" if image else "low-dimensional"
        raise ValueError(f"LeRobot conversion requires exactly one {expected} {kind} feature")
    return matches[0]


def _joint_names(metadata: dict[str, Any], size: int) -> list[str]:
    names = metadata.get("joint_names")
    if not isinstance(names, list) or not all(isinstance(name, str) and name for name in names):
        raise ValueError("LeRobot output.metadata.joint_names must be a non-empty string list")
    if len(names) != size:
        raise ValueError(f"joint_names has {len(names)} entries but state has {size}")
    return names


def _task(sample: Sample, metadata: dict[str, Any]) -> str:
    value = sample.task_label or metadata.get("default_task_label")
    if not isinstance(value, str) or not value.strip():
        raise ValueError("every LeRobot frame requires an episode or default task label")
    return value


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

    image_key, image = _one_feature(first.observation, image=True, kind="observation")
    state_key, state = _one_feature(first.observation, image=False, kind="observation")
    action_key, action = _one_feature(first.action, image=False, kind="action")
    state = np.asarray(state).reshape(-1)
    action = np.asarray(action).reshape(-1)
    if state.shape != action.shape:
        raise ValueError(f"state shape {state.shape} does not match action shape {action.shape}")
    if image.ndim != 3 or image.shape[2] not in (1, 3, 4):
        raise ValueError(f"wrist image must have HWC shape, got {image.shape}")
    names = _joint_names(output.metadata, state.size)
    shape = tuple(int(value) for value in image.shape)
    features = {
        _IMAGE_FEATURE: {
            "dtype": "video",
            "shape": shape,
            "names": ["height", "width", "channels"],
        },
        _STATE_FEATURE: {"dtype": "float32", "shape": state.shape, "names": names},
        _ACTION_FEATURE: {"dtype": "float32", "shape": action.shape, "names": names},
    }
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
        sample_image = np.asarray(sample.observation[image_key])
        sample_state = np.asarray(sample.observation[state_key], dtype=np.float32).reshape(-1)
        sample_action = np.asarray(sample.action[action_key], dtype=np.float32).reshape(-1)
        if sample_image.shape != shape:
            raise ValueError(f"wrist image shape changed from {shape} to {sample_image.shape}")
        if sample_state.shape != state.shape or sample_action.shape != action.shape:
            raise ValueError("state or action shape changed during conversion")
        dataset.add_frame(
            {
                _IMAGE_FEATURE: sample_image.astype(np.uint8, copy=False),
                _STATE_FEATURE: sample_state,
                _ACTION_FEATURE: sample_action,
                "task": _task(sample, output.metadata),
            }
        )

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


def main() -> None:
    parser = argparse.ArgumentParser(description="Convert a DimOS recording to LeRobot")
    parser.add_argument("config", type=Path)
    args = parser.parse_args()
    config = DataPrepConfig.model_validate_json(args.config.read_text())
    path = run_dataprep(config, writer=write)
    print(path)


if __name__ == "__main__":
    main()
