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

"""Short dense color_image clip(s) as MemoryStore image streams."""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path
from typing import Any

from dimos.evals.agents.base import Agent
from dimos.evals.environments.base import Environment, EnvironmentConfig
from dimos.evals.types import RunningEnvironment
from dimos.evals.vqa.clip import DEFAULT_CLIP_MAX_FRAMES, load_clip_manifest

_MULTI_STREAM_NAMES = ("first_clip", "second_clip", "third_clip", "fourth_clip")


class ImageClipConfig(EnvironmentConfig):
    paths: tuple[Path, ...]
    agent_artifacts: tuple[str, ...] = ("clip",)


class ImageClip(Environment):
    """One or more short clips, each a fully-encoded ``image``-like stream.

    A single path uses stream name ``clip``. Multiple paths use
    ``first_clip``, ``second_clip``, … so QuestionAnswer labels them distinctly.
    Each clip must have ``≤ DEFAULT_CLIP_MAX_FRAMES`` frames so the default
    agent does not sparsely subsample.
    """

    config: ImageClipConfig

    def __init__(self, path: Path | Sequence[Path], **kwargs: Any) -> None:
        paths = (path,) if isinstance(path, Path) else tuple(Path(p) for p in path)
        if not paths:
            raise ValueError("ImageClip requires at least one clip directory")
        artifact_names = ("clip",) if len(paths) == 1 else _MULTI_STREAM_NAMES[: len(paths)]
        super().__init__(paths=paths, agent_artifacts=artifact_names, **kwargs)

    def preflight(self, agent: Agent) -> None:
        if agent.config.modules:
            raise RuntimeError(
                f"ImageClip({self.config.paths!r}) launches nothing; "
                f"{type(agent).__name__} adds modules {agent.config.modules!r}"
            )
        for clip_dir in self.config.paths:
            if not clip_dir.is_dir():
                raise FileNotFoundError(f"clip directory does not exist: {clip_dir}")
            manifest = load_clip_manifest(clip_dir)
            if len(manifest.frames) > DEFAULT_CLIP_MAX_FRAMES:
                raise ValueError(
                    f"clip {clip_dir} has {len(manifest.frames)} frames; "
                    f"max {DEFAULT_CLIP_MAX_FRAMES} so QuestionAnswer encodes every frame"
                )

    def start(self, modules: Sequence[str]) -> RunningEnvironment:
        from dimos.memory.store.memory import MemoryStore
        from dimos.msgs.sensor_msgs.Image import Image

        recording = MemoryStore()
        self._resources.callback(recording.stop)
        streams = []
        names = (
            ("clip",)
            if len(self.config.paths) == 1
            else _MULTI_STREAM_NAMES[: len(self.config.paths)]
        )
        if len(self.config.paths) > len(_MULTI_STREAM_NAMES):
            raise ValueError(
                f"ImageClip supports at most {len(_MULTI_STREAM_NAMES)} clips per case"
            )
        artifacts: dict[str, Path] = {}
        for name, clip_dir in zip(names, self.config.paths, strict=True):
            manifest = load_clip_manifest(clip_dir)
            stream = recording.stream(name, Image)
            for entry in manifest.frames:
                frame_path = clip_dir / entry.file
                if not frame_path.is_file():
                    raise FileNotFoundError(f"clip frame missing: {frame_path}")
                image = Image.from_file(frame_path)
                stream.append(image, ts=entry.ts)
            streams.append(stream)
            artifacts[name] = Path(clip_dir)
        return RunningEnvironment(mcp_url="", streams=tuple(streams), artifacts=artifacts)
