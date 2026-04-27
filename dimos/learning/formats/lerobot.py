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

"""LeRobot v2 dataset writer.

Produces a directory layout compatible with HuggingFace LeRobot:
    <output.path>/
        meta/info.json          # schema, fps, total episodes/frames
        meta/episodes.jsonl     # per-episode metadata (length, task)
        data/chunk-000/episode_000000.parquet   # tabular obs+action
        videos/chunk-000/<image_key>/episode_000000.mp4   # encoded image streams
"""

from __future__ import annotations

from collections.abc import Iterator
from pathlib import Path

from dimos.learning.spec import OutputConfig, Sample


def write(samples: Iterator[Sample], output: OutputConfig) -> Path:
    """Write samples in LeRobot v2 layout. Returns the dataset root path."""
    raise NotImplementedError
