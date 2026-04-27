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

"""RLDS / TFDS dataset writer.

Produces a TFDS-on-disk layout (TFRecord shards + dataset_info.json) following
the RLDS Episode/Step protocol used by Open X-Embodiment, RT-X, etc.

Each TF Example encodes one Episode as a sequence of Steps with:
    observation/<key>, action/<key>, reward, discount, is_first, is_last, is_terminal
"""

from __future__ import annotations

from collections.abc import Iterator
from pathlib import Path

from dimos.learning.spec import OutputConfig, Sample


def write(samples: Iterator[Sample], output: OutputConfig) -> Path:
    """Write samples as TFDS/RLDS shards. Returns the dataset directory path."""
    raise NotImplementedError
