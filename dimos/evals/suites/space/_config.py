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

"""SPACE benchmark configuration — one config object per benchmark.

The convention for adding a benchmark to ``dimos/evals/suites/``: give it its
own directory with a ``_config.py`` exposing a module-level ``config`` built
from environment variables. Suites import it; nothing else needs registering
(``list_suites`` discovers the directory, and ``_``-prefixed modules like this
one are internals, not suites).
"""

from __future__ import annotations

import os
from pathlib import Path

from pydantic import BaseModel, Field

from dimos.constants import CACHE_DIR


def _env_path(variable: str, default: Path) -> Path:
    value = os.environ.get(variable)
    return Path(value) if value else default


class SpaceConfig(BaseModel):
    """Where SPACE lives and how its episodes run.

    ``frame_stride`` and the nav settings mirror the benchmark's own defaults
    (``space/configs/gpt.py``: every frame; ``space/evaluate_dmnav.py``:
    ``LOCAL_CONTEXT = 5``, ``max_steps=100``) so a run here is comparable to an
    upstream one. Raise ``frame_stride`` only to trade fidelity for tokens.
    """

    data_dir: Path = Field(
        default_factory=lambda: _env_path(
            "DIMOS_SPACE_DATA_DIR", CACHE_DIR / "space" / "SPACE_data_release"
        )
    )
    repo_dir: Path = Field(
        default_factory=lambda: _env_path(
            "DIMOS_SPACE_REPO", CACHE_DIR / "space" / "ml-space-benchmark"
        )
    )
    frame_stride: int = 1
    nav_local_context: int = 5
    nav_max_steps: int = 100


config = SpaceConfig()
