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

"""Installed message providers, discovered through Python package entry points."""

from __future__ import annotations

from importlib.metadata import entry_points
from pathlib import Path
from types import ModuleType


def providers() -> tuple[ModuleType, ...]:
    return tuple(
        entry.load()
        for entry in sorted(entry_points(group="dimos.messages"), key=lambda item: item.name)
    )


def schema_roots() -> tuple[Path, ...]:
    return tuple(Path(provider.schema_root()) for provider in providers())
