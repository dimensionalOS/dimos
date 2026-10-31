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

"""The shared `$DIMOS_HOME/config.yaml` (`~/.dimos/config.yaml` by default): the settings a
machine's Desktop writes and its gateway reads.

Read on every ask rather than cached, so a setting Desktop writes reaches the next launch without
a restart, and so a test can point `DIMOS_HOME` somewhere else after this module is imported.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any

import yaml


def dimos_home() -> Path:
    """`$DIMOS_HOME`, or `~/.dimos`."""
    return Path(os.environ.get("DIMOS_HOME") or Path.home() / ".dimos")


def config_file() -> Path:
    """The shared `config.yaml` itself, which may not exist."""
    return dimos_home() / "config.yaml"


def setting(section: str, key: str) -> Any:
    """A value from the shared `config.yaml`, None when the file, the section or the key is missing
    (an empty value counts as unset: that is how Desktop clears a setting)."""
    try:
        loaded = yaml.safe_load(config_file().read_text()) or {}
    except (OSError, yaml.YAMLError):
        return None
    value = (loaded.get(section) or {}).get(key) if isinstance(loaded, dict) else None
    return None if value == "" else value
