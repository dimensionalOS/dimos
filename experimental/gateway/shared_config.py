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

"""The gateway's settings: `DIMOS_APP` (what Desktop starts it with), else `<home>/config.yaml`."""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any

import yaml

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# config.yaml (section, key) -> the DIMOS_APP field that wins over it (Desktop's app contract names)
APP_FIELDS = {
    ("dimos-gateway", "socket"): "socket",
    ("recordings", "dir"): "recordingsDir",
    ("zenoh", "namespace"): "zenohNamespace",
    ("zenoh", "connect"): "zenohConnect",
}


def app_env() -> dict[str, Any]:
    """`DIMOS_APP` as JSON; an unusable one is warned about and ignored, since every field has a fallback."""
    text = os.environ.get("DIMOS_APP")
    if not text:
        return {}
    try:
        loaded = json.loads(text)
    except ValueError as error:
        logger.warning("DIMOS_APP is not JSON, so it is ignored", error=str(error))
        return {}
    return loaded if isinstance(loaded, dict) else {}


def dimos_home() -> Path:
    """`DIMOS_APP`'s `home`, else `~/.dimos` (`DIMOS_HOME` is not read)."""
    home = app_env().get("home")
    return Path(home) if home else Path.home() / ".dimos"


def config_file() -> Path:
    return dimos_home() / "config.yaml"


def setting(section: str, key: str) -> Any:
    """`DIMOS_APP`'s field for it, else config.yaml's; None when neither has it (empty counts as unset)."""
    field = APP_FIELDS.get((section, key))
    value = app_env().get(field) if field else None
    if value not in (None, ""):
        return value
    try:
        loaded = yaml.safe_load(config_file().read_text()) or {}
    except (OSError, yaml.YAMLError):
        return None
    value = (loaded.get(section) or {}).get(key) if isinstance(loaded, dict) else None
    return None if value == "" else value
