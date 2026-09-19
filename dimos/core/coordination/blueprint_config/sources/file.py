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

"""The config file source: a JSON object keyed by section."""

from __future__ import annotations

from collections.abc import Mapping
import json
from pathlib import Path
from typing import Any

from dimos.core.coordination.blueprint_config.errors import BlueprintConfigError


def read_config_file(path: Path) -> Mapping[str, Any]:
    try:
        raw = path.read_text()
    except (FileNotFoundError, IsADirectoryError):
        return {}
    except OSError as error:
        raise BlueprintConfigError(f"Could not read config file {path}: {error}") from error
    try:
        values = json.loads(raw)
    except json.JSONDecodeError as error:
        raise BlueprintConfigError(
            f"Invalid JSON in config file {path}: {error.msg} "
            f"(line {error.lineno}, column {error.colno})"
        ) from error
    if not isinstance(values, Mapping):
        raise BlueprintConfigError(f"Config file {path} must contain a JSON object.")
    return values
