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

"""The environment source: `ROOT__FIELD__SUBFIELD=value`, plus bare GlobalConfig names."""

from __future__ import annotations

from collections.abc import Collection, Mapping
import os
from typing import Any

from dotenv import dotenv_values

from dimos.core.coordination.blueprint_config.global_schema import global_environment_names
from dimos.core.coordination.blueprint_config.schema import (
    OptionTarget,
    TargetIdentity,
    coerce_environment_value,
    source_identity,
)
from dimos.core.coordination.blueprint_config.values import deep_set
from dimos.core.global_config import ENV_FILE


def configuration_environment(environ: Mapping[str, str] | None) -> Mapping[str, str]:
    """The process environment over `.env`, unless the caller supplies one."""
    if environ is not None:
        return environ
    if ENV_FILE is None:
        return os.environ
    from_dotenv = {
        key: value for key, value in dotenv_values(ENV_FILE).items() if value is not None
    }
    return {**from_dotenv, **os.environ}


def read_environment(
    environ: Mapping[str, str],
    roots: Mapping[str, str],
    targets: Mapping[TargetIdentity, OptionTarget],
    *,
    known_transports: Collection[str] = (),
) -> dict[str, Any]:
    """Variables under a known root, coerced by their target when the schema has one.

    `roots` maps the lowercased env root to the section root; a variable whose
    root is unknown is someone else's and skipped.
    """
    global_env_names = global_environment_names()
    values: dict[str, Any] = {}

    def set_coerced(path: tuple[str, ...], raw_name: str, value: str) -> None:
        identity = source_identity(path)
        target = targets.get(identity) if identity is not None else None
        coerced = value if target is None else coerce_environment_value(value, target, raw_name)
        deep_set(values, path, coerced)

    for raw_name, value in environ.items():
        global_field = global_env_names.get(raw_name.lower())
        if global_field is not None:
            set_coerced(("g", global_field), raw_name, value)
            continue

        parts = tuple(part.lower().replace("-", "_") for part in raw_name.split("__"))
        if len(parts) < 2 or parts[0] not in roots:
            continue
        if parts[0] == "transports" and (len(parts) < 3 or parts[1] not in known_transports):
            continue
        set_coerced((roots[parts[0]], *parts[1:]), raw_name, value)

    return values
