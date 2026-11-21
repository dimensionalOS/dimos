# Copyright 2025-2026 Dimensional Inc.
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

"""Redaction shared by configuration help and persisted run metadata."""

from collections.abc import Mapping
from typing import Any

# Include old names when loading run metadata written before the migration.
SECRET_CONFIG_FIELDS = frozenset(
    {
        "api_key",
        "app_secret",
        "aes_128_key",
        "relay_key",
        "dimos_api_key",
        "unitree_aes_128_key",
        "typesafe_api_key",
    }
)


def config_field_name(option: str) -> str:
    return option.removeprefix("--").rsplit(".", 1)[-1].replace("-", "_")


def without_secret_values(values: Mapping[str, Any]) -> dict[str, Any]:
    """Remove credential fields, including nested configuration objects."""
    return {
        key: _clean(value)
        for key, value in values.items()
        if config_field_name(key) not in SECRET_CONFIG_FIELDS
    }


def _clean(value: Any) -> Any:
    if isinstance(value, Mapping):
        return without_secret_values(value)
    if isinstance(value, list):
        return [_clean(item) for item in value]
    return value
