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

"""Deterministic serialization and stable-ID helpers."""

from collections.abc import Mapping, Sequence
from hashlib import sha256
import json
from typing import Final, Literal, TypeAlias

from pydantic import BaseModel

JsonValue: TypeAlias = (
    str | bool | None | int | float | Mapping[str, "JsonValue"] | Sequence["JsonValue"]
)
SchemaVersion: TypeAlias = Literal["spatiotemporal-video-qa/v1"]
SCHEMA_VERSION: Final[SchemaVersion] = "spatiotemporal-video-qa/v1"


def canonical_json_bytes(value: JsonValue) -> bytes:
    """Serialize the benchmark's string-only ID preimages deterministically."""
    return json.dumps(
        value,
        ensure_ascii=False,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")


def stable_id(prefix: str, preimage: JsonValue) -> str:
    """Hash one canonical preimage into an opaque stable identifier."""
    return f"{prefix}_{sha256(canonical_json_bytes(preimage)).hexdigest()}"


def canonical_model_json(model: BaseModel) -> str:
    """Serialize one strict record to deterministic canonical JSON."""
    return canonical_json_bytes(model.model_dump(mode="json")).decode("utf-8")
