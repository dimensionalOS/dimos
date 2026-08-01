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

"""Canonical serialization and content-derived identity helpers."""

from __future__ import annotations

import hashlib
from typing import cast

from pydantic import BaseModel

from dimos.benchmark.dimsim.models import SceneOracleView
from dimos.benchmark.spatial.utilities import JsonValue, canonical_json, stable_opaque_id


def model_bytes(model: BaseModel) -> bytes:
    return canonical_json(cast("JsonValue", model.model_dump(mode="json")))


def oracle_view_bytes(view: SceneOracleView) -> bytes:
    return model_bytes(view)


def oracle_view_digest(view: SceneOracleView) -> str:
    return hashlib.sha256(oracle_view_bytes(view)).hexdigest()


def task_id(identity_payload: dict[str, JsonValue]) -> str:
    return stable_opaque_id("dimsim_task", identity_payload)


def outcome_id(task_identifier: str, digest: str, expected: JsonValue) -> str:
    return stable_opaque_id(
        "dimsim_outcome",
        {"task_id": task_identifier, "oracle_view_digest": digest, "expected": expected},
    )
