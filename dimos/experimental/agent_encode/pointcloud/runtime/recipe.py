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

"""Deterministic JSON descriptions of requests, without executing them."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import fields, is_dataclass
import json
import math

import numpy as np
from pydantic import JsonValue


def describe(value: object, omit: tuple[str, ...] = (), depth: int = 0) -> JsonValue:
    """A dataclass becomes ``{"type": name, **fields}`` without the ``omit`` fields at
    any level; a non-finite number becomes ``{"invalid_number": ...}``."""
    if depth > 96:
        return {"invalid_number": "request nesting exceeds 96 levels"}
    if is_dataclass(value) and not isinstance(value, type):
        return {
            "type": type(value).__name__,
            **{
                f.name: describe(getattr(value, f.name), omit, depth + 1)
                for f in fields(value)
                if f.name not in omit
            },
        }
    if isinstance(value, Mapping):
        if all(isinstance(key, str) for key in value):
            return {key: describe(item, omit, depth + 1) for key, item in value.items()}
        return {
            "type": type(value).__name__,
            "items": [
                [describe(key, omit, depth + 1), describe(item, omit, depth + 1)]
                for key, item in value.items()
            ],
        }
    if isinstance(value, np.ndarray):
        return describe(value.tolist(), omit, depth)
    if isinstance(value, np.generic):
        return describe(value.item(), omit, depth)
    if isinstance(value, (tuple, list)):
        return [describe(item, omit, depth + 1) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return {"invalid_number": "nan" if math.isnan(value) else ("inf" if value > 0 else "-inf")}
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return {"type": type(value).__name__}


def canonical(value: JsonValue) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)
