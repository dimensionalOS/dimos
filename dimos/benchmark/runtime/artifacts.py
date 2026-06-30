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

"""Small artifact helpers for runtime demos."""

from __future__ import annotations

import json
from pathlib import Path


def write_json(path: Path, payload: object) -> None:
    """Write JSON artifact with parent directory creation."""

    path.parent.mkdir(parents=True, exist_ok=True)
    model_dump = getattr(payload, "model_dump", None)
    if callable(model_dump):
        value = model_dump(mode="json")
    else:
        value = payload
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")
