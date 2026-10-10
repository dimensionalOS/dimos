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

"""Generate standard entry points from the single author-owned blueprint map."""

from collections.abc import Mapping
from typing import Any

from dimos_build_config import read_project


def dynamic_metadata(settings: Mapping[str, Any], project: Mapping[str, Any]) -> dict[str, Any]:
    if settings:
        raise ValueError("The dimos metadata provider accepts no settings")
    _, table = read_project()
    if not table:
        raise ValueError("The dimos metadata provider requires tool.dimos.package")
    if "dimos.blueprints" in project.get("entry-points", {}):
        raise ValueError("Another declaration already supplies dimos.blueprints")
    return {"entry-points": {"dimos.blueprints": table.get("blueprints", {})}}
