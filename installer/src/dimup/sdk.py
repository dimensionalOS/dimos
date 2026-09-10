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

"""Read the consumer dependency policy from the selected SDK revision."""

from copy import deepcopy
from typing import Any

from dimup.process import SetupError

SDK_URL = "https://github.com/dimensionalOS/dimos.git"


def desktop_extras(manifest: dict[str, Any]) -> list[str]:
    extras = sorted(set(manifest["project"]["optional-dependencies"]) - {"dds", "unitree-dds"})
    if not extras:
        raise SetupError("Selected SDK has no desktop extras.")
    return extras


def consumer_policy(manifest: dict[str, Any]) -> tuple[list[str], dict[str, Any]]:
    extras = desktop_extras(manifest)
    upstream = manifest.get("tool", {}).get("uv", {})
    policy = {
        key: deepcopy(upstream[key])
        for key in (
            "required-version",
            "override-dependencies",
            "constraint-dependencies",
            "exclude-newer",
            "exclude-newer-package",
            "sources",
            "index",
            "extra-build-dependencies",
            "extra-build-variables",
        )
        if key in upstream
    }
    for name, source in policy.get("sources", {}).items():
        entries = source if isinstance(source, list) else [source]
        if any("path" in entry or "workspace" in entry for entry in entries):
            raise SetupError(f"Selected SDK source {name!r} requires a checkout-local dependency.")
    return extras, policy
