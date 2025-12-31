#!/usr/bin/env python3
# Copyright 2025 Dimensional Inc.
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

from __future__ import annotations

import time

from ..support import prompt_tools as p
from ..support.dimos_banner import RenderLogo
from ..support.get_tool_check_results import get_tool_check_results
from ..support.misc import get_project_toml


def phase0():
    logo = RenderLogo(
        glitchyness=0.25,
        stickyness=50,
        fps=30,
        wave_strength=12,
        wave_speed=0.01,
        wave_freq=0.005,
        scrollable=True,
    )

    print("- checking system")
    system_analysis = get_tool_check_results()
    timeout = 0.5
    # timeout = 0.2

    for key, result in system_analysis.items():
        time.sleep(timeout)
        name = result.get("name") or key
        exists = result.get("exists", False)
        version = result.get("version", "") or ""
        note = result.get("note", "") or ""
        if not exists:
            logo.log(f"- ❌ {name} {note}".strip())
        else:
            logo.log(f"- ✅ {name}: {version} {note}".strip())
    toml_data = get_project_toml()
    logo.stop()

    optional = toml_data["project"].get("optional-dependencies", {})
    features = [f for f in optional.keys() if f not in ["cpu"]]
    selected_features = p.pick_many(
        "Which features do you want? (Selecting none is okay)", options=features
    )
    if "sim" in selected_features and "cuda" not in selected_features:
        selected_features.append("cpu")

    return system_analysis, selected_features


if __name__ == "__main__":
    print(phase0())
