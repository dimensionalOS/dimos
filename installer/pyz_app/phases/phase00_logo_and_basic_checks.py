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
from ..support.get_system_analysis import get_system_analysis
from ..support.misc import get_project_toml

def phase0():
    fps = 14
    logo = RenderLogo(
        glitchyness=0.45, # relative quantity of visual artifacting
        stickyness=fps * 0.75, # how many frames to keep an artifact
        fps=fps, # at 30fps it flickers a lot in the MacOS stock terminal. Ironically its fine at 30fps in the VS Code terminal
        color_wave_amplitude=10, # bigger = wider range of colors
        wave_speed=0.01, # bigger = faster
        wave_freq=0.01, # smaller = longer streaks of color
        scrollable=True,
    )

    logo.log("- checking system")
    system_analysis = get_system_analysis()
    # # visually we want cuda to be listed last and os to be first
    timeout = 0.5
    cuda = system_analysis["cuda"]
    del system_analysis["cuda"]
    ordered_analysis = {
        "os": system_analysis["os"],
        **system_analysis,
        "cuda": cuda,
    }
    ordered_analysis["cuda"] = cuda
    
    for key, result in (ordered_analysis.items()):
        name = result.get("name") or key
        exists = result.get("exists", False)
        version = result.get("version", "") or ""
        note = result.get("note", "") or ""
        cross = "\u2718"
        check = "\u2714"
        if not exists:
            logo.log(f"- {p.red(cross)} {name} {note}".strip())
        else:
            logo.log(f"- {p.cyan(check)} {name}: {version} {note}".strip())
        time.sleep(timeout)
    toml_data = get_project_toml()
    logo.stop()

    optional = toml_data["project"].get("optional-dependencies", {})
    features = [f for f in optional.keys() if f not in ["cpu"]]
    p.header("First Phase: Feature Selection")
    selected_features = p.pick_many(
        "Which features do you want? (Selecting none is okay)", options=features
    )
    if "sim" in selected_features and "cuda" not in selected_features:
        selected_features.append("cpu")

    return system_analysis, selected_features


if __name__ == "__main__":
    print(phase0())
