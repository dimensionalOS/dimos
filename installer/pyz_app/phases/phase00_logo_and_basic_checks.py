#!/usr/bin/env python3
from __future__ import annotations

import time

from ..support.dimos_banner import RenderLogo
from ..support.get_tool_check_results import get_tool_check_results
from ..support.misc import get_project_toml
from ..support import prompt_tools as p


def phase0():
    logo = RenderLogo(
        glitchyness=0.35,
        stickyness=18,
        fps=30,
        wave_strength=12,
        wave_speed=0.12,
        wave_freq=0.07,
        scrollable=True,
    )

    logo.log("- checking system")
    system_analysis = get_tool_check_results()
    timeout = 0.5

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

    time.sleep(timeout)
    toml_data = get_project_toml()
    time.sleep(timeout)
    logo.stop()
    p.clear_screen()

    optional = toml_data["project"].get("optional-dependencies", {})
    features = [f for f in optional.keys() if f not in ["cpu"]]
    selected_features = p.pick_many("Which features do you want?", options=features)
    if "sim" in selected_features and "cuda" not in selected_features:
        selected_features.append("cpu")
    exit(0)
    return system_analysis, selected_features


if __name__ == "__main__":
    print(phase0())
