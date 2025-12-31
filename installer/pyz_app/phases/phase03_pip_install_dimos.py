#!/usr/bin/env python3
from __future__ import annotations

from ..support.constants import discordUrl
from ..support.dax import run_command
from ..support import prompt_tools as p


def phase3(_system_analysis, selected_features):
    p.clear_screen()
    p.header("Next Phase: Pip Installing Dimos")
    selected_features_string = ""
    if selected_features:
        selected_features_string = f"[{','.join(selected_features)}]"
    package_name = f"dimos{selected_features_string} @ git+https://github.com/dimensionalOS/dimos.git"
    res = run_command(["pip", "install", package_name], print_command=True)
    if res.code != 0:
        print("")
        p.error(
            f"Failed to pip install dimos 😕\nPlease message us in our discord and we'll help you get it installed!:\n    {p.highlight(discordUrl)}"
        )
        raise SystemExit(1)
