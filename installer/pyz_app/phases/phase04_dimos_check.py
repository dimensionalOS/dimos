#!/usr/bin/env python3
from __future__ import annotations

from ..support.constants import discordUrl
from ..support.dax import run_command
from ..support import prompt_tools as p


def phase4():
    p.clear_screen()
    p.header("Next Phase: Dimos Check")

    def bail(msg: str):
        print("")
        p.error(msg)
        p.error(f"Please message us in our discord and we'll help you get it installed:\n    {discordUrl}")
        raise SystemExit(1)

    checks = [
        {"label": "dimos --version", "cmd": ["dimos", "--version"]},
        {"label": "import dimos (python)", "cmd": ["python", "-c", "import dimos;"]},
    ]

    passed = 0
    for check in checks:
        res = run_command(check["cmd"])
        if res.code != 0:
            bail(f"Failed to run {check['label']} 😕")
        passed += 1
        p.boring_log(f"- {check['label']} succeeded")

    p.boring_log(f"- {passed} Dimos checks passed")
