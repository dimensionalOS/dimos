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

from ..support import prompt_tools as p
from ..support.constants import discord_url
from ..support.dax import run_command


def phase4():
    p.header("Next Phase: Dimos Check")

    def bail(msg: str):
        print("")
        p.error(msg)
        p.error(
            f"Please message us in our discord and we'll help you get it installed:\n    {discord_url}"
        )
        raise SystemExit(1)

    checks = [
        # the dimos CLI isn't install for some reason but dimos-robot is?
        # {"label": "dimos --version", "cmd": ["dimos", "--version"]},
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
    p.confirm("Press enter to continue to next phase")