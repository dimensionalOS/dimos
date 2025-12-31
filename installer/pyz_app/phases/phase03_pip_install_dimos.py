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


def phase3(_system_analysis, selected_features):
    p.header("Next Phase: Pip Installing Dimos")
    # some setup.py's (contact_graspnet_pytorch) require numpy (so pip itself will fail while trying to install them)
    # so we preinstall numpy
    res = run_command(["pip", "install", "numpy"], print_command=True)
    selected_features_string = ""
    if selected_features:
        selected_features_string = f"[{','.join(selected_features)}]"
    package_name = (
        f"dimos{selected_features_string} @ git+https://github.com/dimensionalOS/dimos.git"
    )
    res = run_command(["pip", "install", package_name], print_command=True)
    if res.code != 0:
        print("")
        p.error(
            f"Failed to pip install dimos 😕\nPlease message us in our discord and we'll help you get it installed!:\n    {p.highlight(discord_url)}"
        )
        raise SystemExit(1)
    
    p.confirm("Press enter to continue to next phase")