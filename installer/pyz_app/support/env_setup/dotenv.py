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

from pathlib import Path
import re

from .. import prompt_tools as p
from ..constants import dimos_env_vars
from ..installer_status import installer_status
from ..misc import add_git_ignore_patterns


def setup_dotenv(project_path: str | Path, env_path: str | Path) -> bool:
    project_path = Path(project_path)
    env_path = Path(env_path)
    template_repo = bool(installer_status.get("template_repo"))

    env_exists = env_path.is_file()

    if not env_exists:
        print("Dimos involves setting several project specific environment variables.")
        print(f"We highly recommend having these in a git-ignored {p.highlight('.env')} file.")
        print(f'''I don't see a {p.highlight('.env')} file''')
        print()
        if template_repo or installer_status.get("non_interactive"):
            create_env = True
        else:
            create_env = p.ask_yes_no("Can I create one for you?")
        if not create_env:
            print("- Okay, I'll assume you will manage env vars yourself:")
            for name, value in dimos_env_vars.items():
                print(f"  {name}={value}")
            return False
        env_path.write_text("")
        add_git_ignore_patterns(project_path, ["/.env"], {"comment": "Added by dimos setup"})

    try:
        env_text = env_path.read_text()
    except FileNotFoundError:
        env_text = ""

    existing_vars = {
        match.group(1)
        for line in env_text.split("\n")
        if (match := re.match(r"^(?:export\s+)?([A-Za-z_][A-Za-z0-9_]*)\s*=", line.strip()))
    }

    missing_env_vars = [
        f"{name}={value}" for name, value in dimos_env_vars.items() if name not in existing_vars
    ]

    if missing_env_vars:
        needs_trailing_newline = len(env_text) > 0 and not env_text.endswith("\n")
        additions = ("\n" if needs_trailing_newline else "") + "\n".join(missing_env_vars) + "\n"
        env_path.write_text(env_text + additions)
        p.boring_log(f"- appended {len(missing_env_vars)} env var(s) to .env")
    else:
        p.boring_log("- all required env vars already exist in .env")
    
    return True


__all__ = ["setup_dotenv"]
