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

import os
from pathlib import Path
import shutil

from ..support import prompt_tools as p
from ..support.constants import discord_url
from ..support.shell_tooling import command_exists, run_command
from ..support.misc import (
    add_git_ignore_patterns,
    ensure_git_and_lfs,
    ensure_port_audio,
    ensure_python,
    get_project_directory,
)
from ..support.installer_status import installer_status
from ..support.venv import activate_venv, get_venv_dirs_at, purge_broken_external_venv


def phase2(system_analysis, selected_features):
    p.header("Next Phase: Check Install of Vital System Dependencies")
    try:
        
        if has_ifconfig:= command_exists("ifconfig"):
            p.boring_log("- ifconfig found")
        else:
            p.error("- ifconfig not found")
        
        if has_route:= command_exists("route"):
            p.boring_log("- route found")
        else:
            p.error("- route not found")
        
        if has_sysctl:= command_exists("sysctl"):
            p.boring_log("- sysctl found")
        else:
            p.error("- sysctl not found")
        
        python_cmd = ensure_python()
        ensure_git_and_lfs()
        ensure_port_audio()

        if not (has_ifconfig and has_route and has_sysctl):
            print("- ifconfig, route, and sysctl are required for the installer to function")
            print(
                "- Please install these system dependencies and re-run this command from the terminal"
            )
            raise SystemExit(1)

        if selected_features and "cuda" in selected_features:
            if not system_analysis.get("cuda", {}).get("exists", False):
                p.error("you selected the CUDA feature but I don't see CUDA support in your system")

        ensure_venv_active(python_cmd)
        
    except Exception as error:
        print("")
        print("")
        p.error("One of the vital dependencies was missing or had versioning issues")
        p.error(f"    error: {getattr(error, 'message', None) or error}")
        p.error(f"Message us in the discord if you're having trouble: {p.highlight(discord_url)}")
        if p.ask_yes_no(
            "It is recommended to STOP here because of the error. Should I stop here? [y=stop,n=continue]"
        ):
            raise SystemExit(1)
        raise error
    
    print(f'''✅ passed all checks for vital system dependencies''')
    p.confirm("Press enter to continue to next phase")


DEFAULT_VENV_NAME = "venv"


def ensure_venv_active(python_cmd: str):
    p.boring_log(f"- checking if in python virtual environment")
    active_venv = os.environ.get("VIRTUAL_ENV")
    if active_venv:
        p.boring_log(f"- detected active virtual environment: {active_venv}")
        return active_venv

    project_directory = get_project_directory()
    possible_venv_dirs = get_venv_dirs_at(project_directory)
    
    purge_broken_external_venv()
    
    if len(possible_venv_dirs) == 1:
        activate_venv(possible_venv_dirs[0])
    elif len(possible_venv_dirs) > 1:
        print("- multiple python virtual environments found")
        print("- Dimos needs to be installed to a python virtual environment")
        chosen = p.pick_one("Choose a virtual environment to activate:", options=possible_venv_dirs)
        activate_venv(chosen)
    else:
        print("- Dimos needs to be installed to a python virtual environment")
        if not p.ask_yes_no("Can I setup a Python virtual environment for you?"):
            raise RuntimeError(
                "- ❌ A virtual environment is required to install dimos. Please set one up then rerun this command."
            )
        venv_dir = Path(project_directory) / DEFAULT_VENV_NAME
        if venv_dir.exists():
            p.warning(f'it appears there is a corrupt venv at {venv_dir}')
            if p.ask_yes_no("Can I delete the corrupt venv?"):
                p.boring_log(f"- deleting corrupt venv at {venv_dir}")
                shutil.rmtree(venv_dir)
            else:
                p.warning(f'you are probably going to get an error if we continue but okay')
        
        p.boring_log(f"- creating virtual environment at {venv_dir}")
        venv_res = run_command(
            [python_cmd, "-m", "venv", str(venv_dir)], dry_run=installer_status["dry_run"], print_command=True
        )
        if venv_res.code != 0:
            raise RuntimeError(
                "- ❌ Failed to create virtual environment. Please create one manually and rerun this command."
            )
        add_git_ignore_patterns(
            project_directory, [f"/{DEFAULT_VENV_NAME}"], {"comment": "Added by dimos setup"}
        )
        activate_venv(venv_dir)
        p.boring_log("- ✅ virtual environment activated")
        return str(venv_dir)

    return os.environ.get("VIRTUAL_ENV")
