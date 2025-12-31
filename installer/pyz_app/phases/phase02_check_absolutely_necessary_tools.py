#!/usr/bin/env python3
from __future__ import annotations

import os
from pathlib import Path

from ..support.constants import discordUrl
from ..support.dax import command_exists, run_command
from ..support.misc import (
    add_git_ignore_patterns,
    ensure_git_and_lfs,
    ensure_port_audio,
    ensure_python,
    get_project_directory,
)
from ..support import prompt_tools as p
from ..support.venv import activate_venv, get_venv_dirs_at


def phase2(system_analysis, selected_features):
    p.clear_screen()
    p.header("Next Phase: Check Install of Vital System Dependencies")
    try:
        has_ifconfig = command_exists("ifconfig")
        has_route = command_exists("route")
        has_sysctl = command_exists("sysctl")
        python_cmd = ensure_python()
        ensure_git_and_lfs()
        ensure_port_audio()

        if not (has_ifconfig and has_route and has_sysctl):
            print("- ifconfig, route, and sysctl are required for the installer to function")
            print("- Please install these system dependencies and re-run this command from the terminal")
            raise SystemExit(1)

        if selected_features and "cuda" in selected_features:
            if not system_analysis.get("cuda", {}).get("exists"):
                p.error("you selected the CUDA feature but I don't see CUDA support in your system")

        ensure_venv_active(python_cmd)
    except Exception as error:
        print("")
        print("")
        p.error("One of the vital dependencies was missing or had versioning issues")
        p.error(f"    error: {getattr(error, 'message', None) or error}")
        p.error(f"Message us in the discord if you're having trouble: {p.highlight(discordUrl)}")
        if p.ask_yes_no("It is NOT recommended to continue. Would you like to stop the setup? [y=exit, n=continue]"):
            raise SystemExit(1)


DEFAULT_VENV_NAME = "venv"


def ensure_venv_active(python_cmd: str):
    active_venv = os.environ.get("VIRTUAL_ENV")
    if active_venv:
        p.boring_log(f"- detected active virtual environment: {active_venv}")
        return active_venv

    p.clear_screen()
    project_directory = get_project_directory()
    possible_venv_dirs = get_venv_dirs_at(project_directory)

    if len(possible_venv_dirs) == 1:
        activate_venv(possible_venv_dirs[0])
    elif len(possible_venv_dirs) > 1:
        print("- multiple python virtual environments found")
        print("- Dimos needs to be installed to a python virtual environment")
        chosen = p.pick_one("Choose a virtual environment to activate:", options=possible_venv_dirs)
        activate_venv(chosen)
    else:
        print("- Dimos needs to be installed to a python virtual environment")
        if not p.confirm("Can I setup a Python virtual environment for you?"):
            raise RuntimeError("- ❌ A virtual environment is required to install dimos. Please set one up then rerun this command.")
        venv_dir = Path(project_directory) / DEFAULT_VENV_NAME
        p.boring_log(f"- creating virtual environment at {venv_dir}")
        venv_res = run_command([python_cmd, "-m", "venv", str(venv_dir)])
        if venv_res.code != 0:
            raise RuntimeError("- ❌ Failed to create virtual environment. Please create one manually and rerun this command.")
        add_git_ignore_patterns(project_directory, [f"/{DEFAULT_VENV_NAME}"], {"comment": "Added by dimos setup"})
        activate_venv(venv_dir)
        p.boring_log("- ✅ virtual environment activated")
        return str(venv_dir)

    return os.environ.get("VIRTUAL_ENV")
