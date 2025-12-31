#!/usr/bin/env python3
from __future__ import annotations

from ..support.env_setup.direnv import setup_direnv
from ..support.env_setup.dotenv import setup_dotenv
from ..support.misc import get_project_directory
from ..support import prompt_tools as p


def phase5():
    p.clear_screen()
    p.header("Next Phase: Environment configuration")

    project_path = get_project_directory()
    env_path = f"{project_path}/.env"
    envrc_path = f"{project_path}/.envrc"

    has_dotenv = setup_dotenv(project_path, env_path)
    if not has_dotenv:
        return

    setup_direnv(envrc_path)
    print()
    print()
    print("Setup Complete!")