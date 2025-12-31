#!/usr/bin/env python3
from __future__ import annotations

import re
from pathlib import Path

from ..constants import dimosEnvVars
from ..misc import add_git_ignore_patterns
from .. import prompt_tools as p


def setup_dotenv(project_path: str | Path, env_path: str | Path) -> bool:
    project_path = Path(project_path)
    env_path = Path(env_path)

    env_exists = env_path.is_file()

    if not env_exists:
        print(
            "Dimos involves setting several project specific environment variables.\n"
            f"We highly recommend having these in a git-ignored {p.highlight('.env')} file.\n"
        )
        if not p.ask_yes_no(f"I don't see a {p.highlight('.env')} file, can I create one for you?"):
            print("- Okay, I'll assume you will manage env vars yourself:")
            for name, value in dimosEnvVars.items():
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

    missing_env_vars = [f"{name}={value}" for name, value in dimosEnvVars.items() if name not in existing_vars]

    if missing_env_vars:
        needs_trailing_newline = len(env_text) > 0 and not env_text.endswith("\n")
        additions = ("\n" if needs_trailing_newline else "") + "\n".join(missing_env_vars) + "\n"
        env_path.write_text(env_text + additions)
        p.boring_log(f"- appended {len(missing_env_vars)} env var(s) to .env")
    else:
        p.boring_log("- all required env vars already exist in .env")

    return True


__all__ = ["setup_dotenv"]
