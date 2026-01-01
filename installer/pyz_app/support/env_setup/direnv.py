#!/usr/bin/env python3
from __future__ import annotations

import re
from pathlib import Path

from ..shell_tooling import command_exists
from .. import prompt_tools as p


def setup_direnv(envrc_path: str | Path) -> None:
    envrc_path = Path(envrc_path)

    if not command_exists("direnv"):
        p.boring_log("- direnv not detected; skipping .envrc setup")
        venv = p.highlight((envrc_path.parent / "venv").as_posix())
        p.sub_header(
            f"- In the future don't forget to: {p.highlight(f'source {venv}/bin/activate')}\n"
            "  (each time you create a new terminal and cd to the project)"
        )
        return

    envrc_exists = envrc_path.is_file()
    envrc_text = envrc_path.read_text() if envrc_exists else ""

    if not envrc_exists:
        print("direnv detected but no .envrc found.")
        if not p.ask_yes_no("Create one to auto-activate the virtual environment?"):
            p.boring_log("- skipping .envrc creation")
            return
        envrc_path.write_text(envrc_text)
        p.boring_log("- created .envrc")

    has_venv_activation = bool(
        re.search(r"(^|;)\s*(source|\.)\s+.*[v]?env.*/bin/activate", envrc_text, flags=re.IGNORECASE)
    )
    if not has_venv_activation:
        add_activation = p.ask_yes_no(
            "It looks like there is a .envrc file, but I don't see a python virtual environment "
            "activation in there. Is it okay if I add a python virtual env activation to the .envrc?"
        )
        if add_activation:
            block = "\n".join(
                [
                    "for venv in venv .venv env; do",
                    '  if [[ -f "$venv/bin/activate" ]]; then',
                    '    . "$venv/bin/activate"',
                    "    break",
                    "  fi",
                    "done",
                ]
            )
            needs_newline = len(envrc_text) > 0 and not envrc_text.endswith("\n")
            envrc_text = envrc_text + ("\n" if needs_newline else "") + block + "\n"
            envrc_path.write_text(envrc_text)
            p.boring_log("- added venv activation to .envrc")

    has_dotenv = "dotenv_if_exists" in envrc_text
    if not has_dotenv:
        print(f"I don't see {p.highlight('dotenv_if_exists')} in .envrc.")
        if p.ask_yes_no("Can I add it so the .env file is loaded automatically?"):
            needs_newline = len(envrc_text) > 0 and not envrc_text.endswith("\n")
            envrc_text = envrc_text + ("\n" if needs_newline else "") + "dotenv_if_exists\n"
            envrc_path.write_text(envrc_text)
            p.boring_log("- added dotenv_if_exists to .envrc")


__all__ = ["setup_direnv"]
