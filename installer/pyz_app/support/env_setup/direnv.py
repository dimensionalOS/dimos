#!/usr/bin/env python3
from __future__ import annotations

import re
from pathlib import Path

from ..shell_tooling import command_exists
from .. import prompt_tools as p
from ..installer_status import installer_status

def ask_if_not_template_repo(prompt: str) -> bool:
    if installer_status.get("template_repo"):
        return True
    return p.ask_yes_no(prompt)

def setup_direnv(envrc_path: str | Path) -> bool:
    envrc_path = Path(envrc_path)

    if not command_exists("direnv"):
        p.boring_log("- direnv not detected; skipping .envrc setup")
        venv = p.highlight((envrc_path.parent / "venv").as_posix())
        p.sub_header(
            f"- In the future don't forget to: {p.highlight(f'source {venv}/bin/activate')}\n"
            "  (each time you create a new terminal and cd to the project)"
        )
        return False

    envrc_exists = envrc_path.is_file()
    envrc_text = envrc_path.read_text() if envrc_exists else ""

    add_activation = False
    if not envrc_exists:
        print(f"{p.highlight('direnv')} detected but no {p.highlight('.envrc')} file found.")
        if not ask_if_not_template_repo("Can I create one for you? (for automatic venv activation)"):
            add_activation = True
            p.boring_log("- skipping .envrc creation")
            return False
        envrc_path.write_text(envrc_text)
        p.boring_log("- created .envrc")

    has_venv_activation = bool(
        re.search(r"(^|;)\s*(source|\.)\s+.*[v]?env.*/bin/activate", envrc_text, flags=re.IGNORECASE)
    )
    if not has_venv_activation:
        if not add_activation:
            print(f"It looks like there is a {p.highlight('.envrc')} file")
            print(f"But it seems to not include auto venv activation.")
            add_activation = ask_if_not_template_repo(f"Is it okay if I add a python virtual env activation to the {p.highlight('.envrc')}?")
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
        print(f"I don't see {p.highlight('dotenv_if_exists')} in the {p.highlight('.envrc')}.")
        if ask_if_not_template_repo("Can I add it so the .env file is loaded automatically?"):
            needs_newline = len(envrc_text) > 0 and not envrc_text.endswith("\n")
            envrc_text = envrc_text + ("\n" if needs_newline else "") + "dotenv_if_exists\n"
            envrc_path.write_text(envrc_text)
            p.boring_log("- added dotenv_if_exists to .envrc")
    
    p.sub_header(f"- Don't forget to call {p.highlight('direnv allow')} to enable the .envrc!")
    return True


__all__ = ["setup_direnv"]
