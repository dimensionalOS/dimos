# Copyright 2026 Dimensional Inc.
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

"""`dimos autoresearch` commands; the implementation lives in dimos.autoresearch."""

from __future__ import annotations

from pathlib import Path

import typer

autoresearch_app = typer.Typer(help="Autoresearch sessions", no_args_is_help=True)


@autoresearch_app.command()
def export(
    session_dir: Path = typer.Argument(..., help="A session folder with a frozen candidate"),
    name: str | None = typer.Option(None, "--name", help="Skill name; default: from the task"),
    root: Path | None = typer.Option(
        None, "--root", help="Where the skill folder goes; default: the agent's user skills folder"
    ),
    force: bool = typer.Option(
        False, "--force", help="Export even with no held-out score or a held-out pass rate of 0"
    ),
) -> None:
    """Export a session's frozen program as an Agent Skill folder. Needs no running stack.
    Refused while the program has no held-out score or passed no held-out trial, unless --force."""
    # Imported here so `dimos --help` does not load the robot API.
    from dimos.autoresearch.session import Session
    from dimos.autoresearch.skill import USER_SKILLS_DIR, export_skill

    print(export_skill(Session(session_dir), root or USER_SKILLS_DIR, name, force=force))
