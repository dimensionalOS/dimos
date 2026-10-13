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

"""Enter the selected consumer environment before importing the robot CLI."""

from __future__ import annotations

import os
from pathlib import Path

from dimos.package.project import consumer_project, uv_environment

_DISPATCH_PROJECT = "DIMOS_CONSUMER_PROJECT"


def dispatch(argv: list[str]) -> list[str]:
    explicit: Path | None = None
    if argv[:1] == ["--project"]:
        if len(argv) < 3:
            raise ValueError("--project requires a consumer directory and command")
        explicit = Path(argv[1])
        argv = argv[2:]
    if argv[:1] == ["package"]:
        from dimos.package.cli import main

        main([*(["--project", str(explicit)] if explicit else []), *argv[1:]])
        return []
    if not any(argument in {"run", "list"} for argument in argv):
        if explicit:
            raise ValueError("--project selects the environment for package, list or run")
        return argv
    if os.environ.get(_DISPATCH_PROJECT):
        if explicit:
            raise ValueError("Cannot change consumer project inside a dispatched command")
        return argv
    if explicit is None:
        try:
            project = consumer_project(None)
        except ValueError:
            return argv
    else:
        project = consumer_project(explicit)
    if not (project / "uv.lock").is_file():
        raise ValueError("Consumer uv.lock is missing; run package update before list/run")
    environment = uv_environment()
    environment[_DISPATCH_PROJECT] = str(project)
    # -I prevents the caller's checkout/PYTHONPATH from bypassing the selected environment.
    program = "from dimos.cli.entry import main; main()"
    os.execvpe(
        "uv",
        [
            "uv",
            "run",
            "--project",
            str(project),
            "--locked",
            "--no-default-groups",
            "python",
            "-I",
            "-c",
            program,
            *argv,
        ],
        environment,
    )
    return argv
