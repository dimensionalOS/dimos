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

"""Lightweight package command entry, independent of robot runtime imports."""

from __future__ import annotations

import argparse
from pathlib import Path
import subprocess
import sys

from dimos.package.project import add, consumer_project, remove, sync, update
from dimos.package.sources import resolve_source


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(prog="dimos package")
    parser.add_argument("--project", type=Path)
    commands = parser.add_subparsers(dest="command", required=True)
    install = commands.add_parser("add", help="Add a dependency using uv")
    install.add_argument("source")
    install.add_argument("--editable", action="store_true")
    install.add_argument("--subdirectory")
    install.add_argument("--extra", action="append", default=[])
    selector = install.add_mutually_exclusive_group()
    selector.add_argument("--tag")
    selector.add_argument("--branch")
    selector.add_argument("--rev")
    synchronize = commands.add_parser("sync", help="Install the existing consumer lock")
    upgrade = commands.add_parser("update", help="Explicitly refresh dependency resolutions")
    upgrade.add_argument("names", nargs="*")
    uninstall = commands.add_parser("remove", help="Remove distribution dependencies using uv")
    uninstall.add_argument("names", nargs="+")
    for command in (install, synchronize, upgrade, uninstall):
        command.add_argument("--project", type=Path, default=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    try:
        if args.command != "add":
            project = consumer_project(args.project)
            if args.command == "sync":
                sync(project)
            elif args.command == "update":
                update(project, tuple(args.names))
            else:
                remove(project, tuple(args.names))
            return
        source = resolve_source(
            args.source,
            tag=args.tag,
            branch=args.branch,
            rev=args.rev,
            subdirectory=args.subdirectory,
        )
        project = consumer_project(args.project, create=True)
        add(
            project,
            source,
            editable=args.editable,
            tag=args.tag,
            branch=args.branch,
            rev=args.rev,
            extras=tuple(args.extra),
        )
    except (ValueError, OSError, subprocess.CalledProcessError) as error:
        print(f"Package command failed: {error}", file=sys.stderr)
        raise SystemExit(
            error.returncode if isinstance(error, subprocess.CalledProcessError) else 2
        ) from error
