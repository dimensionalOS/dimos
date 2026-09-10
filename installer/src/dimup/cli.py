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

"""The DimOS bootstrap CLI. Runtime commands belong to the application."""

import argparse
import os
from pathlib import Path
import sys

from dimup.process import Runner, SetupError
from dimup.project import create
from dimup.setup import prepare


def main() -> None:
    parser = argparse.ArgumentParser(prog="dimup", description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("setup", help="Prepare this machine for DimOS development")
    init = commands.add_parser("init", help="Create a DimOS SDK application")
    init.add_argument("directory", type=Path)
    init.add_argument("--ref", default="main", help="SDK branch or commit (default: main)")
    args = parser.parse_args()
    state = Path(os.environ.get("XDG_STATE_HOME", Path.home() / ".local/state"))
    try:
        if args.command == "setup":
            prepare(Runner(state / "dimup/setup.log"))
        else:
            create(args.directory, args.ref)
    except (SetupError, OSError) as error:
        print(str(error), file=sys.stderr)
        raise SystemExit(1) from error


if __name__ == "__main__":
    main()
