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

from rich.console import Console
from rich.text import Text

from dimup.process import Runner, SetupError
from dimup.setup import prepare


def main() -> None:
    parser = argparse.ArgumentParser(prog="dimup", description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("setup", help="Prepare this machine for DimOS development")
    parser.parse_args()
    state = Path(os.environ.get("XDG_STATE_HOME", Path.home() / ".local/state"))
    try:
        prepare(Runner(state / "dimup/setup.log"))
    except (SetupError, OSError) as error:
        Console(stderr=True).print(Text(str(error), style="red"))
        raise SystemExit(1) from error
    except KeyboardInterrupt:
        Console(stderr=True).print("Interrupted. Installation files and logs have been kept.")
        raise SystemExit(130) from None


if __name__ == "__main__":
    main()
