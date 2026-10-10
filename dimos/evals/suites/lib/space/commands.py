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

"""Acquire pinned SPACE inputs or score responses saved by ``dimos evals run``."""

from __future__ import annotations

import argparse
from collections.abc import Sequence
from contextlib import ExitStack
import json
from pathlib import Path
import signal
from types import FrameType

from dimos.evals.suites.lib.space.data import SpacePaths, setup
from dimos.evals.suites.lib.space.report import score_run


def _terminate(signum: int, _frame: FrameType | None) -> None:
    # Let process cleanup finish even if the terminal sends another signal.
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, signal.SIG_IGN)
    raise SystemExit(128 + signum)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subcommands = parser.add_subparsers(dest="command", required=True)
    subcommands.add_parser("setup", help="Acquire pinned source and one verified task file")
    score = subcommands.add_parser("score", help="Replay saved replies without model calls")
    score.add_argument("run_dir", type=Path)
    args = parser.parse_args(argv)
    paths = SpacePaths()
    with ExitStack() as handlers:
        for sig in (signal.SIGTERM, signal.SIGHUP):
            previous = signal.signal(sig, _terminate)
            handlers.callback(signal.signal, sig, previous)
        result = setup(paths) if args.command == "setup" else score_run(paths, args.run_dir)
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0 if result.get("complete", True) else 1


if __name__ == "__main__":
    raise SystemExit(main())
