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

"""Run GraspGenX once and save an annotated point-cloud PNG."""

import argparse
from collections.abc import Sequence
import os
from pathlib import Path

from .demo import run_contributor_demo


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(os.environ.get("DIMOS_GRASPGENX_OUTPUT", "graspgenx-ycb-demo.png")),
        help="Destination PNG path.",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    result = run_contributor_demo(output_path=args.output)
    print(
        f"graspgenx-ycb-demo complete candidates={result.candidate_count} "
        f"image={result.image_path}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
