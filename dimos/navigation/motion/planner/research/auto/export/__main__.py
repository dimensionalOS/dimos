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

"""CLI for the lab exporter: python -m dimos.navigation.motion.planner.research.auto.export <dest>"""

import argparse
from pathlib import Path

from .exporter import run

if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("dest", type=Path, help="directory to create the lab in")
    ap.add_argument(
        "--force",
        action="store_true",
        help="overwrite a non-empty dest / allow a dirty source package",
    )
    ap.add_argument("--no-venv", action="store_true", help="skip uv sync (use ambient python)")
    ap.add_argument("--no-warm", action="store_true", help="skip cache warming")
    ap.add_argument("--no-build", action="store_true", help="skip the cargo/battery smoke test")
    args = ap.parse_args()
    run(
        args.dest,
        force=args.force,
        no_venv=args.no_venv,
        no_warm=args.no_warm,
        no_build=args.no_build,
    )
