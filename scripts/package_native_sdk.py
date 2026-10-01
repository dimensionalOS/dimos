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

"""Create a relocatable native SDK source bundle without fetching dependencies."""

import argparse
from pathlib import Path
import shutil


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    for name in ("dimos-module", "dimos-module-macros", "dimos-lcm-transport"):
        source = root / "native" / "rust" / name
        target = args.output / "rust" / name
        if target.exists():
            shutil.rmtree(target)
        shutil.copytree(source, target, ignore=shutil.ignore_patterns("target"))
        manifest = target / "Cargo.toml"
        manifest.write_text(
            manifest.read_text().replace(', path = "../../../dimos/message_codegen"', "")
        )
    (args.output / "rust" / "Cargo.toml").write_text(
        '[workspace]\nmembers = ["dimos-module", "dimos-module-macros", "dimos-lcm-transport"]\nresolver = "2"\n'
    )
    print(args.output.resolve())


if __name__ == "__main__":
    main()
