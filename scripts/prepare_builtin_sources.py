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

"""Explicitly prepare pinned upstream data and local DimOS message inputs."""

import argparse
from pathlib import Path

from dimos.message_codegen.build import ROOT, prepare_builtin_sources as prepare


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cache", type=Path, default=ROOT / "build/builtin-source-cache")
    parser.add_argument("--offline", action="store_true")
    args = parser.parse_args()
    print(prepare(args.cache, offline=args.offline))


if __name__ == "__main__":
    main()
