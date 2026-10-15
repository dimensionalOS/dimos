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
import shutil

from dimos_cdrgen.source_inputs import prepare_sources

ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "packages/dimos-generated"


def prepare(cache: Path, *, offline: bool = False, project: Path = PROJECT) -> Path:
    upstream = cache / "upstream"
    prepare_sources(project / "schema-sources.json", upstream, cache / "archives", offline=offline)
    target = project / "interfaces"
    if target.exists():
        shutil.rmtree(target)
    shutil.copytree(upstream, target)
    shutil.copytree(project / "custom/dimos_msgs/msg", target / "dimos_msgs/msg")
    shutil.copyfile(project / "custom/dimos_msgs/LICENSE", target / "licenses/dimos_msgs-LICENSE")
    return target


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cache", type=Path, default=ROOT / "build/builtin-source-cache")
    parser.add_argument("--offline", action="store_true")
    args = parser.parse_args()
    print(prepare(args.cache, offline=args.offline))


if __name__ == "__main__":
    main()
