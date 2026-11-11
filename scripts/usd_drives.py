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


from __future__ import annotations

import argparse
from importlib import import_module
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--layer", type=Path, required=True, help="the .usda to rewrite in place")
    parser.add_argument(
        "--from",
        dest="source",
        required=True,
        help="module:callable returning {joint: (kp, kd, target)}",
    )
    args = parser.parse_args()

    from dimos.robot.assets.usd import usd_drives, usd_with_drives

    module, _, name = args.source.partition(":")
    drives = getattr(import_module(module), name)()
    layer = usd_with_drives(args.layer.read_text(), drives)
    args.layer.write_text(layer)
    written = usd_drives(layer)
    print(f"wrote {len(written)} drive(s) into {args.layer}")
    for joint, (kp, kd, target) in sorted(written.items()):
        print(f"  {joint}: kp={kp:g} kd={kd:g} target={target:g}")


if __name__ == "__main__":
    main()
