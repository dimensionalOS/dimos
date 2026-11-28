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

"""Select rigid-body contact surfaces, excluding game movement/query helpers.

A wheel on a MuJoCo robot is a physical rigid body. RAGE's test_vehicle_wheel
bit is a query category, not the physical layer for wheels. It also admits
character stair ramps and vehicle-only blockers over the detailed treads.
"""

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

ROBOT_CONTACT_MASK = 1 << 13
CONTACT_POLICY = "rigid-object-v1"
CONTACT_LIMIT = (
    "Robot collision uses physical-object contacts; vehicle-wheel query helpers, "
    "character stair ramps and query-only blockers are excluded."
)


def bind_catalog(source: Path, output: Path) -> dict[str, Any]:
    """Rebind existing placements without reusing a differently filtered cache.

    Scenes and named spawns must be prepared against the resulting catalog hash.
    The source catalog, immutable placement database and decoder stay unchanged.
    """
    if output.exists():
        raise ValueError("Choose a new catalog output path")
    original = source.read_bytes()
    manifest: dict[str, Any] = json.loads(original)
    if manifest["version"] != 1:
        raise ValueError("Unsupported world catalog")
    manifest["database"] = str((source.parent / manifest["database"]).resolve())
    manifest["include_mask"] = ROBOT_CONTACT_MASK
    manifest["contact_policy"] = CONTACT_POLICY
    manifest["parent_catalog_sha256"] = hashlib.sha256(original).hexdigest()
    manifest["limits"] = [
        item for item in manifest["limits"] if not item.startswith("Robot collision uses ")
    ] + [CONTACT_LIMIT]
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = bind_catalog(args.source, args.output)
    print(json.dumps({"include_mask": result["include_mask"], "policy": result["contact_policy"]}))


if __name__ == "__main__":
    main()
