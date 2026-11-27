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

"""Bind an immutable collision catalog to the renderer's authored-state selection."""

import argparse
import json
from pathlib import Path
import re
from typing import Any

from dimos.experimental.gta_sim2.world_index import file_digest


def disabled_maps(path: Path, expected_digest: str | None = None) -> dict[str, str]:
    if expected_digest is not None and file_digest(path) != expected_digest:
        raise ValueError("Scene-state profile changed; bind a new catalog before loading")
    value = json.loads(path.read_text())
    maps = value.get("disabled_maps")
    if not isinstance(maps, dict) or any(
        not isinstance(key, str)
        or re.fullmatch(r"[a-z0-9_]+\.rpf@[0-9a-f]+", key) is None
        or not isinstance(reason, str)
        or not reason.strip()
        for key, reason in maps.items()
    ):
        raise ValueError("Scene-state disabled_maps must name exact resources with reasons")
    return maps


def bind_catalog(source: Path, profile: Path, output: Path) -> dict[str, Any]:
    """Create a distinct catalog identity without modifying the source database."""
    if output.exists():
        raise ValueError("Choose a new catalog output path")
    maps = disabled_maps(profile)
    manifest: dict[str, Any] = json.loads(source.read_text())
    manifest["database"] = str((source.parent / manifest["database"]).resolve())
    manifest["scene_state"] = {
        "path": str(profile.resolve()),
        "sha256": file_digest(profile),
        "name": json.loads(profile.read_text()).get("name", "Authored state selection"),
        "disabled_maps": len(maps),
    }
    manifest["parent_catalog_sha256"] = file_digest(source)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("source", "profile", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(bind_catalog(args.source, args.profile, args.output)["scene_state"]))


if __name__ == "__main__":
    main()
