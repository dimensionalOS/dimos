#!/usr/bin/env python3

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

"""
Generate pip_dependency_database.py for the pyz app by aggregating JSON files
from installer/dep_database and embedding both the resulting DATA and the
project pyproject.toml into the output file as raw triple-quoted strings.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import sys

installer_root = Path(__file__).resolve().parent.parent
toml_source = installer_root.parent / "pyproject.toml"
dep_dir = installer_root / "dep_database"
dependency_out_path = installer_root / "pyz_app" / "bundled_files" / "pip_dependency_database.json"
toml_link_path = installer_root / "pyz_app" / "bundled_files" / "pyproject.toml"


def _read_dep_json(dep_dir: Path) -> dict:
    aggregated: dict[str, object] = {}
    for path in sorted(dep_dir.glob("*.json")):
        name = path.stem.lower()
        try:
            aggregated[name] = json.loads(path.read_text())
        except Exception as exc:  # pragma: no cover - build-time guard
            print(f"{name} had an error: {exc}", file=sys.stderr)
    return aggregated


def main() -> None:
    #
    # make the aggregated json
    #
    aggregated_data = _read_dep_json(dep_dir)
    dependency_out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(dependency_out_path, "w", encoding="utf-8") as outfile:
        json.dump(aggregated_data, outfile, indent=2, sort_keys=True)

    #
    # hardlink the pyproject.toml
    #
    toml_link_path.parent.mkdir(parents=True, exist_ok=True)
    try:
        if toml_link_path.exists():
            try:
                if toml_link_path.samefile(toml_source):
                    return
            except FileNotFoundError:
                pass
            toml_link_path.unlink()
        os.link(toml_source, toml_link_path)
    except Exception as exc:
        print(f"Failed to hardlink pyproject.toml: {exc}", file=sys.stderr)


if __name__ == "__main__":
    main()
