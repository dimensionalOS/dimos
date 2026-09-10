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

"""Build-time helpers that do not import the DimOS runtime."""

import os
from pathlib import Path
import tarfile

try:
    import tomllib
except ImportError:
    import tomli as tomllib


def native_files(root: Path) -> list[Path]:
    """Collect native build roots, preserving workspace and sibling dependencies."""
    workspace = tomllib.loads((root / "Cargo.toml").read_text())["workspace"]
    roots = {root / member for member in workspace["members"]}
    roots.update({root / "native", root / "dimos/hardware/sensors/lidar/common"})
    for directory, dirs, files in os.walk(root / "dimos"):
        dirs[:] = [
            name
            for name in dirs
            if name not in {"target", "build", "node_modules", ".git", "__pycache__"}
        ]
        if "flake.nix" in files or "Cargo.toml" in files:
            roots.add(Path(directory))
    paths = {root / "Cargo.toml", root / "Cargo.lock"}
    for source in roots:
        for directory, dirs, files in os.walk(source):
            dirs[:] = [
                name
                for name in dirs
                if name not in {"target", "build", "node_modules", ".git", "result", "__pycache__"}
            ]
            for name in files:
                path = Path(directory) / name
                if not path.is_symlink() and path.suffix not in {".pyc", ".pcap", ".db", ".rrd"}:
                    paths.add(path)
    return sorted(path for path in paths if path.is_file())


def bundle_native_sources(root: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tarfile.open(destination, "w") as archive:
        for path in native_files(root):
            info = archive.gettarinfo(str(path), arcname=str(path.relative_to(root)))
            info.uid = info.gid = info.mtime = 0
            info.uname = info.gname = ""
            with path.open("rb") as source:
                archive.addfile(info, source)
