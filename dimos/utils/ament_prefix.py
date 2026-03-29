# Copyright 2025-2026 Dimensional Inc.
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

"""Fake ament index so xacro resolves $(find pkg) without a ROS workspace.

xacro's $(find pkg) calls ament_index_python.get_package_share_directory(),
which looks for packages under directories listed in AMENT_PREFIX_PATH.
We create the expected directory structure with symlinks to our LFS data
so that resolution works natively — no monkey-patching required.
"""

from __future__ import annotations

import os
from pathlib import Path
import tempfile
import threading

_lock = threading.Lock()
_prefix_dir: Path | None = None
_registered: dict[str, Path] = {}


def _get_prefix_dir() -> Path:
    global _prefix_dir
    if _prefix_dir is None:
        _prefix_dir = Path(tempfile.gettempdir()) / "dimos_ament_prefix"
    return _prefix_dir


def ensure_ament_packages(package_paths: dict[str, Path]) -> None:
    """Register packages so ament_index_python can find them.

    Creates a fake ament prefix with symlinks to actual data directories.
    Safe to call multiple times — skips already-registered packages.
    """
    if not package_paths:
        return

    with _lock:
        prefix = _get_prefix_dir()
        resource_dir = prefix / "share" / "ament_index" / "resource_index" / "packages"
        resource_dir.mkdir(parents=True, exist_ok=True)

        for pkg_name, pkg_path in package_paths.items():
            resolved = Path(pkg_path).resolve()

            # Skip if already registered with the same target
            if _registered.get(pkg_name) == resolved:
                continue

            # Create marker file (ament_index_python checks this exists)
            marker = resource_dir / pkg_name
            marker.write_text("")

            # Create symlink: <prefix>/share/<pkg_name> -> actual data dir
            share_link = prefix / "share" / pkg_name
            if share_link.is_symlink() or share_link.exists():
                share_link.unlink()
            share_link.symlink_to(resolved)

            _registered[pkg_name] = resolved

        # Prepend to AMENT_PREFIX_PATH if not already there
        prefix_str = str(prefix)
        current = os.environ.get("AMENT_PREFIX_PATH", "")
        if prefix_str not in current.split(os.pathsep):
            os.environ["AMENT_PREFIX_PATH"] = (
                f"{prefix_str}{os.pathsep}{current}" if current else prefix_str
            )
