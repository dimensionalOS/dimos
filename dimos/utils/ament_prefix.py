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

"""Resolve xacro $(find pkg) to custom package paths.

xacro's $(find pkg) calls ament_index_python which requires a ROS workspace.
Since DimOS downloads robot descriptions via LFS to custom paths, we need
to redirect package resolution.

When ament_index_python is available (ROS environment), we set up a fake
ament index with symlinks so resolution works natively. Otherwise, we
temporarily patch xacro's _find handler as a fallback.
"""

from __future__ import annotations

from collections.abc import Iterator
import contextlib
import os
from pathlib import Path
import tempfile
import threading

_lock = threading.Lock()

# Ament index state
_prefix_dir: Path | None = None
_ament_registered: dict[str, Path] = {}

_has_ament: bool
try:
    from ament_index_python.packages import (  # type: ignore[import-not-found]
        get_package_share_directory as _ament_get,  # noqa: F401
    )

    _has_ament = True
except ImportError:
    _has_ament = False


def _setup_ament_index(package_paths: dict[str, Path]) -> None:
    """Create fake ament index entries so get_package_share_directory() works."""
    global _prefix_dir

    if _prefix_dir is None:
        _prefix_dir = Path(tempfile.gettempdir()) / "dimos_ament_prefix"

    prefix = _prefix_dir
    resource_dir = prefix / "share" / "ament_index" / "resource_index" / "packages"
    resource_dir.mkdir(parents=True, exist_ok=True)

    for pkg_name, pkg_path in package_paths.items():
        resolved = Path(pkg_path).resolve()
        if _ament_registered.get(pkg_name) == resolved:
            continue

        # Marker file for ament_index_python
        (resource_dir / pkg_name).write_text("")

        # Symlink: <prefix>/share/<pkg_name> -> actual data dir
        share_link = prefix / "share" / pkg_name
        if share_link.is_symlink() or share_link.exists():
            share_link.unlink()
        share_link.symlink_to(resolved)

        _ament_registered[pkg_name] = resolved

    # Prepend to AMENT_PREFIX_PATH
    prefix_str = str(prefix)
    current = os.environ.get("AMENT_PREFIX_PATH", "")
    if prefix_str not in current.split(os.pathsep):
        os.environ["AMENT_PREFIX_PATH"] = (
            f"{prefix_str}{os.pathsep}{current}" if current else prefix_str
        )


@contextlib.contextmanager
def _patch_xacro_find(package_paths: dict[str, Path]) -> Iterator[None]:
    """Fallback: temporarily patch xacro's _find when ament is unavailable."""
    from xacro import substitution_args  # type: ignore[import-untyped]

    original_find = substitution_args._find

    def custom_find(resolved: str, a: str, args: list[str], context: dict[str, str]) -> str:
        pkg_name = args[0] if args else ""
        if pkg_name in package_paths:
            pkg_path = str(Path(package_paths[pkg_name]).resolve())
            return resolved.replace(f"$({a})", pkg_path)
        return str(original_find(resolved, a, args, context))

    substitution_args._find = custom_find
    try:
        yield
    finally:
        substitution_args._find = original_find


def ensure_ament_packages(package_paths: dict[str, Path]) -> None:
    """Register packages so xacro $(find pkg) resolves to our paths.

    Uses ament_index_python when available, otherwise stores paths for
    the monkey-patch fallback used in process_xacro().
    """
    if not package_paths or not _has_ament:
        return

    with _lock:
        _setup_ament_index(package_paths)


def process_xacro(path: Path, package_paths: dict[str, Path], xacro_args: dict[str, str]) -> str:
    """Process a xacro file to URDF XML, resolving $(find pkg) from package_paths.

    Uses ament_index_python when available, falls back to patching xacro otherwise.
    """
    import xacro  # type: ignore[import-not-found,import-untyped]

    ensure_ament_packages(package_paths)

    if _has_ament:
        doc = xacro.process_file(str(path), mappings=xacro_args)
    else:
        with _patch_xacro_find(package_paths):
            doc = xacro.process_file(str(path), mappings=xacro_args)

    return str(doc.toprettyxml(indent="  "))
