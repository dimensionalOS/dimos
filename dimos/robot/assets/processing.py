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

"""Universal robot asset rendering helpers."""

from __future__ import annotations

from collections.abc import Mapping
import hashlib
import os
from pathlib import Path
import re
from typing import Literal

from dimos.robot.assets.git_cache import DEFAULT_ROBOT_ASSET_CACHE_ROOT
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

PackageUriMode = Literal["preserve", "absolute"]

DERIVED_ASSET_CACHE_ROOT = DEFAULT_ROBOT_ASSET_CACHE_ROOT / "derived"
_RENDERED_URDF_CACHE_ROOT = DERIVED_ASSET_CACHE_ROOT / "rendered_urdfs"


def render_urdf(
    urdf_path: Path | str | os.PathLike[str],
    package_paths: Mapping[str, Path | str | os.PathLike[str]] | None = None,
    xacro_args: Mapping[str, str] | None = None,
    *,
    package_uri_mode: PackageUriMode = "preserve",
) -> Path:
    """Render a URDF or Xacro artifact into a cached plain URDF file.

    This is the universal robot-description stage: it expands Xacro with the
    supplied ROS package roots and can optionally rewrite ``package://`` URIs to
    absolute filesystem paths. Consumer-specific cleanup, such as Drake-only tag
    stripping, belongs in the consumer adapter.
    """
    if package_uri_mode not in ("preserve", "absolute"):
        raise ValueError(f"Unsupported package URI mode: {package_uri_mode!r}")

    source_path = Path(os.fspath(urdf_path))
    resolved_package_paths = normalize_package_paths(package_paths or {})
    resolved_xacro_args = dict(xacro_args or {})

    cache_key = _generate_render_key(
        source_path,
        resolved_package_paths,
        resolved_xacro_args,
        package_uri_mode,
    )
    rendered_stem = _rendered_urdf_stem(source_path)
    cache_path = _RENDERED_URDF_CACHE_ROOT / cache_key / rendered_stem
    cache_path.mkdir(parents=True, exist_ok=True)
    rendered_urdf = cache_path / f"{rendered_stem}.urdf"

    if rendered_urdf.exists():
        logger.debug(f"Using cached rendered URDF: {rendered_urdf}")
        return rendered_urdf

    if source_path.suffix == ".xacro":
        urdf_content = _process_xacro(source_path, resolved_package_paths, resolved_xacro_args)
    else:
        urdf_content = source_path.read_text()

    if package_uri_mode == "absolute":
        urdf_content = resolve_package_uris(urdf_content, resolved_package_paths)

    rendered_urdf.write_text(urdf_content)
    logger.info(f"Rendered URDF cached at: {rendered_urdf}")
    return rendered_urdf


def resolve_package_uris(
    urdf_content: str,
    package_paths: Mapping[str, Path | str | os.PathLike[str]],
) -> str:
    """Rewrite ``package://`` URIs in URDF XML to absolute filesystem paths."""
    resolved_package_paths = normalize_package_paths(package_paths)
    pattern = r"""package://([^/]+)/(.+?)(["'<>\s])"""

    def replace_uri(match: re.Match[str]) -> str:
        pkg_name = match.group(1)
        rel_path = match.group(2)
        suffix = match.group(3)

        if pkg_name in resolved_package_paths:
            full_path = resolved_package_paths[pkg_name] / rel_path
            if full_path.exists():
                return f"{full_path}{suffix}"
            logger.warning(f"File not found: {full_path}")

        return match.group(0)

    return re.sub(pattern, replace_uri, urdf_content)


def normalize_package_paths(
    package_paths: Mapping[str, Path | str | os.PathLike[str]],
) -> dict[str, Path]:
    return {
        package_name: Path(os.fspath(package_path)).resolve()
        for package_name, package_path in package_paths.items()
    }


def _generate_render_key(
    urdf_path: Path,
    package_paths: Mapping[str, Path],
    xacro_args: Mapping[str, str],
    package_uri_mode: PackageUriMode,
) -> str:
    processing_version = "urdf-render-v2"
    mtime = urdf_path.stat().st_mtime if urdf_path.exists() else 0
    key_data = repr(
        (
            processing_version,
            str(urdf_path),
            mtime,
            sorted((name, str(path)) for name, path in package_paths.items()),
            _render_dependency_fingerprints(urdf_path, package_paths),
            sorted(xacro_args.items()),
            package_uri_mode,
        )
    )
    return hashlib.sha256(key_data.encode()).hexdigest()[:16]


def _rendered_urdf_stem(source_path: Path) -> str:
    """Return a stable output stem without duplicated URDF suffixes."""
    return source_path.stem.removesuffix(".urdf")


def _render_dependency_fingerprints(
    urdf_path: Path,
    package_paths: Mapping[str, Path],
) -> tuple[tuple[str, str], ...]:
    """Fingerprint files that can affect Xacro expansion or URI rewriting."""
    roots = list(package_paths.values()) or [urdf_path.parent]
    fingerprints = []
    for root in sorted(set(roots), key=str):
        fingerprints.append((str(root), _directory_fingerprint(root)))
    return tuple(fingerprints)


def _directory_fingerprint(root: Path) -> str:
    digest = hashlib.sha256()
    if not root.exists():
        digest.update(b"missing")
        return digest.hexdigest()[:16]

    for path in sorted(root.rglob("*")):
        if ".git" in path.parts:
            continue
        try:
            stat = path.stat()
        except OSError:
            continue
        if not path.is_file():
            continue
        relative_path = path.relative_to(root)
        digest.update(str(relative_path).encode())
        digest.update(str(stat.st_size).encode())
        digest.update(path.read_bytes())
    return digest.hexdigest()[:16]


def _process_xacro(
    xacro_path: Path,
    package_paths: dict[str, Path],
    xacro_args: dict[str, str],
) -> str:
    try:
        from dimos.utils.ament_prefix import process_xacro
    except ImportError:
        raise ImportError(
            "xacro is required for processing .xacro files. "
            "Install the manipulation extra: pip install dimos[manipulation]"
        )

    return process_xacro(xacro_path, package_paths, xacro_args)
