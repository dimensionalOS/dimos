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

"""Writable source workspaces for installed native modules, not a build runner."""

from collections.abc import Iterator
from contextlib import contextmanager
from hashlib import sha256
from importlib.resources import files
import json
import os
from pathlib import Path
import platform

from filelock import FileLock

from dimos.constants import CACHE_DIR

_IGNORED = {".git", ".venv", "__pycache__", "target", "build", "dist"}


@contextmanager
def package_source_workspace(
    package: str,
    source_dir: str,
    executable: str,
    command: str,
    extra_env: dict[str, str],
) -> Iterator[tuple[str, str]]:
    """Yield locked writable paths; the caller owns the single shared build path.

    Update only source-owned files. Preserve generated outputs for the native
    builder's incremental work, but invalidate an executable after source changes
    or an interrupted/failed preparation. Installed sources are never modified.
    """
    resource = files(package)
    if not isinstance(resource, Path):
        raise TypeError("source_package requires an unpacked wheel or editable package")
    package_root = resource.resolve()
    source = package_root / source_dir
    if not source.resolve().is_relative_to(package_root):
        raise ValueError("source_dir must stay inside source_package")
    if not source.is_dir():
        raise FileNotFoundError(f"Packaged native source directory is missing: {source}")
    inputs: dict[str, tuple[bytes, int]] = {}
    digest = sha256()
    for path in sorted(source.rglob("*")):
        relative = path.relative_to(source)
        if _IGNORED.intersection(relative.parts):
            continue
        if path.is_symlink():
            raise ValueError(f"Packaged native sources must not contain symlinks: {relative}")
        if path.is_file():
            data, mode = path.read_bytes(), path.stat().st_mode & 0o777
            inputs[relative.as_posix()] = data, mode
            digest.update(relative.as_posix().encode() + b"\0")
            digest.update(str(mode).encode() + b"\0" + sha256(data).digest())
    if not inputs:
        raise ValueError("Packaged native source directory is empty")
    revision = digest.hexdigest()
    # Stable across source edits, isolated across locations and explicit recipes.
    # Ambient toolchain changes use the existing explicit force-build flags.
    identity = (str(source), command, executable, extra_env, platform.system(), platform.machine())
    key = sha256(json.dumps(identity, sort_keys=True).encode()).hexdigest()
    entry = CACHE_DIR / "native-packages" / key
    entry.mkdir(parents=True, exist_ok=True)
    workspace = entry / "source"
    artifact = workspace / executable
    complete = entry / "complete"
    manifest = entry / "sources.json"
    with FileLock(entry / "build.lock"):
        workspace.mkdir(exist_ok=True)
        if not complete.exists() or complete.read_text() != revision:
            complete.unlink(missing_ok=True)
            previous = set(json.loads(manifest.read_text())) if manifest.exists() else set()
            removed = previous - inputs.keys()
            for name in removed:
                (workspace / name).unlink(missing_ok=True)
            # Only prune empty source-owned directories, never generated outputs.
            directories = {
                parent
                for name in removed
                for parent in (workspace / name).parents
                if parent != workspace and parent.is_relative_to(workspace)
            }
            for directory in sorted(directories, key=lambda path: len(path.parts), reverse=True):
                if directory.is_dir() and not any(directory.iterdir()):
                    directory.rmdir()
            for name, (data, mode) in inputs.items():
                destination = workspace / name
                destination.parent.mkdir(parents=True, exist_ok=True)
                # Preserve timestamps of unchanged inputs for Cargo/CMake.
                if not destination.is_file() or destination.read_bytes() != data:
                    destination.write_bytes(data)
                destination.chmod(mode)
            temporary = entry / "sources.tmp"
            temporary.write_text(json.dumps(sorted(inputs)))
            temporary.replace(manifest)
            artifact.unlink(missing_ok=True)
        if artifact.is_file() and not os.access(artifact, os.X_OK):
            artifact.unlink()
        # Failure in the shared builder leaves no marker, so retry cannot reuse
        # a partially written executable. Other generated build outputs survive.
        complete.unlink(missing_ok=True)
        yield str(workspace), str(artifact)
        if not artifact.resolve().is_relative_to(workspace):
            raise ValueError("Built executable must stay inside the cached source tree")
        if not artifact.is_file() or not os.access(artifact, os.X_OK):
            raise FileNotFoundError(f"Build did not produce an executable file: {artifact}")
        marker = entry / "complete.tmp"
        marker.write_text(revision)
        marker.replace(complete)
