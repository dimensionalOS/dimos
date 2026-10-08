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

"""Private preparation of native sources shipped in a Python package."""

from collections.abc import Callable
from hashlib import sha256
from importlib.resources import files
import json
import os
from pathlib import Path
import platform
import shlex
import shutil
import subprocess

from filelock import FileLock

from dimos.constants import CACHE_DIR

_IGNORED = {".git", ".venv", "__pycache__", "target", "build", "dist"}
_BUILD_ENV = (
    "PATH",
    "CC",
    "CXX",
    "CFLAGS",
    "CXXFLAGS",
    "CPPFLAGS",
    "LDFLAGS",
    "RUSTFLAGS",
    "CARGO_ENCODED_RUSTFLAGS",
    "CARGO_BUILD_TARGET",
    "CARGO_TARGET_DIR",
    "RUSTUP_TOOLCHAIN",
    "CMAKE_GENERATOR",
    "CMAKE_PREFIX_PATH",
    "PKG_CONFIG_PATH",
)


def _toolchain(command: str, env: dict[str, str]) -> dict[str, str]:
    """Fingerprint common builders; arbitrary shell dependencies still need a forced rebuild."""
    words = set(shlex.split(command))
    names = words.intersection({"cargo", "rustc", "cmake", "nix"})
    if "cargo" in names:
        names.add("rustc")
    if "cmake" in names:
        names.update((env.get("CC", "cc"), env.get("CXX", "c++")))
    result = {}
    for name in sorted(names):
        args = shlex.split(name)
        executable = shutil.which(args[0], path=env.get("PATH"))
        if executable is None:
            result[name] = "missing"
            continue
        probe = subprocess.run(
            [executable, *args[1:], "--version"],
            env=env,
            capture_output=True,
            text=True,
            timeout=10,
            check=True,
        )
        result[name] = executable + "\n" + probe.stdout + probe.stderr
    return result


def prepare_package_source(
    package: str,
    source_dir: str,
    executable: str,
    command: str,
    extra_env: dict[str, str],
    build: Callable[[Path, Path], None],
    *,
    rebuild: bool,
) -> tuple[str, str]:
    """Snapshot a complete source directory, then build under a cross-process lock.

    Each recipe gets a separate writable tree. Nothing is generated in site-packages.
    Failed or interrupted builds have no completion marker and are recreated on retry.
    Stop dependent runs before forcing a rebuild or deleting their cache.
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
    # Include file contents and modes, so editable changes invalidate without a version bump.
    inputs: list[tuple[Path, bytes, int]] = []
    digest = sha256()
    for path in sorted(source.rglob("*")):
        relative = path.relative_to(source)
        if _IGNORED.intersection(relative.parts):
            continue
        if path.is_symlink():
            raise ValueError(f"Packaged native sources must not contain symlinks: {relative}")
        if path.is_file():
            data, mode = path.read_bytes(), path.stat().st_mode & 0o777
            inputs.append((relative, data, mode))
            digest.update(relative.as_posix().encode() + b"\0")
            digest.update(str(mode).encode() + b"\0" + sha256(data).digest())
    if not inputs:
        raise ValueError("Packaged native source directory is empty")
    env = {**os.environ, **extra_env}
    identity = {
        "schema": 1,
        "package": package,
        "source_dir": source_dir,
        "source": digest.hexdigest(),
        "executable": executable,
        "command": command,
        "platform": platform.platform(),
        "machine": platform.machine(),
        "environment": {key: env.get(key) for key in _BUILD_ENV},
        "extra_env": extra_env,
        "toolchain": _toolchain(command, env),
    }
    key = sha256(json.dumps(identity, sort_keys=True).encode()).hexdigest()
    entry = CACHE_DIR / "native-packages" / key
    entry.mkdir(parents=True, exist_ok=True)
    snapshot = entry / "source"
    artifact = snapshot / executable
    complete = entry / "complete"
    with FileLock(entry / "build.lock"):
        if (
            rebuild
            or not complete.is_file()
            or not artifact.is_file()
            or not os.access(artifact, os.X_OK)
        ):
            complete.unlink(missing_ok=True)
            shutil.rmtree(snapshot, ignore_errors=True)
            snapshot.mkdir()
            for relative, data, mode in inputs:
                destination = snapshot / relative
                destination.parent.mkdir(parents=True, exist_ok=True)
                destination.write_bytes(data)
                destination.chmod(mode)
            build(snapshot, artifact)
            if not artifact.resolve().is_relative_to(snapshot):
                raise ValueError("Built executable must stay inside the cached source tree")
            if not artifact.is_file() or not os.access(artifact, os.X_OK):
                raise FileNotFoundError(f"Build did not produce an executable file: {artifact}")
            marker = entry / "complete.tmp"
            marker.write_text(key)
            marker.replace(complete)
        return str(snapshot), str(artifact)
