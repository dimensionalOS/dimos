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

"""Package-owned runtime projects using ordinary Python/uv dependency metadata."""

from __future__ import annotations

from dataclasses import dataclass
from hashlib import sha256
from importlib.metadata import distribution as installed_distribution
from importlib.resources import files
from importlib.util import find_spec
import json
import os
from pathlib import Path
import platform
import shutil
import sys
import tempfile

from filelock import FileLock

from dimos.constants import CACHE_DIR

_IGNORED = {"__pycache__", ".git", ".venv", "node_modules", "build", "dist"}
_PROVENANCE_ENV = "DIMOS_ISOLATED_PROVENANCE"


def _source_files(root: Path) -> list[Path]:
    return sorted(
        path
        for path in root.rglob("*")
        if path.is_file()
        and not _IGNORED.intersection(path.relative_to(root).parts)
        and path.suffix not in {".pyc", ".pyo"}
    )


def _digest(root: Path, paths: list[Path]) -> str:
    digest = sha256()
    for path in paths:
        relative = path.relative_to(root).as_posix()
        digest.update(relative.encode() + b"\0")
        digest.update(sha256(path.read_bytes()).digest())
    return digest.hexdigest()


def contract_fingerprint(distribution: str, package: str) -> dict[str, str]:
    """Compare Python contract code, even for editable installs with unchanged versions.

    This is an alignment check, not an artifact signature or a native ABI guarantee.
    Deployment tooling remains responsible for locking full distribution hashes.
    """
    spec = find_spec(package)
    if spec is None or not spec.submodule_search_locations:
        raise ValueError(f"Shared contract {package!r} must be an importable package")
    roots = list(spec.submodule_search_locations)
    if len(roots) != 1:
        raise ValueError(f"Shared contract {package!r} must have one package directory")
    root = Path(roots[0]).resolve()
    sources = [path for path in _source_files(root) if path.suffix in {".py", ".pyi"}]
    if not sources:
        raise ValueError(f"Shared contract {package!r} contains no Python source")
    metadata = installed_distribution(distribution)
    payload = sorted(
        (str(path), path.hash.mode, path.hash.value)
        for path in metadata.files or ()
        if path.hash is not None
        and not any(part.endswith((".dist-info", ".egg-info")) for part in path.parts)
        and ".." not in path.parts
    )
    direct_url = json.loads(metadata.read_text("direct_url.json") or "{}")
    if direct_url.get("dir_info", {}).get("editable"):
        payload = []
    return {
        "version": metadata.version,
        "python_source": _digest(root, sources),
        "payload": sha256(json.dumps(payload).encode()).hexdigest() if payload else "editable",
    }


def verify_environment() -> None:
    """Reject a child that resolved different DimOS or shared contract code."""
    for expected in json.loads(os.environ.get(_PROVENANCE_ENV, "[]")):
        actual = contract_fingerprint(expected["distribution"], expected["package"])
        if actual != expected["fingerprint"]:
            raise RuntimeError(
                f"Isolated Python contract mismatch for {expected['distribution']!r}: "
                f"expected {expected['fingerprint']}, got {actual}; install matching "
                "DimOS/contract artifacts or use matching editable sources"
            )


@dataclass(frozen=True)
class PackageRuntime:
    project: Path
    provenance: str

    @property
    def environment(self) -> Path:
        return self.project.parent / ".venv"

    @property
    def prepare_lock(self) -> FileLock:
        return FileLock(self.project.parent / "prepare.lock")


@dataclass(frozen=True)
class PackageProject:
    """A filesystem-installed package's runtime project, resolved only at build.

    ``shared_packages`` adds (distribution, import package) contracts, such as
    application message bindings. DimOS and the owning contract are always checked.
    uv resolves the runtime's own pyproject/lock from configured indexes/wheelhouses.
    """

    package: str
    path: str
    distribution: str
    shared_packages: tuple[tuple[str, str], ...] = ()

    def resolve(self) -> PackageRuntime:
        relative = Path(self.path)
        if relative.is_absolute() or ".." in relative.parts or not relative.parts:
            raise ValueError("PackageProject.path must be a non-empty relative package directory")
        resource = files(self.package)
        if not isinstance(resource, Path):
            raise TypeError("PackageProject requires an unpacked wheel or editable package")
        package_root = resource.resolve()
        source = (package_root / relative).resolve()
        if not source.is_relative_to(package_root):
            raise ValueError("PackageProject.path must stay inside its owning package")
        if not (source / "pyproject.toml").is_file():
            raise FileNotFoundError(
                f"Packaged runtime manifest is missing: {source / 'pyproject.toml'}"
            )
        paths = _source_files(source)
        if any(path.is_symlink() for path in paths):
            raise ValueError("Packaged runtime files must not be symlinks")
        contracts = dict(
            (("dimos", "dimos"), (self.distribution, self.package), *self.shared_packages)
        )
        provenance = json.dumps(
            [
                {
                    "distribution": distribution,
                    "package": package,
                    "fingerprint": contract_fingerprint(distribution, package),
                }
                for distribution, package in sorted(contracts.items())
            ],
            sort_keys=True,
        )
        identity = json.dumps(
            {
                "source": _digest(source, paths),
                "provenance": provenance,
                "python": sys.version,
                "platform": platform.platform(),
            },
            sort_keys=True,
        )
        cache = CACHE_DIR / "isolated-python" / sha256(identity.encode()).hexdigest()
        cache.mkdir(parents=True, exist_ok=True)
        project = cache / "project"
        with FileLock(cache / "prepare.lock"):
            if not project.is_dir():
                staging = Path(tempfile.mkdtemp(prefix="project-", dir=cache))
                try:
                    for path in paths:
                        target = staging / path.relative_to(source)
                        target.parent.mkdir(parents=True, exist_ok=True)
                        shutil.copy2(path, target)
                    staging.rename(project)
                finally:
                    if staging.exists():
                        shutil.rmtree(staging)
        return PackageRuntime(project, provenance)
