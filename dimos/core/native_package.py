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

"""Prepare pinned Nix packages for installed DimOS and local checkouts."""

from dataclasses import dataclass
import json
import os
from pathlib import Path
import re
import shutil
import subprocess

from filelock import FileLock

from dimos.constants import CACHE_DIR, DIMOS_PROJECT_ROOT
from dimos.utils.cache import cache_usage_guard
from dimos.utils.logging_config import setup_logger

logger = setup_logger()
_PACKAGE_DIR = Path(__file__).resolve().parents[1]


@dataclass(frozen=True)
class NativePackage:
    flake_dir: str
    attribute: str
    executable: str


def native_packages() -> dict[str, NativePackage]:
    """Load the catalog also consumed by the dependency-free CI publisher."""
    data = json.loads((_PACKAGE_DIR / "native_packages.json").read_text())
    return {name: NativePackage(**spec) for name, spec in data.items()}


def source_revision() -> str:
    """The immutable source revision carried by wheels and source distributions."""
    metadata = _PACKAGE_DIR / "_native_revision.json"
    if not metadata.is_file():
        raise RuntimeError("DimOS distribution is missing native source revision metadata")
    revision = json.loads(metadata.read_text()).get("revision", "")
    if not isinstance(revision, str) or not re.fullmatch(r"[0-9a-f]{40}", revision):
        raise RuntimeError("DimOS distribution has invalid native source revision metadata")
    return revision


def package_reference(package: NativePackage) -> str:
    """Use local Git inputs only when the imported package belongs to a checkout."""
    if (DIMOS_PROJECT_ROOT / ".git").exists():
        flake = DIMOS_PROJECT_ROOT / package.flake_dir
        return f"{flake}#{package.attribute}"
    return (
        f"github:dimensionalOS/dimos/{source_revision()}"
        f"?dir={package.flake_dir}#{package.attribute}"
    )


def _nix(arguments: list[str]) -> str:
    command = [
        "nix",
        "--extra-experimental-features",
        "nix-command flakes",
        "--extra-substituters",
        "https://dimensionalos.cachix.org",
        "--extra-trusted-public-keys",
        "dimensionalos.cachix.org-1:20ynj6TjpoD3qTxkdNoeHtgs2G2pNvgAq1EQYLTHJXI=",
        *arguments,
    ]
    # Stdout is machine-readable; stream stderr so evaluation and builds remain visible.
    with subprocess.Popen(command, stdout=subprocess.PIPE, text=True) as process:
        assert process.stdout is not None
        output = process.stdout.read()
        status = process.wait()
    if status:
        raise RuntimeError(
            f"Native package preparation failed (exit {status}): {' '.join(command)}"
        )
    return output.strip()


def ensure_native_package(package_id: str) -> Path:
    """Reuse, substitute, or build a package and return its immutable executable path.

    Evaluate on every preparation so edits to native inputs cannot reuse a stale
    checkout result. Nix owns input hashing and substitution; no Git dirty-bit
    heuristic or Python build cache is involved.
    """
    packages = native_packages()
    if package_id not in packages:
        raise ValueError(
            f"Unknown native package {package_id!r}; choose from {', '.join(packages)}"
        )
    if shutil.which("nix") is None:
        raise RuntimeError(
            "Native packages require Nix with flakes enabled. "
            "See https://github.com/dimensionalOS/dimos/blob/main/docs/installation/nix.md and configure the dimensionalos Cachix cache."
        )
    package = packages[package_id]
    reference = package_reference(package)
    logger.info("Preparing native package", package=package_id, source=reference)
    with cache_usage_guard():
        derivation = _nix(["eval", "--raw", "--no-update-lock-file", f"{reference}.drvPath"])
        if not derivation.startswith("/nix/store/") or not derivation.endswith(".drv"):
            raise RuntimeError(f"Nix returned an invalid derivation path: {derivation!r}")
        directory = CACHE_DIR / "native" / Path(derivation).name
        directory.mkdir(parents=True, exist_ok=True)
        with FileLock(directory / "prepare.lock"):
            result = directory / "result"
            executable = result / "bin" / package.executable
            if not executable.is_file():
                _nix(
                    [
                        "build",
                        "-L",
                        f"{derivation}^out",
                        "--out-link",
                        str(result),
                        "--max-jobs",
                        "1",
                        "--cores",
                        "2",
                    ]
                )
            if not executable.is_file() or not os.access(executable, os.X_OK):
                raise RuntimeError(f"Native package {package_id!r} did not provide {executable}")
            return executable.resolve()
