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

"""Invoke a cargo-shaped builder on the generated crate and collect the binary."""

from __future__ import annotations

from pathlib import Path
import shutil
import subprocess

from dimos.cli.bake.errors import BakeError

BUILDERS = ("cargo", "cross", "zigbuild")

_INVOCATION = {
    "cargo": ["cargo", "build"],
    "cross": ["cross", "build"],
    "zigbuild": ["cargo", "zigbuild"],
}


def build_command(builder: str, *, target: str | None = None, debug: bool = False) -> list[str]:
    if builder not in _INVOCATION:
        raise BakeError(f"unknown --builder {builder!r}; choose from {', '.join(BUILDERS)}")
    cmd = list(_INVOCATION[builder])
    # Pin the output dir artifact_path reads, against an inherited CARGO_TARGET_DIR.
    # Relative so it also resolves inside a cross container.
    cmd.extend(["--target-dir", "target"])
    if not debug:
        cmd.append("--release")
    if target:
        cmd.extend(["--target", target])
    return cmd


def target_dir_name(target: str) -> str:
    """The output directory for a target triple, with any glibc suffix stripped."""
    return target.split(".", 1)[0]


def artifact_path(
    crate_dir: Path, host: str, *, target: str | None = None, debug: bool = False
) -> Path:
    profile = "debug" if debug else "release"
    out = crate_dir / "target"
    if target:
        out = out / target_dir_name(target)
    return out / profile / host


def build_host(
    crate_dir: Path,
    host: str,
    *,
    builder: str = "cargo",
    target: str | None = None,
    debug: bool = False,
) -> Path:
    """Compile the generated crate and return the path to the built binary."""
    cmd = build_command(builder, target=target, debug=debug)
    if shutil.which(cmd[0]) is None:
        raise BakeError(f"`{cmd[0]}` is not on PATH")
    result = subprocess.run(cmd, cwd=crate_dir, check=False)
    if result.returncode != 0:
        raise BakeError(f"{' '.join(cmd)} failed with exit {result.returncode}")
    artifact = artifact_path(crate_dir, host, target=target, debug=debug)
    if not artifact.exists():
        raise BakeError(f"build succeeded but {artifact} is missing")
    return artifact


def install(artifact: Path, out: Path) -> int:
    """Copy the built binary to `out`, returning its size in bytes."""
    out.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(artifact, out)
    return out.stat().st_size
