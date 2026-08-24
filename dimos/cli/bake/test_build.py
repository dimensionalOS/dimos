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

"""Unit tests for the bake build helpers."""

from __future__ import annotations

from pathlib import Path
import subprocess

import pytest

from dimos.cli.bake import build
from dimos.cli.bake.build import (
    TARGET_DIR,
    artifact_path,
    build_command,
    build_host,
    install,
    target_dir_name,
)
from dimos.cli.bake.codegen import crate_dir
from dimos.cli.bake.errors import BakeError


def test_build_command_per_builder() -> None:
    crate = crate_dir("go2-nav")
    assert build_command("cargo", crate) == [
        "cargo",
        "build",
        "--target-dir",
        "../../../target",
        "--release",
    ]
    assert build_command("cargo", crate, debug=True)[-1] != "--release"
    assert build_command("cross", crate, target="aarch64-unknown-linux-musl")[:2] == [
        "cross",
        "build",
    ]
    assert build_command("zigbuild", crate)[:2] == ["cargo", "zigbuild"]
    assert build_command("cargo", crate, target="aarch64-unknown-linux-musl")[-2:] == [
        "--target",
        "aarch64-unknown-linux-musl",
    ]
    with pytest.raises(BakeError, match="unknown --builder"):
        build_command("make", crate)


def test_build_command_targets_the_shared_workspace_dir() -> None:
    crate = crate_dir("go2-nav")
    cmd = build_command("cargo", crate)
    given = cmd[cmd.index("--target-dir") + 1]
    assert (crate / given).resolve() == TARGET_DIR


def test_target_dir_name_strips_the_zigbuild_glibc_suffix() -> None:
    assert target_dir_name("aarch64-unknown-linux-gnu.2.31") == "aarch64-unknown-linux-gnu"
    assert target_dir_name("aarch64-unknown-linux-gnu") == "aarch64-unknown-linux-gnu"


def test_artifact_path_follows_profile_and_target() -> None:
    assert artifact_path("host") == TARGET_DIR / "release" / "host"
    assert artifact_path("host", debug=True) == TARGET_DIR / "debug" / "host"
    assert (
        artifact_path("host", target="aarch64-unknown-linux-musl")
        == TARGET_DIR / "aarch64-unknown-linux-musl" / "release" / "host"
    )
    assert (
        artifact_path("host", target="aarch64-unknown-linux-gnu.2.31")
        == TARGET_DIR / "aarch64-unknown-linux-gnu" / "release" / "host"
    )


def test_build_host_requires_the_builder_on_path(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(build.shutil, "which", lambda _: None)
    with pytest.raises(BakeError, match="not on PATH"):
        build_host(tmp_path, "host")


def test_build_host_surfaces_a_failed_build(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(build.shutil, "which", lambda _: "/usr/bin/cargo")
    monkeypatch.setattr(
        build.subprocess,
        "run",
        lambda *a, **k: subprocess.CompletedProcess(a, returncode=101),
    )
    with pytest.raises(BakeError, match="exit 101"):
        build_host(tmp_path, "host")


def test_build_host_reports_a_missing_artifact(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(build.shutil, "which", lambda _: "/usr/bin/cargo")
    monkeypatch.setattr(
        build.subprocess,
        "run",
        lambda *a, **k: subprocess.CompletedProcess(a, returncode=0),
    )
    with pytest.raises(BakeError, match="is missing"):
        build_host(tmp_path, "host")


def test_install_copies_the_binary_and_reports_its_size(tmp_path: Path) -> None:
    artifact = tmp_path / "target" / "release" / "host"
    artifact.parent.mkdir(parents=True)
    artifact.write_bytes(b"\x7fELF host")
    artifact.chmod(0o755)
    out = tmp_path / "deploy" / "bin" / "host"
    assert install(artifact, out) == len(b"\x7fELF host")
    assert out.read_bytes() == b"\x7fELF host"
    assert out.stat().st_mode & 0o111
