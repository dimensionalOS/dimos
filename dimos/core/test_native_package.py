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

"""Native preparation must work without Python-adjacent build directories."""

from contextlib import nullcontext
from io import StringIO
import json
from pathlib import Path
import subprocess
from unittest.mock import MagicMock

import pytest

from dimos.core import native_package
from dimos.core.native_module import NativeModuleConfig
from dimos.experimental.memory.rust_recorder import RustRecorder
from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2
from dimos.hardware.sensors.lidar.livox.module import Mid360
from dimos.hardware.sensors.lidar.pointlio.module import PointLio


@pytest.fixture
def isolated_packages(tmp_path, monkeypatch):
    monkeypatch.setattr(native_package, "CACHE_DIR", tmp_path / "cache")
    monkeypatch.setattr(native_package, "DIMOS_PROJECT_ROOT", tmp_path / "installed")
    package_dir = tmp_path / "installed" / "dimos"
    package_dir.mkdir(parents=True)
    catalog = native_package._PACKAGE_DIR / "native_packages.json"
    (package_dir / "native_packages.json").write_text(catalog.read_text())
    (package_dir / "_native_revision.json").write_text(json.dumps({"revision": "a" * 40}))
    monkeypatch.setattr(native_package, "_PACKAGE_DIR", package_dir)
    monkeypatch.setattr(native_package, "cache_usage_guard", nullcontext)
    return package_dir


def test_installed_package_uses_pinned_source(isolated_packages):
    package = native_package.native_packages()["dimos-memory-recorder"]
    assert native_package.package_reference(package) == (
        "github:dimensionalOS/dimos/"
        + "a" * 40
        + "?dir=dimos/experimental/memory/rust#dimos-memory-recorder"
    )


def test_checkout_uses_local_inputs(isolated_packages):
    (isolated_packages.parent / ".git").write_text("gitdir: /unused")
    package = native_package.native_packages()["dimos-memory-recorder"]
    assert native_package.package_reference(package) == (
        f"{isolated_packages.parent}/dimos/experimental/memory/rust#dimos-memory-recorder"
    )


@pytest.mark.parametrize("metadata", [None, {"revision": "main"}, {"revision": 42}])
def test_installed_package_never_guesses_revision(isolated_packages, metadata):
    path = isolated_packages / "_native_revision.json"
    if metadata is None:
        path.unlink()
    else:
        path.write_text(json.dumps(metadata))
    with pytest.raises(RuntimeError, match="revision metadata"):
        native_package.source_revision()


def test_preparation_reuses_exact_outputs_and_rechecks_inputs(isolated_packages, tmp_path, mocker):
    mocker.patch.object(native_package.shutil, "which", return_value="/bin/nix")
    generation = ["first"]
    builds = []

    def nix(arguments):
        if arguments[0] == "eval":
            return f"/nix/store/{generation[0]}.drv"
        builds.append(arguments)
        result = Path(arguments[arguments.index("--out-link") + 1])
        output = tmp_path / generation[0]
        executable = output / "bin" / "dimos-memory-recorder"
        executable.parent.mkdir(parents=True)
        executable.write_text("#!/bin/sh\nexit 0\n")
        executable.chmod(0o755)
        result.symlink_to(output, target_is_directory=True)
        return ""

    mocker.patch.object(native_package, "_nix", side_effect=nix)
    first = native_package.ensure_native_package("dimos-memory-recorder")
    assert native_package.ensure_native_package("dimos-memory-recorder") == first
    assert len(builds) == 1
    generation[0] = "changed"
    second = native_package.ensure_native_package("dimos-memory-recorder")
    assert second != first
    assert first.is_file()  # Other worktrees/runs retain their rooted result.
    assert len(builds) == 2
    assert builds[0][-4:] == ["--max-jobs", "1", "--cores", "2"]


def test_missing_nix_has_installation_guidance(isolated_packages, mocker):
    mocker.patch.object(native_package.shutil, "which", return_value=None)
    with pytest.raises(RuntimeError, match="require Nix"):
        native_package.ensure_native_package("mid360")


def test_failed_evaluation_does_not_attempt_build(isolated_packages, mocker):
    mocker.patch.object(native_package.shutil, "which", return_value="nix")
    nix = mocker.patch.object(native_package, "_nix", side_effect=RuntimeError("download failed"))
    with pytest.raises(RuntimeError, match="download failed"):
        native_package.ensure_native_package("pointlio")
    assert nix.call_count == 1


def test_failed_nix_command_reports_status(mocker):
    process = MagicMock(spec=subprocess.Popen)
    process.__enter__.return_value = process
    process.stdout = StringIO("")
    process.wait.return_value = 7
    mocker.patch.object(native_package.subprocess, "Popen", return_value=process)
    with pytest.raises(RuntimeError, match="exit 7"):
        native_package._nix(["build", "example"])


@pytest.mark.parametrize(
    "module_class, package_id",
    [
        (RustRecorder, "dimos-memory-recorder"),
        (Mid360, "mid360"),
        (PointLio, "pointlio"),
        (FastLio2, "fastlio2"),
    ],
)
def test_modules_use_prepared_executable_without_source_cwd(
    module_class, package_id, tmp_path, mocker
):
    executable = tmp_path / "store" / "bin" / "native"
    prepare = mocker.patch(
        "dimos.core.native_module.ensure_native_package", return_value=executable
    )
    module = module_class()
    try:
        module._maybe_build()
        assert module._argv({})[0] == str(executable)
        assert module.config.cwd is None
        assert "native_package" not in module.config.to_config_dict()
        prepare.assert_called_once_with(package_id)
    finally:
        module.stop()


def test_package_does_not_accept_checkout_build_configuration():
    with pytest.raises(ValueError, match="cannot be combined"):
        NativeModuleConfig(native_package="mid360", cwd="cpp")
