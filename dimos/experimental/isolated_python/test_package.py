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

import json
import subprocess

import pytest

from dimos.experimental.isolated_python.module import IsolatedPythonModule
from dimos.experimental.isolated_python.package import (
    PackageProject,
    contract_fingerprint,
    verify_environment,
)


@pytest.fixture
def packaged_project(tmp_path, monkeypatch, mocker):
    mocker.patch.dict("sys.modules")
    package = tmp_path / "package_contract"
    package.mkdir()
    (package / "__init__.py").write_text("VALUE = 1\n")
    runtime = package / "runtime"
    runtime.mkdir()
    (runtime / "pyproject.toml").write_text('[project]\nname = "runtime"\nversion = "0.1"\n')
    (runtime / "runtime.py").write_text("VALUE = 2\n")
    metadata = tmp_path / "package_contract-0.1.dist-info"
    metadata.mkdir()
    (metadata / "METADATA").write_text("Name: package-contract\nVersion: 0.1\n")
    monkeypatch.syspath_prepend(str(tmp_path))
    monkeypatch.setattr("dimos.experimental.isolated_python.package.CACHE_DIR", tmp_path / "cache")
    return PackageProject("package_contract", "runtime", "package-contract"), runtime


def test_packaged_project_is_snapshotted_and_reuses_cache(packaged_project):
    project, source = packaged_project
    first = project.resolve()
    assert first.project != source
    assert (first.project / "runtime.py").read_text() == "VALUE = 2\n"
    assert project.resolve() == first
    (source / "uv.lock").write_text("version = 1\n")
    changed = project.resolve()
    assert changed.environment != first.environment
    assert not (first.project / "uv.lock").exists()
    assert (changed.project / "uv.lock").read_text() == "version = 1\n"


def test_runtime_source_and_host_contract_edits_invalidate_cache(packaged_project):
    project, source = packaged_project
    first = project.resolve()
    (source / "runtime.py").write_text("VALUE = 3\n")
    second = project.resolve()
    assert second.environment != first.environment
    (source.parent / "__init__.py").write_text("VALUE = 4\n")
    assert project.resolve().environment != second.environment


@pytest.mark.parametrize("path", ["..", "../runtime", "/runtime", ""])
def test_package_project_cannot_escape_its_package(packaged_project, path):
    with pytest.raises(ValueError, match="relative package directory"):
        PackageProject("package_contract", path, "package-contract").resolve()


def test_missing_runtime_manifest_reports_package_path(packaged_project):
    project, source = packaged_project
    (source / "pyproject.toml").unlink()
    with pytest.raises(FileNotFoundError, match="Packaged runtime manifest"):
        project.resolve()


def test_child_rejects_same_version_with_different_contract_code(packaged_project, monkeypatch):
    _, source = packaged_project
    expected = contract_fingerprint("package-contract", "package_contract")
    monkeypatch.setenv(
        "DIMOS_ISOLATED_PROVENANCE",
        json.dumps(
            [
                {
                    "distribution": "package-contract",
                    "package": "package_contract",
                    "fingerprint": expected,
                }
            ]
        ),
    )
    verify_environment()
    (source.parent / "__init__.py").write_text("VALUE = 99\n")
    with pytest.raises(RuntimeError, match="contract mismatch.*package-contract"):
        verify_environment()


def test_package_preparation_is_lazy_and_does_not_overlay_checkout(
    packaged_project, mocker, monkeypatch
):
    project, _ = packaged_project

    class PackagedContract(IsolatedPythonModule):
        package_project = project
        implementation = "runtime:Implementation"

    resolve = mocker.spy(PackageProject, "resolve")
    checkout = mocker.patch("dimos.experimental.isolated_python.module.get_project_root")
    run = mocker.patch(
        "dimos.experimental.isolated_python.module.subprocess.run",
        return_value=subprocess.CompletedProcess([], 0, "", ""),
    )
    monkeypatch.setenv("PYTHONPATH", "/host/dependencies")
    module = PackagedContract()
    try:
        PackagedContract.blueprint()
        resolve.assert_not_called()
        module._run_prepare()
        command = run.call_args.args[0]
        assert "--project" in command
        assert "--with-editable" not in command
        assert "verify_environment()" in command[-1]
        assert "PYTHONPATH" not in run.call_args.kwargs["env"]
        assert "--no-sync" in module._launch_command(7)
        checkout.assert_not_called()
    finally:
        module.stop()
