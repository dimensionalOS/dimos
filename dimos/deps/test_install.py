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

import os
from pathlib import Path
import subprocess
import sys

import pytest
from pytest_mock import MockerFixture

from dimos.core import run_registry
from dimos.core.run_registry import RunEntry
from dimos.deps import install
from dimos.deps.install import (
    LEAKING_VARIABLES,
    PrepareError,
    Target,
    find_uv,
    parse_uv_version,
    pip_install_command,
    prepare,
    refuse_active_runs,
    refuse_backend_switch,
    refuse_system_python,
    sync_command,
    uv_environment,
    verify_providers,
)


@pytest.fixture
def venv(tmp_path: Path) -> Target:
    return Target(
        python=tmp_path / "venv" / "bin" / "python",
        prefix=tmp_path / "venv",
        base_prefix=tmp_path / "base",
    )


@pytest.fixture
def registry_dir(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    path = tmp_path / "runs"
    monkeypatch.setattr(run_registry, "REGISTRY_DIR", path)
    return path


def _entry(environment: str) -> RunEntry:
    return RunEntry(
        run_id="20260101-000000-abcd-unitree-go2",
        pid=os.getpid(),
        blueprint="unitree-go2",
        started_at="2026-01-01T00:00:00+00:00",
        log_dir="/tmp/log",
        environment=environment,
    )


def test_sync_command_targets_the_running_interpreter_and_lock(
    venv: Target, tmp_path: Path
) -> None:
    command = sync_command(
        ["uv"], tmp_path, venv, ["runtime-common", "runtime-unitree"], "cuda", offline=True
    )

    assert command == [
        "uv",
        "sync",
        "--locked",
        "--inexact",
        "--no-default-groups",
        "--project",
        str(tmp_path),
        "--python",
        str(venv.python),
        "--offline",
        "--extra",
        "runtime-common",
        "--extra",
        "runtime-unitree",
        "--extra",
        "cuda",
    ]


def test_repair_reinstalls_only_the_gpu_onnxruntime(venv: Target, tmp_path: Path) -> None:
    command = sync_command(
        ["uv"],
        tmp_path,
        venv,
        ["runtime-common"],
        "cuda",
        offline=False,
        reinstall="onnxruntime-gpu",
    )

    assert command[-2:] == ["--reinstall-package", "onnxruntime-gpu"]
    assert "--offline" not in command


def test_pip_install_command_installs_one_lock_into_the_interpreter(venv: Target) -> None:
    lock = Path("/pkg/dimos/deps/locks/pylock.runtime-common-cpu.toml")

    command = pip_install_command(["uv"], venv, lock, offline=True)

    assert command == [
        "uv",
        "pip",
        "install",
        "--python",
        str(venv.python),
        "-r",
        str(lock),
        "--offline",
    ]


def test_uv_environment_scrubs_redirecting_variables_and_keeps_the_rest(
    venv: Target, monkeypatch: pytest.MonkeyPatch
) -> None:
    for key in LEAKING_VARIABLES:
        monkeypatch.setenv(key, "leaked")
    monkeypatch.setenv("HTTPS_PROXY", "http://proxy.example:3128")

    env = uv_environment(venv, offline=False)

    assert env["UV_PROJECT_ENVIRONMENT"] == str(venv.prefix)
    assert env["UV_PYTHON_DOWNLOADS"] == "never"
    assert env["UV_PREVIEW_FEATURES"] == "pylock"
    assert env["HTTPS_PROXY"] == "http://proxy.example:3128"
    assert "UV_PYTHON" not in env
    assert "VIRTUAL_ENV" not in env
    assert "UV_OFFLINE" not in env


def test_uv_environment_offline_sets_uv_offline(venv: Target) -> None:
    assert uv_environment(venv, offline=True)["UV_OFFLINE"] == "1"


def test_system_python_is_refused(tmp_path: Path) -> None:
    system = Target(python=tmp_path / "bin/python", prefix=tmp_path, base_prefix=tmp_path)

    with pytest.raises(PrepareError, match="not a virtualenv"):
        refuse_system_python(system)


def test_active_run_in_this_environment_is_refused(venv: Target, registry_dir: Path) -> None:
    _entry(str(venv.prefix)).save()

    with pytest.raises(PrepareError, match="stop it first"):
        refuse_active_runs(venv)


def test_active_run_with_unknown_environment_is_refused(venv: Target, registry_dir: Path) -> None:
    _entry("").save()

    with pytest.raises(PrepareError, match="stop it first"):
        refuse_active_runs(venv)


def test_active_run_elsewhere_is_ignored(venv: Target, registry_dir: Path) -> None:
    _entry("/somewhere/else").save()

    refuse_active_runs(venv)


def test_find_uv_rejects_old_versions(mocker: MockerFixture) -> None:
    mocker.patch("dimos.deps.install.shutil.which", return_value="/usr/bin/uv")
    mocker.patch(
        "dimos.deps.install.subprocess.run",
        return_value=subprocess.CompletedProcess([], 0, stdout="uv 0.9.24 (x)\n", stderr=""),
    )

    with pytest.raises(PrepareError, match="too old"):
        find_uv()


def test_find_uv_falls_back_to_the_python_module(mocker: MockerFixture) -> None:
    mocker.patch("dimos.deps.install.shutil.which", return_value=None)
    run = mocker.patch(
        "dimos.deps.install.subprocess.run",
        return_value=subprocess.CompletedProcess([], 0, stdout="uv 0.9.25\n", stderr=""),
    )

    assert find_uv() == [sys.executable, "-m", "uv"]
    run.assert_called_once_with(
        [sys.executable, "-m", "uv", "--version"], capture_output=True, text=True, check=True
    )


def test_find_uv_missing_gives_the_install_hint(mocker: MockerFixture) -> None:
    mocker.patch("dimos.deps.install.shutil.which", return_value=None)
    mocker.patch("dimos.deps.install.subprocess.run", side_effect=FileNotFoundError)

    with pytest.raises(PrepareError, match="astral.sh/uv/install.sh"):
        find_uv()


def test_parse_uv_version_rejects_garbage() -> None:
    with pytest.raises(PrepareError, match="could not read"):
        parse_uv_version("something else entirely")


def test_cpu_backend_into_a_cuda_environment_is_refused(
    venv: Target, mocker: MockerFixture
) -> None:
    mocker.patch("dimos.deps.install.installed_version", return_value="1.24.1")

    with pytest.raises(PrepareError, match="prepared with --backend cuda"):
        refuse_backend_switch(venv, "cpu")


def test_verify_providers_reports_the_failing_check(venv: Target, mocker: MockerFixture) -> None:
    mocker.patch(
        "dimos.deps.install.subprocess.run",
        return_value=subprocess.CompletedProcess(
            [],
            1,
            stdout="",
            stderr="cv2 has no 'legacy' module: opencv-contrib-python was overwritten",
        ),
    )

    with pytest.raises(PrepareError, match="opencv-contrib-python was overwritten"):
        verify_providers(venv, "cpu")


@pytest.fixture
def prepared(venv: Target, mocker: MockerFixture) -> list[list[str]]:
    """Patch every boundary of prepare(); return the recorded subprocess commands."""
    commands: list[list[str]] = []

    def fake_run(command: list[str], **kwargs: object) -> subprocess.CompletedProcess[str]:
        commands.append(list(command))
        return subprocess.CompletedProcess(command, 0, stdout="verified\n", stderr="")

    mocker.patch.object(Target, "current", return_value=venv)
    mocker.patch("dimos.deps.install.refuse_active_runs")
    mocker.patch("dimos.deps.install.find_uv", return_value=["uv"])
    mocker.patch("dimos.deps.install.installed_version", return_value=None)
    mocker.patch("dimos.deps.install.subprocess.run", side_effect=fake_run)
    return commands


def test_checkout_prepare_syncs_then_relayers_gpu_onnxruntime_then_verifies(
    prepared: list[list[str]], venv: Target, tmp_path: Path, mocker: MockerFixture
) -> None:
    mocker.patch("dimos.deps.install.checkout_root", return_value=tmp_path)

    prepare(["runtime-unitree"], "cuda", offline=False)

    sync, repair, verify = prepared
    assert sync[:3] == ["uv", "sync", "--locked"]
    assert "--reinstall-package" not in sync
    assert repair == [*sync, "--reinstall-package", "onnxruntime-gpu"]
    assert verify[0] == str(venv.python) and verify[-1] == "CUDAExecutionProvider"


def test_release_prepare_installs_each_bundle_lock_in_order(
    prepared: list[list[str]], venv: Target, tmp_path: Path, mocker: MockerFixture
) -> None:
    mocker.patch("dimos.deps.install.checkout_root", return_value=None)
    locks = tmp_path / "locks"
    locks.mkdir()
    for bundle in ("runtime-common", "runtime-drone"):
        (locks / f"pylock.{bundle}-cpu.toml").write_text("")
    mocker.patch(
        "dimos.deps.install.lock_path",
        side_effect=lambda bundle, backend: locks / f"pylock.{bundle}-{backend}.toml",
    )

    prepare(["runtime-common", "runtime-drone"], "cpu", offline=True)

    first, second, verify = prepared
    assert first[-3:] == ["-r", str(locks / "pylock.runtime-common-cpu.toml"), "--offline"]
    assert second[-3:] == ["-r", str(locks / "pylock.runtime-drone-cpu.toml"), "--offline"]
    assert verify[-1] == "CPUExecutionProvider"


def test_release_prepare_without_lock_artifacts_fails_before_installing(
    prepared: list[list[str]], tmp_path: Path, mocker: MockerFixture
) -> None:
    mocker.patch("dimos.deps.install.checkout_root", return_value=None)
    mocker.patch(
        "dimos.deps.install.lock_path", side_effect=lambda bundle, backend: tmp_path / "absent.toml"
    )

    with pytest.raises(PrepareError, match="lacks its lock artifacts"):
        prepare(["runtime-common"], "cpu", offline=False)
    assert prepared == []


def test_installer_failure_is_propagated(
    venv: Target, tmp_path: Path, mocker: MockerFixture
) -> None:
    mocker.patch.object(Target, "current", return_value=venv)
    mocker.patch("dimos.deps.install.refuse_active_runs")
    mocker.patch("dimos.deps.install.find_uv", return_value=["uv"])
    mocker.patch("dimos.deps.install.installed_version", return_value=None)
    mocker.patch("dimos.deps.install.checkout_root", return_value=tmp_path)
    mocker.patch(
        "dimos.deps.install.subprocess.run",
        return_value=subprocess.CompletedProcess([], 3, stdout="", stderr=""),
    )

    with pytest.raises(PrepareError, match="exited with status 3"):
        prepare(["runtime-common"], "cpu", offline=False)


def test_checkout_root_is_this_checkout() -> None:
    root = install.checkout_root()

    assert root is not None
    assert (root / "uv.lock").is_file()
