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

from pathlib import Path
import sys

import pytest

from dimos.utils.mjpython import (
    allow_mjpython_to_start_after_a_crash,
    mjpython_env,
    mjpython_executable,
    prepare_mjpython_launch,
)


@pytest.fixture
def fake_venv(tmp_path, monkeypatch):
    """A venv layout whose mjpython sits next to sys.executable, as MuJoCo installs it."""
    bindir = tmp_path / "bin"
    bindir.mkdir()
    (bindir / "python").touch()
    mjpython = bindir / "mjpython"
    mjpython.touch()
    mjpython.chmod(0o755)

    monkeypatch.setattr(sys, "executable", str(bindir / "python"))
    monkeypatch.setattr(sys, "platform", "darwin")
    return mjpython


def test_resolves_mjpython_next_to_the_interpreter(fake_venv, monkeypatch):
    monkeypatch.setenv("PATH", "")
    assert mjpython_executable() == fake_venv


def test_missing_mjpython_names_the_path_it_looked_at(fake_venv):
    fake_venv.unlink()
    with pytest.raises(RuntimeError, match=str(fake_venv)):
        mjpython_executable()


def test_non_executable_mjpython_is_rejected(fake_venv):
    fake_venv.chmod(0o644)
    with pytest.raises(RuntimeError, match="missing from this environment"):
        mjpython_executable()


def test_env_puts_libpython_on_the_dylib_search_path(monkeypatch, tmp_path):
    monkeypatch.setattr(sys, "platform", "darwin")
    monkeypatch.setattr("sysconfig.get_config_var", lambda name: str(tmp_path))
    assert mjpython_env({})["DYLD_LIBRARY_PATH"] == str(tmp_path)


def test_env_prepends_rather_than_clobbering(monkeypatch, tmp_path):
    monkeypatch.setattr(sys, "platform", "darwin")
    monkeypatch.setattr("sysconfig.get_config_var", lambda name: str(tmp_path))
    env = mjpython_env({"DYLD_LIBRARY_PATH": "/already/here"})
    assert env["DYLD_LIBRARY_PATH"] == f"{tmp_path}:/already/here"


def test_env_left_alone_when_libdir_does_not_exist(monkeypatch):
    monkeypatch.setattr(sys, "platform", "darwin")
    monkeypatch.setattr("sysconfig.get_config_var", lambda name: None)
    assert "DYLD_LIBRARY_PATH" not in mjpython_env({})


def test_env_is_untouched_off_macos(monkeypatch, tmp_path):
    monkeypatch.setattr(sys, "platform", "linux")
    monkeypatch.setattr("sysconfig.get_config_var", lambda name: str(tmp_path))
    assert mjpython_env({}) == {}


def test_executable_refuses_to_resolve_off_macos(monkeypatch):
    monkeypatch.setattr(sys, "platform", "linux")
    with pytest.raises(RuntimeError, match="only used on macOS"):
        mjpython_executable()


def test_crash_prompt_is_disabled_for_mujocos_bundle_only(monkeypatch):
    monkeypatch.setattr(sys, "platform", "darwin")
    calls = []
    monkeypatch.setattr("subprocess.run", lambda cmd, **kw: calls.append(cmd))

    allow_mjpython_to_start_after_a_crash()

    assert calls == [
        ["defaults", "write", "org.mujoco.mjpython", "ApplePersistenceIgnoreState", "-bool", "true"]
    ]


def test_crash_prompt_is_left_alone_off_macos(monkeypatch):
    monkeypatch.setattr(sys, "platform", "linux")
    monkeypatch.setattr("subprocess.run", lambda cmd, **kw: pytest.fail("ran defaults off macOS"))
    allow_mjpython_to_start_after_a_crash()


def test_preparing_a_launch_disables_the_crash_prompt(fake_venv, monkeypatch):
    calls = []
    monkeypatch.setattr("subprocess.run", lambda cmd, **kw: calls.append(cmd))

    executable, env = prepare_mjpython_launch()

    assert executable == fake_venv
    assert env is not None
    assert len(calls) == 1


@pytest.mark.skipif(sys.platform != "darwin", reason="mjpython only exists on macOS")
def test_the_real_mjpython_starts_after_being_killed_uncleanly():
    """The regression this module exists for: a Ctrl-C'd sim bricks every later launch.

    Without the persistence default, AppKit blocks on an invisible "reopen windows?"
    prompt and mjpython never reaches Python at all.
    """
    import subprocess

    mjpython = Path(sys.executable).with_name("mjpython")
    if not mjpython.is_file():
        pytest.skip("mjpython is not installed in this environment")

    executable, env = prepare_mjpython_launch()
    hung = subprocess.Popen([str(executable), "-c", "import time; time.sleep(60)"], env=env)
    try:
        hung.kill()
        hung.wait(timeout=30)

        result = subprocess.run(
            [str(executable), "-c", "print('alive')"],
            env=env,
            capture_output=True,
            text=True,
            timeout=60,
        )
    finally:
        hung.kill()
    assert result.returncode == 0, result.stderr
    assert "alive" in result.stdout


@pytest.mark.skipif(sys.platform != "darwin", reason="mjpython only exists on macOS")
def test_the_real_mjpython_can_load_libpython():
    """The regression this module exists for: uv venvs cannot dlopen libpython unaided."""
    import subprocess

    mjpython = Path(sys.executable).with_name("mjpython")
    if not mjpython.is_file():
        pytest.skip("mjpython is not installed in this environment")

    allow_mjpython_to_start_after_a_crash()
    result = subprocess.run(
        [str(mjpython), "-c", "import mujoco; print(mujoco.__version__)"],
        env=mjpython_env(),
        capture_output=True,
        text=True,
        timeout=120,
    )
    assert result.returncode == 0, result.stderr
