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

"""Installation boundary test, explicitly selected with ``-m native_e2e``."""

import importlib.util
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys

import pytest

from dimos.utils.testing.waiting import wait_until


@pytest.mark.native_e2e
@pytest.mark.skipif(sys.platform != "linux", reason="POSIX native package acceptance runs on Linux")
def test_installed_native_package_is_discovered_and_runs_outside_its_sources(tmp_path):
    for command in ("cmake", "cargo", "uv"):
        if shutil.which(command) is None:
            pytest.skip(f"external package acceptance requires {command}")
    for package in ("build", "scikit_build_core"):
        if importlib.util.find_spec(package) is None:
            pytest.skip(f"external package acceptance requires {package}")

    root = Path(__file__).resolve().parents[2]
    source = tmp_path / "source"
    shutil.copytree(root / "examples/packages/native", source)
    wheels = tmp_path / "wheels"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "build",
            "--wheel",
            "--no-isolation",
            "--outdir",
            str(wheels),
            str(source),
        ],
        check=True,
        capture_output=True,
        text=True,
        timeout=120,
    )
    (wheel,) = wheels.glob("*.whl")
    assert "-py3-none-linux_" in wheel.name
    installed = tmp_path / "installed"
    subprocess.run(
        [
            "uv",
            "pip",
            "install",
            "--python",
            sys.executable,
            "--target",
            str(installed),
            "--offline",
            "--no-deps",
            str(wheel),
        ],
        check=True,
        capture_output=True,
        text=True,
        timeout=30,
    )
    shutil.rmtree(source)
    report = tmp_path / "report.txt"
    env = {
        **os.environ,
        "PYTHONPATH": str(installed),
        "DIMOS_PACKAGE_REPORT": str(report),
        "XDG_STATE_HOME": str(tmp_path / "state"),
        "XDG_CACHE_HOME": str(tmp_path / "cache"),
    }
    discovery = subprocess.run(
        [
            sys.executable,
            "-c",
            "import sys; from dimos.robot.external_blueprints import "
            "list_external_blueprint_names; assert 'dimos-external-native.probe' in "
            "list_external_blueprint_names(); assert 'dimos_external_native.module' not in sys.modules",
        ],
        cwd=tmp_path,
        env=env,
        check=True,
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert discovery.returncode == 0
    cli = [sys.executable, "-c", "from dimos.cli.entry import main; main()"]
    listed = subprocess.run(
        [*cli, "list"],
        cwd=tmp_path,
        env=env,
        check=True,
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert "dimos-external-native.probe" in listed.stdout
    with (tmp_path / "run.log").open("w+") as log:
        proc = subprocess.Popen(
            [
                *cli,
                "--transport",
                "zenoh",
                "--viewer",
                "none",
                "--n-workers",
                "1",
                "--no-serve-coordinator-rpc",
                "run",
                "dimos-external-native.probe",
            ],
            cwd=tmp_path,
            env=env,
            stdout=log,
            stderr=subprocess.STDOUT,
        )
        try:
            wait_until(
                lambda: (
                    report.exists()
                    and "serve_coordinator_rpc is off" in (tmp_path / "run.log").read_text()
                )
                or proc.poll() is not None,
                timeout=45,
            )
            log.seek(0)
            assert report.exists(), log.read()
            ready = report.read_text().splitlines()[0]
            assert ready.startswith("ready ")
            assert ready.endswith("hello from an installed native package")
            child_pid = int(ready.split()[1])
        finally:
            proc.send_signal(signal.SIGTERM)
            try:
                proc.wait(timeout=30)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=10)
        assert report.read_text().splitlines()[-1] == "stopped"
        with pytest.raises(ProcessLookupError):
            os.kill(child_pid, 0)
