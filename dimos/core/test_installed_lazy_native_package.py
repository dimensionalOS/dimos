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

"""Source-only installs do no native work; selected blueprint targets prepare lazily."""

import importlib.util
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import zipfile

import pytest

from dimos.utils.testing.waiting import wait_until


@pytest.mark.native_e2e
@pytest.mark.skipif(sys.platform != "linux", reason="Rust package acceptance runs on Linux")
@pytest.mark.parametrize("installed_host", [False, True], ids=["source-install", "host-wheel"])
def test_source_package_install_list_and_selected_target(tmp_path, installed_host):
    for tool in ("uv", "cargo"):
        if shutil.which(tool) is None:
            pytest.skip(f"lazy package acceptance requires {tool}")
    for package in ("build", "setuptools", "wheel"):
        if importlib.util.find_spec(package) is None:
            pytest.skip(f"lazy package acceptance requires {package}")
    wheelhouse = os.environ.get("DIMOS_PACKAGE_WHEELHOUSE")
    if installed_host and not wheelhouse:
        pytest.skip("set DIMOS_PACKAGE_WHEELHOUSE to a host wheel with source_package support")

    root = Path(__file__).resolve().parents[2]
    source = tmp_path / "source"
    shutil.copytree(root / "examples/packages/lazy-native", source)
    guards = tmp_path / "guards"
    guards.mkdir()
    calls = tmp_path / "compiler-calls"
    for name in ("cargo", "rustc", "cmake", "cc", "c++"):
        guard = guards / name
        guard.write_text('#!/bin/sh\necho invoked >> "$COMPILER_CALLS"\nexit 97\n')
        guard.chmod(0o755)
    env = {
        **os.environ,
        "PATH": str(guards) + os.pathsep + os.environ["PATH"],
        "COMPILER_CALLS": str(calls),
        "UV_CACHE_DIR": subprocess.check_output(["uv", "cache", "dir"], text=True).strip(),
        "XDG_CACHE_HOME": str(tmp_path / "cache"),
        "XDG_STATE_HOME": str(tmp_path / "state"),
        "XDG_DATA_HOME": str(tmp_path / "data"),
        "DIMOS_PACKAGE_REPORT": str(tmp_path / "report.txt"),
    }
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
        env=env,
        check=True,
        capture_output=True,
        text=True,
        timeout=60,
    )
    (wheel,) = wheels.glob("*.whl")
    assert wheel.name.endswith("py3-none-any.whl")
    with zipfile.ZipFile(wheel) as archive:
        names = archive.namelist()
        assert "dimos_lazy_native/native/Cargo.lock" in names
        assert "dimos_lazy_native/native/src/bin/other.rs" in names
        assert not any("/target/" in name for name in names)

    if installed_host:
        host = tmp_path / "host"
        subprocess.run(
            ["uv", "venv", str(host), "--python", sys.executable], check=True, capture_output=True
        )
        python = str(host / "bin/python")
        assert wheelhouse is not None
        (host_wheel,) = Path(wheelhouse).glob("dimos-*.whl")
        subprocess.run(
            ["uv", "pip", "install", "--python", python, "--offline", str(host_wheel), str(wheel)],
            env=env,
            check=True,
            capture_output=True,
            text=True,
            timeout=120,
        )
        env.pop("PYTHONPATH", None)
        env.pop("PYTHONHOME", None)
        provenance = subprocess.check_output(
            [python, "-c", "import dimos.core.native_module as m; print(m.__file__)"],
            cwd=tmp_path,
            env=env,
            text=True,
        )
        assert str(host) in provenance
        installed = next(host.glob("lib/python*/site-packages/dimos_lazy_native"))
    else:
        python = sys.executable
        prefix = tmp_path / "installed"
        # Also test pip's local-source build path, with compilers deliberately unavailable.
        subprocess.run(
            [
                "uv",
                "pip",
                "install",
                "--python",
                python,
                "--offline",
                "--no-deps",
                "--no-build-isolation",
                "--target",
                str(prefix),
                str(source),
            ],
            env=env,
            check=True,
            capture_output=True,
            text=True,
            timeout=60,
        )
        env["PYTHONPATH"] = str(prefix)
        installed = prefix / "dimos_lazy_native"
    shutil.rmtree(source)
    original = {
        path.relative_to(installed): path.read_bytes()
        for path in installed.rglob("*")
        if path.is_file()
    }
    cli = [python, "-c", "from dimos.cli.entry import main; main()"]
    listed = subprocess.check_output([*cli, "list"], cwd=tmp_path, env=env, text=True)
    assert "dimos-lazy-native.probe" in listed
    assert "dimos-lazy-native.other" in listed
    assert not calls.exists()
    cache = tmp_path / "cache/dimos/native-packages"
    assert not cache.exists()
    env["PATH"] = os.environ["PATH"]

    report = tmp_path / "report.txt"
    artifact_times = []
    for iteration in range(2):
        report.unlink(missing_ok=True)
        log_path = tmp_path / f"run-{iteration}.log"
        with log_path.open("w") as log:
            process = subprocess.Popen(
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
                    "dimos-lazy-native.probe",
                ],
                cwd=tmp_path,
                env=env,
                stdin=subprocess.DEVNULL,
                stdout=log,
                stderr=subprocess.STDOUT,
            )
            try:
                wait_until(
                    lambda log_path=log_path, process=process: (
                        report.exists() and "serve_coordinator_rpc is off" in log_path.read_text()
                    )
                    or process.poll() is not None,
                    timeout=60,
                )
                assert report.exists(), log_path.read_text()
                assert (
                    report.read_text().strip().endswith("hello from a lazily built native package")
                )
                child_pid = int(report.read_text().split()[1])
            finally:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=10)
            assert report.read_text().splitlines()[-1] == "stopped"
            with pytest.raises(ProcessLookupError):
                os.kill(child_pid, 0)
        (artifact,) = cache.glob("*/source/target/release/package_probe")
        artifact_times.append(artifact.stat().st_mtime_ns)
        assert not list(cache.glob("*/source/target/release/package_other"))
    assert artifact_times[0] == artifact_times[1]
    assert not (installed / "native/target").exists()
    assert all((installed / path).read_bytes() == data for path, data in original.items())
    assert not (tmp_path / "data/dimos/repo").exists()
