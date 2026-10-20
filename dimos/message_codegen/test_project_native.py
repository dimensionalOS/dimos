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

"""Compile the exact external examples with normal installed CMake/Cargo hooks."""

import json
import os
from pathlib import Path
import shutil
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[2]
pytestmark = pytest.mark.native_e2e


def run(command, project, env, *, succeeds=True):
    result = subprocess.run(
        [str(value) for value in command],
        cwd=project,
        env=env,
        capture_output=True,
        text=True,
        check=False,
        timeout=600,
    )
    if succeeds:
        assert result.returncode == 0, result.stdout + result.stderr
    else:
        assert result.returncode != 0, result.stdout + result.stderr
    return result


@pytest.fixture(scope="module")
def external_project(tmp_path_factory):
    executable = os.environ.get("DIMOS_EXTERNAL_NATIVE_PYTHON")
    if not executable:
        pytest.skip(
            "Set DIMOS_EXTERNAL_NATIVE_PYTHON to an installed review wheel with message-build tools"
        )
    project = tmp_path_factory.mktemp("native-message-project") / "project"
    shutil.copytree(ROOT / "examples/external-messages", project)
    env = dict(os.environ)
    env.pop("PYTHONPATH", None)
    env["PATH"] = str(Path(executable).parent) + os.pathsep + env["PATH"]
    env["DIMOS_MESSAGE_PYTHON"] = executable
    env["CARGO_TARGET_DIR"] = os.environ.get("DIMOS_EXTERNAL_CARGO_TARGET", str(project / "target"))
    if "DIMOS_MESSAGE_NATIVE_CACHE" not in env:
        env["DIMOS_MESSAGE_NATIVE_CACHE"] = str(project / "build/native-cache")
    # The interpreter path must retain its venv symlink rather than resolve to
    # the base interpreter, so all consumers use the installed review package.
    python = Path(executable)
    run([python, "-m", "pip", "install", "--no-deps", "--no-build-isolation", "."], project, env)
    prefix = run(
        [python, "-m", "dimos.message_codegen.native", "cmake-prefix"], project, env
    ).stdout.strip()
    configure = [
        "cmake",
        "-S",
        ".",
        "-B",
        "build/cpp",
        "-DCMAKE_PREFIX_PATH=" + prefix,
        "-DPython3_EXECUTABLE=" + str(python),
    ]
    build_cpp = ["cmake", "--build", "build/cpp", "--parallel", "2"]
    cargo = ["cargo", "build", *(["--offline"] if env.get("DIMOS_OFFLINE") == "1" else [])]
    return project, python, env, configure, build_cpp, cargo


def test_normal_builds_share_header_and_exchange_cdr(external_project):
    project, python, env, configure, build_cpp, cargo = external_project
    run([python, "-I", "demo_reading.py", "write", "python.cdr"], project, env)
    run(configure, project, env)
    run(build_cpp, project, env)
    run(["build/cpp/telemetry_relay", "python.cdr", "cpp.cdr"], project, env)
    run([python, "-m", "dimos.message_codegen.native", "configure-cargo"], project, env)
    run([*cargo, "--example", "relay"], project, env)
    run(
        [Path(env["CARGO_TARGET_DIR"]) / "debug/examples/relay", "cpp.cdr", "rust.cdr"],
        project,
        env,
    )
    result = run([python, "-I", "demo_reading.py", "verify", "rust.cdr"], project, env)
    assert "one shared Header identity" in result.stdout


def test_normal_native_builds_reconcile_edits_renames_and_deletions(external_project):
    project, python, env, configure, build_cpp, cargo = external_project
    source = project / "interfaces/telemetry_msgs/msg/DeviceReading.msg"
    source.write_text(source.read_text() + "uint32 status\n")
    transient = source.with_name("Transient.msg")
    transient.write_text("uint32 value\n")
    cpp = project / "cpp/relay.cpp"
    cpp.write_text(
        cpp.read_text().replace("value.value += 1.0;", "value.status = 9; value.value += 1.0;")
    )
    run(configure, project, env)
    run(build_cpp, project, env)
    run([python, "-m", "dimos.message_codegen.native", "configure-cargo"], project, env)
    run([*cargo, "--lib"], project, env)
    renamed = source.with_name("HTTPReading.msg")
    source.rename(renamed)
    cpp.write_text(
        cpp.read_text()
        .replace("device_reading.hpp", "http_reading.hpp")
        .replace("DeviceReading", "HTTPReading")
    )
    run(configure, project, env)
    run(build_cpp, project, env)
    run([*cargo, "--lib"], project, env)
    transient.unlink()
    run(configure, project, env)
    run(build_cpp, project, env)
    run([*cargo, "--lib"], project, env)
    plan = json.loads((project / "build/cpp/dimos-messages.json").read_text())
    prefix = Path(plan["prefix"])
    assert list(prefix.rglob("http_reading.hpp"))
    assert not list(prefix.rglob("device_reading.hpp"))
    assert not list(prefix.rglob("transient.hpp"))
    roots = list(
        Path(env["CARGO_TARGET_DIR"]).glob(
            "debug/build/acme-telemetry-messages-*/out/dimos-message-bindings/telemetry_msgs/msg"
        )
    )
    roots = [
        directory
        for directory in roots
        if str(project / "pyproject.toml")
        in json.loads((directory.parents[2] / "dimos-messages.json").read_text())["inputs"]
    ]
    assert roots
    for directory in roots:
        assert not (directory / "device_reading.rs").exists()
        assert not (directory / "transient.rs").exists()
        assert (directory / "http_reading.rs").is_file()
    manifest = json.loads((roots[0].parents[2] / "dimos-messages.json").read_text())
    state = Path(manifest["generation"])
    previous = state.read_bytes()
    renamed.unlink()
    failed = run([*cargo, "--lib"], project, env, succeeds=False)
    assert "No messages found" in failed.stderr
    assert state.read_bytes() == previous


def test_installed_sdk_modules_exchange_cdr_through_actual_coordinator(tmp_path):
    executable = os.environ.get("DIMOS_EXTERNAL_NATIVE_PYTHON")
    dependencies = os.environ.get("DIMOS_CDR_NATIVE_SDK_PREFIX")
    if not executable or not dependencies:
        pytest.skip("Supply an installed review wheel and native SDK dependency prefixes")
    project = tmp_path / "project"
    shutil.copytree(ROOT / "examples/external-messages", project)
    python = Path(executable)
    env = dict(os.environ)
    env.pop("PYTHONPATH", None)
    env["PATH"] = str(python.parent) + os.pathsep + env["PATH"]
    env["DIMOS_MESSAGE_PYTHON"] = executable
    env["CARGO_TARGET_DIR"] = env.get("DIMOS_EXTERNAL_CARGO_TARGET", str(project / "target"))
    run([python, "-m", "pip", "install", "--no-deps", "--no-build-isolation", "."], project, env)
    prefix = run(
        [python, "-m", "dimos.message_codegen.native", "cmake-prefix"], project, env
    ).stdout.strip()
    run([python, "-m", "dimos.message_codegen.native", "configure-cargo"], project, env)
    configure = [
        "cmake",
        "-S",
        ".",
        "-B",
        "build/cpp",
        "-DDIMOS_BUILD_MODULE_EXAMPLE=ON",
        "-DCMAKE_PREFIX_PATH=" + prefix + ";" + dependencies,
        "-DPKG_CONFIG_USE_CMAKE_PREFIX_PATH=FALSE",
        "-DPython3_EXECUTABLE=" + str(python),
    ]
    for variable, target in (
        ("DIMOS_CDR_NATIVE_PFR_SOURCE", "PFR"),
        ("DIMOS_CDR_NATIVE_JSON_SOURCE", "NLOHMANN_JSON"),
    ):
        if variable in env:
            configure.append("-DFETCHCONTENT_SOURCE_DIR_" + target + "=" + env[variable])
    run(configure, project, env)
    run(["cmake", "--build", "build/cpp", "--parallel", "2"], project, env)
    run(
        [
            "cargo",
            "build",
            "--locked",
            "--manifest-path",
            "modules/Cargo.toml",
            "--jobs",
            "2",
            *(["--offline"] if env.get("DIMOS_OFFLINE") == "1" else []),
        ],
        project,
        env,
    )
    result = run(
        [
            python,
            "demo_modules.py",
            "--cpp",
            "build/cpp/telemetry_module",
            "--rust",
            Path(env["CARGO_TARGET_DIR"]) / "debug/telemetry_module",
        ],
        project,
        env,
    )
    assert (
        "PASS: coordinator Python -> C++ -> Rust -> Python over loopback Zenoh CDR" in result.stdout
    )
