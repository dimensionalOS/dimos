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

"""Exercise the installer's extras selection without installing packages or system tools."""

import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

INSTALL_SCRIPT = Path(__file__).with_name("install.sh")
INSTALL_SELECTION = """\
source "$1"
INSTALL_DIR="$2"
INSTALL_TEST_PYTHON="$3"
shift 3
INSTALL_MODE=library
NON_INTERACTIVE=1
DETECTED_OS=ubuntu
DETECTED_ARCH="${INSTALL_TEST_ARCH:-x86_64}"
DETECTED_GPU=none
project_cmd() {
    "$INSTALL_TEST_PYTHON" -c 'import json, sys; print(json.dumps(sys.argv[1:]))' "$@" \
        >> "$INSTALL_DIR/commands.jsonl"
}
prompt_multi() {
    case "$1" in
        *platforms*) PROMPT_RESULT="${INSTALL_TEST_PLATFORMS:-}";;
        *) PROMPT_RESULT="${INSTALL_TEST_FEATURES:-}";;
    esac
}
parse_args "$@"
prompt_extras
resolve_extras
install_library_dependencies
"""


@pytest.fixture
def installer(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    for key in os.environ:
        if key.startswith(("DIMOS_", "INSTALL_TEST_")):
            monkeypatch.delenv(key)
    # Only the capability probe runs this stub. All installation commands are recorded.
    dimos = tmp_path / ".venv/bin/dimos"
    dimos.parent.mkdir(parents=True)
    dimos.write_text('#!/bin/sh\nexit "${INSTALL_TEST_PREPARE_STATUS:-0}"\n')
    dimos.chmod(0o755)
    return tmp_path


def _install(installer: Path, *args: str, **settings: str) -> list[list[str]]:
    commands = installer / "commands.jsonl"
    commands.write_text("")
    result = subprocess.run(
        [
            "bash",
            "-c",
            INSTALL_SELECTION,
            "installer-test",
            str(INSTALL_SCRIPT),
            str(installer),
            sys.executable,
            *args,
        ],
        cwd=installer,
        env={**os.environ, **settings},
        stdin=subprocess.DEVNULL,
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    return [json.loads(line) for line in commands.read_text().splitlines()]


@pytest.mark.parametrize(
    ("extras", "names"),
    [
        ("unitree", ["unitree-go2"]),
        ("drone", ["drone-basic"]),
        ("manipulation", ["xarm7-planner-coordinator"]),
        ("sim,control,unitree", ["unitree-go2"]),
        ("planning,learning", ["xarm7-planner-coordinator"]),
        ("unitree-dds", ["unitree-g1-teleop"]),
        ("spot", ["spot"]),
        ("runtime-common", ["replay"]),
        ("runtime-unitree", ["unitree-go2"]),
        ("runtime-manipulation", ["xarm7-planner-coordinator"]),
        ("runtime-unitree-dds", ["unitree-g1-teleop"]),
        ("runtime-drone", ["drone-basic"]),
        ("runtime-spot", ["spot"]),
        ("unitree,drone,unitree", ["unitree-go2", "drone-basic"]),
        ("agents,perception,web", ["replay"]),
    ],
)
def test_explicit_extras_prepare_the_selected_stacks(
    installer: Path, extras: str, names: list[str]
) -> None:
    commands = _install(installer, "--extras", extras)

    assert commands == [
        ["uv", "pip", "install", "--python", ".venv/bin/python", "dimos"],
        [".venv/bin/dimos", "prepare", *names, "--backend", "cpu"],
    ]


def test_environment_extras_and_prompt_choices_select_the_same_stacks(installer: Path) -> None:
    environment = _install(installer, DIMOS_EXTRAS="drone,unitree")
    prompted = _install(
        installer,
        INSTALL_TEST_PLATFORMS="Drone (Mavlink / DJI)\nUnitree (Go2, G1, B1)",
    )

    assert prompted == environment
    assert environment[-1] == [
        ".venv/bin/dimos",
        "prepare",
        "drone-basic",
        "unitree-go2",
        "--backend",
        "cpu",
    ]


def test_unbundled_extras_are_installed_before_preparing_locked_dependencies(
    installer: Path,
) -> None:
    commands = _install(installer, "--extras", "unitree,scene,dds,cuda")

    assert commands[1:] == [
        ["uv", "pip", "install", "--python", ".venv/bin/python", "dimos[scene,dds]"],
        [".venv/bin/dimos", "prepare", "unitree-go2", "--backend", "cuda"],
    ]


def test_all_installs_each_stack_and_scene(installer: Path) -> None:
    commands = _install(installer, "--extras", "all")

    assert commands[1:] == [
        ["uv", "pip", "install", "--python", ".venv/bin/python", "dimos[scene]"],
        [
            ".venv/bin/dimos",
            "prepare",
            "xarm7-planner-coordinator",
            "unitree-go2",
            "replay",
            "drone-basic",
            "spot",
            "--backend",
            "cpu",
        ],
    ]


def test_all_on_arm_omits_scene_and_keeps_the_selected_stacks(installer: Path) -> None:
    commands = _install(installer, "--extras", "all", INSTALL_TEST_ARCH="aarch64")

    assert len(commands) == 2
    assert commands[-1] == [
        ".venv/bin/dimos",
        "prepare",
        "xarm7-planner-coordinator",
        "unitree-go2",
        "replay",
        "drone-basic",
        "spot",
        "--backend",
        "cpu",
    ]


def test_no_cuda_applies_before_preparation(installer: Path) -> None:
    commands = _install(installer, "--extras", "unitree,cuda", "--no-cuda")

    assert commands[-1] == [".venv/bin/dimos", "prepare", "unitree-go2", "--backend", "cpu"]


def test_releases_without_prepare_install_the_requested_extras(installer: Path) -> None:
    commands = _install(installer, "--extras", "unitree", INSTALL_TEST_PREPARE_STATUS="1")

    assert commands[-1] == [
        "uv",
        "pip",
        "install",
        "--python",
        ".venv/bin/python",
        "--torch-backend",
        "cpu",
        "dimos[unitree,cpu]",
    ]


def test_all_on_releases_without_prepare_uses_existing_feature_extras(installer: Path) -> None:
    commands = _install(installer, "--extras", "all", INSTALL_TEST_PREPARE_STATUS="1")

    assert commands[-1] == [
        "uv",
        "pip",
        "install",
        "--python",
        ".venv/bin/python",
        "--torch-backend",
        "cpu",
        "dimos[manipulation,unitree,learning,mapping,misc,webrtc,apriltag,drone,spot,scene,cpu]",
    ]
