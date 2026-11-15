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
import subprocess
import sys
from typing import Any

import pytest

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.visualization.rerun.bridge import RerunBridgeModule
from experimental.gateway import launches, shared_config
from experimental.gateway.launches import LaunchConfig


def test_parse_takes_both_forms_and_refuses_a_mix() -> None:
    assert launches.parse({"robot_ip": "x"}).global_ == {"robot_ip": "x"}
    structured = launches.parse({"global": {"replay": True}, "modules": {"cam": {"fps": 5}}})
    assert (structured.global_, structured.modules) == ({"replay": True}, {"cam": {"fps": 5}})
    with pytest.raises(ValueError):
        launches.parse({"global": {}, "robot_ip": "x"})
    with pytest.raises(ValueError):
        launches.parse([1])


def test_secrets_go_in_the_environment_not_the_command_line() -> None:
    launch = LaunchConfig(
        {"typesafe_api_key": "k", "robot_ip": "1.2.3.4"}, {"cam/x": {"token": "t", "fps": 5}}
    )
    args = launches.run_args("demo", launch)
    assert args == ["--robot-ip=1.2.3.4", "run", "demo", "--cam-x.fps=5"]
    env = launches.secret_env(launch.global_, launch.modules, launch.secret_paths())
    assert env == {"TYPESAFE_API_KEY": "k", "CAM_X__TOKEN": "t"}


def test_flags_spell_values_as_dimos_reads_them() -> None:
    assert launches.global_flags(
        {"replay": True, "dtop": False, "n_workers": 2, "robot_ips": ["a"]}
    ) == [
        "--no-dtop",
        "--n-workers=2",
        "--replay",
        '--robot-ips=["a"]',
    ]
    launch = LaunchConfig({"replay": True}, {}, args=["--api-key", "x"])
    assert launches.run_args("demo", launch) == [
        *("run", "demo"),
        *("--replay=true", "--api-key", "x"),
    ]
    assert launches.shown_args(["--api-key", "x", "--token=y", "--fps=3"]) == [
        "--api-key",
        launches.HIDDEN,
        f"--token={launches.HIDDEN}",
        "--fps=3",
    ]


@pytest.mark.parametrize(
    ("values", "expected", "flags"),
    [
        (
            {"replay": "true", "n_workers": 3.0},
            {"replay": True, "n_workers": 3},
            ["--n-workers=3", "--replay"],
        ),
        (
            {"replay": "false", "n_workers": "3"},
            {"replay": False, "n_workers": 3},
            ["--n-workers=3", "--no-replay"],
        ),
        (
            {"zenoh_gossip": "no", "robot_ip": "yes", "robot_width": "0.5"},
            {"zenoh_gossip": False, "robot_ip": "yes", "robot_width": 0.5},
            ["--robot-ip=yes", "--robot-width=0.5", "--no-zenoh-gossip"],
        ),
        ({"replay": None}, {"replay": None}, []),
        ({}, {}, []),
    ],
)
def test_normalization_preserves_only_explicit_overrides(
    values: dict[str, Any],
    expected: dict[str, Any],
    flags: list[str],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("REPLAY", "true")
    monkeypatch.setenv("N_WORKERS", "9")
    original = dict(values)

    normalized = launches.normalize_global(values)

    assert normalized == expected
    assert launches.global_flags(normalized) == flags
    assert values == original


@pytest.mark.parametrize("values", [{"replay": "maybe"}, {"n_workers": 3.5}, {"robot_iq": None}])
def test_invalid_normalization_leaves_input_unchanged(values: dict[str, Any]) -> None:
    original = dict(values)
    with pytest.raises(launches.LaunchConfigError):
        launches.normalize_global(values)
    assert values == original


@pytest.mark.parametrize(("replay", "expected"), [("yes", "True"), ("no", "False")])
def test_normalized_flags_are_accepted_by_the_real_cli(
    home: Path, monkeypatch: pytest.MonkeyPatch, replay: str, expected: str
) -> None:
    monkeypatch.setenv("REPLAY", "true")
    monkeypatch.setenv("N_WORKERS", "9")
    launch = LaunchConfig({"replay": replay, "n_workers": 3.0}).normalized()

    result = subprocess.run(
        [
            sys.executable,
            "-m",
            "dimos.cli.dimos",
            *launches.global_flags(launch.global_),
            "show-config",
        ],
        cwd=home,
        capture_output=True,
        text=True,
        timeout=30,
    )

    assert result.returncode == 0, result.stdout + result.stderr
    assert f"replay: {expected}" in result.stdout.splitlines()
    assert "n_workers: 3" in result.stdout.splitlines()


def test_dotted_module_fields_become_one_object_flag() -> None:
    flags = launches.module_flags(
        {"controlcoordinator": {"addresses.xarm_arm": "192.0.2.1", "addresses.piper_arm": "can0"}}
    )
    assert flags == ['--controlcoordinator.addresses={"piper_arm":"can0","xarm_arm":"192.0.2.1"}']


def test_launch_env_defaults_are_settings_dimos_reads() -> None:
    parsed = BlueprintConfigParser(RerunBridgeModule.blueprint()).parse(
        environ=launches.launch_env_defaults()
    )
    kwargs = parsed.module_kwargs(RerunBridgeModule.name)
    assert (kwargs["rerun_open"], kwargs["rerun_web"]) == ("none", True)


@pytest.mark.parametrize("enabled", [True, False, None])
def test_topic_metadata_follows_the_desktop_runtime_setting(
    home: Path, enabled: bool | None
) -> None:
    if enabled is not None:
        (home / "dimos_home").mkdir()
        (home / "dimos_home" / "config.yaml").write_text(
            f"topics:\n  enabled: {str(enabled).lower()}\n"
        )
        shared_config.shared_config.cache_clear()
    defaults = launches.launch_env_defaults()
    assert defaults.get("TOPIC_METADATA") == ("true" if enabled is True else None)
