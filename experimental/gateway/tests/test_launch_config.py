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

from pydantic import BaseModel, Field
import pytest

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.module import Module, ModuleConfig
from dimos.visualization.rerun.bridge import RerunBridgeModule
from experimental.gateway import launches, shared_config
from experimental.gateway.introspect import parser_schema, target_paths
from experimental.gateway.launches import LaunchConfig


class AuthConfig(BaseModel):
    token: str = ""
    region: str = ""
    identity: str = ""


class ExposureConfig(BaseModel):
    gain: float = 1.0


class ProfileConfig(BaseModel):
    exposure: ExposureConfig = Field(default_factory=ExposureConfig)


class CameraConfig(ModuleConfig):
    auth: AuthConfig = Field(default_factory=AuthConfig)
    profile: ProfileConfig = Field(default_factory=ProfileConfig)
    credentials: dict[str, str] = Field(default_factory=dict)
    accounts: list[dict[str, str]] = Field(default_factory=list)


class Camera(Module):
    config: CameraConfig


@pytest.fixture
def camera_parser() -> tuple[BlueprintConfigParser, dict[str, list[str]]]:
    blueprint = Camera.blueprint().namespace("cam")
    return BlueprintConfigParser(blueprint), target_paths(parser_schema(blueprint))


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
        {"robot_ip": "1.2.3.4"},
        {"cam/x": {"api_key": "k", "token": "t", "auth.token": "n", "fps": 5}},
    )
    targets = {"cam/x": ["api_key", "token", "auth.token", "fps"]}
    args = launches.run_args("demo", launch, targets)
    assert args == ["--robot-ip=1.2.3.4", "run", "demo", "--cam-x.fps=5"]
    env = launches.secret_env(launch.global_, launch.modules, launch.secret_paths(), targets)
    assert env == {"CAM_X__API_KEY": "k", "CAM_X__TOKEN": "t", "CAM_X__AUTH__TOKEN": "n"}


@pytest.mark.parametrize(
    "fields",
    [
        {
            "auth.token": "nested-secret",
            "auth.region": "west",
            "auth.identity": "explicit-secret",
            "profile.exposure.gain": 3.0,
        },
        {
            "auth": {
                "token": "nested-secret",
                "region": "west",
                "identity": "explicit-secret",
            },
            "profile": {"exposure": {"gain": 3.0}},
        },
        {
            "auth": {"token": "nested-secret"},
            "auth.region": "west",
            "auth.identity": "explicit-secret",
            "profile.exposure": {"gain": 3.0},
        },
    ],
    ids=["dotted", "objects", "mixed"],
)
def test_nested_model_values_round_trip_with_secret_and_public_siblings(
    camera_parser: tuple[BlueprintConfigParser, dict[str, list[str]]], fields: dict[str, Any]
) -> None:
    parser, targets = camera_parser
    launch = LaunchConfig(modules={"cam/camera": fields}, secrets=["cam/camera.auth.identity"])

    args = launches.run_args("demo", launch, targets)
    env = launches.secret_env(launch.global_, launch.modules, launch.secret_paths(), targets)
    parsed = parser.parse(args[2:], environ=env)

    assert args == [
        "run",
        "demo",
        "--cam-camera.auth.region=west",
        "--cam-camera.profile.exposure.gain=3.0",
    ]
    assert env == {
        "CAM_CAMERA__AUTH__IDENTITY": "explicit-secret",
        "CAM_CAMERA__AUTH__TOKEN": "nested-secret",
    }
    assert parsed.module_kwargs("cam/camera") == {
        "auth": {"token": "nested-secret", "region": "west", "identity": "explicit-secret"},
        "profile": {"exposure": {"gain": 3.0}},
    }
    assert parsed.file_and_cli == {
        "cam/camera": {"auth": {"region": "west"}, "profile": {"exposure": {"gain": "3.0"}}}
    }


@pytest.mark.parametrize(
    "fields",
    [
        {"credentials.api_key": "dictionary-secret", "credentials.region": "west"},
        {"credentials": {"api_key": "dictionary-secret", "region": "west"}},
    ],
    ids=["dotted", "object"],
)
def test_dictionary_secret_and_public_sibling_survive_parser_round_trip(
    camera_parser: tuple[BlueprintConfigParser, dict[str, list[str]]], fields: dict[str, Any]
) -> None:
    parser, targets = camera_parser
    launch = LaunchConfig(modules={"cam/camera": fields})

    args = launches.run_args("demo", launch, targets)
    env = launches.secret_env(launch.global_, launch.modules, launch.secret_paths(), targets)
    parsed = parser.parse(args[2:], environ=env)

    assert args == ["run", "demo"]
    assert set(env) == {"CAM_CAMERA__CREDENTIALS"}
    assert parsed.module_kwargs("cam/camera")["credentials"] == {
        "api_key": "dictionary-secret",
        "region": "west",
    }


@pytest.mark.parametrize(
    "fields",
    [
        {"credentials.ServiceA": "explicit-secret", "credentials.region": "west"},
        {"credentials": {"ServiceA": "explicit-secret", "region": "west"}},
    ],
    ids=["dotted", "object"],
)
def test_explicit_dictionary_secret_preserves_key_case(
    camera_parser: tuple[BlueprintConfigParser, dict[str, list[str]]], fields: dict[str, Any]
) -> None:
    parser, targets = camera_parser
    launch = LaunchConfig(
        modules={"cam/camera": fields}, secrets=["cam/camera.credentials.ServiceA"]
    )

    args = launches.run_args("demo", launch, targets)
    env = launches.secret_env(launch.global_, launch.modules, launch.secret_paths(), targets)
    parsed = parser.parse(args[2:], environ=env)

    assert args == ["run", "demo"]
    assert parsed.module_kwargs("cam/camera")["credentials"] == {
        "ServiceA": "explicit-secret",
        "region": "west",
    }


def test_explicit_descendant_secret_protects_a_json_encoded_collection(
    camera_parser: tuple[BlueprintConfigParser, dict[str, list[str]]],
) -> None:
    parser, targets = camera_parser
    credentials = '{"ServiceA":"explicit-secret","region":"west"}'
    launch = LaunchConfig(
        modules={"cam/camera": {"credentials": credentials}},
        secrets=["cam/camera.credentials.ServiceA"],
    )

    args = launches.run_args("demo", launch, targets)
    env = launches.secret_env(launch.global_, launch.modules, launch.secret_paths(), targets)
    parsed = parser.parse(args[2:], environ=env)

    assert args == ["run", "demo"]
    assert env == {"CAM_CAMERA__CREDENTIALS": credentials}
    assert parsed.module_kwargs("cam/camera")["credentials"] == {
        "ServiceA": "explicit-secret",
        "region": "west",
    }
    assert launches.redact(launch.global_, launch.modules, launch.secret_paths()) == (
        {},
        {"cam/camera": {"credentials": launches.HIDDEN}},
    )


def test_module_and_model_field_aliases_round_trip_with_secret_values(
    camera_parser: tuple[BlueprintConfigParser, dict[str, list[str]]],
) -> None:
    parser, targets = camera_parser
    launch = LaunchConfig(
        modules={
            "CAM/CAMERA": {
                "AUTH": {
                    "TOKEN": "automatic-secret",
                    "REGION": "west",
                    "IDENTITY": "explicit-secret",
                },
                "profile.exposure.GAIN": 4.0,
                "FRAME-ID": "camera-frame",
            }
        },
        secrets=["cam/camera.auth.identity"],
    )

    args = launches.run_args("demo", launch, targets)
    env = launches.secret_env(launch.global_, launch.modules, launch.secret_paths(targets), targets)
    parsed = parser.parse(args[2:], environ=env)

    assert env == {
        "CAM_CAMERA__AUTH__TOKEN": "automatic-secret",
        "CAM_CAMERA__AUTH__IDENTITY": "explicit-secret",
    }
    assert "automatic-secret" not in " ".join(args)
    assert "explicit-secret" not in " ".join(args)
    assert parsed.module_kwargs("cam/camera") == {
        "frame_id": "camera-frame",
        "auth": {"token": "automatic-secret", "region": "west", "identity": "explicit-secret"},
        "profile": {"exposure": {"gain": 4.0}},
    }
    assert launches.redact(launch.global_, launch.modules, launch.secret_paths(targets)) == (
        {},
        {
            "CAM/CAMERA": {
                "AUTH": {
                    "TOKEN": launches.HIDDEN,
                    "REGION": "west",
                    "IDENTITY": launches.HIDDEN,
                },
                "profile.exposure.GAIN": 4.0,
                "FRAME-ID": "camera-frame",
            }
        },
    )


def test_explicit_secret_aliases_preserve_case_sensitive_dictionary_keys(
    camera_parser: tuple[BlueprintConfigParser, dict[str, list[str]]],
) -> None:
    parser, targets = camera_parser
    credentials = {
        "ServiceA": "explicit-secret",
        "servicea": "public-lowercase",
        "Service-A": "public-hyphen",
    }
    launch = LaunchConfig(
        modules={"CAM/CAMERA": {"CREDENTIALS": credentials}},
        secrets=["cam/camera.credentials.ServiceA"],
    )

    paths = launch.secret_paths(targets)
    args = launches.run_args("demo", launch, targets)
    env = launches.secret_env(launch.global_, launch.modules, paths, targets)
    parsed = parser.parse(args[2:], environ=env)

    assert args == ["run", "demo"]
    assert parsed.module_kwargs("cam/camera")["credentials"] == credentials
    assert launches.redact(launch.global_, launch.modules, paths) == (
        {},
        {
            "CAM/CAMERA": {
                "CREDENTIALS": {
                    "ServiceA": launches.HIDDEN,
                    "servicea": "public-lowercase",
                    "Service-A": "public-hyphen",
                }
            }
        },
    )


def test_conflicting_aliases_of_one_model_field_are_rejected(
    camera_parser: tuple[BlueprintConfigParser, dict[str, list[str]]],
) -> None:
    _, targets = camera_parser
    launch = LaunchConfig(modules={"cam/camera": {"frame_id": "one", "FRAME-ID": "two"}})

    with pytest.raises(launches.LaunchConfigError, match="conflicting"):
        launches.run_args("demo", launch, targets)


def test_atomic_collections_keep_arbitrary_keys_and_list_secret_values(
    camera_parser: tuple[BlueprintConfigParser, dict[str, list[str]]],
) -> None:
    parser, targets = camera_parser
    credentials = {"Service-A": "hyphen-secret", "service.with.dots": "dot-secret"}
    accounts = [{"api_key": "list-secret", "region": "west"}, {"region": "east"}]
    launch = LaunchConfig(
        modules={"cam/camera": {"credentials": credentials, "accounts": accounts}},
        secrets=["cam/camera.credentials.Service-A", "cam/camera.credentials.service.with.dots"],
    )

    args = launches.run_args("demo", launch, targets)
    env = launches.secret_env(launch.global_, launch.modules, launch.secret_paths(), targets)
    parsed = parser.parse(args[2:], environ=env)

    assert args == ["run", "demo"]
    assert set(env) == {"CAM_CAMERA__CREDENTIALS", "CAM_CAMERA__ACCOUNTS"}
    assert parsed.module_kwargs("cam/camera") == {
        "credentials": credentials,
        "accounts": accounts,
    }


def test_redaction_preserves_nested_shapes_and_public_values() -> None:
    launch = LaunchConfig(
        global_={"options": {"token": "global-secret", "label": "public"}},
        modules={
            "cam/camera": {
                "auth.token": "dotted-secret",
                "credentials": {"ServiceA": "explicit-secret", "region": "west"},
                "accounts": [{"password": "list-secret", "region": "east"}],
            }
        },
        secrets=["cam/camera.credentials.ServiceA"],
    )

    global_, modules = launches.redact(launch.global_, launch.modules, launch.secret_paths())

    assert global_ == {"options": {"token": launches.HIDDEN, "label": "public"}}
    assert modules == {
        "cam/camera": {
            "auth.token": launches.HIDDEN,
            "credentials": {"ServiceA": launches.HIDDEN, "region": "west"},
            "accounts": [{"password": launches.HIDDEN, "region": "east"}],
        }
    }
    assert launch.modules["cam/camera"]["accounts"][0]["password"] == "list-secret"


def test_explicit_secret_inside_a_list_preserves_and_redacts_the_collection(
    camera_parser: tuple[BlueprintConfigParser, dict[str, list[str]]],
) -> None:
    parser, targets = camera_parser
    accounts = [{"identity": "explicit-secret", "region": "west"}, {"region": "east"}]
    launch = LaunchConfig(
        modules={"cam/camera": {"accounts": accounts}},
        secrets=["cam/camera.accounts.0.identity"],
    )

    args = launches.run_args("demo", launch, targets)
    env = launches.secret_env(launch.global_, launch.modules, launch.secret_paths(), targets)
    parsed = parser.parse(args[2:], environ=env)

    assert args == ["run", "demo"]
    assert parsed.module_kwargs("cam/camera")["accounts"] == accounts
    assert launches.redact(launch.global_, launch.modules, launch.secret_paths()) == (
        {},
        {
            "cam/camera": {
                "accounts": [
                    {"identity": launches.HIDDEN, "region": "west"},
                    {"region": "east"},
                ]
            }
        },
    )


def test_explicit_parent_secret_redacts_dotted_and_object_children() -> None:
    launch = LaunchConfig(
        modules={"cam": {"auth.region": "west", "profile": {"exposure": {"gain": 3}}}},
        secrets=["cam.auth", "cam.profile"],
    )

    assert launches.redact(launch.global_, launch.modules, launch.secret_paths()) == (
        {},
        {"cam": {"auth.region": launches.HIDDEN, "profile": launches.HIDDEN}},
    )


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


@pytest.mark.parametrize("connect", ["tcp/10.21.31.106:7447", None])
def test_zenoh_connect_follows_the_desktop_runtime_setting(home: Path, connect: str | None) -> None:
    if connect is not None:
        (home / "dimos_home").mkdir()
        (home / "dimos_home" / "config.yaml").write_text(f"zenoh:\n  connect: {connect}\n")
        shared_config.shared_config.cache_clear()
    defaults = launches.launch_env_defaults()
    assert defaults.get("ZENOH_CONNECT") == connect
    parsed = BlueprintConfigParser(RerunBridgeModule.blueprint()).parse(environ=defaults)
    assert parsed.global_config.get("zenoh_connect", "") == (connect or "")
