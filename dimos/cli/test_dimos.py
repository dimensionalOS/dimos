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

from collections.abc import Iterator
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
from typing import Any

import pytest
from pytest_mock import MockerFixture
from typer.testing import CliRunner

import dimos.cli.commands.info as info
import dimos.cli.commands.lifecycle as lifecycle
from dimos.cli.commands.lifecycle import _with_relay_bridge
import dimos.cli.commands.mcp as mcp
from dimos.cli.dimos import main, normalize_argv
import dimos.cli.spy.run_spy as run_spy
from dimos.core.coordination.blueprints import Blueprint, ChoiceAtom, autoconnect
import dimos.core.coordination.module_coordinator as module_coordinator
import dimos.core.coordination.process_lifecycle as process_lifecycle
import dimos.core.daemon as core_daemon
from dimos.core.daemon import install_signal_handlers
from dimos.core.global_config import global_config
from dimos.core.module import Module, ModuleConfig
import dimos.core.run_registry as run_registry
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.robot import external_blueprints as external
import dimos.robot.get_all_blueprints as get_all_blueprints
from dimos.robot.get_all_blueprints import get_by_name_or_exit
from dimos.robot.manipulators.a1z.connection import A1ZConnection
from dimos.robot.manipulators.a750.connection import A750Connection
from dimos.robot.manipulators.piper.connection import PiperConnection
from dimos.robot.manipulators.xarm.connection import XArmConnection
from dimos.robot.unitree.g1.effectors.high_level.webrtc import G1HighLevelWebRtc
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.robot.unitree.go2.fleet_connection import Go2FleetConnection
import dimos.utils.cache as cache_utils
import dimos.utils.logging_config as logging_config


class RunConfigA(ModuleConfig):
    map_file: str | None = None
    entity_prefix: str = "world"
    daemon: str = "module-default"


class RunModuleA(Module):
    config: RunConfigA


class RunConfigB(ModuleConfig):
    map_file: str | None = None
    lookahead: float = 1.0


class RunModuleB(Module):
    config: RunConfigB


@pytest.mark.parametrize(
    ("argv", "expected"),
    [
        # Bare `--simulation` (legacy flag form) followed by the subcommand:
        # the default backend is injected so click doesn't eat `run`.
        (
            ["dimos", "--simulation", "run", "go2"],
            ["dimos", "--simulation", "mujoco", "run", "go2"],
        ),
        # Bare `--simulation` followed by another option, or nothing.
        (["dimos", "--simulation", "-d", "run"], ["dimos", "--simulation", "mujoco", "-d", "run"]),
        (["dimos", "--simulation"], ["dimos", "--simulation", "mujoco"]),
        (["dimos", "--record", "run", "go2"], ["dimos", "--record", "sqlite", "run", "go2"]),
        (["dimos", "--record", "sqlite", "run"], ["dimos", "--record", "sqlite", "run"]),
        (["dimos", "--record", "mcap", "run"], ["dimos", "--record", "mcap", "run"]),
        # Explicit simulator — left untouched.
        (["dimos", "--simulation", "mujoco", "run"], ["dimos", "--simulation", "mujoco", "run"]),
        (["dimos", "--simulation", "dimsim", "run"], ["dimos", "--simulation", "dimsim", "run"]),
        (["dimos", "--simulation=dimsim", "run"], ["dimos", "--simulation=dimsim", "run"]),
        # No `--simulation` at all — left untouched.
        (["dimos", "run", "go2"], ["dimos", "run", "go2"]),
    ],
)
def test_normalize_argv(argv: list[str], expected: list[str]) -> None:
    assert normalize_argv(argv) == expected


def test_dotenv_in_the_working_directory_is_read(tmp_path: Path) -> None:
    # The package where upward search finds this .env, like a checkout's.
    install = tmp_path / "install"
    install.mkdir()
    (install / "dimos").symlink_to(Path(__file__).resolve().parents[1])
    (install / ".env").write_text("DIMOS_TEST_BOTH=next-to-the-code\n")
    work = tmp_path / "work"
    work.mkdir()
    (work / ".env").write_text(
        "DIMOS_TEST_WORK=working-directory\n"
        "DIMOS_TEST_BOTH=working-directory\n"
        "DIMOS_TEST_EXPORTED=working-directory\n"
    )
    names = ("DIMOS_TEST_WORK", "DIMOS_TEST_BOTH", "DIMOS_TEST_EXPORTED")
    # A file, because python-dotenv searches from the working directory under `python -c`.
    script = tmp_path / "print_env.py"
    script.write_text(f"import os, dimos.cli.dimos\nprint(*map(os.getenv, {names}))\n")

    result = subprocess.run(
        [sys.executable, str(script)],
        cwd=work,
        # Built here, so a failure does not print the whole environment as a local.
        env={k: v for k, v in os.environ.items() if k != "PYTEST_VERSION"}
        | {"PYTHONPATH": str(install), "DIMOS_TEST_EXPORTED": "environment"},
        capture_output=True,
        text=True,
        check=True,
    )

    assert result.stdout.split() == ["working-directory", "next-to-the-code", "environment"]


@pytest.mark.parametrize(
    ("args", "expected"),
    [
        ([], {}),
        (
            [
                "--arg",
                "x=0.5",
                "--arg",
                "enabled=true",
                "-a",
                "n=3",
                "-a",
                "name=sample",
                "-a",
                "missing=null",
            ],
            {"x": 0.5, "enabled": True, "n": 3, "name": "sample", "missing": None},
        ),
    ],
)
def test_mcp_values_reach_adapter(
    monkeypatch: pytest.MonkeyPatch, args: list[str], expected: dict[str, Any]
) -> None:
    calls: list[tuple[str, dict[str, Any]]] = []

    class Adapter:
        def call_tool(self, name: str, arguments: dict[str, Any]) -> dict[str, Any]:
            calls.append((name, arguments))
            return {"content": [{"text": "fixture result"}]}

    monkeypatch.setattr(mcp, "_get_adapter", lambda timeout: Adapter())
    result = CliRunner().invoke(main, ["mcp", "call", "inspect", *args])
    assert result.exit_code == 0, result.output + repr(result.exception)
    assert calls == [("inspect", expected)]
    assert "fixture result" in result.output


def test_mcp_malformed_argument_rejected_before_adapter(monkeypatch: pytest.MonkeyPatch) -> None:
    def forbidden(*args: Any) -> None:
        raise AssertionError("invalid argument reached adapter")

    monkeypatch.setattr(mcp, "_get_adapter", forbidden)
    result = CliRunner().invoke(main, ["mcp", "call", "inspect", "--arg", "invalid"])
    assert result.exit_code == 2, result.output + repr(result.exception)
    assert "expected KEY=VALUE" in result.output


def test_global_help_explains_dtop() -> None:
    result = CliRunner().invoke(main, ["--help"])

    assert result.exit_code == 0, result.output
    output_plain = re.sub(r"\x1b\[[0-9;]*m", "", result.output)
    output_plain = " ".join(output_plain.replace("│", "").split())
    assert "--dtop" in output_plain
    assert "Publish live resource stats for dtop" in output_plain
    assert "Override dtop in GlobalConfig" not in output_plain


def test_global_config_flag_applies_before_subcommand():
    """A GlobalConfig flag placed before the subcommand (e.g. --transport) must be
    applied by the root callback so every subcommand sees it -- not just
    run/show-config, which no longer apply it themselves."""
    runner = CliRunner()
    original = global_config.transport
    try:
        result = runner.invoke(main, ["--transport", "zenoh", "show-config"])
        assert result.exit_code == 0, result.output
        assert "transport: zenoh" in result.output
    finally:
        global_config.update(transport=original)


def test_show_config_contains_only_global_settings():
    result = CliRunner().invoke(main, ["show-config"])
    assert result.exit_code == 0, result.output
    assert "transport:" in result.output
    assert "relay_key:" not in result.output
    assert "mcp_port:" not in result.output


def test_show_config_redacts_secrets_including_nested_settings(mocker: MockerFixture) -> None:
    config = mocker.patch.object(info, "global_config")
    config.model_dump.return_value = {
        "transport": "zenoh",
        "api_key": "private-api-key",
        "connection": {"aes_128_key": "private-aes-key", "host": "localhost"},
    }

    result = CliRunner().invoke(main, ["show-config"])

    assert result.exit_code == 0, result.output
    assert "transport: zenoh" in result.output
    assert "connection: {'host': 'localhost'}" in result.output
    assert "private-" not in result.output


def test_rerun_bridge_resolves_its_settings_like_a_blueprint_module(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    import dimos.visualization.rerun.bridge as bridge

    recorded: dict[str, Any] = {}
    monkeypatch.setattr(bridge, "run_bridge", lambda **config: recorded.update(config))
    monkeypatch.setenv("RERUNBRIDGEMODULE__HOST", "0.0.0.0")

    result = CliRunner().invoke(
        main,
        ["rerun-bridge", "--rerun-open", "none", "--rerunbridgemodule.memory-limit", "4GB"],
    )

    assert result.exit_code == 0, result.output
    assert recorded == {
        "host": "0.0.0.0",
        "rerun_open": "none",
        "memory_limit": "4GB",
        "rerun_web": True,
    }
    help_output = CliRunner().invoke(main, ["rerun-bridge", "--help"]).output
    assert "--rerunbridgemodule.host" in help_output


def test_rerun_bridge_applies_global_options_given_after_the_command(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    import dimos.visualization.rerun.bridge as bridge

    seen: list[str] = []
    monkeypatch.setattr(bridge, "run_bridge", lambda **config: seen.append(global_config.transport))

    result = CliRunner().invoke(main, ["rerun-bridge", "--transport", "lcm"])

    assert result.exit_code == 0, result.output
    assert seen == ["lcm"]


def test_run_composition_leaves_blueprint_alone_when_relay_disabled() -> None:
    class Config(ModuleConfig):
        pass

    class TestModule(Module):
        config: Config

    original_local_relay = global_config.local_relay
    original_relay_url = global_config.relay_url
    global_config.update(local_relay=False, relay_url=None)
    try:
        source = TestModule.blueprint()
        assert _with_relay_bridge(source) is source
    finally:
        global_config.update(
            local_relay=original_local_relay,
            relay_url=original_relay_url,
        )


def test_list_blueprints_groups_builtin_and_external(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        external,
        "list_external_blueprint_names",
        lambda: ["my-test-stack.demo", "my-test-stack.keyboard-teleop"],
    )

    result = CliRunner().invoke(main, ["list"])

    assert result.exit_code == 0
    assert "Built-in blueprints:" in result.output
    assert "  unitree-go2" in result.output
    assert "demo-agent" not in result.output
    assert "External blueprints:" in result.output
    assert "  my-test-stack.demo" in result.output
    assert "  my-test-stack.keyboard-teleop" in result.output


def test_list_blueprints_without_external_names(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(external, "list_external_blueprint_names", lambda: [])

    result = CliRunner().invoke(main, ["list"])

    assert result.exit_code == 0
    assert "Built-in blueprints:" in result.output
    assert "External blueprints:" not in result.output


def test_list_blueprints_reports_external_discovery_errors(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    def raise_error() -> list[str]:
        raise external.ExternalBlueprintError("external metadata is invalid")

    monkeypatch.setattr(external, "list_external_blueprint_names", raise_error)

    result = CliRunner().invoke(main, ["list"])

    assert result.exit_code == 1
    assert "external metadata is invalid" in result.output


@pytest.fixture
def isolated_cache_locks(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(cache_utils, "_CACHE_LOCK_DIR", tmp_path / "cache-users")
    monkeypatch.setattr(cache_utils, "_CACHE_GATE_PATH", tmp_path / "cache-clean.lock")


@pytest.fixture
def stubbed_run(
    isolated_cache_locks: None,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> Iterator[dict[str, Any]]:
    """Drive `dimos run` through parsing without starting processes."""
    recorded: dict[str, Any] = {}
    original_global_config = global_config.model_dump()

    class FakeCoordinator:
        n_modules = 1
        transports: dict[tuple[str, type], Any] = {}

        @classmethod
        def build(
            cls,
            blueprint: Any,
            parsed_config: Any = None,
            *,
            reserve_rpc_name: str | None = None,
            module_rpc_namespace: str | None = None,
            remote_module_refs: Any = None,
        ) -> "FakeCoordinator":
            recorded["blueprint"] = blueprint
            recorded["parsed_config"] = parsed_config
            return cls()

        def health_check(self) -> bool:
            return True

        def start_rpc_service(self, *, name: str | None = None) -> None:
            return None

        def stop(self) -> None:
            return None

        def suppress_console(self) -> None:
            return None

        def loop(self) -> None:
            return None

    class FakeEntry:
        def __init__(self, **fields: Any) -> None:
            recorded["entry"] = fields
            self.run_id = fields["run_id"]
            self.log_dir = fields["log_dir"]

        def save(self) -> None:
            return None

        def remove(self) -> None:
            return None

    blueprints = {
        "alpha": RunModuleA.blueprint(),
        "beta": RunModuleB.blueprint(),
    }
    modules = {
        "runmodulea": RunModuleA.blueprint(),
        "runmoduleb": RunModuleB.blueprint(),
    }

    global_config.update(local_relay=False, relay_url=None)
    monkeypatch.setattr(module_coordinator, "ModuleCoordinator", FakeCoordinator)
    monkeypatch.setattr(run_registry, "RunEntry", FakeEntry)
    monkeypatch.setattr(run_registry, "cleanup_stale", lambda: 0)
    monkeypatch.setattr(run_registry, "generate_run_id", lambda name: f"test-{name}")
    monkeypatch.setattr(process_lifecycle, "spawn_watchdog", lambda *args, **kwargs: None)
    monkeypatch.setattr(core_daemon, "install_signal_handlers", lambda *args, **kwargs: None)
    monkeypatch.setattr(lifecycle, "LOG_DIR", tmp_path / "logs")
    monkeypatch.setattr(logging_config, "set_run_log_dir", lambda path: None)
    monkeypatch.setattr(get_all_blueprints, "get_by_name_or_exit", blueprints.__getitem__)
    monkeypatch.setattr(
        get_all_blueprints,
        "get_module_by_name_or_exit",
        lambda name: modules[name.lower()],
    )
    monkeypatch.setenv("DIMOS_RUN_ID", "")

    try:
        yield recorded
    finally:
        global_config.update(**original_global_config)


@pytest.mark.parametrize("signum", [signal.SIGINT, signal.SIGTERM])
def test_foreground_logs_signal_and_requests_shutdown_once(
    stubbed_run: dict[str, Any],
    monkeypatch: pytest.MonkeyPatch,
    mocker: MockerFixture,
    signum: signal.Signals,
) -> None:
    handlers = {}
    monkeypatch.setattr(core_daemon, "install_signal_handlers", install_signal_handlers)
    mocker.patch("signal.signal", side_effect=lambda sig, handler: handlers.update({sig: handler}))
    shutdown = mocker.patch.object(module_coordinator.ModuleCoordinator, "shutdown", create=True)
    mocker.patch.object(
        module_coordinator.ModuleCoordinator,
        "loop",
        side_effect=lambda: handlers[signum](signum, None),
    )
    log = mocker.patch.object(core_daemon.logger, "info")

    result = CliRunner().invoke(main, ["run", "alpha"])

    assert result.exit_code == 0, result.output + repr(result.exception)
    log.assert_called_once_with(
        "Received signal, shutting down", signal=signum, run_id="test-alpha"
    )
    shutdown.assert_called_once_with()


@pytest.fixture
def stubbed_choices(stubbed_run, mocker: MockerFixture, monkeypatch: pytest.MonkeyPatch):
    mapping = autoconnect(
        ChoiceAtom(
            name="PointClearing",
            default="raycasting",
            choices={
                "raycasting": RunModuleA.blueprint(),
                "column_carving": RunModuleB.blueprint(),
            },
        ),
        ChoiceAtom(
            name="Viewer",
            default="off",
            choices={"on": RunModuleA.blueprint(instance_name="viewer"), "off": autoconnect()},
        ),
    )
    mocker.patch.object(get_all_blueprints, "get_by_name_or_exit", return_value=mapping)
    monkeypatch.setenv("TOPIC_METADATA", "false")
    return stubbed_run


@pytest.mark.parametrize(
    ("options", "expected"),
    [
        ([], RunModuleA),
        (["--choice", "PointClearing=column_carving"], RunModuleB),
        (["--choice=PointClearing=column_carving"], RunModuleB),
    ],
)
def test_run_resolves_choices_before_build(stubbed_choices, options, expected) -> None:
    result = CliRunner().invoke(main, ["run", "mapping", *options])

    assert result.exit_code == 0, result.output + repr(result.exception)
    assert isinstance(stubbed_choices["blueprint"], Blueprint)
    assert [atom.module for atom in stubbed_choices["blueprint"].blueprints] == [expected]


def test_run_supports_multiple_choices(stubbed_choices) -> None:
    result = CliRunner().invoke(
        main,
        ["run", "mapping", "--choice", "PointClearing=column_carving", "--choice", "Viewer=on"],
    )

    assert result.exit_code == 0, result.output + repr(result.exception)
    assert [atom.name for atom in stubbed_choices["blueprint"].blueprints] == [
        "runmoduleb",
        "viewer",
    ]


def test_run_parses_configuration_for_the_selected_branch(stubbed_choices) -> None:
    result = CliRunner().invoke(
        main,
        [
            "run",
            "mapping",
            "--runmoduleb.lookahead",
            "2.5",
            "--choice",
            "PointClearing=column_carving",
        ],
    )

    assert result.exit_code == 0, result.output + repr(result.exception)
    assert stubbed_choices["parsed_config"].module_kwargs("runmoduleb")["lookahead"] == 2.5


def test_run_rejects_configuration_for_unselected_modules(stubbed_choices) -> None:
    result = CliRunner().invoke(
        main,
        [
            "run",
            "mapping",
            "--choice",
            "PointClearing=column_carving",
            "--runmodulea.entity-prefix",
            "unused",
        ],
    )

    assert result.exit_code == 2, result.output
    assert "runmodulea.entity-prefix" in result.output
    assert stubbed_choices == {}


def test_run_help_uses_the_selected_branch(stubbed_choices) -> None:
    result = CliRunner().invoke(
        main, ["run", "mapping", "--choice", "PointClearing=column_carving", "--help"]
    )

    assert result.exit_code == 0, result.output
    assert "--runmoduleb.lookahead" in result.output
    assert "--runmodulea.entity-prefix" not in result.output
    assert stubbed_choices == {}


@pytest.mark.parametrize(
    ("options", "message"),
    [
        (["--choice", "PointClearing"], "expected NAME=OPTION"),
        (["--choice", "=raycasting"], "expected NAME=OPTION"),
        (["--choice", "PointClearing="], "expected NAME=OPTION"),
        (["--choice", "PointClearing=typo"], "Invalid option 'typo'"),
        (["--choice", "Typo=raycasting"], "Unknown or inactive choices: Typo"),
        (
            ["--choice", "PointClearing=raycasting", "--choice", "PointClearing=column_carving"],
            "Choice 'PointClearing' was specified more than once",
        ),
    ],
)
def test_run_rejects_invalid_choices_before_starting(stubbed_choices, options, message) -> None:
    result = CliRunner().invoke(main, ["run", "mapping", *options])

    assert result.exit_code == 2, result.output
    assert message in result.output
    assert stubbed_choices == {}


def test_run_requires_a_selection_when_no_default_exists(
    stubbed_run, mocker: MockerFixture
) -> None:
    mapping = autoconnect(ChoiceAtom(name="Required", choices={"alpha": RunModuleA.blueprint()}))
    mocker.patch.object(get_all_blueprints, "get_by_name_or_exit", return_value=mapping)

    result = CliRunner().invoke(main, ["run", "mapping"])

    assert result.exit_code == 2, result.output
    assert "Choice 'Required' is required" in result.output
    assert "--choice Required=<option>" in result.output
    assert stubbed_run == {}


def test_run_resolves_nested_choices_loaded_from_the_registry(stubbed_run, monkeypatch) -> None:
    inner = autoconnect(
        ChoiceAtom(
            name="PointClearing",
            choices={
                "raycasting": RunModuleA.blueprint(),
                "column_carving": RunModuleB.blueprint(),
            },
        )
    )
    exported = autoconnect(inner.global_config(n_workers=2)).global_config(n_workers=4)
    monkeypatch.setattr(sys.modules[__name__], "registered_choices", exported, raising=False)
    monkeypatch.setitem(
        get_all_blueprints.all_blueprints, "mapping", f"{__name__}:registered_choices"
    )
    monkeypatch.setattr(get_all_blueprints, "get_by_name_or_exit", get_by_name_or_exit)
    monkeypatch.setenv("TOPIC_METADATA", "false")

    result = CliRunner().invoke(
        main,
        [
            "run",
            "mapping",
            "--choice",
            "PointClearing=column_carving",
            "--runmoduleb.lookahead=2.5",
        ],
    )

    assert result.exit_code == 0, result.output + repr(result.exception)
    assert [atom.module for atom in stubbed_run["blueprint"].blueprints] == [RunModuleB]
    assert stubbed_run["parsed_config"].module_kwargs("runmoduleb")["lookahead"] == 2.5
    assert stubbed_run["blueprint"].global_config_overrides == {"n_workers": 4}


@pytest.mark.parametrize("run_options", [[], ["--daemon"]])
def test_dimsim_default_transport_fails_before_run_registration(
    stubbed_run, mocker, run_options
) -> None:
    mocker.patch.object(
        get_all_blueprints, "get_by_name_or_exit", return_value=GO2Connection.blueprint()
    )
    fork = mocker.patch.object(
        lifecycle, "fork_daemon", side_effect=AssertionError("started daemon")
    )

    result = CliRunner().invoke(
        main, ["--simulation", "dimsim", "run", "unitree-go2", *run_options]
    )

    assert result.exit_code == 2, result.output
    fork.assert_not_called()
    assert "DimSim publishes over LCM only" in result.output
    assert "--transport lcm" in result.output
    assert "entry" not in stubbed_run
    assert "parsed_config" not in stubbed_run


@pytest.mark.parametrize("run_options", [[], ["--daemon"]])
@pytest.mark.parametrize(
    "import_error",
    ["No module named roboplan", "libroboplan_core.so: undefined symbol: example"],
)
def test_run_reports_missing_dependency_before_starting(
    stubbed_run: dict[str, Any],
    mocker: MockerFixture,
    run_options: list[str],
    import_error: str,
) -> None:
    hint = (
        f"Failed to import roboplan.core: {import_error}. "
        "Install the manipulation extra before selecting the roboplan backend."
    )
    mocker.patch.object(
        RunModuleA, "check_dependencies", side_effect=ManipulationModule.check_dependencies
    )
    mocker.patch(
        "dimos.manipulation.manipulation_module.import_module",
        side_effect=ImportError(import_error),
    )
    fork_daemon = mocker.patch.object(lifecycle, "fork_daemon")

    result = CliRunner().invoke(main, ["run", "alpha", "beta", *run_options])

    assert result.exit_code == 1
    assert f"Error: {hint}" in result.output
    assert result.output.count(hint) == 1
    assert stubbed_run == {}
    fork_daemon.assert_not_called()


@pytest.mark.parametrize("run_options", [[], ["--daemon"]])
@pytest.mark.parametrize(
    ("module", "dependency", "hint"),
    [
        (A1ZConnection, "a1z.robots.get_robot", "bin/hardware/a1z/setup"),
        (A750Connection, "a750_control", "uv pip install a750-control"),
        (PiperConnection, "piper_sdk", "uv sync --extra control"),
        (XArmConnection, "xarm.wrapper", "uv sync --extra control"),
    ],
)
@pytest.mark.parametrize(
    "import_error", [ModuleNotFoundError("SDK missing"), ImportError("undefined symbol: example")]
)
def test_run_reports_unavailable_arm_sdk_before_starting(
    stubbed_run, mocker, run_options, module, dependency, hint, import_error
) -> None:
    # Exercise the arm's check without resolving hardware URDFs during config validation.
    mocker.patch.object(RunModuleA, "check_dependencies", side_effect=module.check_dependencies)
    import_sdk = mocker.patch(f"{module.__module__}.import_module", side_effect=import_error)
    fork_daemon = mocker.patch.object(lifecycle, "fork_daemon")

    result = CliRunner().invoke(main, ["run", "alpha", *run_options])

    assert result.exit_code == 1, result.output
    assert "SDK unavailable" in result.output
    assert str(import_error) in result.output
    assert result.output.count(hint) == 1
    assert "Traceback" not in result.output
    assert stubbed_run == {}
    import_sdk.assert_called_once_with(dependency)
    fork_daemon.assert_not_called()


def test_run_leaves_hosted_dependency_checks_to_the_host(
    stubbed_run: dict[str, Any],
    mocker: MockerFixture,
) -> None:
    check_dependencies = mocker.patch.object(
        RunModuleA, "check_dependencies", side_effect=ImportError("dependency missing locally")
    )
    mocker.patch.object(
        get_all_blueprints,
        "get_by_name_or_exit",
        return_value=RunModuleA.blueprint().hosted(host="robot-host"),
    )
    run_hosted = mocker.patch.object(lifecycle, "_run_hosted")

    result = CliRunner().invoke(main, ["run", "alpha"])

    assert result.exit_code == 0, result.output
    check_dependencies.assert_not_called()
    run_hosted.assert_called_once()
    assert "blueprint" not in stubbed_run


def test_hosted_run_announces_host_discovery_before_waiting(
    stubbed_run: dict[str, Any], mocker: MockerFixture
) -> None:
    mocker.patch.object(
        get_all_blueprints,
        "get_by_name_or_exit",
        return_value=RunModuleA.blueprint().hosted(host="robot-host"),
    )
    mocker.patch("dimos.hosted.machine.local_host", return_value=None)
    mocker.patch("dimos.hosted.client.root_rpc")
    mocker.patch("signal.signal")
    echo = mocker.spy(lifecycle.typer, "echo")

    def wait_for_hosts(*args: Any, **kwargs: Any) -> None:
        echo.assert_any_call("Waiting for Hosts...")
        raise TimeoutError("no live Host named or identified by 'robot-host'")

    mocker.patch("dimos.hosted.deploy.wait_for_hosts", side_effect=wait_for_hosts)

    result = CliRunner().invoke(main, ["--zenoh-connect", "tcp/127.0.0.1:7447", "run", "alpha"])

    assert result.exit_code == 1, result.output
    assert "Waiting for Hosts..." in result.output
    assert "Error: no live Host named or identified by 'robot-host'" in result.output


@pytest.mark.parametrize(
    ("module", "options", "message"),
    [
        (GO2Connection, [], "No robot IP specified"),
        (GO2Connection, ["--robot-ip", ""], "No robot IP specified"),
        (GO2Connection, ["--go2connection.ip", " "], "No robot IP specified"),
        (Go2FleetConnection, [], "No robot IPs specified"),
        (Go2FleetConnection, ["--go2fleetconnection.ips", "[]"], "No robot IPs specified"),
        (G1HighLevelWebRtc, [], "No robot IP specified"),
    ],
)
def test_run_rejects_missing_robot_addresses_before_build(
    stubbed_run, monkeypatch, tmp_path, module, options, message
) -> None:
    monkeypatch.setattr(get_all_blueprints, "get_by_name_or_exit", lambda _: module.blueprint())
    monkeypatch.setenv("ROBOT_IP", "null")
    monkeypatch.setenv("ROBOT_IPS", "null")
    monkeypatch.setenv("REPLAY", "false")
    monkeypatch.setenv("SIMULATION", "")

    result = CliRunner().invoke(
        main, ["run", "robot", "--config", str(tmp_path / "absent.json"), *options]
    )

    assert result.exit_code == 2, result.output
    assert "Invalid blueprint configuration" in result.output
    assert message in result.output
    assert "blueprint" not in stubbed_run
    assert "entry" not in stubbed_run


@pytest.mark.parametrize(
    ("exec_handoff", "expected_run_id"),
    [(True, "launcher-run"), (False, "test-alpha")],
)
def test_daemon_reuses_identity_only_for_exec_handoff(
    stubbed_run: dict[str, Any],
    monkeypatch: pytest.MonkeyPatch,
    exec_handoff: bool,
    expected_run_id: str,
) -> None:
    monkeypatch.setenv("DIMOS_RUN_ID", "launcher-run")
    if exec_handoff:
        monkeypatch.setenv("_DIMOS_DAEMON_STATUS_FD", "7")
    else:
        monkeypatch.delenv("_DIMOS_DAEMON_STATUS_FD", raising=False)
    monkeypatch.setattr(lifecycle, "fork_daemon", lambda path: (123, 7))
    monkeypatch.setattr(lifecycle, "read_daemon_status", lambda fd: {"ok": True, "n_modules": 1})

    result = CliRunner().invoke(main, ["run", "alpha", "--daemon"])

    assert result.exit_code == 0, result.output
    assert f"Run ID:    {expected_run_id}" in result.output


def test_run_rejects_record_topics_matching_nothing(stubbed_run: dict[str, Any]) -> None:
    result = CliRunner().invoke(
        main, ["--record", "sqlite", "--record-topics", "nope", "run", "alpha"]
    )

    assert result.exit_code == 2
    assert "matched none of" in result.output
    assert "blueprint" not in stubbed_run  # failed before build


def test_python_recording_rejects_rust_encoding_threads(
    stubbed_run: dict[str, Any],
) -> None:
    result = CliRunner().invoke(
        main,
        [
            "--record",
            "sqlite",
            "--record-engine",
            "python",
            "--record-encoding-threads",
            "4",
            "run",
            "alpha",
        ],
    )

    assert result.exit_code == 2
    output_plain = re.sub(r"\x1b\[[0-9;]*m", "", result.output)
    assert "valid only with" in output_plain
    assert "--record-engine rust" in output_plain
    assert "blueprint" not in stubbed_run


def test_python_mcap_is_rejected_before_build(
    stubbed_run: dict[str, Any],
) -> None:
    result = CliRunner().invoke(
        main, ["--record", "mcap", "--record-engine", "python", "run", "alpha"]
    )

    assert result.exit_code == 2
    output_plain = re.sub(r"\x1b\[[0-9;]*m", "", result.output)
    assert "MCAP recording requires --record-engine rust" in output_plain
    assert "blueprint" not in stubbed_run


def test_run_parses_spaced_and_equals_config_flags(stubbed_run: dict[str, Any]) -> None:
    result = CliRunner().invoke(
        main,
        [
            "run",
            "alpha",
            "beta",
            "--entity-prefix=hall",
            "--lookahead",
            "2.5",
        ],
    )

    assert result.exit_code == 0, result.output
    parsed = stubbed_run["parsed_config"]
    assert parsed.module_kwargs("runmodulea")["entity_prefix"] == "hall"
    assert parsed.module_kwargs("runmoduleb")["lookahead"] == 2.5
    assert stubbed_run["entry"]["blueprint"] == "alpha-beta"
    assert stubbed_run["entry"]["cli_args"] == ["alpha", "beta"]


@pytest.mark.parametrize(
    "argv",
    [
        ["--robot-ip", "192.168.0.116", "run", "alpha"],
        ["run", "alpha", "--robot-ip", "192.168.0.116"],
    ],
)
def test_run_accepts_global_config_flags_before_and_after_blueprint(
    argv: list[str],
    stubbed_run: dict[str, Any],
) -> None:
    result = CliRunner().invoke(main, argv)

    assert result.exit_code == 0, result.output
    assert stubbed_run["parsed_config"].global_config["robot_ip"] == "192.168.0.116"


def test_after_run_global_config_is_applied_before_blueprint_resolution(
    stubbed_run: dict[str, Any],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    observed_robot_ips: list[str | None] = []

    def resolve(name: str) -> Any:
        observed_robot_ips.append(global_config.robot_ip)
        assert name == "alpha"
        return RunModuleA.blueprint()

    monkeypatch.setattr(get_all_blueprints, "get_by_name_or_exit", resolve)

    result = CliRunner().invoke(
        main,
        ["run", "alpha", "--robot-ip", "192.0.2.42"],
    )

    assert result.exit_code == 0, result.output
    assert observed_robot_ips == ["192.0.2.42"]


def test_qualified_global_relay_flag_is_applied_before_composition(
    stubbed_run: dict[str, Any],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    observed_relay_values: list[tuple[bool, str | None]] = []

    def compose(blueprint: Any) -> Any:
        observed_relay_values.append((global_config.local_relay, global_config.relay_url))
        return blueprint

    monkeypatch.setattr(lifecycle, "_with_relay_bridge", compose)

    result = CliRunner().invoke(main, ["run", "alpha", "--g.local-relay=true"])

    assert result.exit_code == 0, result.output
    assert observed_relay_values == [(True, None)]
    assert stubbed_run["parsed_config"].global_config["local_relay"] is True


def test_run_relay_ca_flag_configures_composed_module(
    stubbed_run: dict[str, Any],
) -> None:
    result = CliRunner().invoke(
        main,
        ["run", "alpha", "--local-relay", "--relaybridgemodule.relay-ca", "/ca.pem"],
    )

    assert result.exit_code == 0, result.output
    assert stubbed_run["parsed_config"].module_kwargs("relaybridgemodule")["relay_ca"] == "/ca.pem"


def test_run_rejects_ambiguous_short_config_flag(stubbed_run: dict[str, Any]) -> None:
    result = CliRunner().invoke(main, ["run", "alpha", "beta", "--map-file", "office"])

    assert result.exit_code == 2
    assert "--runmodulea.map-file" in result.output
    assert "--runmoduleb.map-file" in result.output
    assert "parsed_config" not in stubbed_run
    assert "entry" not in stubbed_run


def test_run_accepts_qualified_config_flag(stubbed_run: dict[str, Any]) -> None:
    result = CliRunner().invoke(
        main,
        ["run", "alpha", "beta", "--runmoduleb.map-file=office"],
    )

    assert result.exit_code == 0, result.output
    assert stubbed_run["parsed_config"].module_kwargs("runmoduleb")["map_file"] == "office"
    assert "map_file" not in stubbed_run["parsed_config"].module_kwargs("runmodulea")


def test_run_options_still_work_after_dynamic_config_flags(
    stubbed_run: dict[str, Any],
) -> None:
    result = CliRunner().invoke(
        main,
        ["run", "alpha", "beta", "--entity-prefix", "hall", "--disable", "RunModuleB"],
    )

    assert result.exit_code == 0, result.output
    assert stubbed_run["parsed_config"].module_kwargs("runmodulea")["entity_prefix"] == "hall"
    assert stubbed_run["blueprint"].disabled_modules_tuple == (RunModuleB,)


def test_run_help_without_blueprint(isolated_cache_locks: None) -> None:
    result = CliRunner().invoke(main, ["run", "--help"])

    assert result.exit_code == 0, result.output
    output_plain = re.sub(r"\x1b\[[0-9;]*m", "", result.output)
    assert "--daemon" in output_plain


def test_run_help_lists_dynamic_flags_without_starting(
    stubbed_run: dict[str, Any], mocker: MockerFixture
) -> None:
    check_dependencies = mocker.patch.object(
        RunModuleA, "check_dependencies", side_effect=AssertionError("help checked dependencies")
    )

    result = CliRunner().invoke(main, ["run", "alpha", "--help"])

    assert result.exit_code == 0, result.output
    check_dependencies.assert_not_called()
    # In CI, GITHUB_ACTIONS makes typer emit ANSI styling that splits option
    # names mid-string, so strip the escape codes before matching.
    output_plain = re.sub(r"\x1b\[[0-9;]*m", "", result.output)
    assert "--map-file" in output_plain
    assert "--runmodulea.daemon" in output_plain
    assert output_plain.count("--daemon") == 1
    assert "Publish live resource stats for dtop" in output_plain
    assert "parsed_config" not in stubbed_run
    assert "entry" not in stubbed_run


@pytest.mark.parametrize("legacy_flag", ["-o", "--option"])
def test_run_rejects_removed_legacy_option(
    legacy_flag: str,
    stubbed_run: dict[str, Any],
) -> None:
    result = CliRunner().invoke(
        main,
        ["run", "alpha", legacy_flag, "runmodulea.entity_prefix=hall"],
    )

    assert result.exit_code == 2
    assert "removed" in result.output.lower()
    assert "--map-file" in result.output
    assert "parsed_config" not in stubbed_run


def test_run_reports_external_resolution_errors(
    isolated_cache_locks: None,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    def raise_error(name: str):
        raise external.ExternalBlueprintError(
            "Failed to load external blueprint "
            f"{name!r} from entry point 'my_test_stack.missing:demo_blueprint': "
            "ModuleNotFoundError: No module named 'my_test_stack.missing'"
        )

    monkeypatch.setattr(
        "dimos.robot.get_all_blueprints.resolve_external_blueprint_by_name",
        raise_error,
    )

    result = CliRunner().invoke(main, ["run", "my-test-stack.demo"])

    assert result.exit_code == 1
    assert "Failed to load external blueprint 'my-test-stack.demo'" in result.output
    assert "my_test_stack.missing:demo_blueprint" in result.output


def test_run_reports_unknown_bare_blueprint(isolated_cache_locks: None) -> None:
    result = CliRunner().invoke(main, ["run", "missing-bare-blueprint"])

    assert result.exit_code == 1
    assert "Unknown blueprint or module: missing-bare-blueprint" in result.output


@pytest.fixture
def spy_main_argv(monkeypatch):
    """Stub run_spy.main and capture the sys.argv the lcmspy alias hands it."""
    captured: list[list[str]] = []
    monkeypatch.setattr(sys, "argv", ["dimos"])
    monkeypatch.setattr(run_spy, "main", lambda: captured.append(list(sys.argv)))
    return captured


def test_lcmspy_alias_prepends_lcm_transport(spy_main_argv):
    result = CliRunner().invoke(main, ["lcmspy"])
    assert result.exit_code == 0, result.output
    assert spy_main_argv == [["spy", "--transport", "lcm"]]


def test_lcmspy_alias_rejects_transport_override(spy_main_argv):
    result = CliRunner().invoke(main, ["lcmspy", "--transport", "zenoh"])
    assert result.exit_code == 1
    assert "LCM-only" in result.output
    assert spy_main_argv == []  # never reaches the spy


def test_spy_cmd_rejects_stray_positional(monkeypatch):
    # A stray positional must fail loudly, not silently launch the TUI.
    monkeypatch.setattr(sys, "argv", ["dimos"])
    result = CliRunner().invoke(main, ["spy", "foo"])
    assert result.exit_code == 1
    assert "unexpected" in result.output.lower()


def test_lcmspy_rejects_stray_positional(monkeypatch):
    monkeypatch.setattr(sys, "argv", ["dimos"])
    result = CliRunner().invoke(main, ["lcmspy", "foo"])
    assert result.exit_code == 1
    assert "unexpected" in result.output.lower()


def test_spy_rejects_root_transport(monkeypatch, spy_main_argv):
    # A root-level `--transport` (before the subcommand) sets the stack backend,
    # which the spy ignores. Rather than silently show all transports, error and
    # point at the subcommand-level filter.
    monkeypatch.setattr(sys, "argv", ["dimos"])
    original = global_config.transport
    try:
        result = CliRunner().invoke(main, ["--transport", "zenoh", "spy"])
    finally:
        global_config.update(transport=original)
    assert result.exit_code == 2
    assert "dimos spy --transport" in result.output
    assert spy_main_argv == []  # never reaches the spy


def test_run_registered_keeps_the_entry_from_readiness_until_exit(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(run_registry, "REGISTRY_DIR", tmp_path / "runs")
    entry = run_registry.RunEntry("run-1", 1, "bp", "now", str(tmp_path))
    seen: list[bool] = []

    def fake_run(blueprint: Any, config: Any, *, on_ready: Any, **_: Any) -> None:
        seen.append(entry.registry_path.exists())
        on_ready(None)
        seen.append(entry.registry_path.exists())
        raise SystemExit(0)

    monkeypatch.setattr(lifecycle, "run_coordinator", fake_run)
    monkeypatch.setattr(lifecycle, "_mcp_url", lambda *_: "http://127.0.0.1:9990/mcp")
    with pytest.raises(SystemExit):
        lifecycle._run_registered("bp", "cfg", entry)  # type: ignore[arg-type]

    assert seen == [False, True]
    assert entry.mcp_url == "http://127.0.0.1:9990/mcp"
    assert not entry.registry_path.exists()
