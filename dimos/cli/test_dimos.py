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
from pathlib import Path
import re
import sys
from typing import Any

import pytest
from typer.testing import CliRunner

import dimos.cli.commands.lifecycle as lifecycle
from dimos.cli.commands.lifecycle import _with_relay_bridge
from dimos.cli.dimos import main, normalize_argv
import dimos.cli.spy.run_spy as run_spy
import dimos.core.coordination.module_coordinator as module_coordinator
import dimos.core.coordination.process_lifecycle as process_lifecycle
from dimos.core.global_config import global_config
from dimos.core.module import Module, ModuleConfig
import dimos.core.run_registry as run_registry
from dimos.robot import external_blueprints as external
import dimos.robot.get_all_blueprints as get_all_blueprints
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
        def build(cls, blueprint: Any, parsed_config: Any = None) -> "FakeCoordinator":
            recorded["blueprint"] = blueprint
            recorded["parsed_config"] = parsed_config
            return cls()

        def health_check(self) -> bool:
            return True

        def stop(self) -> None:
            return None

        def suppress_console(self) -> None:
            return None

        def loop(self) -> None:
            return None

    class FakeEntry:
        def __init__(self, **fields: Any) -> None:
            recorded["entry"] = fields

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
    monkeypatch.setattr(lifecycle, "install_signal_handlers", lambda *args, **kwargs: None)
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


def test_run_rejects_record_topics_matching_nothing(stubbed_run: dict[str, Any]) -> None:
    result = CliRunner().invoke(main, ["--record", "--record-topics", "nope", "run", "alpha"])

    assert result.exit_code == 2
    assert "matched none of" in result.output
    assert "blueprint" not in stubbed_run  # failed before build


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


def test_run_help_lists_dynamic_flags_without_starting(stubbed_run: dict[str, Any]) -> None:
    result = CliRunner().invoke(main, ["run", "alpha", "--help"])

    assert result.exit_code == 0, result.output
    # In CI, GITHUB_ACTIONS makes typer emit ANSI styling that splits option
    # names mid-string, so strip the escape codes before matching.
    output_plain = re.sub(r"\x1b\[[0-9;]*m", "", result.output)
    assert "--map-file" in output_plain
    assert "--runmodulea.daemon" in output_plain
    assert output_plain.count("--daemon") == 1
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
