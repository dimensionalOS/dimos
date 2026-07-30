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
from typing import Any, Literal

from pydantic import BaseModel, Field
import pytest
from typer.testing import CliRunner

import dimos.cli.dimos as dimos_cli
from dimos.cli.dimos import (
    _normalize_simulation_argv,
    _with_relay_bridge,
    arg_help,
    load_config_args,
    main,
)
import dimos.cli.spy.run_spy as run_spy
from dimos.core.coordination.blueprints import autoconnect
import dimos.core.coordination.module_coordinator as module_coordinator
import dimos.core.coordination.process_lifecycle as process_lifecycle
import dimos.core.coordination.worker_manager_python as worker_manager_python
from dimos.core.global_config import global_config
from dimos.core.module import Module, ModuleConfig
import dimos.core.run_registry as run_registry
from dimos.robot import external_blueprints as external
import dimos.robot.get_all_blueprints as get_all_blueprints
import dimos.utils.cache as cache_utils
import dimos.utils.logging_config as logging_config


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
        # Explicit simulator — left untouched.
        (["dimos", "--simulation", "mujoco", "run"], ["dimos", "--simulation", "mujoco", "run"]),
        (["dimos", "--simulation", "dimsim", "run"], ["dimos", "--simulation", "dimsim", "run"]),
        (["dimos", "--simulation=dimsim", "run"], ["dimos", "--simulation=dimsim", "run"]),
        # No `--simulation` at all — left untouched.
        (["dimos", "run", "go2"], ["dimos", "run", "go2"]),
    ],
)
def test_normalize_simulation_argv(argv: list[str], expected: list[str]):
    assert _normalize_simulation_argv(argv) == expected


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


def test_blueprint_arg_help():
    class ConfigA(ModuleConfig):
        min_interval_sec: float = 0.1
        entity_prefix: str = "world"
        viewer_mode: Literal["native", "web", "connect", "none"] = "native"

    class TestModuleA(Module):
        config: ConfigA

    class ConfigB(ModuleConfig):
        memory_limit: str = "25%"
        ip: str = "127.0.0.1"

    class TestModuleB(Module):
        config: ConfigB

    blueprint = autoconnect(TestModuleA.blueprint(), TestModuleB.blueprint())
    output = arg_help(blueprint.config(), blueprint)
    # List output produces better diff in pytest error output.
    assert output.split("\n") == [
        "    testmodulea:",
        "      * testmodulea.default_rpc_timeout: float (default: 120.0)"
        " [--testmodulea.default-rpc-timeout]",
        "      * testmodulea.frame_id_prefix: str | None (default: None)"
        " [--testmodulea.frame-id-prefix]",
        "      * testmodulea.frame_id: str | None (default: None) [--testmodulea.frame-id]",
        "      * testmodulea.min_interval_sec: float (default: 0.1) [--min-interval-sec]",
        "      * testmodulea.entity_prefix: str (default: world) [--entity-prefix]",
        "      * testmodulea.viewer_mode: typing.Literal['native', 'web', 'connect', 'none']"
        " (default: native) [--viewer-mode]",
        "    testmoduleb:",
        "      * testmoduleb.default_rpc_timeout: float (default: 120.0)"
        " [--testmoduleb.default-rpc-timeout]",
        "      * testmoduleb.frame_id_prefix: str | None (default: None)"
        " [--testmoduleb.frame-id-prefix]",
        "      * testmoduleb.frame_id: str | None (default: None) [--testmoduleb.frame-id]",
        "      * testmoduleb.memory_limit: str (default: 25%) [--testmoduleb.memory-limit]",
        "      * testmoduleb.ip: str (default: 127.0.0.1) [--ip]",
        "",
    ]


def test_load_config_args_merges_cli_g_overrides(tmp_path):
    """CLI flags (--transport, --local-relay, ...) must merge into the g
    subtree built from config file / G__* env / -o g.* args, not replace it:
    a replace silently reverts every other g.* key to its default."""

    class Config(ModuleConfig):
        pass

    class TestModuleG(Module):
        config: Config

    blueprint = TestModuleG.blueprint()
    kwargs = load_config_args(
        blueprint.config(),
        ["g.robot_id=go2-lab", "g.local_relay=false"],
        tmp_path / "config.json",
        cli_g_overrides={"local_relay": True},
    )
    assert kwargs["g"]["robot_id"] == "go2-lab"  # survives the CLI overrides
    assert kwargs["g"]["local_relay"] is True  # the explicit flag wins its own key


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


def test_blueprint_arg_help_nested_config_paths():
    class NestedConfig(BaseModel):
        enabled: bool = True
        mode: str = "auto"

    class Config(ModuleConfig):
        nested: NestedConfig = Field(default_factory=NestedConfig)

    class TestModule(Module):
        config: Config

    blueprint = TestModule.blueprint(nested={"mode": "manual"})
    output = arg_help(blueprint.config(), blueprint)

    assert "      testmodule.nested:" in output
    assert "        * testmodule.nested.enabled: bool (default: True) [--enabled]" in output
    assert "        * testmodule.nested.mode: str (default: manual) [--mode]" in output


def test_help_expands_the_union_variant_matching_the_backend_default():
    class DisabledConfig(BaseModel):
        backend: Literal["disabled"] = "disabled"

    class EnabledConfig(BaseModel):
        backend: Literal["enabled"] = "enabled"
        level: int = 1

    class Config(ModuleConfig):
        nested: DisabledConfig | EnabledConfig = Field(default_factory=DisabledConfig)

    class TestModule(Module):
        config: Config

    blueprint = TestModule.blueprint(nested={"backend": "enabled"})
    output = arg_help(blueprint.config(), blueprint)

    # A broken selection falls back to the first variant, which has no `level`.
    assert "testmodule.nested.level" in output


class RunConfigA(ModuleConfig):
    entity_prefix: str = "world"


class RunModuleA(Module):
    config: RunConfigA


class RunConfigB(ModuleConfig):
    lookahead: float = 1.0


class RunModuleB(Module):
    config: RunConfigB


@pytest.fixture
def stubbed_run(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> dict[str, Any]:
    """Drive `dimos run` to completion without starting any process."""
    recorded: dict[str, Any] = {}

    class FakeCoordinator:
        n_modules = 1

        @classmethod
        def build(cls, blueprint, blueprint_args=None):
            recorded["blueprint"] = blueprint
            recorded["kwargs"] = dict(blueprint_args or {})
            return cls()

        def health_check(self) -> bool:
            return False  # `--daemon` stops here instead of forking

        def stop(self) -> None: ...

        def start_rpc_service(self) -> None: ...

        def loop(self) -> None: ...

    class FakeEntry:
        def __init__(self, **fields: Any) -> None:
            recorded["entry"] = fields

        def save(self) -> None: ...

        def remove(self) -> None: ...

    blueprints = {"alpha": RunModuleA.blueprint(), "beta": RunModuleB.blueprint()}
    monkeypatch.setattr(module_coordinator, "ModuleCoordinator", FakeCoordinator)
    monkeypatch.setattr(run_registry, "RunEntry", FakeEntry)
    monkeypatch.setattr(run_registry, "cleanup_stale", lambda: 0)
    monkeypatch.setattr(process_lifecycle, "spawn_watchdog", lambda *a, **k: None)
    monkeypatch.setattr(dimos_cli, "install_signal_handlers", lambda *a, **k: None)
    monkeypatch.setattr(dimos_cli, "LOG_DIR", tmp_path / "logs")
    monkeypatch.setattr(logging_config, "set_run_log_dir", lambda _log_dir: None)
    monkeypatch.setattr(get_all_blueprints, "get_by_name_or_exit", lambda name: blueprints[name])
    monkeypatch.setattr(
        get_all_blueprints, "get_module_by_name_or_exit", lambda name: blueprints[name.lower()]
    )
    monkeypatch.setenv("DIMOS_RUN_ID", "")
    return recorded


def test_run_keeps_field_flags_out_of_the_run_id_and_registry(
    isolated_cache_locks: None, stubbed_run: dict[str, Any]
) -> None:
    result = CliRunner().invoke(main, ["run", "alpha", "beta", "--entity-prefix", "hall"])

    assert result.exit_code == 0, result.output
    entry = stubbed_run["entry"]
    assert entry["blueprint"] == "alpha-beta"
    assert entry["cli_args"] == ["alpha", "beta"]
    assert entry["run_id"].endswith("-alpha-beta")
    assert stubbed_run["kwargs"]["runmodulea"]["entity_prefix"] == "hall"


def test_run_matches_global_config_fields_after_the_blueprint(
    isolated_cache_locks: None, stubbed_run: dict[str, Any]
) -> None:
    result = CliRunner().invoke(
        main,
        ["run", "alpha", "--robot-ip", "192.168.0.116", "--entity-prefix", "hall"],
    )

    assert result.exit_code == 0, result.output
    assert stubbed_run["kwargs"]["g"]["robot_ip"] == "192.168.0.116"
    assert stubbed_run["kwargs"]["runmodulea"]["entity_prefix"] == "hall"


def test_run_options_still_parse_after_field_flags(
    isolated_cache_locks: None, stubbed_run: dict[str, Any]
) -> None:
    argv = ["run", "alpha", "beta", "--entity-prefix", "hall"]
    result = CliRunner().invoke(main, [*argv, "--disable", "Beta", "-o", "g.dtop=true"])

    assert result.exit_code == 0, result.output
    assert stubbed_run["kwargs"]["runmodulea"]["entity_prefix"] == "hall"
    assert stubbed_run["kwargs"]["g"]["dtop"] == "true"
    assert stubbed_run["blueprint"].disabled_modules_tuple == (RunModuleB,)


def test_run_rejects_an_ambiguous_field_flag(
    isolated_cache_locks: None, stubbed_run: dict[str, Any]
) -> None:
    result = CliRunner().invoke(main, ["run", "alpha", "beta", "--frame-id", "map"])

    assert result.exit_code == 2
    assert "--runmodulea.frame-id" in result.output
    assert "--runmoduleb.frame-id" in result.output
    assert "kwargs" not in stubbed_run  # nothing was built


def test_run_accepts_an_address_flag_for_an_ambiguous_field(
    isolated_cache_locks: None, stubbed_run: dict[str, Any]
) -> None:
    result = CliRunner().invoke(main, ["run", "alpha", "beta", "--runmoduleb.frame-id", "map"])

    assert result.exit_code == 0, result.output
    assert stubbed_run["kwargs"]["runmoduleb"]["frame_id"] == "map"
    assert "frame_id" not in stubbed_run["kwargs"].get("runmodulea", {})


def test_run_help_lists_field_flags(
    isolated_cache_locks: None, stubbed_run: dict[str, Any]
) -> None:
    result = CliRunner().invoke(main, ["run", "alpha", "--help"])

    assert result.exit_code == 0, result.output
    assert "* runmodulea.entity_prefix: str (default: world) [--entity-prefix]" in result.output


def test_nested_blueprint_config_defaults_survive_cli_override(tmp_path, monkeypatch):
    class NestedConfig(BaseModel):
        enabled: bool = True
        mode: str = "auto"

    class Config(ModuleConfig):
        nested: NestedConfig = Field(default_factory=NestedConfig)

    class TestModule(Module):
        config: Config

    class FakeWorker:
        dedicated = False
        module_count = 0

        def reserve_slot(self):
            self.module_count += 1

        def deploy_module(self, _module_class, _global_config, kwargs):
            return kwargs

    monkeypatch.setattr(
        worker_manager_python, "RPCClient", lambda actor, _module_class, _instance_name: actor
    )

    blueprint = TestModule.blueprint(nested={"mode": "manual"})
    blueprint_args = load_config_args(
        blueprint.config(),
        ["testmodule.nested.enabled=false"],
        tmp_path / "config.json",
    )
    worker_manager = worker_manager_python.WorkerManagerPython(global_config)
    worker_manager._started = True
    worker_manager._workers = [FakeWorker()]

    deployed_configs = worker_manager.deploy_parallel(
        [(TestModule, global_config, blueprint.blueprints[0].kwargs.copy())],
        blueprint_args,
    )
    config = Config(**deployed_configs[0])

    assert config.nested.enabled is False
    assert config.nested.mode == "manual"
