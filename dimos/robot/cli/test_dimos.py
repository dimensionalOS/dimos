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
from types import SimpleNamespace
from typing import Literal

from pydantic import BaseModel, Field
import pytest
from typer.testing import CliRunner

from dimos.core.coordination.blueprints import autoconnect
import dimos.core.coordination.worker_manager_python as worker_manager_python
from dimos.core.global_config import global_config
from dimos.core.module import Module, ModuleConfig
from dimos.robot import external_blueprints as external
import dimos.robot.cli.dimos as dimos_cli
from dimos.robot.cli.dimos import _normalize_simulation_argv, arg_help, load_config_args, main
from dimos.utils.cache import CacheCleanResult, CacheIssue
import dimos.utils.cli.spy.run_spy as run_spy


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


def test_cache_clean_refuses_while_dimos_is_running(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        dimos_cli,
        "get_most_recent",
        lambda *, alive_only: SimpleNamespace(run_id="active-run"),
    )

    result = CliRunner().invoke(main, ["cache", "clean"])

    assert result.exit_code == 1
    assert "DimOS run active-run is active" in result.output


def test_cache_clean_force_overrides_active_run(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    cache_dir = tmp_path / "cache"
    cache_dir.mkdir()
    calls: list[bool] = []
    monkeypatch.setattr(
        dimos_cli,
        "get_most_recent",
        lambda *, alive_only: SimpleNamespace(run_id="active-run"),
    )
    monkeypatch.setattr(dimos_cli, "CACHE_DIR", cache_dir)
    monkeypatch.setattr(
        dimos_cli,
        "clean_caches",
        lambda *, force: calls.append(force) or CacheCleanResult(),
    )

    result = CliRunner().invoke(main, ["cache", "clean", "--force", "--yes"])

    assert result.exit_code == 0
    assert calls == [True]
    assert "Warning: cleaning caches while DimOS run active-run is active." in result.output
    assert "Force mode:" in result.output
    assert "Removes robot Git checkouts with local changes or local-only commits" in result.output


def test_cache_clean_defaults_confirmation_to_no(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    cache_dir = tmp_path / "cache"
    deno_dir = cache_dir / "deno"
    deno_dir.mkdir(parents=True)
    unknown_entry = cache_dir / "custom.bin"
    unknown_entry.write_bytes(b"cache")
    calls: list[bool] = []
    monkeypatch.setattr(dimos_cli, "CACHE_DIR", cache_dir)
    monkeypatch.setattr(dimos_cli, "get_most_recent", lambda *, alive_only: None)
    monkeypatch.setattr(
        dimos_cli,
        "clean_caches",
        lambda *, force: calls.append(force) or CacheCleanResult(),
    )

    result = CliRunner().invoke(main, ["cache", "clean"], input="\n")

    assert result.exit_code == 0
    assert calls == []
    assert "DimOS cache cleanup" in result.output
    assert f"Cache root:\n  {cache_dir}" in result.output
    assert f"- {unknown_entry} (DimOS cache entry)" in result.output
    assert f"- {deno_dir} (downloaded Deno runtime)" in result.output
    assert "Preserved:" not in result.output
    assert "Continue? [y/N]:" in result.output
    assert "Cache cleanup cancelled." in result.output


def test_cache_clean_reports_partial_cleanup(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    checkout = Path("/cache/robot_assets/sources/key/robot")
    cache_dir = tmp_path / "cache"
    cache_dir.mkdir()
    monkeypatch.setattr(dimos_cli, "CACHE_DIR", cache_dir)
    monkeypatch.setattr(dimos_cli, "get_most_recent", lambda *, alive_only: None)
    monkeypatch.setattr(
        dimos_cli,
        "clean_caches",
        lambda *, force: CacheCleanResult(
            skipped=[CacheIssue(checkout, "Git checkout has local changes")]
        ),
    )

    result = CliRunner().invoke(main, ["cache", "clean", "--yes"])

    assert result.exit_code == 1
    assert f"Skipped cache: {checkout} (Git checkout has local changes)" in result.output


def test_cache_clean_without_cache_is_a_noop(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    cache_dir = tmp_path / "missing-cache"
    monkeypatch.setattr(dimos_cli, "CACHE_DIR", cache_dir)
    monkeypatch.setattr(dimos_cli, "get_most_recent", lambda *, alive_only: None)

    result = CliRunner().invoke(main, ["cache", "clean"])

    assert result.exit_code == 0
    assert result.output == f"No DimOS cache found at {cache_dir}.\n"


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
        "      * testmodulea.default_rpc_timeout: float (default: 120.0)",
        "      * testmodulea.frame_id_prefix: str | None (default: None)",
        "      * testmodulea.frame_id: str | None (default: None)",
        "      * testmodulea.min_interval_sec: float (default: 0.1)",
        "      * testmodulea.entity_prefix: str (default: world)",
        "      * testmodulea.viewer_mode: typing.Literal['native', 'web', 'connect', 'none'] (default: native)",
        "    testmoduleb:",
        "      * testmoduleb.default_rpc_timeout: float (default: 120.0)",
        "      * testmoduleb.frame_id_prefix: str | None (default: None)",
        "      * testmoduleb.frame_id: str | None (default: None)",
        "      * testmoduleb.memory_limit: str (default: 25%)",
        "      * testmoduleb.ip: str (default: 127.0.0.1)",
        "",
    ]


def test_blueprint_arg_help_extra_args():
    """Test defaults passed to .blueprint() override."""

    class ConfigA(ModuleConfig):
        frame_id_prefix: str | None = None
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

    module_a = TestModuleA.blueprint(frame_id_prefix="foo", viewer_mode="web")
    blueprint = autoconnect(module_a, TestModuleB.blueprint(ip="1.1.1.1"))
    output = arg_help(blueprint.config(), blueprint)
    # List output produces better diff in pytest error output.
    assert output.split("\n") == [
        "    testmodulea:",
        "      * testmodulea.default_rpc_timeout: float (default: 120.0)",
        "      * testmodulea.frame_id_prefix: str | None (default: foo)",
        "      * testmodulea.frame_id: str | None (default: None)",
        "      * testmodulea.min_interval_sec: float (default: 0.1)",
        "      * testmodulea.entity_prefix: str (default: world)",
        "      * testmodulea.viewer_mode: typing.Literal['native', 'web', 'connect', 'none'] (default: web)",
        "    testmoduleb:",
        "      * testmoduleb.default_rpc_timeout: float (default: 120.0)",
        "      * testmoduleb.frame_id_prefix: str | None (default: None)",
        "      * testmoduleb.frame_id: str | None (default: None)",
        "      * testmoduleb.memory_limit: str (default: 25%)",
        "      * testmoduleb.ip: str (default: 1.1.1.1)",
        "",
    ]


def test_blueprint_arg_help_required():
    """Test required arguments."""

    class Config(ModuleConfig):
        foo: int
        spam: str = "eggs"

    class TestModule(Module):
        config: Config

    blueprint = TestModule.blueprint()
    output = arg_help(blueprint.config(), blueprint)
    assert output.split("\n") == [
        "    testmodule:",
        "      * testmodule.default_rpc_timeout: float (default: 120.0)",
        "      * testmodule.frame_id_prefix: str | None (default: None)",
        "      * testmodule.frame_id: str | None (default: None)",
        "      * [Required] testmodule.foo: int",
        "      * testmodule.spam: str (default: eggs)",
        "",
    ]


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


def test_run_reports_external_resolution_errors(monkeypatch: pytest.MonkeyPatch) -> None:
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


def test_run_reports_unknown_bare_blueprint() -> None:
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
    assert "        * testmodule.nested.enabled: bool (default: True)" in output
    assert "        * testmodule.nested.mode: str (default: manual)" in output


def test_blueprint_arg_help_uses_nested_backend_defaults():
    class DisabledConfig(BaseModel):
        backend: Literal["disabled"] = "disabled"

    class EnabledConfig(BaseModel):
        backend: Literal["enabled"] = "enabled"
        level: int = 1

    class Config(ModuleConfig):
        nested: DisabledConfig | EnabledConfig = Field(default_factory=DisabledConfig)

    class TestModule(Module):
        config: Config

    blueprint = TestModule.blueprint(nested={"backend": "enabled", "level": 3})
    output = arg_help(blueprint.config(), blueprint)

    assert "      testmodule.nested:" in output
    assert (
        "        * testmodule.nested.backend: typing.Literal['enabled'] (default: enabled)"
        in output
    )
    assert "        * testmodule.nested.level: int (default: 3)" in output


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
