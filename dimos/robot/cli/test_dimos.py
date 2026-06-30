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

from typing import Literal

import pytest
from typer.testing import CliRunner

from dimos.core.coordination.blueprints import autoconnect
import dimos.core.coordination.module_coordinator as module_coordinator
from dimos.core.module import Module, ModuleConfig
from dimos.core.runtime_environment import (
    MissingPythonProjectFileError,
    PythonProjectRuntimeEnvironment,
    PythonVenvRuntimeEnvironment,
)
from dimos.core.runtime_prepare import (
    RuntimePrepareCommand,
    RuntimePrepareError,
    RuntimePreparePlan,
    prepare_blueprint_runtimes,
    prepare_runtime_plan,
    select_runtime_prepare_plan,
)
from dimos.robot.cli.dimos import _normalize_simulation_argv, arg_help, main
import dimos.robot.get_all_blueprints as registry


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


class RuntimePrepareModuleA(Module):
    pass


class RuntimePrepareModuleB(Module):
    pass


class RuntimePrepareModuleC(Module):
    pass


def _project(tmp_path, name: str, *, pixi: bool = False):
    project = tmp_path / name
    project.mkdir()
    (project / "pyproject.toml").write_text("[project]\nname = 'demo'\nversion = '0.0.1'\n")
    if pixi:
        (project / "pixi.toml").write_text("[workspace]\nchannels = []\nplatforms = []\n")
    return project


def test_runtime_prepare_selects_active_project_runtimes_only(tmp_path):
    active_project = PythonProjectRuntimeEnvironment("active-project", _project(tmp_path, "active"))
    disabled_project = PythonProjectRuntimeEnvironment(
        "disabled-project", _project(tmp_path, "disabled")
    )
    direct_venv = PythonVenvRuntimeEnvironment("direct", tmp_path / "venv" / "bin" / "python")
    blueprint = (
        autoconnect(
            RuntimePrepareModuleA.blueprint(),
            RuntimePrepareModuleB.blueprint(),
            RuntimePrepareModuleC.blueprint(),
        )
        .runtime_environments(active_project, disabled_project, direct_venv)
        .runtime_placements(
            {
                RuntimePrepareModuleA: "active-project",
                RuntimePrepareModuleB: "disabled-project",
                RuntimePrepareModuleC: "direct",
            }
        )
        .disabled_modules(RuntimePrepareModuleB)
    )

    plan = select_runtime_prepare_plan(blueprint)

    assert [command.argv for command in plan.commands] == [
        ["uv", "venv", "--seed"],
        ["uv", "sync"],
    ]
    assert {command.runtime_name for command in plan.commands} == {"active-project"}
    assert plan.no_op_runtime_names == ["direct"]


def test_runtime_prepare_runtime_unknown_and_unused_errors(tmp_path):
    active_project = PythonProjectRuntimeEnvironment("active-project", _project(tmp_path, "active"))
    unused_project = PythonProjectRuntimeEnvironment("unused-project", _project(tmp_path, "unused"))
    blueprint = (
        RuntimePrepareModuleA.blueprint()
        .runtime_environments(active_project, unused_project)
        .runtime_placements({RuntimePrepareModuleA: "active-project"})
    )

    with pytest.raises(RuntimePrepareError, match="Unknown runtime environment 'missing'"):
        select_runtime_prepare_plan(blueprint, "missing")
    with pytest.raises(RuntimePrepareError, match="not used by the active blueprint"):
        select_runtime_prepare_plan(blueprint, "unused-project")


def test_runtime_prepare_active_missing_runtime_is_contextual_error():
    blueprint = RuntimePrepareModuleA.blueprint().runtime_placements(
        {RuntimePrepareModuleA: "missing-runtime"}
    )

    with pytest.raises(RuntimePrepareError) as exc_info:
        select_runtime_prepare_plan(blueprint)

    message = str(exc_info.value)
    assert "missing-runtime" in message
    assert "RuntimePrepareModuleA" in message
    assert "not registered" in message

    with pytest.raises(RuntimePrepareError) as explicit_exc_info:
        select_runtime_prepare_plan(blueprint, "missing-runtime")

    explicit_message = str(explicit_exc_info.value)
    assert "missing-runtime" in explicit_message
    assert "RuntimePrepareModuleA" in explicit_message
    assert "not registered" in explicit_message


def test_runtime_prepare_direct_venv_no_op_success(tmp_path):
    blueprint = (
        RuntimePrepareModuleA.blueprint()
        .runtime_environments(PythonVenvRuntimeEnvironment("direct", tmp_path / "python"))
        .runtime_placements({RuntimePrepareModuleA: "direct"})
    )
    output: list[str] = []
    calls: list[object] = []

    prepare_blueprint_runtimes(
        blueprint, "direct", runner=lambda *args, **kwargs: calls.append(args), output=output.append
    )  # type: ignore[arg-type]

    assert calls == []
    assert "no prepare step is required" in output[0]


def test_runtime_prepare_uv_commands_rerun_every_invocation(tmp_path):
    blueprint = (
        RuntimePrepareModuleA.blueprint()
        .runtime_environments(
            PythonProjectRuntimeEnvironment("project", _project(tmp_path, "project"))
        )
        .runtime_placements({RuntimePrepareModuleA: "project"})
    )
    calls: list[tuple[list[str], object]] = []

    def runner(argv, *, check, cwd):
        calls.append((argv, cwd))

    prepare_blueprint_runtimes(blueprint, runner=runner)  # type: ignore[arg-type]
    prepare_blueprint_runtimes(blueprint, runner=runner)  # type: ignore[arg-type]

    project = tmp_path / "project"
    assert calls == [
        (["uv", "venv", "--seed"], project),
        (["uv", "sync"], project),
        (["uv", "venv", "--seed"], project),
        (["uv", "sync"], project),
    ]


def test_runtime_prepare_pixi_commands(tmp_path):
    blueprint = (
        RuntimePrepareModuleA.blueprint()
        .runtime_environments(
            PythonProjectRuntimeEnvironment("project", _project(tmp_path, "project", pixi=True))
        )
        .runtime_placements({RuntimePrepareModuleA: "project"})
    )
    calls: list[list[str]] = []

    def runner(argv, *, check, cwd):
        calls.append(argv)

    prepare_blueprint_runtimes(blueprint, runner=runner)  # type: ignore[arg-type]

    assert calls == [
        ["pixi", "install"],
        ["pixi", "run", "uv", "venv", "-p", ".pixi/envs/default/bin/python", "--seed"],
        ["pixi", "run", "uv", "sync"],
    ]


def test_runtime_prepare_command_os_error_is_contextual(tmp_path):
    plan = RuntimePreparePlan(
        commands=[
            RuntimePrepareCommand(
                runtime_name="project",
                project=tmp_path,
                convention="uv",
                argv=[str(tmp_path / "missing-uv")],
            )
        ],
        no_op_runtime_names=[],
    )

    with pytest.raises(RuntimePrepareError) as exc_info:
        prepare_runtime_plan(plan)

    message = str(exc_info.value)
    assert "project" in message
    assert str(tmp_path) in message
    assert "missing-uv" in message


def test_run_invalid_project_runtime_error_preserves_pyproject_context(tmp_path, monkeypatch):
    project = tmp_path / "project-without-pyproject"
    project.mkdir()

    blueprint = RuntimePrepareModuleA.blueprint()
    monkeypatch.setattr(registry, "get_by_name_or_exit", lambda name: blueprint)

    def build(_blueprint, _kwargs):
        raise MissingPythonProjectFileError(
            runtime_name="project-env",
            project=project,
            missing_file=project / "pyproject.toml",
        )

    monkeypatch.setattr(module_coordinator.ModuleCoordinator, "build", build)

    result = CliRunner().invoke(
        main,
        ["--viewer", "none", "run", "demo-blueprint", "--config", str(tmp_path / "missing.json")],
        env={"COLUMNS": "240"},
    )

    assert result.exit_code != 0
    assert "demo-blueprint" in result.output
    assert "project-env" in result.output
    assert "project-without-pyproject" in result.output
    assert "pyproject.toml" in result.output


def test_runtime_prepare_cli_parse_and_run_like_semantics(tmp_path, monkeypatch):
    project = _project(tmp_path, "project")
    blueprint = (
        autoconnect(RuntimePrepareModuleA.blueprint(), RuntimePrepareModuleB.blueprint())
        .runtime_environments(PythonProjectRuntimeEnvironment("project", project))
        .runtime_placements({RuntimePrepareModuleA: "project", RuntimePrepareModuleB: "project"})
    )
    module_b_blueprint = RuntimePrepareModuleB.blueprint()
    calls: list[list[str]] = []

    import dimos.robot.get_all_blueprints as registry

    monkeypatch.setattr(registry, "get_by_name_or_exit", lambda name: blueprint)
    monkeypatch.setattr(registry, "get_module_by_name_or_exit", lambda name: module_b_blueprint)

    def runner(argv, *, check, cwd):
        calls.append(argv)

    monkeypatch.setattr("dimos.core.runtime_prepare.subprocess.run", runner)
    result = CliRunner().invoke(
        main,
        [
            "--replay",
            "runtime",
            "prepare",
            "demo-blueprint",
            "--runtime",
            "project",
            "--disable",
            RuntimePrepareModuleB.name,
            "-o",
            "runtimepreparemodulea.frame_id=map",
            "--config",
            str(tmp_path / "missing.json"),
        ],
    )

    assert result.exit_code == 0, result.output
    assert calls == [["uv", "venv", "--seed"], ["uv", "sync"]]
    assert "Preparing runtime 'project'" in result.output


def test_runtime_prepare_cli_command_exists():
    result = CliRunner().invoke(main, ["runtime", "prepare", "--help"])

    assert result.exit_code == 0, result.output
    assert "--runtime" in result.output
