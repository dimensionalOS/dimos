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

from __future__ import annotations

from importlib import import_module
from pathlib import Path
import sys
from unittest.mock import Mock

from packaging import markers
from packaging.requirements import Requirement
from packaging.utils import canonicalize_name
import pytest
from typer.testing import CliRunner

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib

from dimos.cli.dimos import main
from dimos.core import run_registry
from dimos.core.coordination.blueprints import Blueprint, ChoiceAtom, MetaBlueprint, autoconnect
from dimos.robot.all_blueprints import all_blueprints, all_modules
from dimos.robot.get_all_blueprints import OptionalDependencyError, get_by_name, get_by_name_or_exit


def test_resolve_string_blueprint():
    bp = get_by_name("demo-mcp-stress-test")
    assert bp is not None


def test_resolve_string_module():
    bp = get_by_name("webcam-module")
    assert bp is not None


def test_resolve_string_blueprint_or_exit():
    bp = get_by_name_or_exit("demo-mcp-stress-test")
    assert bp is not None


def test_resolve_string_module_or_exit():
    bp = get_by_name_or_exit("webcam-module")
    assert bp is not None


@pytest.mark.parametrize("lookup", [get_by_name, get_by_name_or_exit])
def test_exported_composition_stays_unresolved(lookup) -> None:
    blueprint = lookup("demo-capabilities")

    assert isinstance(blueprint, MetaBlueprint)
    assert {atom.module.__name__ for atom in blueprint.solidify().blueprints} == {
        "DemoSensors",
        "DemoRobotActions",
        "DemoMonitoring",
        "McpServer",
        "Agent",
    }


def test_resolve_unknown_name():
    with pytest.raises(ValueError, match="Unknown blueprint or module"):
        get_by_name("nonexistent-blueprint-xyz")


@pytest.mark.parametrize("lookup", [get_by_name, get_by_name_or_exit])
def test_registry_preserves_unresolved_choices(monkeypatch: pytest.MonkeyPatch, lookup) -> None:
    mapping = autoconnect(ChoiceAtom(name="Required", choices={"empty": Blueprint(blueprints=())}))
    monkeypatch.setattr(sys.modules[__name__], "mapping", mapping, raising=False)
    monkeypatch.setitem(all_blueprints, "test-mapping", f"{__name__}:mapping")

    assert lookup("test-mapping") is mapping


@pytest.mark.parametrize(
    ("name", "target", "module", "extra"),
    [
        ("blueprint", "can:blueprint", "can", "control"),
        ("module", "mujoco.Module", "mujoco", "sim"),
    ],
)
def test_run_reports_missing_extra(
    monkeypatch: pytest.MonkeyPatch, name: str, target: str, module: str, extra: str
) -> None:
    registry = all_blueprints if name == "blueprint" else all_modules
    monkeypatch.setitem(registry, name, target)
    monkeypatch.setitem(sys.modules, module, None)

    result = CliRunner().invoke(main, ["run", name])

    assert result.exit_code == 1
    assert f"dimos[{extra}]" in result.output


def test_disabled_module_reports_its_missing_extra(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setitem(all_modules, "test-module", "mujoco.Module")
    monkeypatch.setitem(sys.modules, "mujoco", None)

    result = CliRunner().invoke(main, ["run", "demo-mcp-stress-test", "--disable", "test-module"])

    assert result.exit_code == 1
    assert "dimos[sim]" in result.output


@pytest.mark.parametrize("missing", ["mujoco.internal", "numpy"])
def test_lookup_preserves_unrelated_import_errors(
    monkeypatch: pytest.MonkeyPatch, missing: str
) -> None:
    monkeypatch.setitem(all_blueprints, "broken", f"{missing}:blueprint")
    monkeypatch.setitem(sys.modules, missing, None)

    with pytest.raises(ModuleNotFoundError) as error:
        get_by_name("broken")
    assert error.value.name == missing


def test_lookup_names_alternative_extras(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setitem(all_blueprints, "needs-pygame", "pygame:blueprint")
    monkeypatch.setitem(sys.modules, "pygame", None)

    with pytest.raises(OptionalDependencyError) as error:
        get_by_name("needs-pygame")
    assert "dimos[control]" in str(error.value)
    assert "dimos[sim]" in str(error.value)


@pytest.mark.parametrize(
    ("platform", "machine", "python", "message"),
    [
        ("linux", "x86_64", "3.10.12", "no compatible build for this Python or platform"),
        ("linux", "x86_64", "3.11.15", "Install 'dimos[mapping]'"),
        ("darwin", "arm64", "3.10.12", "Install 'dimos[mapping]'"),
    ],
)
def test_run_reports_platform_specific_extra(
    monkeypatch: pytest.MonkeyPatch, platform: str, machine: str, python: str, message: str
) -> None:
    environment = markers.default_environment()
    environment.update(
        sys_platform=platform,
        platform_machine=machine,
        python_version=".".join(python.split(".")[:2]),
        python_full_version=python,
    )
    monkeypatch.setattr(markers, "default_environment", lambda: environment.copy())
    monkeypatch.setitem(all_blueprints, "needs-gtsam", "gtsam:blueprint")
    monkeypatch.setitem(sys.modules, "gtsam", None)

    result = CliRunner().invoke(main, ["--replay", "run", "needs-gtsam"])

    assert result.exit_code == 1
    assert "dimos[mapping]" in result.output
    assert message in result.output


def test_optional_import_aliases_match_pyproject() -> None:
    with (Path(__file__).resolve().parents[2] / "pyproject.toml").open("rb") as f:
        project = tomllib.load(f)
    optional = {
        canonicalize_name(Requirement(spec).name)
        for group in project["project"]["optional-dependencies"].values()
        for spec in group
    }
    expected = {
        module: canonicalize_name(name)
        for name, module in project["tool"]["deptry"]["package_module_name_map"].items()
        if canonicalize_name(name) in optional
        and canonicalize_name(module) != canonicalize_name(name)
    }
    aliases = import_module("dimos.utils.optional_dependencies")._DISTRIBUTIONS
    assert {module: name for module, name in aliases.items() if "." not in module} == expected


@pytest.mark.parametrize(
    ("args", "missing", "extra"),
    [
        (
            ["spot", "--spothighlevel.username=test", "--spothighlevel.password=test"],
            "bosdyn.client",
            "spot",
        ),
        (["keyboard-teleop-module"], "pygame", "control"),
        (["unitree-g1-coordinator"], "unitree_sdk2py", "unitree-dds"),
        (["g1-whole-body-connection"], "unitree_sdk2py", "unitree-dds"),
        (["unitree-go2"], "gtsam", "mapping"),
        (["go2-nav-viewer-mesh"], "warp", "cuda"),
        (["coordinator-flowbase"], "portal", "misc"),
        (
            [
                "piper-connection",
                "--piperconnection.can-port=test-can",
                # An explicit joint map avoids fetching the vendor URDF to infer it.
                '--piperconnection.joint-commands={"joint1":["position"]}',
            ],
            "piper_sdk",
            "control",
        ),
    ],
)
def test_run_checks_deferred_packages_before_allocating_workers(
    monkeypatch: pytest.MonkeyPatch, args: list[str], missing: str, extra: str
) -> None:
    monkeypatch.setitem(sys.modules, missing, None)
    allocation = Mock(side_effect=RuntimeError("allocation reached"))
    monkeypatch.setattr(run_registry, "cleanup_stale", allocation)

    result = CliRunner().invoke(main, ["--viewer", "none", "--replay", "run", *args])

    assert result.exit_code == 1
    assert f"dimos[{extra}]" in result.output
    allocation.assert_not_called()


def test_go2_replay_with_incompatible_gtsam_gives_a_supported_environment(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    env = markers.default_environment()
    env.update(
        sys_platform="linux",
        platform_machine="x86_64",
        python_version="3.10",
        python_full_version="3.10.12",
    )
    monkeypatch.setattr(markers, "default_environment", lambda: env.copy())
    monkeypatch.setitem(sys.modules, "gtsam", None)
    allocation = Mock(side_effect=RuntimeError("allocation reached"))
    monkeypatch.setattr(run_registry, "cleanup_stale", allocation)

    result = CliRunner().invoke(main, ["--viewer", "none", "--replay", "run", "unitree-go2"])

    assert result.exit_code == 1
    assert 'python_full_version >= "3.11"' in result.output
    assert "Install " not in result.output
    allocation.assert_not_called()


@pytest.mark.parametrize("name", ["open-arm-mini-connection", "dual-open-yam-leader-connection"])
def test_feetech_reader_checks_only_configured_serial_ports(
    name: str, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setitem(sys.modules, "scservo_sdk", None)
    allocation = Mock(side_effect=RuntimeError("allocation reached"))
    monkeypatch.setattr(run_registry, "cleanup_stale", allocation)
    args = ["--viewer", "none", "--replay", "run", name]
    module = name.replace("-", "")

    CliRunner().invoke(main, [*args, f"--{module}.mock-positions={{}}"])
    allocation.assert_called_once()
    allocation.reset_mock()
    result = CliRunner().invoke(main, [*args, f"--{module}.left-serial-port=configured"])

    assert result.exit_code == 1
    assert "leader-teleop" in result.output
    allocation.assert_not_called()
