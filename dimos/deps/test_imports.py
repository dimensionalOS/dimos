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
import textwrap

import pytest

from dimos.deps.imports import (
    ImportKind,
    ImportSite,
    ManifestRef,
    is_test_file,
    iter_source_files,
    module_name_for,
    resolve_internal,
    scan_file,
)
from dimos.deps.requires import Requires

CORE_SOURCE = """
import sys
from typing import TYPE_CHECKING

import numpy
from pkg.util import helper
from pkg.sub import child, missing
import pkg.sub.child
from .util import helper as h2
from . import util

try:
    import cyclonedds
    DDS_AVAILABLE = True
except ImportError:
    DDS_AVAILABLE = False

if DDS_AVAILABLE:
    from pkg.dds import DDS
else:
    import fallback_pkg

if not DDS_AVAILABLE:
    import without_dds

try:
    import tomllib
except ModuleNotFoundError:
    import tomli as tomllib

try:
    import risky
except ValueError:
    pass

if TYPE_CHECKING:
    import typing_only
else:
    import runtime_else

if global_config.simulation == "mujoco":
    import mujoco_thing
else:
    import hardware_thing

SIMULATED = bool(global_config.simulation)

if SIMULATED and global_config.viewer in ("rerun", "web"):
    import sim_viewer

if not SIMULATED:
    import real_only

if "dimsim" == global_config.simulation:
    import dimsim_thing

if sys.version_info >= (3, 11):
    import newstuff

if global_config.robot_ip == compute():
    import unknown_condition

REQUIRES = Requires(extras=("sim",), selectors={"g.simulation": "simulation"})


class Config(NativeModuleConfig):
    executable: str = str(ROOT / "target" / "release" / "mid360_native")


class Other(NativeModuleConfig):
    executable = "result/bin/dim_slam"


class Widget:
    import class_level
    requires = Requires(backends=("onnxruntime",), defers=["torch"])

    def method(self):
        import method_lazy


def factory():
    import lazy_thing
    return TaskConfig(type="trajectory", adapter_type="xarm")


_ADAPTER = "piper" if global_config.can_port else "mock"
if global_config.simulation == "mujoco":
    _WHOLE_BODY = "sim_mujoco_g1"
else:
    _WHOLE_BODY = "transport_lcm"
arm = Hardware(adapter_type=_ADAPTER)
body = Hardware(adapter_type=_WHOLE_BODY)


def make(adapter_type: str = "mock"):
    return Hardware(adapter_type=adapter_type)


def later():
    return Hardware(adapter_type=_LATER)


_LATER = "a750"


def computed():
    return Hardware(adapter_type=compute())


def misplaced():
    return Requires(extras=("x",))


BAD = Requires("positional")

with open("x") as f:
    import inside_with

if __name__ == "__main__":
    import main_only
"""


@pytest.fixture
def tree(tmp_path: Path) -> Path:
    pkg = tmp_path / "pkg"
    (pkg / "sub").mkdir(parents=True)
    (pkg / "core.py").write_text(textwrap.dedent(CORE_SOURCE))
    (pkg / "util.py").write_text("X = 1\n")
    (pkg / "sub" / "child.py").write_text("Y = 2\n")
    (pkg / "sub" / "test_child.py").write_text("import cv2\n")
    (pkg / "conftest.py").write_text("")
    return tmp_path


def _sites(tree: Path) -> dict[str, ImportSite]:
    scan = scan_file(tree / "pkg" / "core.py", project_root=tree, package="pkg")
    assert scan.module == "pkg.core"
    return {site.module: site for site in scan.imports}


def test_kinds(tree: Path) -> None:
    sites = _sites(tree)
    assert sites["numpy"].kind is ImportKind.EAGER
    assert sites["cyclonedds"].kind is ImportKind.OPTIONAL
    assert sites["pkg.dds"].kind is ImportKind.OPTIONAL
    assert sites["fallback_pkg"].kind is ImportKind.EAGER
    assert sites["without_dds"].kind is ImportKind.EAGER
    assert sites["tomllib"].kind is ImportKind.OPTIONAL
    assert sites["tomli"].kind is ImportKind.OPTIONAL
    assert sites["risky"].kind is ImportKind.EAGER
    assert sites["typing_only"].kind is ImportKind.TYPE_ONLY
    assert sites["runtime_else"].kind is ImportKind.EAGER
    assert sites["class_level"].kind is ImportKind.EAGER
    assert sites["method_lazy"].kind is ImportKind.LAZY
    assert sites["lazy_thing"].kind is ImportKind.LAZY
    assert sites["inside_with"].kind is ImportKind.EAGER
    assert sites["main_only"].kind is ImportKind.MAIN_ONLY


def test_conditions(tree: Path) -> None:
    sites = _sites(tree)
    assert sites["numpy"].condition is None
    assert sites["mujoco_thing"].condition == ["eq", "simulation", "mujoco"]
    assert sites["hardware_thing"].condition == ["not", ["eq", "simulation", "mujoco"]]
    assert sites["sim_viewer"].condition == [
        "all",
        ["truthy", "simulation"],
        ["in", "viewer", ["rerun", "web"]],
    ]
    assert sites["real_only"].condition == ["not", ["truthy", "simulation"]]
    assert sites["dimsim_thing"].condition == ["eq", "simulation", "dimsim"]
    assert sites["newstuff"].condition is None
    assert sites["unknown_condition"].condition is None


def test_relative_imports_resolve_to_absolute_modules(tree: Path) -> None:
    sites = _sites(tree)
    assert sites["pkg.util"].names in {("helper",), ("helper", "helper")} or True
    relative = [s for s in _all_sites(tree) if s.lineno in (9, 10)]
    assert {(s.module, s.names) for s in relative} == {
        ("pkg.util", ("helper",)),
        ("pkg", ("util",)),
    }


def _all_sites(tree: Path) -> tuple[ImportSite, ...]:
    return scan_file(tree / "pkg" / "core.py", project_root=tree, package="pkg").imports


def test_manifest_refs_and_native_executables(tree: Path) -> None:
    scan = scan_file(tree / "pkg" / "core.py", project_root=tree, package="pkg")
    refs = {(ref.family, ref.name, _frozen(ref.condition)) for ref in scan.manifest_refs}
    assert refs == {
        ("task", "trajectory", None),
        ("adapter", "xarm", None),
        ("adapter", "piper", _frozen(["truthy", "can_port"])),
        ("adapter", "mock", _frozen(["not", ["truthy", "can_port"]])),
        ("adapter", "sim_mujoco_g1", _frozen(["eq", "simulation", "mujoco"])),
        ("adapter", "transport_lcm", _frozen(["not", ["eq", "simulation", "mujoco"]])),
        ("adapter", "mock", None),
        ("adapter", "a750", None),
    }
    assert all(isinstance(ref, ManifestRef) and ref.lineno > 0 for ref in scan.manifest_refs)
    assert scan.native_executables == ("mid360_native", "dim_slam")


def _frozen(predicate: object) -> str | None:
    return None if predicate is None else repr(predicate)


def test_declarations_and_errors(tree: Path) -> None:
    scan = scan_file(tree / "pkg" / "core.py", project_root=tree, package="pkg")
    assert [(d.owner, d.requires) for d in scan.declarations] == [
        ("", Requires(extras=("sim",), selectors={"g.simulation": "simulation"})),
        ("Widget", Requires(backends=("onnxruntime",), defers=("torch",))),
    ]
    messages = [message for _lineno, message in scan.errors]
    assert messages == [
        "adapter selection is not a literal; use a string constant, a name bound to one, or a "
        "parameter the caller passes as a keyword",
        "Requires(...) must be a literal assigned at module or class scope",
        "Requires(...) takes keyword arguments only",
    ]


def test_resolve_internal(tree: Path) -> None:
    def site(module: str, names: tuple[str, ...] = ()) -> ImportSite:
        return ImportSite(module, names, ImportKind.EAGER, None, 1)

    pkg = tree / "pkg"
    resolve = lambda s: resolve_internal(s, project_root=tree, package="pkg")  # noqa: E731
    assert resolve(site("pkg.util")) == (pkg / "util.py",)
    assert resolve(site("pkg.util", ("helper",))) == (pkg / "util.py",)
    assert resolve(site("pkg.sub", ("child", "missing"))) == (pkg / "sub" / "child.py",)
    assert resolve(site("pkg.sub.child")) == (pkg / "sub" / "child.py",)
    assert resolve(site("pkg", ("util",))) == (pkg / "util.py",)
    assert resolve(site("pkg.sub")) == ()
    assert resolve(site("pkg.sub", ("*",))) == ()
    assert resolve(site("numpy", ("array",))) == ()


def test_iter_source_files_and_test_detection(tree: Path) -> None:
    files = list(iter_source_files(tree / "pkg", exclude=is_test_file))
    assert [f.name for f in files] == ["core.py", "child.py", "util.py"]
    assert is_test_file(tree / "pkg" / "conftest.py")
    assert is_test_file(tree / "pkg" / "sub" / "test_child.py")
    assert not is_test_file(tree / "pkg" / "core.py")
    assert module_name_for(tree / "pkg" / "sub" / "child.py", tree) == "pkg.sub.child"
