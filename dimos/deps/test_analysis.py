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
import textwrap

import pytest

from dimos.deps.analysis import Analyzer, Requirements, merge_declarations
from dimos.deps.lock import LockIndex
from dimos.deps.requires import Requires
from dimos.deps.rules import GlobalRule

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib

LOCK = """
[[package]]
name = "dimos"
version = "0"
source = { editable = "." }
dependencies = [{ name = "numpy" }]

[package.optional-dependencies]
sim = [{ name = "mujoco" }, { name = "pygame" }]
perception = [{ name = "torch" }]
control = [{ name = "xarm-python-sdk" }, { name = "pygame" }]
web = [{ name = "fastapi" }]

[[package]]
name = "numpy"
version = "1"

[[package]]
name = "mujoco"
version = "1"

[[package]]
name = "torch"
version = "1"

[[package]]
name = "xarm-python-sdk"
version = "1"

[[package]]
name = "pygame"
version = "1"

[[package]]
name = "fastapi"
version = "1"
dependencies = [{ name = "starlette" }]

[[package]]
name = "starlette"
version = "1"
"""

PYPROJECT = """
[project]
name = "dimos"
dependencies = ["numpy"]

[project.optional-dependencies]
sim = ["mujoco", "pygame"]
perception = ["torch"]
control = ["xarm-python-sdk", "pygame"]
web = ["fastapi"]
base = ["dimos[web,perception]"]
"""

FILES = {
    "deps/requires.py": "class Requires: ...\n",
    "robot/blueprint.py": """
        from dimos.core.module import Module
        from dimos.robot.connection import Connection
        from dimos.hardware.helpers import arm
        from dimos.hardware.coordinator import Coordinator
        from dimos.core.global_config import global_config

        if global_config.simulation == "mujoco":
            from dimos.simulation.engine import Engine
        else:
            from dimos.hardware.real import Real
        """,
    "core/module.py": "import numpy\n",
    "core/global_config.py": "global_config = None\n",
    "robot/connection.py": """
        from dimos.deps.requires import Requires

        try:
            import cv2
            HAS_CV = True
        except ImportError:
            HAS_CV = False
        if HAS_CV:
            from dimos.perception.vision import Vision

        REQUIRES = Requires(selectors={"g.unitree_connection_type": "connection"})

        def make_connection():
            from dimos.simulation.engine import Engine
        """,
    "robot/_registry.py": """
        CONNECTION_FACTORIES = {
            "mujoco": "dimos.simulation.engine:Engine",
            "webrtc": "dimos.robot.webrtc:Webrtc",
        }
        """,
    "robot/webrtc.py": "import numpy\n",
    "perception/vision.py": """
        from dimos.deps.requires import Requires

        REQUIRES = Requires(extras=("perception",))

        import torch
        """,
    "simulation/engine.py": """
        from dimos.deps.requires import Requires

        REQUIRES = Requires(extras=("sim",), subprocesses=("dimos.simulation.process",))

        import mujoco
        """,
    "simulation/process.py": """
        from dimos.deps.requires import Requires

        REQUIRES = Requires(extras=("sim",), backends=("onnxruntime",))

        import mujoco
        """,
    "hardware/real.py": "import numpy\n",
    "hardware/helpers.py": """
        def arm():
            return Hardware(adapter_type="xarm")
        """,
    "hardware/coordinator.py": """
        from dimos.deps.requires import Requires


        class Coordinator:
            requires = Requires(selectors={"hardware": "adapter"})
        """,
    "hardware/manipulators/xarm/_registry.py": """
        ADAPTER_FACTORIES = {"xarm": "dimos.hardware.manipulators.xarm.adapter:XArmAdapter"}
        """,
    "hardware/manipulators/xarm/adapter.py": """
        from dimos.deps.requires import Requires

        REQUIRES = Requires(extras=("control",))


        class XArmAdapter:
            def __init__(self):
                import xarm
        """,
    "web/server.py": """
        from dimos.deps.requires import Requires

        REQUIRES = Requires(extras=("web",), tools=("deno",))

        import fastapi
        """,
}


@pytest.fixture
def project(tmp_path: Path) -> Path:
    for rel, source in FILES.items():
        path = tmp_path / "dimos" / rel
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(textwrap.dedent(source))
    return tmp_path


def make_analyzer(project: Path) -> Analyzer:
    lock = LockIndex.from_documents(tomllib.loads(PYPROJECT), tomllib.loads(LOCK))
    return Analyzer(project, lock)


@pytest.fixture
def analyzer(project: Path) -> Analyzer:
    return make_analyzer(project)


def _names(project: Path, closure_files: dict[Path, Path | None]) -> set[str]:
    return {p.relative_to(project / "dimos").as_posix() for p in closure_files}


def test_closure_follows_eager_manifests_subprocesses_and_scenario_branches(
    analyzer: Analyzer, project: Path
) -> None:
    root = project / "dimos" / "robot" / "blueprint.py"
    closure = analyzer.closure([root], {"simulation": ""})
    assert _names(project, closure.files) == {
        "robot/blueprint.py",
        "core/module.py",
        "core/global_config.py",
        "robot/connection.py",
        "deps/requires.py",
        "hardware/helpers.py",
        "hardware/real.py",
        "hardware/coordinator.py",
        "hardware/manipulators/xarm/adapter.py",
    }
    adapter = project / "dimos" / "hardware" / "manipulators" / "xarm" / "adapter.py"
    assert closure.reasons[adapter] == "adapter registry manifest 'xarm'"
    assert closure.chain(adapter)[-2:] == [project / "dimos" / "hardware" / "helpers.py", adapter]

    sim = analyzer.closure([root], {"simulation": "mujoco"})
    sim_names = _names(project, sim.files)
    assert {"simulation/engine.py", "simulation/process.py"} <= sim_names
    assert "hardware/real.py" not in sim_names
    process = project / "dimos" / "simulation" / "process.py"
    assert sim.reasons[process] == "subprocess 'dimos.simulation.process'"


def test_requirements_come_from_declarations(analyzer: Analyzer) -> None:
    result = analyzer.analyze_root("blueprint", "dimos.robot.blueprint:blueprint")
    assert not result.findings and not result.warnings
    assert result.base.extras == {"control": frozenset({"hardware.manipulators.xarm.adapter"})}
    assert result.base.selectors == {
        "g": {"unitree_connection_type": "connection"},
        "coordinator": {"hardware": "adapter"},
    }
    mujoco = result.variants["mujoco"]
    assert mujoco.extras == {"sim": frozenset({"simulation.engine", "simulation.process"})}
    assert mujoco.backends == {"onnxruntime"} and not mujoco.selectors
    assert "dimsim" not in result.variants
    assert result.to_json()["selectors"] == {
        "coordinator": {"hardware": "adapter"},
        "g": {"unitree_connection_type": "connection"},
    }


def test_moving_an_import_into_a_constructor_changes_nothing(project: Path) -> None:
    before = make_analyzer(project).analyze_root("blueprint", "dimos.robot.blueprint:blueprint")
    (project / "dimos" / "simulation" / "engine.py").write_text(
        textwrap.dedent(
            """
            from dimos.deps.requires import Requires

            REQUIRES = Requires(extras=("sim",), subprocesses=("dimos.simulation.process",))


            class Engine:
                def __init__(self):
                    import mujoco
            """
        )
    )
    after = make_analyzer(project).analyze_root("blueprint", "dimos.robot.blueprint:blueprint")
    assert after == before


def test_registry_entries_are_the_implementation_closures(analyzer: Analyzer) -> None:
    manifests = analyzer.manifests()
    xarm, _findings, _warnings = analyzer.analyze_files([manifests[("adapter", "xarm")]])
    assert xarm.extras == {"control": frozenset({"hardware.manipulators.xarm.adapter"})}
    mujoco, _findings, _warnings = analyzer.analyze_files([manifests[("connection", "mujoco")]])
    assert set(mujoco.extras) == {"sim"} and mujoco.backends == {"onnxruntime"}
    webrtc, _findings, _warnings = analyzer.analyze_files([manifests[("connection", "webrtc")]])
    assert webrtc.is_empty()
    assert analyzer.families() == {"adapter", "connection"}


def _check(project: Path, source: str) -> tuple[Requirements, list[str]]:
    """Audit one fresh file; a new analyzer so earlier versions of it are not cached."""
    bad = project / "dimos" / "core" / "bad.py"
    bad.write_text(textwrap.dedent(source))
    analyzer = make_analyzer(project)
    requirements, findings, _warnings = analyzer.check_file(analyzer.scan(bad))
    return requirements, [f"{f.message}. {f.hint}" for f in findings]


def test_lazy_import_needs_a_declaration_or_defers(project: Path) -> None:
    _requirements, messages = _check(project, "def f():\n    import torch\n")
    assert messages == [
        "distribution 'torch' is not provided by core or by this file's declarations "
        "(extras: none). declare Requires(extras=('perception',)) in this file, add 'torch' "
        "to defers when the caller declares it, or move the import"
    ]
    requirements, messages = _check(
        project,
        "from dimos.deps.requires import Requires\n"
        'REQUIRES = Requires(defers=("torch",))\n'
        "def f():\n    import torch\n",
    )
    assert not messages and requirements.is_empty()
    _requirements, messages = _check(
        project,
        'from dimos.deps.requires import Requires\nREQUIRES = Requires(defers=("torch",))\n',
    )
    assert messages == ["defers 'torch' but never imports it lazily. remove it from defers"]
    _requirements, messages = _check(
        project, "try:\n    import torch\nexcept ImportError:\n    torch = None\n"
    )
    assert not messages


def test_declared_extras_cover_eager_and_lazy_imports(project: Path) -> None:
    requirements, messages = _check(
        project,
        "from dimos.deps.requires import Requires\n"
        'REQUIRES = Requires(extras=("control",))\n'
        "import pygame\n"
        "def f():\n    import xarm\n",
    )
    assert not messages
    assert requirements.extras == {"control": frozenset({"core.bad"})}


def test_declaration_checks(project: Path) -> None:
    _requirements, messages = _check(
        project,
        "from dimos.deps.requires import Requires\n"
        'REQUIRES = Requires(extras=("base", "nope"), selectors={"hardware": "adapter", "x": "nope"})\n',
    )
    assert messages == [
        "extra 'base' is an aggregate. declare the extra that provides the import: perception, web",
        "unknown extra 'nope'. declare an extra from pyproject.toml",
        "selector 'hardware' must be declared on the module class that owns the field. move it "
        "into the class body as `requires = Requires(selectors=...)`",
        "no registry manifest defines family 'nope'. known families: adapter, connection",
    ]


def test_transitive_availability_is_a_finding(project: Path) -> None:
    _requirements, messages = _check(
        project,
        'from dimos.deps.requires import Requires\nREQUIRES = Requires(extras=("web",))\nimport starlette\n',
    )
    assert messages == [
        "distribution 'starlette' is not provided by core or by this file's declarations "
        "(extras: web). only reachable through web: fastapi -> starlette; declare it directly in "
        "[project.optional-dependencies] web and run uv lock"
    ]


def test_unknown_import_is_a_finding(project: Path) -> None:
    _requirements, messages = _check(project, "import totally_unknown_pkg\n")
    assert len(messages) == 1 and messages[0].startswith("unknown import name")


def test_rule_without_trigger(analyzer: Analyzer) -> None:
    rule = GlobalRule("relay", when=["truthy", "local_relay"], roots=("web/server.py",))
    requirements, findings, _warnings = analyzer.analyze_rule(rule)
    assert not findings
    assert requirements.extras == {"web": frozenset({"web.server"})}
    assert requirements.tools == {"deno"}


def test_why_finds_chains(analyzer: Analyzer, project: Path) -> None:
    root = project / "dimos" / "robot" / "blueprint.py"
    chains = analyzer.why([root], "control", {"simulation": ""})
    assert [p.name for p in chains[0]] == ["blueprint.py", "helpers.py", "adapter.py"]
    assert analyzer.why([root], "xarm", {"simulation": ""}) == chains
    assert analyzer.why([root], "torch", {"simulation": ""}) == []
    assert analyzer.why([root], "sim", {"simulation": "mujoco"})


def test_requirements_union_and_delta() -> None:
    base = Requirements(
        extras={"web": frozenset({"a"})},
        tools=frozenset({"deno"}),
        selectors={"g": {"simulation": "simulation"}},
    )
    more = Requirements(
        extras={"web": frozenset({"b"}), "sim": frozenset({"c"})},
        backends=frozenset({"onnxruntime"}),
        selectors={"g": {"simulation": "simulation"}, "coordinator": {"hardware": "adapter"}},
    )
    union = base.union(more)
    assert union.extras == {"web": frozenset({"a", "b"}), "sim": frozenset({"c"})}
    assert union.selectors == {
        "g": {"simulation": "simulation"},
        "coordinator": {"hardware": "adapter"},
    }
    delta = union.added_over(base)
    assert delta.extras == {"sim": frozenset({"c"})} and delta.backends == {"onnxruntime"}
    assert delta.selectors == {"coordinator": {"hardware": "adapter"}}
    assert delta.to_json() == {
        "backends": ["onnxruntime"],
        "extras": {"sim": ["c"]},
        "selectors": {"coordinator": {"hardware": "adapter"}},
    }
    assert Requirements().is_empty()


def test_merge_declarations_keeps_order_and_dedups() -> None:
    merged = merge_declarations(
        [
            Requires(extras=("web", "agents"), tools=("deno",)),
            Requires(extras=("agents", "sim"), selectors={"g.simulation": "simulation"}),
        ]
    )
    assert merged == Requires(
        extras=("web", "agents", "sim"),
        tools=("deno",),
        selectors={"g.simulation": "simulation"},
    )
