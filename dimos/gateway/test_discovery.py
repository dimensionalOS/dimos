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

"""The discovery cache with a fake `discover` child (crashes, hangs, a cache on disk, changes to the checkout), the
module ranking, module config introspection, and one real scan of the checkout."""

from __future__ import annotations

import dataclasses
import enum
import json
from pathlib import Path
import sys
import textwrap
from typing import Any, Literal

from pydantic import BaseModel, Field
import pytest

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.gateway import discover
from dimos.gateway.discovery import Discovery, code_changed, rank_modules
from dimos.gateway.models import DimosEvent

# Stands in for `python -m dimos.gateway.discover`: blueprints named crash-* kill the child, hang-* never answer,
# broken-* don't import; every other one has a module `<robot>-connection` and the shared `viewer`.
FAKE_DISCOVER = textwrap.dedent(
    """
    import json, os, sys, time
    MARKER = "@@DIMOS_SERVER@@"
    here = os.path.dirname(os.path.abspath(__file__))
    def emit(value):
        print("noise from an import")
        print(MARKER + json.dumps(value), flush=True)
    names = json.load(open(os.path.join(here, "names.json")))
    command = sys.argv[1]
    with open(os.path.join(here, "calls.log"), "a") as log:
        log.write(command + "\\n")
    if command == "names":
        emit(names)
        sys.exit(0)
    request = json.loads(sys.stdin.read() or "{}")
    known = set(request.get("known", []))
    def module(name, robot):
        cls = f"pkg.{name}.Cls"
        if cls not in known:
            known.add(cls)
            emit({"kind": "module", "name": name, "class": cls, "doc": "", "inputs": [],
                  "outputs": [{"name": "image", "type": "msgs.Image"}], "skills": [], "config": []})
    for name in request.get("blueprints", []):
        emit({"kind": "start", "name": name})
        if name.startswith("crash"):
            os._exit(9)
        if name.startswith("hang"):
            time.sleep(60)
        robot = name.split("-")[0]
        record = {"kind": "blueprint", "name": name, "ref": f"dimos.robot.{robot}.bp:x", "builtin": True,
                  "robot": robot, "optional_dependency": False, "import_error": None, "import_traceback": None,
                  "missing_module": None}
        if name.startswith("broken"):
            emit({**record, "importable": False, "modules": [], "optional_dependency": True,
                  "import_error": "ModuleNotFoundError: No module named 'sdk'", "missing_module": "sdk"})
            continue
        refs = [{"name": m, "class": f"pkg.{m}.Cls", "module": m, "streams": []}
                for m in (f"{robot}-connection", "viewer")]
        emit({**record, "importable": True, "modules": refs})
        module(f"{robot}-connection", robot)
        module("viewer", robot)
    for name in request.get("modules", []):
        emit({"kind": "start", "name": "module:" + name})
        module(name, None)
    emit({"kind": "end"})
    """
)


@pytest.fixture
def fake_discover(tmp_path: Path) -> Any:
    """`fake_discover(blueprints, modules)`: (the child's command, the file it logs its sub-commands to)."""

    def make(blueprints: list[str], modules: list[str] = ()) -> tuple[list[str], Path]:  # type: ignore[assignment]
        folder = tmp_path / "fake"
        folder.mkdir(exist_ok=True)
        (folder / "discover.py").write_text(FAKE_DISCOVER)
        (folder / "names.json").write_text(
            json.dumps({"blueprints": blueprints, "modules": list(modules)})
        )
        return [sys.executable, str(folder / "discover.py")], folder / "calls.log"

    return make


def keyed(digest: str, **parts: Any) -> Any:
    return lambda _: {
        "digest": digest,
        "commit": "c1",
        "dirty": {},
        "packages": "p",
        "python": "py",
        **parts,
    }


def discovery(
    tmp_path: Path, command: list[str], events: list[dict[str, Any]], **kwargs: Any
) -> Discovery:
    return Discovery(
        tmp_path, events.append, cache_dir=tmp_path / "cache", command=command, **kwargs
    )


async def test_a_crash_or_hang_costs_one_blueprint(
    tmp_path: Path, fake_discover: Any, check_model: Any
) -> None:
    command, _ = fake_discover(
        ["go2-a", "crash-b", "go2-c", "hang-d", "g1-e", "broken-f"], ["extra-mod"]
    )
    events: list[dict[str, Any]] = []
    found = discovery(tmp_path, command, events, item_timeout=5, key=keyed("k1"))
    await found.check("startup")
    status = found.status
    assert status["state"] == "done" and status["reason"] == "startup"
    assert (status["blueprints_total"], status["blueprints_done"]) == (6, 6)
    assert (status["importable"], status["not_importable"]) == (3, 3)
    assert (status["modules_total"], status["modules_done"]) == (1, 1)
    crash = found.blueprint("crash-b")
    assert (
        crash and not crash["importable"] and "crashed its process (exit" in crash["import_error"]
    )
    hang = found.blueprint("hang-d")
    assert hang and "no answer in 5 s" in hang["import_error"]
    broken = found.blueprint("broken-f")
    assert broken and broken["missing_module"] == "sdk" and broken["optional_dependency"]
    assert [b["name"] for b in found.blueprint_list()][:3] == ["go2-a", "crash-b", "go2-c"]
    assert {m["name"] for m in found.module_list()} == {
        "go2-connection",
        "g1-connection",
        "viewer",
        "extra-mod",
    }
    assert (
        found.find_module("viewer")
        and found.find_module("Cls")
        and found.find_module("pkg.viewer.Cls")
    )
    assert any("crash-b" in e for e in status["errors"])
    # events: the start, progress, the end, each a valid DimosEvent
    assert events[0]["status"]["state"] == "scanning" and events[-1]["status"]["state"] == "done"
    for event in events:
        check_model(DimosEvent, event, "a discovery event")


async def test_a_restart_answers_from_disk_without_a_child(
    tmp_path: Path, fake_discover: Any
) -> None:
    command, calls = fake_discover(["go2-a", "g1-b"])
    await discovery(tmp_path, command, [], key=keyed("k1")).check("startup")
    scans = calls.read_text().count("scan")
    events: list[dict[str, Any]] = []
    again = discovery(tmp_path, command, events, key=keyed("k1"))
    again.load_cached()
    assert again.blueprint("go2-a")  # served before the key is even checked
    await again.check("startup")
    assert calls.read_text().count("scan") == scans
    assert again.status["state"] == "done" and not again.status["stale"]
    assert events[-1]["status"]["state"] == "done"


async def test_a_change_to_code_rescans_and_serves_the_old_answer_meanwhile(
    tmp_path: Path, fake_discover: Any
) -> None:
    (tmp_path / "pyproject.toml").write_text(PACKAGES_PYPROJECT)
    command, calls = fake_discover(["go2-a"])
    found = discovery(tmp_path, command, [], key=keyed("k1"))
    await found.check("startup")
    # a README edit isn't code: the answer is re-keyed, nothing is imported
    found.key = keyed("k2", dirty={"README.md": "1:2"})
    before = calls.read_text().count("scan")
    await found.check("changed")
    assert calls.read_text().count("scan") == before and found.status["key"] == "k2"
    # a .py edit is: the old answer is served (stale) until the new one is in
    command, _ = fake_discover(["go2-a", "g1-new"])
    found.key = keyed("k3", dirty={"dimos/robot/x.py": "1:2"})
    seen: list[Any] = []
    found.send = lambda event: seen.append(
        (
            event["status"]["state"],
            event["status"]["stale"],
            [b["name"] for b in found.blueprint_list()],
        )
    )
    await found.check("changed")
    assert ("scanning", True, ["go2-a"]) in seen
    assert seen[-1] == ("done", False, ["go2-a", "g1-new"])


PACKAGES_PYPROJECT = '[tool.setuptools.packages.find]\ninclude = ["dimos*"]\n'


def test_code_changed_asks_whether_an_import_can_change(tmp_path: Path) -> None:
    (tmp_path / "pyproject.toml").write_text(PACKAGES_PYPROJECT)
    old = {"commit": "c", "dirty": {"dimos/a.py": "1"}, "packages": "p", "python": "py"}

    def changed(path: str) -> bool:
        return code_changed(tmp_path, old, {**old, "dirty": {**old["dirty"], path: "2"}})

    assert not code_changed(tmp_path, old, {**old})
    for path in ("README.md", "docs/usage/cli.md", "web/app.js", ".github/workflows/ci.yml"):
        assert not changed(path), path
    # inside the package any file can matter (a yaml a module reads while importing); Python anywhere; packaging
    for path in (
        "dimos/robot/go2/params.yaml",
        "dimos/a.py",
        "scripts/tool.py",
        "uv.lock",
        "pyproject.toml",
    ):
        assert changed(path), path
    assert code_changed(tmp_path, old, {**old, "dirty": {}})  # a.py reverted is a change too
    assert code_changed(tmp_path, old, {**old, "packages": "q"})
    # no package list to go by: everything counts
    (tmp_path / "pyproject.toml").unlink()
    assert changed("README.md")


def blueprint(
    name: str, robot: str | None, *modules: str, importable: bool = True
) -> dict[str, Any]:
    return {
        "name": name,
        "robot": robot,
        "importable": importable,
        "modules": [{"name": m, "class": f"pkg.{m}", "module": m, "streams": []} for m in modules],
    }


def test_modules_rank_by_how_specific_they_are_to_the_robot() -> None:
    records = [
        blueprint("go2-basic", "go2", "go2-connection", "rerun", "mapper"),
        blueprint("go2-nav", "go2", "go2-connection", "rerun", "planner", "mapper"),
        blueprint("go2-agent", "go2", "go2-connection", "rerun", "agent"),
        blueprint("go2-broken", "go2", importable=False),
        blueprint("g1-basic", "g1", "g1-connection", "rerun", "planner"),
        blueprint("arm-basic", "xarm", "xarm-driver", "rerun"),
        blueprint("loose", None, "rerun"),
    ]
    ranked = rank_modules(records, "go2")
    assert ranked is not None
    assert (ranked["blueprints"], ranked["blueprints_importable"], ranked["robots_total"]) == (
        4,
        3,
        3,
    )
    names = [m["name"] for m in ranked["modules"]]
    assert names[0] == "go2-connection" and names[-1] == "rerun"
    top = ranked["modules"][0]
    assert top["score"] == pytest.approx(round(1.0 * __import__("math").log(3), 4))
    assert (top["in_robot_blueprints"], top["robot_blueprints"], top["robots_using"]) == (3, 3, 1)
    rerun = ranked["modules"][-1]
    assert rerun["score"] == 0 and rerun["blueprint_count"] == 6 and rerun["robots_using"] == 3
    assert names.index("mapper") < names.index(
        "planner"
    )  # same robots, used by more go2 blueprints
    assert rank_modules(records, "spot") is None


class Mode(enum.Enum):
    FAST = "fast"
    SAFE = "safe"


class FakeConfig(BaseModel):
    model_config = {"arbitrary_types_allowed": True}
    rate: float = Field(10.0, description="Hz")
    mode: Mode = Mode.FAST
    backend: Literal["webrtc", "ros"] = "webrtc"
    maybe: Literal["a", "b"] | None = None
    ip: str
    host: str = None  # type: ignore[assignment]
    sizes: tuple[int, int] = (1, 2)
    on_frame: Any = print
    thing: object = Field(default_factory=object)


class FakeModule:
    config: FakeConfig


@dataclasses.dataclass
class DataConfig:
    count: int = 3
    tags: list[str] = dataclasses.field(default_factory=list, metadata={"description": "labels"})


class DataModule:
    config: DataConfig


def test_config_fields_flag_what_json_cant_carry() -> None:
    fields = {f["name"]: f for f in discover.config_fields(FakeModule)}
    assert fields["rate"]["default"] == 10.0 and fields["rate"]["description"] == "Hz"
    assert fields["mode"]["enum"] == ["fast", "safe"] and fields["mode"]["default"] == "fast"
    assert fields["backend"]["enum"] == ["webrtc", "ros"]
    assert fields["maybe"]["enum"] == ["a", "b"] and fields["maybe"]["default"] is None
    assert fields["ip"]["required"] and fields["ip"]["json_compatible"]
    assert fields["sizes"]["json_compatible"] and fields["sizes"]["default"] == [1, 2]
    assert fields["host"]["default"] is None
    for name in ("rate", "mode", "backend", "maybe", "ip", "host", "sizes"):
        assert fields[name]["json_compatible"] and fields[name]["reason"] is None, name
    for name in ("on_frame", "thing"):
        assert not fields[name]["json_compatible"] and fields[name]["reason"], name
    data = {f["name"]: f for f in discover.config_fields(DataModule)}
    assert data["count"]["default"] == 3 and data["count"]["json_compatible"]
    assert data["tags"]["default"] == [] and data["tags"]["description"] == "labels"


async def test_a_real_scan_of_the_checkout(tmp_path: Path) -> None:
    """The real child on the real checkout, for two blueprints and a module: either it imports and has modules
    whose streams have types, or it says why not."""
    events: list[dict[str, Any]] = []
    real = Discovery(DIMOS_PROJECT_ROOT, events.append, cache_dir=tmp_path / "cache")
    real.command = [sys.executable, "-m", "dimos.gateway.discover"]
    names = await real.child_answer("names")
    assert "unitree-go2-basic" in names["blueprints"] and "go2-connection" in names["modules"]
    await real.scan_items(["coordinator-mock", "unitree-go2-basic"], ["go2-connection"])
    for name in ("coordinator-mock", "unitree-go2-basic"):
        record = real.blueprint(name)
        assert record is not None, name
        if record["importable"]:
            assert record["modules"] and record["import_error"] is None
            streams = [s for m in record["modules"] for s in m["streams"]]
            assert streams and all(
                s["type"] and s["direction"] in ("in", "out", "inout") for s in streams
            )
        else:
            assert record["import_error"], name
    assert real.blueprint("unitree-go2-basic")["robot"] == "go2"  # type: ignore[index]
    connection = real.find_module("go2-connection")
    if connection is not None:
        assert connection["class"].endswith("GO2Connection")
        assert {f["name"] for f in connection["config"]} and all(
            isinstance(f["json_compatible"], bool) for f in connection["config"]
        )
