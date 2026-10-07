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

"""introspect.py on real blueprints and modules, its answers checked against the API's models: a change to how dimos
describes blueprints, modules, streams or skills fails here instead of on a Launcher page."""

from pathlib import Path
from typing import Any

import pytest

from dimos.core.module import ModuleConfig
from dimos.gateway import blueprints, introspect, models
from dimos.robot import all_blueprints as registry, get_all_blueprints

ROOT = Path(__file__).parents[2]
SHOWN = introspect.SHOWN_BASE_FIELDS


def test_a_blueprint_lists_its_modules_and_streams(check_model: Any) -> None:
    answer = introspect.blueprint("demo-camera")
    check_model(models.Blueprint, answer, "the blueprint")
    camera = next(m for m in answer["modules"] if m["class"].endswith(".CameraModule"))
    assert {
        "name": "color_image",
        "type": "dimos.msgs.sensor_msgs.Image.Image",
        "direction": "out",
    } in [{k: v for k, v in s.items() if k != "topic"} for s in camera["streams"]]
    stream = next(s for s in camera["streams"] if s["name"] == "color_image")
    assert stream["topic"] == "/color_image"
    # the module's docstring, methods and where its code is (a file GET /dimos/source can read)
    documented = next(m for m in answer["modules"] if m["summary"])
    assert documented["doc"].startswith(documented["summary"][:40])
    assert camera["file"] == "dimos/hardware/sensors/camera/module.py"
    assert (
        (ROOT / camera["file"])
        .read_text()
        .splitlines()[camera["line"] - 1]
        .startswith("class CameraModule")
    )
    assert not {"build", "start", "stop"} & {m["name"] for m in camera["rpcs"]}
    # where the blueprint itself is defined: the line that assigns it
    defined = (ROOT / answer["file"]).read_text().splitlines()[answer["line"] - 1]
    assert defined.startswith("demo_camera")


def test_where_a_blueprint_or_a_module_is_defined() -> None:
    go2 = introspect.blueprint_source("unitree-go2-basic")
    assert go2["file"].startswith("dimos/robot/unitree/go2/")
    assert (
        (ROOT / go2["file"])
        .read_text()
        .splitlines()[go2["line"] - 1]
        .startswith("unitree_go2_basic")
    )
    camera = introspect.blueprint_source("camera-module")
    assert (ROOT / camera["file"]).read_text().splitlines()[camera["line"] - 1].startswith("class ")
    assert introspect.blueprint_source("no-such-blueprint") == {"file": None, "line": None}


def test_a_modules_rpcs_and_skills_carry_signatures_and_docs() -> None:
    from dimos.agents.annotation import skill
    from dimos.core.core import rpc
    from dimos.core.module import Module

    class Arm(Module):
        """Moves an arm.

        More about it."""

        @rpc
        def home(self, speed: float = 1.0) -> bool:
            """Back to the home pose."""
            return True

        @skill
        def wave(self, times: int) -> str:
            """Wave at someone."""
            return "ok"

    found = introspect.methods(Arm)
    assert found["rpcs"] == [
        {
            "name": "home",
            "params": [{"name": "speed", "type": "float", "default": "1.0"}],
            "return_type": "bool",
            "doc": "Back to the home pose.",
        }
    ]
    assert [(m["name"], m["doc"]) for m in found["skills"]] == [("wave", "Wave at someone.")]
    assert introspect.own_doc(Arm) == "Moves an arm.\n\nMore about it."

    class Bare(Module):
        pass

    # Module's own docstring describes every module, not this one
    assert introspect.own_doc(Bare) == ""
    assert introspect.source_of(Bare)["line"] is not None


def test_a_blueprints_config_and_a_modules_own(check_model: Any) -> None:
    for name in ("demo-camera", "camera-module"):
        answer = introspect.config(name)
        check_model(
            models.BlueprintConfig, blueprints.shown_config(name, answer), f"{name}'s config"
        )
        for module in answer["modules"]:
            assert "error" not in module, module
            names = {arg["name"] for arg in module["args"]}
            assert SHOWN <= names and not names & introspect.INTERNAL_FIELDS


def test_every_module_config_field_is_hidden_or_shown_on_purpose() -> None:
    assert introspect.INTERNAL_FIELDS.isdisjoint(SHOWN)
    unclassified = set(ModuleConfig.model_fields) - introspect.INTERNAL_FIELDS - SHOWN
    assert not unclassified, (
        f"ModuleConfig has new fields {unclassified}: add each to introspect.INTERNAL_FIELDS (hidden from a person) "
        "or SHOWN_BASE_FIELDS"
    )
    gone = (introspect.INTERNAL_FIELDS | SHOWN) - set(ModuleConfig.model_fields)
    assert not gone, f"ModuleConfig no longer has {gone}: drop them from introspect.py"


def test_an_unknown_name_is_an_error() -> None:
    with pytest.raises(ValueError, match="Unknown blueprint or module"):
        introspect.blueprint("no-such-blueprint")


def test_the_catalog(monkeypatch: pytest.MonkeyPatch, check_model: Any) -> None:
    some_blueprints = {
        name: registry.all_blueprints[name] for name in ("demo-camera", "unitree-go2-basic")
    }
    some_modules = {"camera-module": registry.all_modules["camera-module"]}
    for where in (registry, get_all_blueprints):
        monkeypatch.setattr(where, "all_blueprints", some_blueprints)
        monkeypatch.setattr(where, "all_modules", some_modules)
    answer = introspect.catalog()
    check_model(models.Catalog, answer, "the catalog")
    assert answer["errors"] == []
    assert [b["name"] for b in answer["blueprints"]] == ["demo-camera", "unitree-go2-basic"]
    assert "camera-module" in answer["blueprints"][0]["modules"]
    assert {s["module"] for s in answer["skills"]} <= {m["name"] for m in answer["modules"]}


async def test_the_child_process_answers_with_a_real_blueprint() -> None:
    answer = await blueprints.introspect(ROOT, ["blueprint", "demo-camera"])
    assert answer == introspect.blueprint("demo-camera")
