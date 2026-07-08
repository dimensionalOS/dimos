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

"""CI guards: control tasks must never silently vanish from the registry."""

from __future__ import annotations

import importlib
import importlib.util
import inspect
from pathlib import Path
from typing import TYPE_CHECKING, Any

import pytest

from dimos.control.routing import CONSUMABLE_STREAMS, Routing, StreamBinding, TaskBindings
from dimos.control.tasks.registry import ControlTaskRegistry, control_task_registry

if TYPE_CHECKING:
    from types import ModuleType

# Heavy optional dependencies; a task factory failing on one of these still
# passes IF the dependency is not installed. Anything else (path typo,
# internal breakage) fails CI.
OPTIONAL_TASK_MODULES = {"onnxruntime", "pinocchio"}

# Task dirs that intentionally register nothing.
UNREGISTERED_TASK_DIRS: set[str] = set()


def test_every_task_dir_has_a_manifest() -> None:
    pkg = importlib.import_module("dimos.control.tasks")
    checked = 0
    for root in pkg.__path__:
        for child in sorted(Path(root).iterdir()):
            if not child.is_dir() or child.name.startswith(("_", ".")):
                continue
            if not any(not f.name.startswith("_") for f in child.rglob("*.py")):
                continue
            if child.name in UNREGISTERED_TASK_DIRS:
                continue
            manifest = child / "_registry.py"
            assert manifest.exists(), (
                f"{child} contains task code but no _registry.py; discover() would silently skip it"
            )
            manifest_mod = importlib.import_module(f"dimos.control.tasks.{child.name}._registry")
            names = set(manifest_mod.TASK_FACTORIES)
            assert names, f"{manifest} declares no tasks"
            missing = names - set(control_task_registry.available())
            assert not missing, f"{manifest} declares {missing} missing from available()"
            checked += 1
    assert checked > 0


def test_declared_task_factory_paths_resolve() -> None:
    for name, factory_path in sorted(control_task_registry._factory_paths.items()):
        module_name, attr = factory_path.split(":", maxsplit=1)
        try:
            module = importlib.import_module(module_name)
        except ModuleNotFoundError as exc:
            root = (exc.name or "").partition(".")[0]
            if root in OPTIONAL_TASK_MODULES and importlib.util.find_spec(root) is None:
                continue
            pytest.fail(f"{name}: importing {module_name!r} failed: {exc}")
        factory = getattr(module, attr, None)
        assert factory is not None, f"{name}: {module_name!r} has no attribute {attr!r}"
        assert callable(factory), f"{name}: {factory_path!r} is not callable"


def _manifest_modules() -> list[ModuleType]:
    pkg = importlib.import_module("dimos.control.tasks")
    modules = []
    for root in pkg.__path__:
        for child in sorted(Path(root).iterdir()):
            if not child.is_dir() or child.name.startswith(("_", ".")):
                continue
            if not (child / "_registry.py").exists():
                continue
            modules.append(importlib.import_module(f"dimos.control.tasks.{child.name}._registry"))
    return modules


def test_task_cards_are_well_formed() -> None:
    checked = 0
    for module in _manifest_modules():
        factories = module.TASK_FACTORIES
        consumes_by_type = getattr(module, "TASK_CONSUMES", {})
        exposes_by_type = getattr(module, "TASK_EXPOSES", {})
        for label, per_type in (
            ("TASK_CONSUMES", consumes_by_type),
            ("TASK_EXPOSES", exposes_by_type),
        ):
            unknown = set(per_type) - set(factories)
            assert not unknown, (
                f"{module.__name__}.{label} declares {sorted(unknown)} not in TASK_FACTORIES"
            )
        for task_type, streams in consumes_by_type.items():
            for stream, spec in streams.items():
                where = f"{module.__name__}: {task_type!r} stream {stream!r}"
                assert stream in CONSUMABLE_STREAMS, (
                    f"{where} not in allowed set {sorted(CONSUMABLE_STREAMS)}"
                )
                handler, routing = spec
                assert isinstance(handler, str) and handler, f"{where}: bad handler {handler!r}"
                Routing(routing)  # raises on unknown routing strings
                checked += 1
    assert checked > 0


def test_seeded_cards_load_into_registry() -> None:
    servo = control_task_registry.bindings_for("servo")
    assert servo.consumes == (
        StreamBinding("joint_command", "on_joint_command", Routing.CLAIM_OVERLAP),
    )
    velocity = control_task_registry.bindings_for("velocity")
    assert velocity.consumes == (
        StreamBinding("joint_command", "on_joint_command", Routing.CLAIM_OVERLAP),
    )
    cartesian = control_task_registry.bindings_for("cartesian_ik")
    assert cartesian.consumes == (
        StreamBinding(
            "coordinator_cartesian_command", "on_cartesian_command", Routing.BY_TASK_NAME
        ),
    )
    teleop = control_task_registry.bindings_for("teleop_ik")
    assert teleop.consumes == (
        StreamBinding(
            "coordinator_cartesian_command", "on_cartesian_command", Routing.BY_TASK_NAME
        ),
        StreamBinding("teleop_buttons", "on_teleop_buttons", Routing.BROADCAST),
    )
    for empty_card in ("trajectory", "g1_groot_wbc"):
        # Present in _bindings distinguishes a seeded empty card from no card at all.
        assert empty_card in control_task_registry._bindings, f"{empty_card} card not seeded"
        assert control_task_registry.bindings_for(empty_card) == TaskBindings()


def test_declared_handlers_exist_on_task_classes() -> None:
    checked = 0
    for task_type, bindings in sorted(control_task_registry._bindings.items()):
        if not bindings.consumes:
            continue
        factory_path = control_task_registry._factory_paths[task_type]
        module_name, _ = factory_path.split(":", maxsplit=1)
        try:
            module = importlib.import_module(module_name)
        except ModuleNotFoundError as exc:
            root = (exc.name or "").partition(".")[0]
            if root in OPTIONAL_TASK_MODULES and importlib.util.find_spec(root) is None:
                continue
            pytest.fail(f"{task_type}: importing {module_name!r} failed: {exc}")
        task_classes = [
            cls
            for _, cls in inspect.getmembers(module, inspect.isclass)
            if cls.__module__ == module.__name__
            and all(hasattr(cls, attr) for attr in ("compute", "claim", "is_active"))
        ]
        assert task_classes, f"{task_type}: no task class found in {module_name!r}"
        for binding in bindings.consumes:
            assert any(callable(getattr(cls, binding.handler, None)) for cls in task_classes), (
                f"{task_type}: no task class in {module_name!r} defines handler "
                f"{binding.handler!r} declared for stream {binding.stream!r}"
            )
            checked += 1
    assert checked > 0


def test_bindings_for_unknown_type_is_empty() -> None:
    bindings = control_task_registry.bindings_for("definitely_not_registered")
    assert bindings == TaskBindings()
    assert bindings.consumes == ()
    assert not bindings.exposes
    with pytest.raises(TypeError):
        bindings.exposes["poison"] = "x:Y"


def test_register_bindings_runtime_and_conflict() -> None:
    reg = ControlTaskRegistry()
    reg.register_bindings(
        "fake_runtime", consumes={"joint_command": ("on_joint_command", "claim_overlap")}
    )
    assert reg.bindings_for("fake_runtime").consumes == (
        StreamBinding("joint_command", "on_joint_command", Routing.CLAIM_OVERLAP),
    )
    # Identical re-registration is a no-op and keeps the original source attribution.
    reg.register_bindings(
        "fake_runtime", consumes={"joint_command": ("on_joint_command", "claim_overlap")}
    )
    reg.register_bindings(
        "fake_runtime",
        consumes={"joint_command": ("on_joint_command", "claim_overlap")},
        source="other._registry",
    )
    assert reg._binding_sources["fake_runtime"] == "register_bindings()"
    with pytest.raises(ValueError, match="fake_runtime"):
        reg.register_bindings(
            "fake_runtime", consumes={"joint_command": ("on_joint_command", "broadcast")}
        )


def test_register_bindings_rejects_bad_streams_and_routing() -> None:
    reg = ControlTaskRegistry()
    with pytest.raises(ValueError, match="later PR"):
        reg.register_bindings("fake_twist", consumes={"twist_command": ("on_twist", "broadcast")})
    with pytest.raises(ValueError, match="allowed"):
        reg.register_bindings("fake_stream", consumes={"no_such_port": ("on_x", "broadcast")})
    with pytest.raises(ValueError, match="routing"):
        reg.register_bindings("fake_routing", consumes={"joint_command": ("on_x", "round_robin")})
    with pytest.raises(ValueError, match="module:Model"):
        reg.register_bindings("fake_exposes", exposes={"do_thing": "not_a_path"})


class _HandlerlessTask:
    name = "handlerless"

    def claim(self) -> None:
        raise NotImplementedError

    def is_active(self) -> bool:
        return False

    def compute(self, state: Any) -> None:
        return None


class _HandledTask(_HandlerlessTask):
    name = "handled"

    def on_joint_command(self, msg: Any, t_now: float) -> bool:
        return True


def _make_handlerless_task(cfg: Any, hardware: Any) -> _HandlerlessTask:
    return _HandlerlessTask()


def _make_handled_task(cfg: Any, hardware: Any) -> _HandledTask:
    return _HandledTask()


def test_create_fails_loudly_on_missing_handler() -> None:
    reg = ControlTaskRegistry()
    card = {"joint_command": ("on_joint_command", "claim_overlap")}

    reg.register_path("fake_missing_handler", f"{__name__}:_make_handlerless_task")
    reg.register_bindings("fake_missing_handler", consumes=card, source="fake_manifest._registry")
    with pytest.raises(
        TypeError, match=r"fake_missing_handler.+on_joint_command.+fake_manifest\._registry"
    ):
        reg.create("fake_missing_handler", None)

    reg.register_path("fake_with_handler", f"{__name__}:_make_handled_task")
    reg.register_bindings("fake_with_handler", consumes=card)
    task = reg.create("fake_with_handler", None)
    assert isinstance(task, _HandledTask)
