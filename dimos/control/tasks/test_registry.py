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
import importlib.resources
import importlib.util
import inspect
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


def _task_dirs() -> list[Any]:
    """Task subpackage directories under ``dimos.control.tasks``.

    Enumerated via ``importlib.resources`` so the guard sees the same dirs
    however dimos is installed (editable, wheel, or zip), not only a source
    checkout. The result is intentionally independent of the registry's own
    discovery: these tests exist to catch dirs the registry silently skips.
    """
    root = importlib.resources.files("dimos.control.tasks")
    dirs = (
        child
        for child in root.iterdir()
        if child.is_dir() and not child.name.startswith(("_", "."))
    )
    return sorted(dirs, key=lambda child: child.name)


def _contains_task_code(directory: Any) -> bool:
    """True if the dir tree holds a non-underscore ``.py`` module (real task code)."""
    for child in directory.iterdir():
        if child.is_dir():
            if _contains_task_code(child):
                return True
        elif child.name.endswith(".py") and not child.name.startswith("_"):
            return True
    return False


def test_every_task_dir_has_a_manifest() -> None:
    checked = 0
    for child in _task_dirs():
        if not _contains_task_code(child):
            continue
        if child.name in UNREGISTERED_TASK_DIRS:
            continue
        assert (child / "_registry.py").is_file(), (
            f"{child.name} contains task code but no _registry.py; "
            "discover() would silently skip it"
        )
        manifest_mod = importlib.import_module(f"dimos.control.tasks.{child.name}._registry")
        names = set(manifest_mod.TASK_FACTORIES)
        assert names, f"{child.name}/_registry.py declares no tasks"
        missing = names - set(control_task_registry.available())
        assert not missing, f"{child.name}/_registry.py declares {missing} missing from available()"
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
    modules = []
    for child in _task_dirs():
        if not (child / "_registry.py").is_file():
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
    trajectory = control_task_registry.bindings_for("trajectory")
    assert trajectory.consumes == ()  # command-driven only
    assert trajectory.exposes == frozenset({"execute", "cancel", "get_state"})
    g1 = control_task_registry.bindings_for("g1_groot_wbc")
    assert g1.consumes == (StreamBinding("twist_command", "on_twist_command", Routing.BROADCAST),)
    assert g1.exposes == frozenset({"arm", "disarm", "set_dry_run", "reset_runtime_state"})


def _scannable_task_classes(task_type: str) -> list[type] | None:
    """Task-like classes defined in ``task_type``'s factory module.

    Returns ``None`` when the module needs an optional dependency that is
    not installed (the CI guard skips those); otherwise the non-empty list
    of task classes (an empty list is a hard failure).
    """
    factory_path = control_task_registry._factory_paths[task_type]
    module_name, _ = factory_path.split(":", maxsplit=1)
    try:
        module = importlib.import_module(module_name)
    except ModuleNotFoundError as exc:
        root = (exc.name or "").partition(".")[0]
        if root in OPTIONAL_TASK_MODULES and importlib.util.find_spec(root) is None:
            return None
        pytest.fail(f"{task_type}: importing {module_name!r} failed: {exc}")
    classes = [
        cls
        for _, cls in inspect.getmembers(module, inspect.isclass)
        if cls.__module__ == module.__name__
        and all(hasattr(cls, attr) for attr in ("compute", "claim", "is_active"))
    ]
    assert classes, f"{task_type}: no task class found in {module_name!r}"
    return classes


def test_declared_handlers_exist_on_task_classes() -> None:
    checked = 0
    for task_type in control_task_registry.available():
        bindings = control_task_registry.bindings_for(task_type)
        if not bindings.consumes:
            continue
        task_classes = _scannable_task_classes(task_type)
        if task_classes is None:
            continue
        for binding in bindings.consumes:
            assert any(callable(getattr(cls, binding.handler, None)) for cls in task_classes), (
                f"{task_type}: no task class defines handler "
                f"{binding.handler!r} declared for stream {binding.stream!r}"
            )
            checked += 1
    assert checked > 0


def test_declared_commands_exist_on_task_classes() -> None:
    checked = 0
    for task_type in control_task_registry.available():
        bindings = control_task_registry.bindings_for(task_type)
        if not bindings.exposes:
            continue
        task_classes = _scannable_task_classes(task_type)
        if task_classes is None:
            continue
        for command in sorted(bindings.exposes):
            assert any(callable(getattr(cls, command, None)) for cls in task_classes), (
                f"{task_type}: no task class defines command {command!r} declared in TASK_EXPOSES"
            )
            checked += 1
    assert checked > 0


def test_command_guard_flags_command_absent_from_task_class() -> None:
    # Proves the guard above actually discriminates: a real command resolves
    # to a callable on the task class, a bogus one does not.
    task_classes = _scannable_task_classes("trajectory")
    assert task_classes is not None
    assert any(callable(getattr(cls, "execute", None)) for cls in task_classes)
    assert not any(callable(getattr(cls, "no_such_command", None)) for cls in task_classes)


def test_bindings_for_unknown_type_is_empty() -> None:
    bindings = control_task_registry.bindings_for("definitely_not_registered")
    assert bindings == TaskBindings()


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
    with pytest.raises(ValueError, match="allowed"):
        reg.register_bindings("fake_stream", consumes={"no_such_port": ("on_x", "broadcast")})
    with pytest.raises(ValueError, match="routing"):
        reg.register_bindings("fake_routing", consumes={"joint_command": ("on_x", "round_robin")})


def test_register_bindings_accepts_twist_card() -> None:
    # twist_command left DEFERRED_STREAMS in B3; a twist card now loads.
    reg = ControlTaskRegistry()
    reg.register_bindings(
        "fake_twist", consumes={"twist_command": ("on_twist_command", "broadcast")}
    )
    assert reg.bindings_for("fake_twist").consumes == (
        StreamBinding("twist_command", "on_twist_command", Routing.BROADCAST),
    )


def test_register_bindings_rejects_bad_exposes() -> None:
    reg = ControlTaskRegistry()
    with pytest.raises(TypeError, match="sequence"):
        # A bare string is not a command-name sequence.
        reg.register_bindings("fake_str_exposes", exposes="do_thing")
    with pytest.raises(ValueError, match="private"):
        reg.register_bindings("fake_private_exposes", exposes=["_do_thing"])
    with pytest.raises(ValueError, match="more than once"):
        reg.register_bindings("fake_dup_exposes", exposes=["do_thing", "do_thing"])
    with pytest.raises(TypeError, match="non-empty strings"):
        reg.register_bindings("fake_nonstr_exposes", exposes=[123])


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
