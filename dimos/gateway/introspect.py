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

"""Blueprint introspection, run by the dimos gateway in a child process (`python -m dimos.gateway.introspect ...`).

Importing a blueprint can take seconds, pull in GPU libraries or crash outright; in a child (with a timeout) none of
that reaches the gateway. The answer is the last stdout line, after MARKER (importing dimos can print).

    blueprint <name>  -> {"name", "file", "line", "modules": [{"name", "class", "doc", "summary", "file", "line", "rpcs",
                          "skills", "streams": [{"name", "type", "direction", "topic"}]}]}
    config <name>     -> {"name", "modules": [{"module", "class", "args": [...], "error"?}]}
    catalog           -> {"blueprints", "modules", "skills", "errors"} (slow: imports every blueprint)
    anything failing  -> {"error": "<Type>: <message>"}
"""

from __future__ import annotations

from collections.abc import Callable
import enum
import functools
import importlib
import inspect
import json
import re
import sys
import typing
from typing import Any

MARKER = "@@DIMOS_SERVER@@"

# the catalog lists this many import errors, then how many more (a broken install fails most blueprints the same way)
MAX_ERRORS = 50

# ModuleConfig fields every module has that aren't for a person to set (test_introspect checks every ModuleConfig
# field is either here or in SHOWN_BASE_FIELDS, so a new one is a decision, not an accident)
INTERNAL_FIELDS = {"g", "rpc_transport", "rpc_timeouts", "default_rpc_timeout", "instance_name"}
SHOWN_BASE_FIELDS = {"frame_id", "frame_id_prefix"}


def type_name(kind: Any) -> str:
    module = getattr(kind, "__module__", "")
    name = getattr(kind, "__qualname__", None) or getattr(kind, "__name__", None) or repr(kind)
    return str(name) if module in ("builtins", "") else f"{module}.{name}"


def atoms_of(name: str) -> list[Any]:
    """The modules `dimos run <name>` would start: a built-in blueprint, a module's own blueprint, or an external
    `namespace.name`, resolved the way dimos resolves them."""
    from dimos.robot.get_all_blueprints import get_by_name

    return list(get_by_name(name).active_blueprints)


def blueprint(name: str) -> dict[str, Any]:
    from dimos.gateway.discover import blueprint_modules
    from dimos.robot.get_all_blueprints import get_by_name

    bp = get_by_name(name)
    wired = blueprint_modules(bp, {})
    modules = []
    for atom, entry in zip(bp.active_blueprints, wired, strict=True):
        doc = own_doc(atom.module)
        modules.append(
            {
                "name": atom.name,
                "class": type_name(atom.module),
                "doc": doc,
                "summary": doc.split("\n\n")[0].replace("\n", " ")[:300],
                "streams": entry["streams"],
                **methods(atom.module),
                **source_of(atom.module),
            }
        )
    return {"name": name, **blueprint_source(name), "modules": modules}


def blueprint_source(name: str) -> dict[str, Any]:
    """Where a blueprint is defined: the file and line of its `<attr> = ...` (a module run as a blueprint: its class);
    None for an external one."""
    import ast

    from dimos.robot.all_blueprints import all_blueprints, all_modules

    none: dict[str, Any] = {"file": None, "line": None}
    if name in all_modules:
        path, _, attr = all_modules[name].rpartition(".")
        try:
            return source_of(getattr(importlib.import_module(path), attr))
        except Exception:
            return none
    if name not in all_blueprints:
        return none
    path, _, attr = all_blueprints[name].partition(":")
    try:
        file = inspect.getsourcefile(importlib.import_module(path))
        if file is None:
            return none
        with open(file, encoding="utf-8") as source:
            tree = ast.parse(source.read())
    except Exception:
        return none
    line = next(
        (
            node.lineno
            for node in tree.body
            if (
                isinstance(node, ast.Assign)
                and any(getattr(t, "id", None) == attr for t in node.targets)
            )
            or (isinstance(node, ast.AnnAssign) and getattr(node.target, "id", None) == attr)
        ),
        1,
    )
    return {"file": relative_to_checkout(file), "line": line}


def own_doc(cls: Any) -> str:
    """A class's docstring, from it or the nearest base that isn't dimos's own Module plumbing (whose docstring
    describes every module, not this one)."""
    for base in getattr(cls, "__mro__", (cls,)):
        if str(getattr(base, "__module__", "")).startswith("dimos.core."):
            break
        doc = base.__dict__.get("__doc__")
        if isinstance(doc, str) and doc.strip():
            return inspect.cleandoc(doc)
    return ""


@functools.cache
def base_rpcs() -> frozenset[str]:
    """The RPCs every module inherits (build, start, stop, ...): not worth listing on each one."""
    try:
        from dimos.core.module import Module

        return frozenset(dict(Module.rpcs))
    except Exception:
        return frozenset()


def return_type(fn: Any) -> str | None:
    try:
        annotation = inspect.signature(fn).return_annotation
    except (TypeError, ValueError):
        return None
    if annotation is inspect.Signature.empty:
        return None
    return str(getattr(annotation, "__name__", annotation)).replace("typing.", "")


def methods(cls: Any) -> dict[str, list[dict[str, Any]]]:
    """A module's RPC methods and skills (a skill is an RPC an agent can call), each with its signature and docstring;
    the RPCs every module inherits are left out."""
    rpcs: list[dict[str, Any]] = []
    skills: list[dict[str, Any]] = []
    try:
        found = sorted(dict(getattr(cls, "rpcs", {}) or {}).items())
    except Exception:
        found = []
    for name, fn in found:
        skill = bool(getattr(fn, "__skill__", False))
        if not skill and name in base_rpcs():
            continue
        entry = {
            "name": name,
            "params": params(fn),
            "return_type": return_type(fn),
            "doc": inspect.getdoc(fn) or "",
        }
        (skills if skill else rpcs).append(entry)
    return {"rpcs": rpcs, "skills": skills}


def source_of(obj: Any) -> dict[str, Any]:
    """Where a class is defined: its file (relative to the dimos checkout it was imported from, when inside it) and
    first line."""
    try:
        file = inspect.getsourcefile(obj)
        line = inspect.getsourcelines(obj)[1]
    except (TypeError, OSError):
        return {"file": None, "line": None}
    if file is None:
        return {"file": None, "line": None}
    return {"file": relative_to_checkout(file), "line": line}


def relative_to_checkout(file: str) -> str:
    """A file relative to the dimos checkout it was imported from, when inside it; else absolute."""
    from pathlib import Path

    import dimos

    path = Path(file).resolve()
    root = Path(dimos.__file__).resolve().parents[1]
    return str(path.relative_to(root)) if path.is_relative_to(root) else str(path)


def jsonable(value: Any) -> Any:
    def text(obj: Any) -> Any:
        if isinstance(obj, enum.Enum):
            return obj.value
        # a repr without its memory address (it changes every run)
        return re.sub(r" at 0x[0-9a-fA-F]+", "", str(obj))

    try:
        return json.loads(json.dumps(value, default=text))
    except Exception:
        return text(repr(value))


def annotation_name(kind: Any) -> str | None:
    if kind is None:
        return None
    if isinstance(kind, type):
        return type_name(kind)
    return str(kind).replace("typing.", "")


def choices(kind: Any) -> list[Any] | None:
    """An Enum's values or a Literal's options (a UI can offer them as a list)."""
    if isinstance(kind, type) and issubclass(kind, enum.Enum):
        return [jsonable(member.value) for member in kind]
    if typing.get_origin(kind) is typing.Literal:
        return [jsonable(option) for option in typing.get_args(kind)]
    return None


def field_schema(kind: Any) -> dict[str, Any] | None:
    """A field's JSON Schema (self-contained: its $refs point into its own $defs), or None when its values aren't JSON
    (a callable, an object) or it is a nested model (`dimos run` sets those field by field, not as a whole)."""
    from pydantic import BaseModel, TypeAdapter

    def has_model(annotation: Any) -> bool:
        if isinstance(annotation, type) and issubclass(annotation, BaseModel):
            return True
        return any(has_model(arg) for arg in typing.get_args(annotation))

    if has_model(kind):
        return None
    try:
        return TypeAdapter(kind).json_schema()
    except Exception:
        return None


def json_safe(kind: Any, value: Any) -> bool:
    """Whether a field's default survives a trip through JSON (so a value read from a form means the same)."""
    from pydantic import TypeAdapter

    try:
        json.dumps(TypeAdapter(kind).dump_python(value, mode="json"))
        return True
    except Exception:
        return False


def config_args(module: Any, overrides: dict[str, Any]) -> list[dict[str, Any]]:
    """A module class's config fields: name, type, default, description, and the blueprint's value if it sets one."""
    kind = typing.get_type_hints(module).get("config")
    fields = getattr(kind, "model_fields", None)
    if fields is None:
        raise TypeError(
            f"{type_name(module)}.config ({annotation_name(kind)}) isn't a pydantic model"
        )
    try:
        from dimos.core.module import ModuleConfig

        base = set(ModuleConfig.model_fields)
    except Exception:
        base = set()
    args = []
    for name, field in fields.items():
        if name in INTERNAL_FIELDS:
            continue
        required = field.is_required()
        if field.default_factory is not None:
            try:
                default = field.default_factory()
            except Exception:
                default = None
        else:
            default = None if required else field.default
        entry = {
            "name": name,
            "type": annotation_name(field.annotation),
            "default": jsonable(default),
            "description": field.description,
            "required": required,
            "base": name in base,
        }
        options = choices(field.annotation)
        if options is not None:
            entry["choices"] = options
        schema = field_schema(field.annotation)
        entry["json_compatible"] = schema is not None and json_safe(field.annotation, default)
        if schema is not None:
            entry["schema"] = schema
        if name in overrides:
            entry["value"] = jsonable(overrides[name])
        args.append(entry)
    return args


def config(name: str) -> dict[str, Any]:
    modules = []
    for atom in atoms_of(name):
        entry: dict[str, Any] = {"module": atom.name, "class": type_name(atom.module)}
        try:
            entry["args"] = config_args(atom.module, dict(atom.kwargs))
        except Exception as error:
            entry["args"] = []
            entry["error"] = f"{type(error).__name__}: {error}"
        modules.append(entry)
    return {"name": name, "modules": modules}


def robot_of(ref: str, name: str | None = None) -> str | None:
    """The robot a blueprint is for, from robots.json: the robot that lists it, else the one whose `dirs` hold its file
    (dimos.robot.unitree.go2.blueprints.basic:x -> "go2"); None for one no robot claims (a demo)."""
    from dimos.gateway import robots

    doc = _robots()
    if name is not None:
        for robot_id, robot in doc["robots"].items():
            if name in robot["blueprints"]:
                return str(robot_id)
    return robots.owner_of(doc, robots.blueprint_file(ref))


@functools.cache
def _robots() -> dict[str, Any]:
    from dimos.gateway import robots

    return robots.load()


def first_line(obj: Any) -> str:
    doc = inspect.getdoc(obj) or ""
    return doc.strip().split("\n\n")[0].replace("\n", " ")[:300]


def params(fn: Any) -> list[dict[str, Any]]:
    try:
        signature = inspect.signature(fn)
    except (TypeError, ValueError):
        return []
    return [
        {
            "name": name,
            "type": None
            if param.annotation is inspect.Parameter.empty
            else getattr(param.annotation, "__name__", str(param.annotation)),
            "default": None if param.default is inspect.Parameter.empty else repr(param.default),
        }
        for name, param in signature.parameters.items()
        if name not in ("self", "cls")
    ]


def catalog() -> dict[str, Any]:
    """Every blueprint, module and skill (for a launcher); one that fails to import is listed in `errors`."""
    from dimos.robot.all_blueprints import all_blueprints, all_modules
    from dimos.robot.get_all_blueprints import get_blueprint_by_name

    errors: list[str] = []
    classes: dict[str, tuple[str, Any]] = {}
    robots: dict[str, set[str]] = {}
    blueprints = []
    # a blueprint file's docstring describes its blueprint only when the file defines just that one
    per_file: dict[str, int] = {}
    for ref in all_blueprints.values():
        per_file[ref.split(":")[0]] = per_file.get(ref.split(":")[0], 0) + 1
    for name in sorted(all_modules):
        try:
            cls = module_class_by_name(name)
            classes[f"{cls.__module__}.{cls.__qualname__}"] = (name, cls)
        except Exception as error:
            errors.append(f"module {name}: {type(error).__name__}: {error}")
    for name, ref in sorted(all_blueprints.items()):
        try:
            bp = get_blueprint_by_name(name)
            robot = robot_of(ref, name)
            ids = []
            for atom in bp.active_blueprints:
                key = f"{atom.module.__module__}.{atom.module.__qualname__}"
                classes.setdefault(key, (atom.module.__name__, atom.module))
                ids.append(classes[key][0])
                if robot:
                    robots.setdefault(key, set()).add(robot)
            file = ref.split(":")[0]
            doc = first_line(importlib.import_module(file)) if per_file.get(file) == 1 else ""
            blueprints.append(
                {"name": name, "ref": ref, "robot": robot, "modules": ids, "doc": doc}
            )
        except Exception as error:
            errors.append(f"blueprint {name}: {type(error).__name__}: {error}")
    modules, skills = [], []
    for key, (module_id, cls) in sorted(classes.items(), key=lambda item: item[1][0]):
        try:
            info = cls.module_info()
            module_skills = [
                {"name": skill_name, "doc": first_line(fn), "params": params(fn)}
                for skill_name, fn in sorted(dict(cls.rpcs).items())
                if getattr(fn, "__skill__", False)
            ]
            entry = {
                "name": module_id,
                "class": key,
                "doc": first_line(cls),
                "robots": sorted(robots.get(key, ())),
                "inputs": [{"name": s.name, "type": s.type_name} for s in info.inputs],
                "outputs": [{"name": s.name, "type": s.type_name} for s in info.outputs],
                "skills": [s["name"] for s in module_skills],
            }
            modules.append(entry)
            skills += [
                {**skill, "module": module_id, "robots": entry["robots"]} for skill in module_skills
            ]
        except Exception as error:
            errors.append(f"module {module_id}: {type(error).__name__}: {error}")
    if len(errors) > MAX_ERRORS:
        errors = [*errors[:MAX_ERRORS], f"... and {len(errors) - MAX_ERRORS} more"]
    return {"blueprints": blueprints, "modules": modules, "skills": skills, "errors": errors}


COMMANDS: dict[str, Callable[..., dict[str, Any]]] = {
    "blueprint": blueprint,
    "config": config,
    "catalog": catalog,
}


def main(argv: list[str]) -> None:
    try:
        result = COMMANDS[argv[0]](*argv[1:])
    except Exception as error:
        result = {"error": f"{type(error).__name__}: {error}"}
    sys.stdout.write("\n" + MARKER + json.dumps(result, default=str) + "\n")
    sys.stdout.flush()


if __name__ == "__main__":
    main(sys.argv[1:])


def module_class_by_name(name: str) -> Any:
    """A registered module's class (all_modules' dotted path), imported; KeyError for a name it doesn't have."""
    from dimos.robot.all_blueprints import all_modules

    module_path, class_name = all_modules[name].rsplit(".", 1)
    return getattr(importlib.import_module(module_path), class_name)
