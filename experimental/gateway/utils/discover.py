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

"""The discovery scan's child process (`python -m experimental.gateway.utils.discover <command>`), run by discovery.py.

Importing blueprints can hang, crash or print; in a child none of that reaches the gateway. Every answer line is
MARKER + JSON (anything else on stdout is an import's noise):

    names      -> {"blueprints": [every registry and external blueprint], "modules": [every registry module]}
    list       -> {"blueprints": [{name, kind}]}: what `dimos list` prints (blueprints.blueprint_list)
    scan       stdin {"blueprints": [names], "modules": [registry names], "known": [classes]} ->
               {"kind": "start", "name"} before each blueprint, then {"kind": "blueprint", ...}, and a
               {"kind": "module", ...} for each module class not in `known`; registry modules last
    packages   -> {"python", "environment", "packages": {name: version}, "dimos_requires", "dimos_extras"}
"""

from __future__ import annotations

import dataclasses
import enum
import json
import sys
import traceback
import types
import typing
from typing import Any

from experimental.gateway.utils.introspect import (
    INTERNAL_FIELDS,
    MARKER,
    annotation_name,
    first_line,
    jsonable,
    module_class_by_name,
    robot_of,
    type_name,
)


def emit(value: dict[str, Any]) -> None:
    sys.stdout.write("\n" + MARKER + json.dumps(value, default=str) + "\n")
    sys.stdout.flush()


def missing_module(error: BaseException) -> str | None:
    """The module a ModuleNotFoundError (anywhere in the cause chain) couldn't find, top-level name."""
    seen: BaseException | None = error
    while seen is not None:
        if isinstance(seen, ModuleNotFoundError) and seen.name:
            return seen.name.split(".")[0]
        seen = seen.__cause__ or seen.__context__
    return None


def describe_error(error: BaseException) -> dict[str, Any]:
    lines = traceback.format_exception(type(error), error, error.__traceback__)
    return {
        "import_error": f"{type(error).__name__}: {error}",
        "import_traceback": "".join(lines)[-4000:],
        "missing_module": missing_module(error),
    }


def enum_values(annotation: Any) -> list[Any] | None:
    """An Enum's values or a Literal's options, also inside Optional/Union (a UI offers them as a list)."""
    if isinstance(annotation, type) and issubclass(annotation, enum.Enum):
        return [jsonable(member.value) for member in annotation]
    origin = typing.get_origin(annotation)
    if origin is typing.Literal:
        return [jsonable(option) for option in typing.get_args(annotation)]
    if origin in (typing.Union, types.UnionType):
        found: list[Any] = []
        for arg in typing.get_args(annotation):
            values = enum_values(arg)
            if values:
                found += [value for value in values if value not in found]
        return found or None
    return None


def json_check(annotation: Any, default: Any, required: bool) -> tuple[bool, str | None, Any]:
    """(the field converts to and from JSON, why not, its default as JSON).

    Compatible means pydantic can write a JSON Schema for the type and the default survives dump -> JSON -> validate
    unchanged, or is plain JSON already (pydantic doesn't validate defaults: `ip: str = None` is common); anything
    else (a class, a callable, a numpy array) is not something a form can edit."""
    from pydantic import TypeAdapter

    try:
        adapter: TypeAdapter[Any] = TypeAdapter(annotation)
        adapter.json_schema()
    except Exception as error:
        return False, f"its type has no JSON form ({type(error).__name__})", jsonable(default)
    if required:
        return True, None, None
    try:
        if json.loads(json.dumps(default)) == default:
            return True, None, default
    except (TypeError, ValueError):
        pass
    try:
        dumped = adapter.dump_python(default, mode="json")
        back = adapter.validate_python(json.loads(json.dumps(dumped)))
        if back != default:
            return False, "its default changes in a JSON round trip", jsonable(default)
        return True, None, dumped
    except Exception as error:
        return (
            False,
            f"its default doesn't convert to JSON ({type(error).__name__})",
            jsonable(default),
        )


def config_fields(module: type) -> list[dict[str, Any]]:
    """Every field of a module's `config` class (pydantic model or dataclass), except ModuleConfig's internals."""
    kind: Any = typing.get_type_hints(module).get("config")
    try:
        from dimos.core.module import ModuleConfig

        base = set(ModuleConfig.model_fields)
    except Exception:
        base = set()
    rows: list[tuple[str, Any, Any, bool, str | None]] = []
    if hasattr(kind, "model_fields"):
        for name, field in kind.model_fields.items():
            required = field.is_required()
            if field.default_factory is not None:
                try:
                    default = field.default_factory()  # type: ignore[call-arg]
                except Exception:
                    default = None
            else:
                default = None if required else field.default
            rows.append((name, field.annotation, default, required, field.description))
    elif isinstance(kind, type) and dataclasses.is_dataclass(kind):
        hints = typing.get_type_hints(kind)
        for item in dataclasses.fields(kind):
            required = (
                item.default is dataclasses.MISSING and item.default_factory is dataclasses.MISSING
            )
            if item.default_factory is not dataclasses.MISSING:
                default = item.default_factory()
            else:
                default = None if required else item.default
            description = item.metadata.get("description") if item.metadata else None
            rows.append(
                (item.name, hints.get(item.name, item.type), default, required, description)
            )
    else:
        raise TypeError(
            f"{type_name(module)}.config ({annotation_name(kind)}) isn't a pydantic model or dataclass"
        )
    fields = []
    for name, annotation, default, required, description in rows:
        if name in INTERNAL_FIELDS:
            continue
        compatible, reason, as_json = json_check(annotation, default, required)
        fields.append(
            {
                "name": name,
                "type": annotation_name(annotation),
                "default": as_json,
                "description": description,
                "required": required,
                "base": name in base,
                "enum": enum_values(annotation),
                "json_compatible": compatible,
                "reason": reason,
            }
        )
    return fields


def stream_refs(module: type) -> list[Any]:
    from dimos.core.coordination.blueprints import BlueprintAtom

    return list(BlueprintAtom.create(module, {}).streams)  # type: ignore[arg-type]


def module_record(name: str, module: type) -> dict[str, Any]:
    record: dict[str, Any] = {
        "kind": "module",
        "name": name,
        "class": f"{module.__module__}.{module.__qualname__}",
        # its own docstring only: an inherited one describes the base class
        "doc": first_line(module) if module.__dict__.get("__doc__") else "",
    }
    try:
        refs = stream_refs(module)
        record["inputs"] = [
            {"name": s.name, "type": type_name(s.type)} for s in refs if s.direction != "out"
        ]
        record["outputs"] = [
            {"name": s.name, "type": type_name(s.type)} for s in refs if s.direction != "in"
        ]
    except Exception as error:
        record["inputs"], record["outputs"] = [], []
        record["error"] = f"streams: {type(error).__name__}: {error}"
    try:
        record["skills"] = sorted(
            skill
            for skill, fn in dict(getattr(module, "rpcs", {}) or {}).items()
            if getattr(fn, "__skill__", False)
        )
    except Exception:
        record["skills"] = []
    try:
        record["config"] = config_fields(module)
    except Exception as error:
        record["config"] = []
        record["config_error"] = f"{type(error).__name__}: {error}"
    return record


def spec_topic(spec: Any) -> str | None:
    """The topic a blueprint's transport_map pins (a TransportSpec's first str arg / `topic`, or a transport's)."""
    kwargs = getattr(spec, "kwargs", None)
    if isinstance(kwargs, dict) and isinstance(kwargs.get("topic"), str):
        return str(kwargs["topic"])
    args = getattr(spec, "args", None)
    if isinstance(args, tuple) and args and isinstance(args[0], str):
        return args[0]
    topic = getattr(spec, "topic", None)
    topic = getattr(topic, "topic", topic)
    return topic if isinstance(topic, str) else None


def blueprint_modules(bp: Any, registry_names: dict[str, str]) -> list[dict[str, Any]]:
    """Its modules with their streams, each stream's topic worked out as the coordinator does: a transport_map pin,
    else `/<name>` when that name is unique in the blueprint, else null (a random topic at run time)."""
    atoms = list(bp.active_blueprints)
    remap = dict(bp.remapping_map)
    pinned = dict(bp.transport_map)
    wired: dict[str, set[Any]] = {}
    for atom in atoms:
        for stream in atom.streams:
            effective = remap.get((atom.name, stream.name), stream.name)
            if isinstance(effective, str):
                wired.setdefault(effective, set()).add(stream.type)
    modules = []
    for atom in atoms:
        streams = []
        for stream in atom.streams:
            effective = remap.get((atom.name, stream.name), stream.name)
            topic = None
            if isinstance(effective, str):
                spec = pinned.get((effective, stream.type))
                if spec is not None:
                    topic = spec_topic(spec)
                elif len(wired.get(effective, ())) == 1:
                    topic = f"/{effective}"
            streams.append(
                {
                    "name": stream.name,
                    "type": type_name(stream.type),
                    "direction": stream.direction,
                    "topic": topic,
                }
            )
        key = f"{atom.module.__module__}.{atom.module.__qualname__}"
        modules.append(
            {
                "name": atom.name,
                "class": key,
                "module": registry_names.get(key, atom.module.__name__),
                "streams": streams,
            }
        )
    return modules


def scan(request: dict[str, Any]) -> None:
    from dimos.robot.all_blueprints import all_blueprints, all_modules
    from dimos.robot.get_all_blueprints import (
        OptionalDependencyError,
        get_by_name,
        load_blueprint,
    )

    known = set(request.get("known", []))
    registry_names = {path: name for name, path in all_modules.items()}

    def module_seen(module: type) -> None:
        key = f"{module.__module__}.{module.__qualname__}"
        if key not in known:
            known.add(key)
            emit(module_record(registry_names.get(key, module.__name__), module))

    for name in request.get("blueprints", []):
        emit({"kind": "start", "name": name})
        ref = all_blueprints.get(name)
        record: dict[str, Any] = {
            "kind": "blueprint",
            "name": name,
            "ref": ref,
            "builtin": ref is not None,
            "robot": robot_of(ref, name) if ref else None,
        }
        try:
            bp = load_blueprint(name) if ref is not None else get_by_name(name)
            record["modules"] = blueprint_modules(bp, registry_names)
            emit(
                {
                    **record,
                    "importable": True,
                    "optional_dependency": False,
                    "import_error": None,
                    "import_traceback": None,
                    "missing_module": None,
                }
            )
            for atom in bp.active_blueprints:
                module_seen(atom.module)
        except BaseException as error:  # SystemExit from an import counts too
            emit(
                {
                    **record,
                    "importable": False,
                    "modules": [],
                    "optional_dependency": isinstance(error, OptionalDependencyError),
                    **describe_error(error),
                }
            )
    for name in request.get("modules", []):
        emit({"kind": "start", "name": f"module:{name}"})
        try:
            module_seen(module_class_by_name(name))
        except BaseException as error:
            emit(
                {
                    "kind": "module-error",
                    "name": name,
                    "class": all_modules.get(name),
                    "unknown": name not in all_modules,
                    **describe_error(error),
                }
            )


def names() -> dict[str, Any]:
    from dimos.robot.all_blueprints import all_blueprints, all_modules

    external: list[str] = []
    errors: list[str] = []
    try:
        from dimos.robot.external_blueprints import list_external_blueprint_names

        external = list_external_blueprint_names()
    except Exception as error:
        errors.append(f"external blueprints: {type(error).__name__}: {error}")
    return {
        "blueprints": sorted(all_blueprints)
        + [name for name in external if name not in all_blueprints],
        "modules": sorted(all_modules),
        "errors": errors,
    }


def packages() -> dict[str, Any]:
    from importlib.metadata import PackageNotFoundError, distributions, metadata, requires

    from packaging.markers import default_environment
    from packaging.utils import canonicalize_name

    found: dict[str, str] = {}
    for dist in distributions():
        name = dist.metadata["Name"]
        if name:
            found[canonicalize_name(name)] = dist.version
    try:
        dimos_requires = requires("dimos") or []
        dimos_extras = metadata("dimos").get_all("Provides-Extra") or []
    except PackageNotFoundError:
        dimos_requires, dimos_extras = [], []
    return {
        "python": sys.executable,
        "environment": dict(default_environment()),
        "packages": found,
        "dimos_requires": dimos_requires,
        "dimos_extras": dimos_extras,
    }


def main(argv: list[str]) -> None:
    command = argv[0] if argv else ""
    try:
        if command == "names":
            emit(names())
        elif command == "scan":
            scan(json.loads(sys.stdin.read() or "{}"))
            emit({"kind": "end"})
        elif command == "packages":
            emit(packages())
        elif command == "list":
            from experimental.gateway.utils.blueprints import blueprint_list

            emit({"blueprints": blueprint_list()})
        else:
            emit({"error": f"unknown command {command!r}"})
    except BaseException as error:
        emit({"kind": "error", "error": f"{type(error).__name__}: {error}"})


if __name__ == "__main__":
    main(sys.argv[1:])
