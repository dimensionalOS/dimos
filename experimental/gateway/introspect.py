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

import asyncio
from collections import Counter, defaultdict
from collections.abc import Awaitable, Callable
import contextlib
import enum
import functools
import importlib
import inspect
import json
import os
from pathlib import Path
import re
import signal
import sys
import time
import typing
from typing import Any

MARKER = "@@DIMOS_GATEWAY@@"


class IntrospectError(Exception):
    pass


class Cache:
    """One shared computation per key, kept for `ttl` seconds from its start; a failure isn't kept."""

    def __init__(self) -> None:
        self.entries: dict[str, tuple[float, asyncio.Future[Any]]] = {}

    def forget(self, key: str) -> None:
        self.entries.pop(key, None)

    async def get(self, key: str, ttl: float, compute: Callable[[], Awaitable[Any]]) -> Any:
        # keys include request args, so expired entries go or they pile up for days
        now = time.monotonic()
        self.entries = {
            cache_key: entry for cache_key, entry in self.entries.items() if entry[0] > now
        }
        if key not in self.entries:
            self.entries[key] = (now + ttl, asyncio.ensure_future(compute()))
        task = self.entries[key][1]
        try:
            return await asyncio.shield(task)
        except Exception:
            if key in self.entries and self.entries[key][1] is task:
                del self.entries[key]
            raise


async def run_child(
    dimos_dir: Path,
    args: list[str],
    stdin: Any = None,
    timeout: float | None = None,
    python: str | None = None,
) -> dict[str, Any]:
    """Introspect in dimos_dir's python, or (python given: another checkout, a run's) with this file in that one."""
    # imported here: this file also runs alone in checkouts whose gateway may have no constants.py
    from experimental.gateway import constants

    if timeout is None:
        timeout = constants.INTROSPECT_TIMEOUT_S
    child = await asyncio.create_subprocess_exec(
        *(child_command(dimos_dir) if python is None else foreign_command(python)),
        *args,
        cwd=dimos_dir,
        stdin=asyncio.subprocess.PIPE,
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE,
        start_new_session=True,
    )
    try:
        stdout, stderr = await asyncio.wait_for(
            child.communicate(json.dumps(stdin).encode()), timeout
        )
    except asyncio.TimeoutError:
        kill_group(child.pid)
        await child.wait()
        raise IntrospectError(f"introspecting {' '.join(args)} took over {timeout:.0f} s")
    _, found, answer = stdout.decode("utf-8", "replace").rpartition(MARKER)
    if not found:
        tail = (stderr.decode("utf-8", "replace").strip().splitlines() or ["no output"])[-1]
        raise IntrospectError(f"introspection failed (exit {child.returncode}): {tail}")
    value: dict[str, Any] = json.loads(answer.strip().splitlines()[0])
    if isinstance(value.get("error"), str):
        raise IntrospectError(value["error"])
    return value


def child_command(dimos_dir: Path) -> list[str]:
    from experimental.gateway.store import python_for

    return [python_for(dimos_dir), "-m", "experimental.gateway.introspect"]


def foreign_command(python: str) -> list[str]:
    # the checkout may have no gateway or an older one, so this file runs there, importing that checkout's dimos;
    # the gateway's checkout goes last on the path, only for the experimental.gateway helpers that one lacks
    run = (
        "import runpy, sys\n"
        "sys.argv.pop(0)\n"
        "sys.path.append(sys.argv.pop(0))\n"
        # a startup hook (a sitecustomize, a venv .pth) may have imported another checkout's dimos already
        "for name in [loaded for loaded in sys.modules if loaded == 'dimos' or loaded.startswith('dimos.')]:\n"
        "    del sys.modules[name]\n"
        "runpy.run_path(sys.argv[0], run_name='__main__')\n"
    )
    here = Path(__file__).resolve()
    return [python, "-c", run, str(here.parents[2]), str(here)]


def kill_group(pid: int) -> None:
    with contextlib.suppress(ProcessLookupError, PermissionError):
        os.killpg(pid, signal.SIGKILL)


def emit(value: dict[str, Any]) -> None:
    sys.stdout.write("\n" + MARKER + json.dumps(value, default=str) + "\n")
    sys.stdout.flush()


def type_name(kind: Any) -> str:
    module = getattr(kind, "__module__", "")
    name = getattr(kind, "__qualname__", None) or getattr(kind, "__name__", None) or repr(kind)
    return str(name) if module in ("builtins", "") else f"{module}.{name}"


def jsonable(value: Any) -> Any:
    def text(unencodable: Any) -> Any:
        if isinstance(unencodable, enum.Enum):
            return unencodable.value
        return re.sub(r" at 0x[0-9a-fA-F]+", "", str(unencodable))

    try:
        return json.loads(json.dumps(value, default=text))
    except (TypeError, ValueError):
        return text(repr(value))


def first_line(target: Any) -> str:
    doc = inspect.getdoc(target) or ""
    return doc.strip().split("\n\n")[0].replace("\n", " ")[:300]


def own_doc(cls: Any) -> str:
    for base in getattr(cls, "__mro__", (cls,)):
        if str(getattr(base, "__module__", "")).startswith("dimos.core."):
            break
        doc = base.__dict__.get("__doc__")
        if isinstance(doc, str) and doc.strip():
            return inspect.cleandoc(doc)
    return ""


def relative_to_checkout(file: str) -> str:
    import dimos

    path = Path(file).resolve()
    root = Path(dimos.__file__).resolve().parents[1]
    return str(path.relative_to(root)) if path.is_relative_to(root) else str(path)


def source_of(target: Any) -> dict[str, Any]:
    try:
        file = inspect.getsourcefile(target)
        line = inspect.getsourcelines(target)[1]
    except (TypeError, OSError):
        return {"file": None, "line": None}
    return {"file": relative_to_checkout(file) if file else None, "line": line if file else None}


def blueprint_source(name: str) -> dict[str, Any]:
    from dimos.robot.all_blueprints import all_blueprints, all_modules

    try:
        if name in all_modules:
            path, _, attribute = all_modules[name].rpartition(".")
            return source_of(getattr(importlib.import_module(path), attribute))
        path, _, attribute = all_blueprints[name].partition(":")
        module = importlib.import_module(path)
        lines = inspect.getsource(module).splitlines()
    except Exception:  # importing a blueprint runs its code, which can raise anything
        return {"file": None, "line": None}
    assignment = re.compile(rf"{re.escape(attribute)}\s*(:|=(?!=))")
    line = next((number for number, text in enumerate(lines, 1) if assignment.match(text)), 1)
    return {"file": relative_to_checkout(inspect.getfile(module)), "line": line}


def params(function: Any) -> list[dict[str, Any]]:
    try:
        signature = inspect.signature(function)
    except (TypeError, ValueError):
        return []
    return [
        {
            "name": name,
            "type": None
            if param.annotation is inspect.Parameter.empty
            else getattr(param.annotation, "__name__", str(param.annotation)),
            "default": None if param.default is inspect.Parameter.empty else repr(param.default),
            "required": param.default is inspect.Parameter.empty,
            "choices": choices(param.annotation),
        }
        for name, param in signature.parameters.items()
        if name not in ("self", "cls")
    ]


def return_type(function: Any) -> str | None:
    try:
        annotation = inspect.signature(function).return_annotation
    except (TypeError, ValueError):
        return None
    if annotation is inspect.Signature.empty:
        return None
    return str(getattr(annotation, "__name__", annotation)).replace("typing.", "")


@functools.cache
def base_rpcs() -> frozenset[str]:
    from dimos.core.module import Module

    return frozenset(dict(Module.rpcs))


def rpcs(cls: Any) -> list[dict[str, Any]]:
    """The module's own RPCs (not every Module's), each saying whether it's also a skill."""
    found = []
    for name, function in sorted(dict(getattr(cls, "rpcs", {}) or {}).items()):
        skill = bool(getattr(function, "__skill__", False))
        if skill or name not in base_rpcs():
            found.append(
                {
                    "name": name,
                    "params": params(function),
                    "return_type": return_type(function),
                    "doc": inspect.getdoc(function) or "",
                    "skill": skill,
                }
            )
    return found


def spec_topic(spec: Any) -> str | None:
    kwargs = getattr(spec, "kwargs", None)
    if isinstance(kwargs, dict) and isinstance(kwargs.get("topic"), str):
        return str(kwargs["topic"])
    args = getattr(spec, "args", None)
    topic = args[0] if isinstance(args, tuple) and args else getattr(spec, "topic", None)
    # a Topic object (LCM/Zenoh specs) carries the string; Zenoh's has its "dimos/" namespace
    topic = getattr(topic, "topic", topic)
    if not isinstance(topic, str):
        return None
    return "/" + topic.removeprefix("dimos/").lstrip("/")


def blueprint_modules(blueprint: Any, registry_names: dict[str, str]) -> list[dict[str, Any]]:
    remap, pinned = dict(blueprint.remapping_map), dict(blueprint.transport_map)
    wired: dict[str, set[Any]] = defaultdict(set)
    for atom in blueprint.active_blueprints:
        for stream in atom.streams:
            effective = remap.get((atom.name, stream.name), stream.name)
            if isinstance(effective, str):
                wired[effective].add(stream.type)

    def wire(atom: Any, stream: Any) -> str | None:
        effective = remap.get((atom.name, stream.name), stream.name)
        return effective if isinstance(effective, str) else None

    def topic(atom: Any, stream: Any) -> str | None:
        effective = wire(atom, stream)
        if effective is None:
            return None
        spec = pinned.get((effective, stream.type))
        if spec is not None:
            return spec_topic(spec)
        return f"/{effective}" if len(wired[effective]) == 1 else None

    return [
        {
            "name": atom.name,
            "class": type_name(atom.module),
            "module": registry_names.get(type_name(atom.module), atom.module.__name__),
            "streams": [
                {
                    "name": stream.name,
                    "type": type_name(stream.type),
                    "direction": stream.direction,
                    # the name it's wired under after .remappings(): streams sharing it are one topic
                    "wire": wire(atom, stream),
                    "topic": topic(atom, stream),
                }
                for stream in atom.streams
            ],
        }
        for atom in blueprint.active_blueprints
    ]


def blueprint_detail(name: str) -> dict[str, Any]:
    from dimos.robot.get_all_blueprints import get_by_name

    blueprint = get_by_name(name)
    modules = []
    for atom, entry in zip(
        blueprint.active_blueprints, blueprint_modules(blueprint, {}), strict=True
    ):
        doc = own_doc(atom.module)
        modules.append(
            {
                "name": atom.name,
                "class": type_name(atom.module),
                "doc": doc,
                "summary": doc.split("\n\n")[0].replace("\n", " ")[:300],
                "streams": entry["streams"],
                "rpcs": rpcs(atom.module),
                **source_of(atom.module),
            }
        )
    return {"name": name, **blueprint_source(name), "modules": modules}


def choices(kind: Any) -> list[Any] | None:
    if isinstance(kind, type) and issubclass(kind, enum.Enum):
        return [jsonable(member.value) for member in kind]
    if typing.get_origin(kind) is typing.Literal:
        return [jsonable(option) for option in typing.get_args(kind)]
    return None


def field_schema(kind: Any) -> dict[str, Any] | None:
    from pydantic import BaseModel, TypeAdapter
    from pydantic.errors import PydanticUserError

    def has_model(annotation: Any) -> bool:
        if isinstance(annotation, type) and issubclass(annotation, BaseModel):
            return True
        return any(has_model(arg) for arg in typing.get_args(annotation))

    if has_model(kind):
        return None
    try:
        return TypeAdapter(kind).json_schema()
    except (PydanticUserError, TypeError, ValueError):
        return None


def json_safe(kind: Any, value: Any) -> bool:
    from pydantic import TypeAdapter
    from pydantic.errors import PydanticUserError

    try:
        json.dumps(TypeAdapter(kind).dump_python(value, mode="json"))
        return True
    except (PydanticUserError, TypeError, ValueError):
        return False


def field_args(
    fields: dict[str, Any], base: set[str], values: dict[str, Any] | None = None
) -> dict[str, dict[str, Any]]:
    """Each config field's type, default, description and choices, by name; `value` is what the blueprint itself sets."""
    from experimental.gateway.launches import is_secret_name

    args = {}
    for name, field in fields.items():
        required, kind = field.is_required(), field.annotation
        try:
            default = None if required else field.get_default(call_default_factory=True)
        except Exception:  # a default factory is the module's own code
            default = None
        entry = {
            "type": type_name(kind) if isinstance(kind, type) else str(kind).replace("typing.", ""),
            "default": jsonable(default),
            "description": field.description,
            "required": required,
            "secret": is_secret_name(name),
            "base": name in base,
        }
        options = choices(kind)
        if options is not None:
            entry["choices"] = options
        schema = field_schema(kind)
        entry["json_compatible"] = schema is not None and json_safe(kind, default)
        if schema is not None:
            entry["schema"] = schema
        if values and name in values:
            entry["value"] = jsonable(values[name])
        args[name] = entry
    return args


def parser_schema(blueprint: Any) -> Any:
    """dimos run's own view of a blueprint's settings (a private method: dimos has no public one)."""
    from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser

    # TODO(core): BlueprintConfigParser should expose its schema publicly (it carries the parser's skip list)
    return BlueprintConfigParser(blueprint)._get_schema()


def settable(schema: Any, root: str) -> set[str]:
    """The fields `dimos run` takes a flag for under `root` (a module's name, or "g")."""
    return {target.path[0] for target in schema.targets if target.root == root}


def module_args(module: Any, values: dict[str, Any] | None = None) -> dict[str, dict[str, Any]]:
    from dimos.core.module import ModuleConfig

    schema = parser_schema(module.blueprint())
    (only,) = schema.modules
    fields = only.config_cls.model_fields
    keep = settable(schema, only.atom.name)
    return field_args(
        {key: field for key, field in fields.items() if key in keep},
        set(ModuleConfig.model_fields),
        values,
    )


def config(name: str) -> dict[str, Any]:
    """A blueprint's config schema: the GlobalConfig and module fields `dimos run` takes flags for."""
    from dimos.core.global_config import GlobalConfig
    from dimos.core.module import ModuleConfig
    from dimos.robot.get_all_blueprints import get_by_name

    schema = parser_schema(get_by_name(name))
    modules: dict[str, Any] = {}
    for module in schema.modules:
        atom = module.atom
        entry: dict[str, Any] = {"class": type_name(atom.module)}
        try:
            keep = settable(schema, atom.name)
            fields = {
                key: field for key, field in module.config_cls.model_fields.items() if key in keep
            }
            entry["args"] = field_args(fields, set(ModuleConfig.model_fields), dict(atom.kwargs))
        except Exception as error:  # a module's config class is its own code
            entry["args"] = {}
            entry["error"] = f"{type(error).__name__}: {error}"
        modules[atom.name] = entry
    keep = settable(schema, "g")
    return {
        "name": name,
        "global": field_args(
            {key: field for key, field in GlobalConfig.model_fields.items() if key in keep}, set()
        ),
        "modules": modules,
    }


def module_class(name: str) -> tuple[str, Any]:
    """A module by registry name or class path, as (its name in the scan, class)."""
    from dimos.robot.all_blueprints import all_modules

    path = all_modules.get(name, name)
    module_path, _, attribute = path.rpartition(".")
    if not module_path:
        raise KeyError(f"no module {name}")
    cls = getattr(importlib.import_module(module_path), attribute)
    return next(
        (registry_name for registry_name, class_path in all_modules.items() if class_path == path),
        attribute,
    ), cls


def module(name: str) -> dict[str, Any]:
    registry, cls = module_class(name)
    record = module_record(registry, cls)
    return {**{key: value for key, value in record.items() if key != "kind"}, **source_of(cls)}


def module_config(name: str) -> dict[str, Any]:
    registry, cls = module_class(name)
    return {"name": registry, "class": type_name(cls), "args": module_args(cls)}


def blueprint_list() -> dict[str, Any]:
    from dimos.robot.all_blueprints import all_blueprints
    from dimos.robot.external_blueprints import list_external_blueprint_names

    builtin = sorted(all_blueprints)
    return {
        "blueprints": [{"name": name, "kind": "builtin"} for name in builtin]
        + [{"name": name, "kind": "external"} for name in list_external_blueprint_names()]
    }


def missing_module(error: BaseException) -> str | None:
    seen: BaseException | None = error
    while seen is not None:
        if isinstance(seen, ModuleNotFoundError) and seen.name:
            return seen.name.split(".")[0]
        seen = seen.__cause__ or seen.__context__
    return None


def module_record(name: str, cls: Any) -> dict[str, Any]:
    from dimos.core.coordination.blueprints import BlueprintAtom

    streams = list(BlueprintAtom.create(cls, {}).streams)
    return {
        "kind": "module",
        "name": name,
        "class": type_name(cls),
        "doc": first_line(cls),
        "inputs": [
            {"name": stream.name, "type": stream.type.__name__}
            for stream in streams
            if stream.direction != "out"
        ],
        "outputs": [
            {"name": stream.name, "type": stream.type.__name__}
            for stream in streams
            if stream.direction != "in"
        ],
        "rpcs": rpcs(cls),
    }


def scan(request: Any) -> dict[str, Any]:
    from dimos.robot.all_blueprints import all_blueprints, all_modules
    from dimos.robot.external_blueprints import list_external_blueprint_names
    from dimos.robot.get_all_blueprints import OptionalDependencyError, get_by_name, load_blueprint

    request = request if isinstance(request, dict) else {}
    registry_names = {path: name for name, path in all_modules.items()}
    known = set(request.get("known") or [])
    names = request.get("blueprints")
    if names is None:
        names = sorted(all_blueprints) + [
            blueprint_name
            for blueprint_name in list_external_blueprint_names()
            if blueprint_name not in all_blueprints
        ]
        emit({"kind": "names", "blueprints": names, "modules": sorted(all_modules)})
    per_file = Counter(path.split(":")[0] for path in all_blueprints.values())

    def seen(name: str, cls: Any) -> None:
        if type_name(cls) not in known:
            known.add(type_name(cls))
            try:
                emit(module_record(name, cls))
            except Exception as error:
                emit({"kind": "error", "error": f"module {name}: {type(error).__name__}: {error}"})

    for name in names:
        emit({"kind": "start", "name": name})
        ref = all_blueprints.get(name)
        record: dict[str, Any] = {
            "kind": "blueprint",
            "name": name,
            "ref": ref,
            "builtin": ref is not None,
        }
        try:
            blueprint = load_blueprint(name) if ref is not None else get_by_name(name)
            file = ref.split(":")[0] if ref else None
            doc = first_line(importlib.import_module(file)) if file and per_file[file] == 1 else ""
            modules = blueprint_modules(blueprint, registry_names)
            emit(
                {
                    **record,
                    "importable": True,
                    "optional_dependency": False,
                    "import_error": None,
                    "missing_module": None,
                    "modules": modules,
                    "doc": doc,
                }
            )
            for atom in blueprint.active_blueprints:
                seen(registry_names.get(type_name(atom.module), atom.module.__name__), atom.module)
        except BaseException as error:
            emit(
                {
                    **record,
                    "importable": False,
                    "optional_dependency": isinstance(error, OptionalDependencyError),
                    "import_error": f"{type(error).__name__}: {error}",
                    "missing_module": missing_module(error),
                    "modules": [],
                    "doc": "",
                }
            )
    for name in request.get("modules", sorted(all_modules)):
        if all_modules.get(name) in known:
            continue
        emit({"kind": "start", "name": f"module:{name}"})
        try:
            path, _, attribute = all_modules[name].rpartition(".")
            seen(name, getattr(importlib.import_module(path), attribute))
        except BaseException as error:
            emit({"kind": "error", "error": f"module {name}: {type(error).__name__}: {error}"})
    return {"kind": "end"}


def packages() -> dict[str, Any]:
    from importlib.metadata import PackageNotFoundError, distributions, metadata, requires

    from packaging.markers import default_environment
    from packaging.utils import canonicalize_name

    found = {
        canonicalize_name(distribution.metadata["Name"]): distribution.version
        for distribution in distributions()
        if distribution.metadata["Name"]
    }
    try:
        dimos_requires, dimos_extras = (
            requires("dimos") or [],
            metadata("dimos").get_all("Provides-Extra") or [],
        )
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
    request = json.loads(sys.stdin.read() or "null")
    commands: dict[str, Callable[..., dict[str, Any]]] = {
        "blueprint": blueprint_detail,
        "config": config,
        "module": module,
        "module_config": module_config,
        "list": blueprint_list,
        "packages": packages,
        "scan": scan,
    }
    try:
        command = commands[argv[0]]
        result = command(*argv[1:], request) if argv[0] == "scan" else command(*argv[1:])
    except Exception as error:
        result = {"error": f"{type(error).__name__}: {error}"}
    emit(result)


if __name__ == "__main__":
    main(sys.argv[1:])
