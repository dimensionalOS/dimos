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
import difflib
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
INTERNAL_FIELDS = {"g", "rpc_transport", "rpc_timeouts", "default_rpc_timeout", "instance_name"}
RUN_VALUE_OPTIONS = {"--disable", "--config", "-c", "--relay-url", "--relay-ca"}
RUN_SWITCHES = {"--local-relay", "--no-local-relay"}
RUN_REFUSED = {
    "--daemon": "the gateway runs dimos in the foreground",
    "-d": "the gateway runs dimos in the foreground",
    "--help": "try `dimos run <blueprint> --help` in a terminal",
}


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
        self.entries = {k: entry for k, entry in self.entries.items() if entry[0] > now}
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
    dimos_dir: Path, args: list[str], stdin: Any = None, timeout: float = 180
) -> dict[str, Any]:
    child = await asyncio.create_subprocess_exec(
        *child_command(dimos_dir),
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
    def text(obj: Any) -> Any:
        if isinstance(obj, enum.Enum):
            return obj.value
        return re.sub(r" at 0x[0-9a-fA-F]+", "", str(obj))

    try:
        return json.loads(json.dumps(value, default=text))
    except Exception:
        return text(repr(value))


def first_line(obj: Any) -> str:
    doc = inspect.getdoc(obj) or ""
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


def source_of(obj: Any) -> dict[str, Any]:
    try:
        file = inspect.getsourcefile(obj)
        line = inspect.getsourcelines(obj)[1]
    except (TypeError, OSError):
        return {"file": None, "line": None}
    return {"file": relative_to_checkout(file) if file else None, "line": line if file else None}


def blueprint_source(name: str) -> dict[str, Any]:
    from dimos.robot.all_blueprints import all_blueprints, all_modules

    try:
        if name in all_modules:
            path, _, attr = all_modules[name].rpartition(".")
            return source_of(getattr(importlib.import_module(path), attr))
        path, _, attr = all_blueprints[name].partition(":")
        module = importlib.import_module(path)
        lines = inspect.getsource(module).splitlines()
    except Exception:
        return {"file": None, "line": None}
    assignment = re.compile(rf"{re.escape(attr)}\s*(:|=(?!=))")
    line = next((n for n, text in enumerate(lines, 1) if assignment.match(text)), 1)
    return {"file": relative_to_checkout(inspect.getfile(module)), "line": line}


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


def return_type(fn: Any) -> str | None:
    try:
        annotation = inspect.signature(fn).return_annotation
    except (TypeError, ValueError):
        return None
    if annotation is inspect.Signature.empty:
        return None
    return str(getattr(annotation, "__name__", annotation)).replace("typing.", "")


@functools.cache
def base_rpcs() -> frozenset[str]:
    from dimos.core.module import Module

    return frozenset(dict(Module.rpcs))


def methods(cls: Any) -> dict[str, list[dict[str, Any]]]:
    rpcs: list[dict[str, Any]] = []
    skills: list[dict[str, Any]] = []
    for name, fn in sorted(dict(getattr(cls, "rpcs", {}) or {}).items()):
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


def blueprint_modules(bp: Any, registry_names: dict[str, str]) -> list[dict[str, Any]]:
    remap, pinned = dict(bp.remapping_map), dict(bp.transport_map)
    wired: dict[str, set[Any]] = defaultdict(set)
    for atom in bp.active_blueprints:
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
        for atom in bp.active_blueprints
    ]


def blueprint(name: str) -> dict[str, Any]:
    from dimos.robot.get_all_blueprints import get_by_name

    bp = get_by_name(name)
    modules = []
    for atom, entry in zip(bp.active_blueprints, blueprint_modules(bp, {}), strict=True):
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


def choices(kind: Any) -> list[Any] | None:
    if isinstance(kind, type) and issubclass(kind, enum.Enum):
        return [jsonable(member.value) for member in kind]
    if typing.get_origin(kind) is typing.Literal:
        return [jsonable(option) for option in typing.get_args(kind)]
    return None


def field_schema(kind: Any) -> dict[str, Any] | None:
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
    from pydantic import TypeAdapter

    try:
        json.dumps(TypeAdapter(kind).dump_python(value, mode="json"))
        return True
    except Exception:
        return False


def config_args(module: Any, overrides: dict[str, Any]) -> list[dict[str, Any]]:
    from dimos.core.module import ModuleConfig

    fields = getattr(typing.get_type_hints(module).get("config"), "model_fields", None)
    if fields is None:
        raise TypeError(f"{type_name(module)}.config isn't a pydantic model")
    args = []
    for name, field in fields.items():
        if name in INTERNAL_FIELDS:
            continue
        required, kind = field.is_required(), field.annotation
        try:
            default = None if required else field.get_default(call_default_factory=True)
        except Exception:
            default = None
        entry = {
            "name": name,
            "type": type_name(kind) if isinstance(kind, type) else str(kind).replace("typing.", ""),
            "default": jsonable(default),
            "description": field.description,
            "required": required,
            "base": name in ModuleConfig.model_fields,
        }
        options = choices(kind)
        if options is not None:
            entry["choices"] = options
        schema = field_schema(kind)
        entry["json_compatible"] = schema is not None and json_safe(kind, default)
        if schema is not None:
            entry["schema"] = schema
        if name in overrides:
            entry["value"] = jsonable(overrides[name])
        args.append(entry)
    return args


def config(name: str) -> dict[str, Any]:
    from dimos.robot.get_all_blueprints import get_by_name

    modules = []
    for atom in get_by_name(name).active_blueprints:
        entry: dict[str, Any] = {"module": atom.name, "class": type_name(atom.module)}
        try:
            entry["args"] = config_args(atom.module, dict(atom.kwargs))
        except Exception as error:
            entry["args"] = []
            entry["error"] = f"{type(error).__name__}: {error}"
        modules.append(entry)
    return {"name": name, "modules": modules}


def check_args(name: str, args: list[str]) -> dict[str, Any]:
    from pydantic import TypeAdapter, ValidationError

    from dimos.cli.dimos import normalize_argv
    from dimos.core.coordination.blueprint_config.merging import _resolve_target, merge_cli
    from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
    from dimos.core.coordination.blueprint_config.schema import normalize_option_name
    from dimos.core.coordination.blueprint_config.sources import (
        global_schema_defaults,
        validate_global_values,
    )
    from dimos.robot.all_blueprints import all_modules
    from dimos.robot.get_all_blueprints import get_by_name

    tokens = normalize_argv(args)
    schema = BlueprintConfigParser(get_by_name(name))._get_schema()

    def target_of(token: str) -> tuple[Any, bool]:
        option = normalize_option_name(token[2:].partition("=")[0])
        target = _resolve_target(option, schema)
        if target is None and option.startswith("no_"):
            positive = _resolve_target(option.removeprefix("no_"), schema)
            if positive is not None and positive.section == "global" and positive.is_bool:
                return positive, True
        return target, False

    checked: list[dict[str, Any]] = []
    index = 0
    while index < len(tokens):
        token = tokens[index]
        option = token.partition("=")[0]
        group = [token]
        entry: dict[str, Any] = {"tokens": group, "target": None, "error": None}
        checked.append(entry)
        index += 1
        if option in RUN_REFUSED:
            entry["error"] = f"{option} isn't for a launch: {RUN_REFUSED[option]}"
            continue
        if option in RUN_VALUE_OPTIONS:
            entry["target"] = f"run {option}"
            if "=" not in token:
                if index >= len(tokens) or tokens[index].startswith("-"):
                    entry["error"] = f"{option} needs a value"
                    continue
                group.append(tokens[index])
                index += 1
            given = token.partition("=")[2] if "=" in token else group[-1]
            if option == "--disable" and given not in all_modules:
                close = difflib.get_close_matches(given, list(all_modules), n=3)
                entry["error"] = f"--disable: no module {given!r}" + (
                    f" (did you mean {', '.join(close)}?)" if close else ""
                )
            elif option in ("--config", "-c") and not Path(given).expanduser().is_file():
                entry["error"] = f"{option}: no file {given}"
            continue
        if option in RUN_SWITCHES:
            entry["target"] = f"run {option}"
            continue
        if not token.startswith("--") or option in ("-o", "--option"):
            entry["error"] = (
                f"{token!r}: one blueprint per launch, and options start with '--'"
                if not token.startswith("-")
                else f"unexpected {token!r}"
            )
            continue
        try:
            target, negated = target_of(token)
        except Exception as error:
            target, negated = None, False
            entry["error"] = str(error)
        if "=" not in token and not negated and index < len(tokens):
            if not tokens[index].startswith("--") and (
                target is not None or not tokens[index].startswith("-")
            ):
                group.append(tokens[index])
                index += 1
        if entry["error"]:
            continue
        modules: dict[str, dict[str, Any]] = defaultdict(dict)
        global_values: dict[str, Any] = {}
        try:
            merge_cli(modules, global_values, {}, tuple(group), schema)
            assert target is not None
            entry["target"] = target.qualified_name
            if target.section == "global":
                validate_global_values({**global_schema_defaults(), **global_values})
            elif target.section == "module":
                value: Any = modules[target.root]
                for part in target.path:
                    value = value[part]
                TypeAdapter(target.annotations[0]).validate_python(value)
        except ValidationError as error:
            entry["error"] = f"--{target.relative_name}: {error.errors()[0]['msg']}"
        except Exception as error:
            text = str(error).strip()
            entry["error"] = text.splitlines()[0] if text else type(error).__name__
    return {"name": name, "args": checked}


def blueprint_list() -> dict[str, Any]:
    from dimos.robot.all_blueprints import all_blueprints
    from dimos.robot.external_blueprints import list_external_blueprint_names

    builtin = sorted(name for name in all_blueprints if not name.startswith("demo-"))
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
    skills = [
        {"name": skill, "doc": first_line(fn), "params": params(fn)}
        for skill, fn in sorted(dict(getattr(cls, "rpcs", {}) or {}).items())
        if getattr(fn, "__skill__", False)
    ]
    return {
        "kind": "module",
        "name": name,
        "class": type_name(cls),
        "doc": first_line(cls),
        "inputs": [
            {"name": s.name, "type": s.type.__name__} for s in streams if s.direction != "out"
        ],
        "outputs": [
            {"name": s.name, "type": s.type.__name__} for s in streams if s.direction != "in"
        ],
        "skills": skills,
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
            n for n in list_external_blueprint_names() if n not in all_blueprints
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
            bp = load_blueprint(name) if ref is not None else get_by_name(name)
            file = ref.split(":")[0] if ref else None
            doc = first_line(importlib.import_module(file)) if file and per_file[file] == 1 else ""
            modules = blueprint_modules(bp, registry_names)
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
            for atom in bp.active_blueprints:
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
            path, _, attr = all_modules[name].rpartition(".")
            seen(name, getattr(importlib.import_module(path), attr))
        except BaseException as error:
            emit({"kind": "error", "error": f"module {name}: {type(error).__name__}: {error}"})
    return {"kind": "end"}


def packages() -> dict[str, Any]:
    from importlib.metadata import PackageNotFoundError, distributions, metadata, requires

    from packaging.markers import default_environment
    from packaging.utils import canonicalize_name

    found = {
        canonicalize_name(d.metadata["Name"]): d.version
        for d in distributions()
        if d.metadata["Name"]
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


COMMANDS: dict[str, Callable[..., dict[str, Any]]] = {
    "blueprint": blueprint,
    "config": config,
    "args": check_args,
    "list": blueprint_list,
    "packages": packages,
    "scan": scan,
}


def main(argv: list[str]) -> None:
    request = json.loads(sys.stdin.read() or "null")
    try:
        command = COMMANDS[argv[0]]
        result = command(*argv[1:], request) if argv[0] in ("args", "scan") else command(*argv[1:])
    except Exception as error:
        result = {"error": f"{type(error).__name__}: {error}"}
    emit(result)


if __name__ == "__main__":
    main(sys.argv[1:])
