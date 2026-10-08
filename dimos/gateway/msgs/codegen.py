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

"""msgs.ts and msgs.js: every dimos message's LCM codec for frontends, generated from dimos/msgs/.

Every message class defined under dimos/msgs/ is resolved to the generated dimos_lcm class whose wire format its
lcm_encode() writes (dimos/web/lcm_codec.py schema_class_for) and its schema exported (export_schema). msgs.ts is
runtime.ts plus those schemas and a typed namespace per package (`geometry_msgs.PoseStamped.decode(bytes)`), for Deno
and TypeScript; msgs.js is msgs.ts bundled to plain JS by `deno bundle`, which the gateway serves at GET
/dimos/msgs.js for pages to import without a build step. A hand-written message (one with no dimos_lcm class, or
whose fingerprint isn't its namesake's) has no schema: it's a warning, listed in both files, never a failure.

    python -m dimos.gateway.msgs           # check: warnings, and exit 1 when a file is stale
    python -m dimos.gateway.msgs --write   # regenerate both (msgs.js needs deno)

msgs.js records the sha256 of the msgs.ts it was bundled from, so its staleness is checked without deno.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import importlib
import inspect
import json
from pathlib import Path
import re
import shutil
import subprocess
from typing import Any

HERE = Path(__file__).parent
DIMOS_DIR = HERE.parents[1]
MSGS_DIR = DIMOS_DIR / "msgs"
RUNTIME_FILE = HERE / "runtime.ts"
TS_FILE = HERE / "msgs.ts"
JS_FILE = HERE / "msgs.js"
WRITE_COMMAND = "python -m dimos.gateway.msgs --write"
_BUILT_FROM = re.compile(r"^// bundled from msgs\.ts sha256:([0-9a-f]{64})$", re.M)
_IDENTIFIER = re.compile(r"^[A-Za-z_$][A-Za-z0-9_$]*$")
_TS_SCALAR = {
    "int8_t": "number",
    "int16_t": "number",
    "int32_t": "number",
    "int64_t": "bigint",
    "float": "number",
    "double": "number",
    "byte": "number",
    "boolean": "boolean",
    "string": "string",
}
_TS_ARRAY = {
    "byte": "Uint8Array",
    "int8_t": "Int8Array",
    "int16_t": "Int16Array",
    "int32_t": "Int32Array",
    "int64_t": "BigInt64Array",
    "float": "Float32Array",
    "double": "Float64Array",
    "boolean": "boolean[]",
    "string": "string[]",
}


@dataclass(frozen=True)
class Missing:
    """A dimos message class with no LCM schema, and why."""

    type_name: str  # its msg_name, "<package>.<Type>"
    where: str  # "dimos/msgs/sensor_msgs/JointCommand.py:JointCommand"
    reason: str

    def warning(self) -> str:
        return f"{self.type_name} ({self.where}): {self.reason}"


@dataclass(frozen=True)
class Scan:
    schemas: dict[str, dict[str, Any]]  # type name -> export_schema(...)
    missing: list[Missing]


def message_classes() -> list[tuple[str, type[Any]]]:
    """Every message class defined in a module under dimos/msgs/ (one with lcm_encode, not a Protocol), with where."""
    found = []
    for path in sorted(MSGS_DIR.rglob("*.py")):
        if path.name.startswith("test_") or path.name == "__init__.py":
            continue
        module_name = ".".join(path.relative_to(DIMOS_DIR.parent).with_suffix("").parts)
        module = importlib.import_module(module_name)
        for name, value in vars(module).items():
            if (
                inspect.isclass(value)
                and value.__module__ == module_name
                and callable(getattr(value, "lcm_encode", None))
                and not getattr(value, "_is_protocol", False)
            ):
                found.append((f"{path.relative_to(DIMOS_DIR.parent)}:{name}", value))
    return found


def scan() -> Scan:
    """Each dimos message's schema (by the type name it writes), and the ones without one."""
    from dimos.web.lcm_codec import export_schema, schema_class_for

    schemas: dict[str, dict[str, Any]] = {}
    missing: dict[str, Missing] = {}
    for where, cls in message_classes():
        type_name = getattr(cls, "msg_name", None) or cls.__name__
        try:
            schema = export_schema(schema_class_for(cls))
        except ValueError as e:
            reason = _reason(type_name, str(e))
            missing[type_name] = Missing(type_name, where, reason)
            continue
        multi = [
            f"{struct}.{field}"
            for struct, rows in schema["structs"].items()
            for field, _, dims in rows
            if dims is not None and len(dims) > 1
        ]
        if multi:
            missing[type_name] = Missing(
                type_name, where, f"multi-dimensional arrays aren't supported ({', '.join(multi)})"
            )
            continue
        schemas[schema["type"]] = schema
    for name in schemas:
        missing.pop(name, None)
    return Scan(schemas, sorted(missing.values(), key=lambda m: m.type_name))


def _reason(type_name: str, error: str) -> str:
    """schema_class_for's refusal, said for a frontend."""
    own = re.search(r"declares its own LCM fingerprint (\w+) but the \S+ schema has (\w+)", error)
    if own:
        return (
            f"hand-written, its own fingerprint {own.group(1)} (dimos_lcm.{type_name} is another message "
            f"of the same name, fingerprint {own.group(2)})"
        )
    if "hand-written" in error or "has no generated" in error:
        return f"hand-written, no dimos_lcm.{type_name} class"
    return error


def warnings(found: Scan | None = None) -> list[str]:
    """One line per dimos message without a schema: frontends can't decode it until they register() a decoder."""
    return [m.warning() for m in (found or scan()).missing]


def _ts_field_type(field_type: str, dims: list[Any] | None) -> str:
    if dims is None:
        return _TS_SCALAR.get(field_type, field_type)
    if field_type in _TS_ARRAY:
        return _TS_ARRAY[field_type]
    return f"{field_type}[]"


def _key(name: str) -> str:
    return name if _IDENTIFIER.match(name) else json.dumps(name)


def generate_ts(found: Scan | None = None) -> str:
    """msgs.ts's text."""
    found = found or scan()
    structs: dict[str, list[list[Any]]] = {}
    for schema in found.schemas.values():
        for name, rows in schema["structs"].items():
            if structs.setdefault(name, rows) != rows:
                raise ValueError(f"two schemas define the struct {name} differently")
    types = {name: found.schemas[name]["fp"] for name in sorted(found.schemas)}
    missing = {m.type_name: f"{m.reason} [{m.where}]" for m in found.missing}

    runtime = RUNTIME_FILE.read_text()
    out = [
        "// GENERATED by `" + WRITE_COMMAND + "` from dimos/msgs/ and dimos_lcm: do not edit.",
        "// Every dimos message's LCM codec, by name, fingerprint, LCM channel or zenoh key. msgs.js is this file as",
        "// plain JS (GET /dimos/msgs.js).",
        "//",
        '//     import { decodeMessage, geometry_msgs } from "../../dimos/msgs.js"',
        '//     z.subscribe("dimos/odom/**", {}, (m) => console.log(decodeMessage(m)))',
        '//     z.put(geometry_msgs.Twist.zenohKey("dimos/cmd_vel"), geometry_msgs.Twist.encode({ linear: { x: 0.3 } }))',
        "//",
        "// dimos messages with no LCM schema (decode them with register()):",
        *(f"//   {m.warning()}" for m in found.missing),
        "",
        "// ---- runtime.ts ----",
        runtime.rstrip("\n"),
        "",
        "// ---- schemas ----",
        "",
        "const STRUCTS: Record<string, Field[]> = {",
        *(f"    {json.dumps(name)}: {json.dumps(structs[name])}," for name in sorted(structs)),
        "}",
        "const TYPES: Record<string, string> = {",
        *(f"    {json.dumps(name)}: {json.dumps(fp)}," for name, fp in types.items()),
        "}",
        "const MISSING: Record<string, string> = {",
        *(f"    {json.dumps(name)}: {json.dumps(reason)}," for name, reason in missing.items()),
        "}",
        "",
        "const registry = createRegistry(STRUCTS, TYPES, MISSING)",
        "export const { decode, decodeChannel, decodeMessage, typeOfChannel, lookup, register, getTypeNames, getMissingTypes } =",
        "    registry",
        "",
    ]
    packages: dict[str, list[str]] = {}
    for name in sorted(structs):
        packages.setdefault(name.split(".")[0], []).append(name)
    for package, names in packages.items():
        out.append(f"export namespace {package} {{")
        for name in names:
            type_name = name.split(".")[1]
            out.append(f"    export interface {type_name} {{")
            for field, field_type, dims in structs[name]:
                out.append(f"        {_key(field)}: {_ts_field_type(field_type, dims)}")
            out.append("    }")
            if name in types:
                out.append(
                    f"    export const {type_name} = lookup({json.dumps(name)}) as MsgType<{type_name}>"
                )
        out.append("}")
    return "\n".join(out) + "\n"


def bundled_from(js_text: str) -> str | None:
    """The sha256 of the msgs.ts a msgs.js was bundled from, as it records it."""
    match = _BUILT_FROM.search(js_text)
    return match.group(1) if match else None


def _sha256(text: str) -> str:
    return hashlib.sha256(text.encode()).hexdigest()


def deno() -> str | None:
    """The deno dimos pins (dimos/utils/deno.py) when it's downloaded, so msgs.js bundles the same everywhere, else
    deno on PATH; never downloads."""
    from dimos.utils.deno import _DENO_CACHE_DIR, DENO_VERSION

    pinned = _DENO_CACHE_DIR / DENO_VERSION / "deno"
    return str(pinned) if pinned.exists() else shutil.which("deno")


def deno_is_pinned(deno_path: str) -> bool:
    """`deno_path` is the version dimos pins (whose `deno bundle` wrote the committed msgs.js)."""
    from dimos.utils.deno import DENO_VERSION

    out = subprocess.run(
        [deno_path, "--version"], capture_output=True, text=True, check=True
    ).stdout
    return out.split()[1] == DENO_VERSION.lstrip("v")


def bundle_js(ts_text: str, deno_path: str) -> str:
    """msgs.ts bundled to plain JS by `deno bundle`, headed by the sha256 of the msgs.ts it came from."""
    import tempfile

    with tempfile.TemporaryDirectory() as tmp:
        source, target = Path(tmp) / "msgs.ts", Path(tmp) / "msgs.js"
        source.write_text(ts_text)
        subprocess.run(
            [
                deno_path,
                "bundle",
                "--quiet",
                "--platform=browser",
                "--format=esm",
                "-o",
                str(target),
                str(source),
            ],
            check=True,
            capture_output=True,
            cwd=tmp,
        )
        body = target.read_text()
    header = (
        "// GENERATED by `"
        + WRITE_COMMAND
        + "`: msgs.ts as plain JS (`deno bundle`); edit runtime.ts or the\n"
        "// generator, never this file. Types: msgs.ts beside it (GET /dimos/msgs.ts).\n"
        f"// bundled from msgs.ts sha256:{_sha256(ts_text)}\n"
    )
    return header + body


def stale_problems(found: Scan | None = None) -> list[str]:
    """msgs.ts isn't what the generator writes now, or msgs.js wasn't bundled from this msgs.ts."""
    problems = []
    ts_text = generate_ts(found)
    if not TS_FILE.exists() or TS_FILE.read_text() != ts_text:
        problems.append(
            f"dimos/gateway/msgs/msgs.ts is stale (a message in dimos/msgs/, dimos_lcm or runtime.ts changed): run "
            f"`{WRITE_COMMAND}`"
        )
    if not JS_FILE.exists() or bundled_from(JS_FILE.read_text()) != _sha256(ts_text):
        problems.append(
            f"dimos/gateway/msgs/msgs.js wasn't bundled from the current msgs.ts: run `{WRITE_COMMAND}` (needs deno)"
        )
    return problems


def write(found: Scan | None = None) -> None:
    """Regenerate msgs.ts, and msgs.js from it (RuntimeError without deno)."""
    ts_text = generate_ts(found)
    deno_path = deno()
    if deno_path is None:
        raise RuntimeError(
            "msgs.js is bundled by deno: install it (https://deno.com/) or run "
            "`python -c 'from dimos.utils.deno import ensure_deno; ensure_deno()'` (the version dimos pins)"
        )
    js_text = bundle_js(ts_text, deno_path)
    TS_FILE.write_text(ts_text)
    JS_FILE.write_text(js_text)
