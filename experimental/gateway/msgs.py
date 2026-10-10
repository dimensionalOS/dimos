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

from dataclasses import dataclass
import importlib
import inspect
import json
from pathlib import Path
import re
import subprocess
import sys
import tempfile
from typing import Any

HERE = Path(__file__).parent / "assets" / "msgs"
DIMOS_DIR = Path(__file__).parents[2] / "dimos"
RUNTIME_FILE = HERE / "runtime.ts"
JS_FILE = HERE / "msgs.js"
IDENTIFIER = re.compile(r"^[A-Za-z_$][A-Za-z0-9_$]*$")
TS_SCALAR = {
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
TS_ARRAY = {
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
    type_name: str
    where: str
    reason: str


@dataclass(frozen=True)
class Scan:
    schemas: dict[str, dict[str, Any]]
    missing: list[Missing]


def message_classes() -> list[tuple[str, type[Any]]]:
    found = []
    for path in sorted((DIMOS_DIR / "msgs").rglob("*.py")):
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


def reason(type_name: str, error: str) -> str:
    own = re.search(r"declares its own LCM fingerprint (\w+) but the \S+ schema has (\w+)", error)
    if own:
        return (
            f"hand-written, its own fingerprint {own.group(1)} (dimos_lcm.{type_name} is another "
            f"message of the same name, fingerprint {own.group(2)})"
        )
    if "hand-written" in error or "has no generated" in error:
        return f"hand-written, no dimos_lcm.{type_name} class"
    return error


def scan() -> Scan:
    from dimos.web.lcm_codec import export_schema, schema_class_for

    schemas: dict[str, dict[str, Any]] = {}
    missing: dict[str, Missing] = {}
    for where, cls in message_classes():
        type_name = getattr(cls, "msg_name", None) or cls.__name__
        try:
            schema = export_schema(schema_class_for(cls))
        except ValueError as error:
            missing[type_name] = Missing(type_name, where, reason(type_name, str(error)))
            continue
        multi = [
            f"{struct}.{field}"
            for struct, rows in schema["structs"].items()
            for field, _, dims in rows
            if dims is not None and len(dims) > 1
        ]
        if multi:
            why = f"multi-dimensional arrays aren't supported ({', '.join(multi)})"
            missing[type_name] = Missing(type_name, where, why)
            continue
        schemas[schema["type"]] = schema
    for name in schemas:
        missing.pop(name, None)
    return Scan(schemas, sorted(missing.values(), key=lambda m: m.type_name))


def field_type(kind: str, dims: list[Any] | None) -> str:
    if dims is None:
        return TS_SCALAR.get(kind, kind)
    return TS_ARRAY.get(kind, f"{kind}[]")


def generate_ts(found: Scan) -> str:
    structs: dict[str, list[list[Any]]] = {}
    for schema in found.schemas.values():
        for name, rows in schema["structs"].items():
            if structs.setdefault(name, rows) != rows:
                raise ValueError(f"two schemas define the struct {name} differently")
    types = {name: found.schemas[name]["fp"] for name in sorted(found.schemas)}
    missing = {m.type_name: f"{m.reason} [{m.where}]" for m in found.missing}
    out = [
        RUNTIME_FILE.read_text().rstrip("\n"),
        "",
        "const STRUCTS: Record<string, Field[]> = {",
        *(f"    {json.dumps(name)}: {json.dumps(structs[name])}," for name in sorted(structs)),
        "}",
        "const TYPES: Record<string, string> = {",
        *(f"    {json.dumps(name)}: {json.dumps(fp)}," for name, fp in types.items()),
        "}",
        "const MISSING: Record<string, string> = {",
        *(f"    {json.dumps(name)}: {json.dumps(why)}," for name, why in missing.items()),
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
            short = name.split(".")[1]
            out.append(f"    export interface {short} {{")
            for field, kind, dims in structs[name]:
                key = field if IDENTIFIER.match(field) else json.dumps(field)
                out.append(f"        {key}: {field_type(kind, dims)}")
            out.append("    }")
            if name in types:
                out.append(
                    f"    export const {short} = lookup({json.dumps(name)}) as MsgType<{short}>"
                )
        out.append("}")
    return "\n".join(out) + "\n"


def bundle_js(ts_text: str) -> str:
    from dimos.utils.deno import ensure_deno

    with tempfile.TemporaryDirectory() as tmp:
        source, target = Path(tmp) / "msgs.ts", Path(tmp) / "msgs.js"
        source.write_text(ts_text)
        subprocess.run(
            [
                str(ensure_deno()),
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
        text = target.read_text().replace("/* @__PURE__ */ ", "")
    return "\n".join(line for line in text.splitlines() if not line.startswith("// ")) + "\n"


if __name__ == "__main__":
    found = scan()
    for missing in found.missing:
        print(
            f"no LCM schema: {missing.type_name} ({missing.where}): {missing.reason}",
            file=sys.stderr,
        )
    JS_FILE.write_text(bundle_js(generate_ts(found)))
