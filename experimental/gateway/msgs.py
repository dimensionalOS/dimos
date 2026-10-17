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
import hashlib
import importlib
import importlib.metadata
import inspect
import json
from pathlib import Path
import re
import subprocess
import sys
import tempfile
from typing import Any

DIMOS_DIR = Path(__file__).parents[2] / "dimos"
RUNTIME_FILE = Path(__file__).parent / "assets" / "msgs" / "runtime.ts"
BUILD_TIMEOUT_S = 120


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


def scan() -> tuple[dict[str, dict[str, Any]], dict[str, str]]:
    """Every message type's LCM schema, and why each type without one has none."""
    from dimos.web.lcm_codec import export_schema, schema_class_for

    schemas: dict[str, dict[str, Any]] = {}
    missing: dict[str, str] = {}
    for where, cls in message_classes():
        type_name = getattr(cls, "msg_name", None) or cls.__name__
        try:
            schema = export_schema(schema_class_for(cls))
        except ValueError as error:
            missing[type_name] = f"{reason(type_name, str(error))} [{where}]"
            continue
        multi = [
            f"{struct}.{field}"
            for struct, rows in schema["structs"].items()
            for field, _, dims in rows
            if dims is not None and len(dims) > 1
        ]
        if multi:
            missing[type_name] = (
                f"multi-dimensional arrays aren't supported ({', '.join(multi)}) [{where}]"
            )
            continue
        schemas[schema["type"]] = schema
    return schemas, {name: why for name, why in sorted(missing.items()) if name not in schemas}


def generate_ts(schemas: dict[str, dict[str, Any]], missing: dict[str, str]) -> str:
    structs: dict[str, list[list[Any]]] = {}
    for schema in schemas.values():
        for name, rows in schema["structs"].items():
            if structs.setdefault(name, rows) != rows:
                raise ValueError(f"two schemas define the struct {name} differently")
    structs = dict(sorted(structs.items()))
    types = {name: schemas[name]["fp"] for name in sorted(schemas)}
    package_names = ", ".join(sorted({name.split(".")[0] for name in types}))
    return f"""import {{ createRegistry, packages }} from {json.dumps(RUNTIME_FILE.as_uri())}
const STRUCTS = {json.dumps(structs)}
const TYPES = {json.dumps(types)}
const registry = createRegistry(STRUCTS, TYPES, {json.dumps(missing)})
export {{ createRegistry }}
export const {{ decode, decodeChannel, decodeMessage, typeOfChannel, lookup, register, getTypeNames, getMissingTypes }} = registry
export const {{ {package_names} }} = packages(TYPES, registry.lookup)
"""


def sdk_decoder() -> Path:
    from dimos.web.relay_bridge.locate import find_web_dir

    return find_web_dir() / "sdk" / "src" / "decoders" / "lcm.ts"


def bundle_js(ts_text: str) -> str:
    from dimos.utils.deno import ensure_deno

    with tempfile.TemporaryDirectory() as tmp:
        source, target, config = (
            Path(tmp) / "msgs.ts",
            Path(tmp) / "msgs.js",
            Path(tmp) / "deno.json",
        )
        source.write_text(ts_text)
        config.write_text(json.dumps({"imports": {"lcm-decoder": sdk_decoder().as_uri()}}))
        subprocess.run(
            [
                str(ensure_deno()),
                "bundle",
                "--quiet",
                "--platform=browser",
                "--format=esm",
                f"--config={config}",
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


def sources_hash() -> str:
    digest = hashlib.sha256(importlib.metadata.version("dimos-lcm").encode())
    sources = [Path(__file__), RUNTIME_FILE, sdk_decoder(), DIMOS_DIR / "web" / "lcm_codec.py"]
    for path in sources + sorted((DIMOS_DIR / "msgs").rglob("*.py")):
        digest.update(f"\0{path.relative_to(DIMOS_DIR.parent)}\0".encode() + path.read_bytes())
    return digest.hexdigest()[:16]


_build_lock = asyncio.Lock()


async def built_js(cache_dir: Path) -> Path:
    """msgs.js for this checkout's messages, built on first ask and kept until a message changes."""
    target = cache_dir / f"msgs-{await asyncio.to_thread(sources_hash)}.js"
    async with _build_lock:
        if target.is_file():
            return target
        cache_dir.mkdir(parents=True, exist_ok=True)
        # a child process, so importing every message type doesn't stay in the gateway's memory
        child = await asyncio.create_subprocess_exec(
            sys.executable,
            "-m",
            "experimental.gateway.msgs",
            str(target),
            cwd=DIMOS_DIR.parent,
            stdout=asyncio.subprocess.DEVNULL,
            stderr=asyncio.subprocess.PIPE,
        )
        try:
            _, err = await asyncio.wait_for(child.communicate(), BUILD_TIMEOUT_S)
        except asyncio.TimeoutError:
            child.kill()
            raise RuntimeError(f"building msgs.js took over {BUILD_TIMEOUT_S} s") from None
        if child.returncode != 0:
            last = err.decode(errors="replace").strip().splitlines()[-1:] or ["no output"]
            raise RuntimeError(f"building msgs.js failed: {last[0]}")
        for old in cache_dir.glob("msgs-*.js"):
            if old != target:
                old.unlink(missing_ok=True)
    return target


if __name__ == "__main__":
    schemas, missing = scan()
    for type_name, why in missing.items():
        print(f"no LCM schema: {type_name}: {why}", file=sys.stderr)
    out = Path(sys.argv[1])
    partial = out.with_suffix(".partial")
    partial.write_text(bundle_js(generate_ts(schemas, missing)))
    partial.replace(out)
