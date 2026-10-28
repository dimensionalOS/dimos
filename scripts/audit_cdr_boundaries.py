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


"""Reject retired message imports across capabilities, tests, and runnable examples."""

from __future__ import annotations

import argparse
import ast
from collections import Counter
import json
from pathlib import Path


def qualified(node: ast.expr, imports: dict[str, str]) -> str:
    if isinstance(node, ast.Name):
        return imports.get(node.id, node.id)
    if isinstance(node, ast.Attribute):
        return qualified(node.value, imports) + "." + node.attr
    return ast.unparse(node)


def legacy(name: str) -> bool:
    if name == "dimos_lcm" or name.startswith("dimos_lcm."):
        return True
    return name == "dimos.msgs" or name.startswith("dimos.msgs.")


# These files read historical bytes or implement the external Habitat Python 3.9 ABI.
# They may import raw LCM classes, but never the retired DimOS wrappers.
RAW_LCM_BOUNDARIES = {
    "dimos/memory/codecs/legacy.py",
    "dimos/memory/codecs/jpeg.py",
    "dimos/memory/codecs/test_generated_recording.py",
    "dimos/memory/store/test_mcap.py",
    "dimos/memory/store/test_mcap_wire.py",
    "dimos/memory/test_recording_migration.py",
    "dimos/simulation/habitat/connection.py",
    "dimos/simulation/habitat/server.py",
    "dimos/simulation/habitat/test_connection.py",
    "dimos/web/relay_bridge/gen_lcm_fixtures.py",
    "dimos/web/test_cockpit.py",
    "dimos/web/test_codecs.py",
    "dimos/web/test_lcm_codec.py",
}


def audit(root: Path, scopes: list[str] | None = None) -> dict:
    imports_found = []
    ports = []
    scopes = scopes or ["dimos", "examples", "native/python", "misc", "scripts"]
    sources = {path for scope in scopes for path in (root / scope).rglob("*.py")}
    for path in sorted(sources):
        relative = path.relative_to(root)
        if any(part in {"build", "target", ".venv", "__pycache__"} for part in relative.parts):
            continue
        if relative.parts[:2] == ("dimos", "msgs"):
            continue
        tree = ast.parse(path.read_text(), filename=str(relative))
        imports = {}
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom):
                for alias in node.names:
                    full = f"{node.module}.{alias.name}"
                    imports[alias.asname or alias.name] = full
                    if legacy(full) and not (
                        full.startswith("dimos_lcm.") and str(relative) in RAW_LCM_BOUNDARIES
                    ):
                        imports_found.append(
                            {"file": str(relative), "line": node.lineno, "symbol": full}
                        )
            elif isinstance(node, ast.Import):
                for alias in node.names:
                    imports[alias.asname or alias.name.split(".")[0]] = (
                        alias.name if alias.asname else alias.name.split(".")[0]
                    )
                    if legacy(alias.name) and not (
                        alias.name.startswith("dimos_lcm") and str(relative) in RAW_LCM_BOUNDARIES
                    ):
                        imports_found.append(
                            {"file": str(relative), "line": node.lineno, "symbol": alias.name}
                        )
        for node in ast.walk(tree):
            if not isinstance(node, ast.AnnAssign) or not isinstance(
                node.annotation, ast.Subscript
            ):
                continue
            annotation = node.annotation
            direction = qualified(annotation.value, imports)
            if direction not in {"dimos.core.stream.In", "dimos.core.stream.Out"}:
                continue
            candidates = [
                qualified(n, imports)
                for n in ast.walk(annotation.slice)
                if isinstance(n, (ast.Name, ast.Attribute))
            ]
            types = sorted({name for name in candidates if legacy(name)})
            if types:
                ports.append(
                    {
                        "file": str(relative),
                        "line": node.lineno,
                        "port": ast.unparse(node.target),
                        "direction": direction.rsplit(".", 1)[1],
                        "legacy_types": types,
                    }
                )
    return {
        "status": "unfinished" if ports or imports_found else "clear",
        "method": "Static imports and typed ports in capabilities, tests, demos, and native Python runtimes. Historical raw-LCM boundaries are explicit exceptions. This does not validate native compilation or runtime behavior.",
        "legacy_port_count": len(ports),
        "legacy_import_count": len(imports_found),
        "ports_by_subsystem": dict(sorted(Counter(p["file"].split("/")[1] for p in ports).items())),
        "legacy_ports": ports,
        "legacy_imports": imports_found,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--output", type=Path)
    parser.add_argument("--scope", action="append", help="Limit the scan to a capability path.")
    parser.add_argument(
        "--check", action="store_true", help="Fail while any legacy imports or ports remain."
    )
    args = parser.parse_args()
    report = audit(args.root.resolve(), args.scope)
    text = json.dumps(report, indent=2) + "\n"
    if args.output:
        args.output.write_text(text)
    else:
        print(text, end="")
    return int(args.check and report["status"] != "clear")


if __name__ == "__main__":
    raise SystemExit(main())
