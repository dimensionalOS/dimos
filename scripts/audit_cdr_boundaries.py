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


"""Inventory legacy runtime imports and typed ports without importing robot modules."""

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
    parts = name.split(".")
    return len(parts) >= 3 and parts[:2] == ["dimos", "msgs"] and parts[2].endswith("_msgs")


def audit(root: Path) -> dict:
    imports_found = []
    ports = []
    for path in sorted((root / "dimos").rglob("*.py")):
        relative = path.relative_to(root)
        if path.name.startswith(("test_", "demo_", "tool_")) or path.name == "conftest.py":
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
                    if legacy(full):
                        imports_found.append(
                            {"file": str(relative), "line": node.lineno, "symbol": full}
                        )
            elif isinstance(node, ast.Import):
                for alias in node.names:
                    imports[alias.asname or alias.name.split(".")[0]] = (
                        alias.name if alias.asname else alias.name.split(".")[0]
                    )
                    if legacy(alias.name):
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
        "method": "Static Python import and typed In/Out scan; no runtime graph or hardware imports. Internal driver values are not automatically wire defects.",
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
    parser.add_argument(
        "--check", action="store_true", help="Fail while any legacy imports or ports remain."
    )
    args = parser.parse_args()
    report = audit(args.root.resolve())
    text = json.dumps(report, indent=2) + "\n"
    if args.output:
        args.output.write_text(text)
    else:
        print(text, end="")
    return int(args.check and report["status"] != "clear")


if __name__ == "__main__":
    raise SystemExit(main())
