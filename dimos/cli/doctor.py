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

"""Read-only diagnostics for a DimOS application or contributor checkout."""

import argparse
import json
from pathlib import Path
import shutil
import subprocess
import sys
from typing import Any

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib


def project_root(path: Path) -> Path:
    for root in (path.resolve(), *path.resolve().parents):
        manifest = root / "pyproject.toml"
        if manifest.is_file():
            config = tomllib.loads(manifest.read_text())
            if "dimos" in config.get("tool", {}).get("uv", {}).get("sources", {}) or (
                config.get("project", {}).get("name") == "dimos" and (root / "dimos").is_dir()
            ):
                return root
    raise ValueError(
        "No DimOS project found. Run inside an application created by dimup init or a checkout prepared by dimup dev."
    )


def probe(python: Path, script: str, *args: str, cwd: Path) -> tuple[bool, str]:
    try:
        result = subprocess.run(
            [str(python), "-I", "-c", script, *args],
            capture_output=True,
            text=True,
            timeout=45,
            check=False,
            cwd=cwd,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        return False, str(error)
    if result.returncode:
        return False, (result.stderr or result.stdout).strip()[-1500:]
    return True, result.stdout.strip()


METADATA_CHECK = """import importlib.metadata as m, json, sys
from pathlib import Path
app = m.distribution(sys.argv[1])
sdk = m.distribution("dimos")
print(json.dumps({
    "editable": json.loads(app.read_text("direct_url.json") or "{}"),
    "sdk": json.loads(sdk.read_text("direct_url.json") or "{}"),
    "entries": [e.name for e in app.entry_points if e.group == "dimos.blueprints"],
    "version": sdk.version,
    "source": str(Path(__import__("dimos").__file__).resolve()),
}))
"""
LIBRARY_CHECK = """import ctypes
from ctypes.util import find_library
import numpy as np
import sqlite3, sqlite_vec
from turbojpeg import TurboJPEG
import zenoh, lcm
with sqlite3.connect(":memory:") as connection:
    if not hasattr(connection, "enable_load_extension"):
        raise RuntimeError(
            "Python lacks SQLite extension loading. Recreate .venv with uv-managed Python: "
            "uv venv --clear --managed-python --python 3.12 && uv sync --locked"
        )
    connection.enable_load_extension(True)
    sqlite_vec.load(connection)
    connection.enable_load_extension(False)
    connection.execute("SELECT vec_version()").fetchone()
for name in ("portaudio", "sndfile"):
    path = find_library(name)
    if not path:
        raise RuntimeError(f"Missing host library: {name}; run dimup setup")
    ctypes.CDLL(path)
codec = TurboJPEG()
image = np.zeros((8, 8, 3), dtype=np.uint8)
assert codec.decode(codec.encode(image)).shape == image.shape
"""


def diagnose(root: Path) -> list[tuple[bool, str, str]]:
    manifest = tomllib.loads((root / "pyproject.toml").read_text())
    name = manifest["project"]["name"]
    contributor = name == "dimos"
    python = root / ".venv/bin/python"
    results = [
        (
            Path(sys.prefix).resolve() == (root / ".venv").resolve(),
            "Active environment",
            "source .dimos/activate.sh selects this project's interpreter",
        )
    ]
    if not python.is_file():
        return [*results, (False, "Application environment", "Run uv sync --locked")]
    ok, detail = probe(python, METADATA_CHECK, name, cwd=root)
    if ok:
        metadata: dict[str, Any] = json.loads(detail)
        app = metadata["editable"]
        editable = app.get("dir_info", {}).get("editable") and app.get("url") == root.as_uri()
        if contributor:
            source = metadata.get("source")
            results.append(
                (
                    bool(editable) and source == str(root / "dimos/__init__.py"),
                    "Editable DimOS checkout",
                    f"Expected {root}; imported {source}. Run uv sync --locked if they differ",
                )
            )
        else:
            results.append(
                (
                    bool(editable),
                    "Editable application",
                    "Run uv sync --locked to install this application",
                )
            )
            results.append(
                (
                    bool(metadata["entries"]),
                    "Blueprint registration",
                    ", ".join(metadata["entries"]) or "Run uv sync to refresh entry points",
                )
            )
            expected = manifest["tool"]["uv"]["sources"]["dimos"]["rev"]
            commit = metadata["sdk"].get("vcs_info", {}).get("commit_id")
            results.append(
                (
                    commit == expected,
                    "SDK revision",
                    f"Expected {expected}; installed {commit}. Run uv sync --locked if they differ",
                )
            )
    else:
        results.append((False, "Installed metadata", f"{detail}\nRun uv sync --locked"))
    for tool in ("uv", "cargo", "nix", "git"):
        found = shutil.which(tool)
        results.append(
            (found is not None, tool, found or "Run dimup setup, then activate the project")
        )
    ok, detail = probe(python, LIBRARY_CHECK, cwd=root)
    results.append((ok, "Native libraries and image codec", detail or "Image round trip passed"))
    return results


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="dimos doctor", description=__doc__)
    parser.add_argument("--project-dir", type=Path, default=Path.cwd())
    args = parser.parse_args(argv)
    try:
        results = diagnose(project_root(args.project_dir))
    except (OSError, ValueError, KeyError, TypeError) as error:
        results = [(False, "Application", str(error))]
    for ok, label, detail in results:
        print(f"{'PASS' if ok else 'FAIL'}  {label}: {detail}")
    return int(any(not ok for ok, _, _ in results))


if __name__ == "__main__":
    raise SystemExit(main())
