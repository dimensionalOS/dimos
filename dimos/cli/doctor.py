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

"""Read-only workspace diagnostics. Only the standard library is imported here."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from typing import Any

import tomllib

PROFILES = {
    "navigation": (
        "dimos.robot.unitree.go2.blueprints.smart.unitree_go2",
        "unitree_go2",
        ["dimos.navigation.replanning_a_star.min_cost_astar_ext", "unitree_webrtc_connect"],
    ),
    "manipulation": (
        "dimos.robot.manipulators.xarm.blueprints.basic",
        "xarm7_planner_coordinator",
        ["roboplan", "pinocchio", "xarm"],
    ),
}

NATIVE_CHECK = """import ctypes, os
from pathlib import Path
prefix = Path(os.environ["CONDA_PREFIX"])
if (prefix / "bin/python").exists():
    raise RuntimeError("Pixi must not own Python")
for name in ("libturbojpeg.so", "libportaudio.so", "libsndfile.so", "libGL.so.1", "libEGL.so.1"):
    ctypes.CDLL(str(prefix / "lib" / name))
"""
IMAGE_CHECK = """import numpy as np
from turbojpeg import TurboJPEG
import zenoh, lcm, cv2, open3d
codec = TurboJPEG()
pixels = np.zeros((8, 8, 3), dtype=np.uint8)
if codec.decode(codec.encode(pixels)).shape != pixels.shape:
    raise RuntimeError("TurboJPEG round trip failed")
"""
PROFILE_CHECK = """import importlib, json, sys
from dimos.core.native_module import NativeModule
module, name, dependencies = json.loads(sys.argv[1])
for dependency in dependencies:
    importlib.import_module(dependency)
blueprint = getattr(importlib.import_module(module), name)
for atom in blueprint.active_blueprints:
    if issubclass(atom.module, NativeModule):
        raise RuntimeError(f"{atom.module.__name__} needs explicit setup-time Nix preparation")
"""
PROJECT_CHECK = """import importlib.metadata as metadata, json, sys
from pathlib import Path
package, root = sys.argv[1:]
dist = metadata.distribution(package)
entries = [entry for entry in dist.entry_points if entry.group == "dimos.blueprints"]
if not entries:
    raise RuntimeError("Project has no registered blueprints; run uv sync")
info = json.loads(dist.read_text("direct_url.json") or "{}")
if not info.get("dir_info", {}).get("editable") or info.get("url") != Path(root).as_uri():
    raise RuntimeError("Project is not installed editable from this workspace; run uv sync")
"""


def probe(command: list[str], timeout: float = 40) -> str | None:
    try:
        result = subprocess.run(
            command, capture_output=True, text=True, timeout=timeout, check=False
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        return str(error)
    if result.returncode:
        detail = (result.stderr or result.stdout).strip()
        return f"exit {result.returncode}: {detail[-1500:]}"
    return None


def workspace(path: Path) -> Path:
    for candidate in (path.resolve(), *path.resolve().parents):
        if (candidate / ".dimos/project.toml").is_file():
            return candidate
    raise ValueError("No workspace configuration found. Run the workspace initializer first.")


def diagnose(root: Path) -> list[tuple[str, str, str]]:
    results: list[tuple[str, str, str]] = []
    try:
        config = tomllib.loads((root / ".dimos/project.toml").read_text())
        tools: dict[str, Any] = json.loads((root / ".dimos/tools.json").read_text())
        if not isinstance(tools, dict):
            raise TypeError("Tool configuration must be an object")
        if config.get("mode") not in {"sdk", "contributor"}:
            raise ValueError("Unknown workspace mode")
        package = ""
        if config["mode"] == "sdk":
            manifest = tomllib.loads((root / "pyproject.toml").read_text())
            package = manifest["project"]["name"]
            if not isinstance(package, str):
                raise TypeError("Project name must be a string")
        profile = config["profile"]
        if profile not in PROFILES:
            raise ValueError(f"Unknown profile {profile!r}")
    except (OSError, ValueError, TypeError, KeyError) as error:
        return [("FAIL", "Workspace configuration", f"{error}; run the initializer with --restore")]

    def check(label: str, error: str | None) -> None:
        results.append(("FAIL" if error else "PASS", label, error or ""))

    expected = root / ".venv"
    check(
        "Workspace Python",
        None
        if Path(sys.prefix).resolve() == expected
        else f"Expected {expected}; source .dimos/activate.sh",
    )
    prefix = root / ".dimos/.pixi/envs/default"
    check(
        "Combined activation",
        None
        if os.environ.get("CONDA_PREFIX") == str(prefix)
        and os.environ.get("VIRTUAL_ENV") == str(expected)
        else "Source .dimos/activate.sh",
    )
    for tool in ("uv", "pixi", "nix"):
        executable = tools.get(tool)
        check(
            tool,
            probe([executable, "--version"])
            if isinstance(executable, str)
            else "Tool path missing; run the initializer with --restore",
        )
    nix = tools.get("nix")
    if isinstance(nix, str):
        check(
            "Nix store",
            probe([nix, "--extra-experimental-features", "nix-command flakes", "store", "info"]),
        )
    for command in (["git", "lfs", "version"], ["ffmpeg", "-version"]):
        check(" ".join(command[:2]), probe(command))
    for label, code, arguments in (
        ("Native libraries", NATIVE_CHECK, []),
        ("Python/native imports and JPEG", IMAGE_CHECK, []),
        ("Profile dependencies", PROFILE_CHECK, [json.dumps(PROFILES[profile])]),
    ):
        check(label, probe([sys.executable, "-c", code, *arguments]))
    if config.get("mode") == "sdk":
        check(
            "Editable project and blueprints",
            probe([sys.executable, "-c", PROJECT_CHECK, package, str(root)]),
        )
    if shutil.which("direnv") is None:
        results.append(
            (
                "INFO",
                "Optional direnv",
                "Not installed; manual activation works. https://direnv.net/docs/installation.html",
            )
        )
    elif not (root / ".envrc").is_file():
        results.append(
            ("INFO", "Optional direnv", "No .envrc; add one sourcing .dimos/activate.sh if desired")
        )
    elif not os.environ.get("DIRENV_DIR"):
        results.append(
            (
                "INFO",
                "Optional direnv",
                "Not loaded here. Review .envrc, run direnv allow, and check https://direnv.net/docs/hook.html",
            )
        )
    else:
        results.append(
            (
                "INFO",
                "Optional direnv",
                "Direnv is active; activation checks above verify the project environment",
            )
        )
    return results


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="dimos doctor", description=__doc__)
    parser.add_argument("--project-dir", type=Path, default=Path.cwd())
    args = parser.parse_args(argv)
    try:
        root = workspace(args.project_dir)
        results = diagnose(root)
    except (OSError, ValueError) as error:
        results = [("FAIL", "Workspace", str(error))]
    for status, label, detail in results:
        print(f"{status:4}  {label}" + (f": {detail}" if detail else ""))
    failed = any(status == "FAIL" for status, _, _ in results)
    if failed:
        print(
            "Activate the workspace or run the initializer with --restore. No files were changed."
        )
    return int(failed)


if __name__ == "__main__":
    raise SystemExit(main())
