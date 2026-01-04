#!/usr/bin/env python3
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

"""Generate dependency prompts for system package discovery using Claude."""

import argparse
import asyncio
import json
from pathlib import Path
import re
import shutil
import sys

from support.claude import run_claude_named_prompts

try:
    import tomllib  # Python 3.11+
except ModuleNotFoundError:  # pragma: no cover
    import tomli as tomllib  # type: ignore


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent.parent
PYPROJECT_PATH = REPO_ROOT / "pyproject.toml"
DEP_DB_DIR = REPO_ROOT / "installer" / "dep_database"

DEP_LIST_KEYS = ["apt_dependencies", "brew_dependencies", "nix_dependencies"]
REQUIRED_KEYS = ["package", *DEP_LIST_KEYS]


def _normalize_requirement(requirement: str) -> str:
    """
    Strip version markers and normalize case.

    Example:
        >>> _normalize_requirement("Torch>=2.0; python_version>='3.9'")
        'torch'
    """
    return re.sub(r"[=>,;].+", "", requirement).strip().lower()


async def _run_cmd(*args: str) -> tuple[int, str, str]:
    """
    Run a command asynchronously and return (code, stdout, stderr).

    Example:
        >>> asyncio.run(_run_cmd("echo", "hi"))
        (0, 'hi', '')
    """
    proc = await asyncio.create_subprocess_exec(
        *args,
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE,
    )
    stdout, stderr = await proc.communicate()
    return proc.returncode, stdout.decode().strip(), stderr.decode().strip()


async def _project_root() -> Path:
    """
    Resolve the git repository root.

    Example:
        >>> asyncio.run(_project_root())  # doctest: +ELLIPSIS
        PosixPath('.../dimos')
    """
    code, stdout, stderr = await _run_cmd("git", "rev-parse", "--show-toplevel")
    if code != 0:
        raise RuntimeError(f"git rev-parse failed: {stderr or stdout}")
    return Path(stdout)


async def _load_dependencies() -> list[str]:
    """
    Load dependency lists from pyproject.toml.

    Example:
        >>> deps = asyncio.run(_load_dependencies())
        >>> isinstance(deps, list)
        True
    """
    raw = await asyncio.to_thread(PYPROJECT_PATH.read_bytes)
    data = tomllib.loads(raw.decode())
    project = data.get("project", {})
    deps = list(project.get("dependencies", []))
    for _, extras in project.get("optional-dependencies", {}).items():
        deps.extend(extras)
    return deps


async def _existing_entry_is_complete(path: Path) -> bool:
    """
    Check whether a dep_database entry has all required keys.

    Example:
        >>> tmp = DEP_DB_DIR / "_example.json"
        >>> tmp.write_text('{"package":"foo","apt_dependencies":[],"brew_dependencies":[],"nix_dependencies":[]}')
        >>> asyncio.run(_existing_entry_is_complete(tmp))
        True
    """
    try:
        raw = await asyncio.to_thread(path.read_text)
        obj = json.loads(raw)
    except Exception:
        return False

    overlap = set(obj.keys()) & set(REQUIRED_KEYS)
    if len(overlap) != len(REQUIRED_KEYS):
        return False
    return all(isinstance(obj.get(key), list) for key in DEP_LIST_KEYS)


def _build_prompt(name: str, requirement: str) -> str:
    """
    Build the Claude prompt for a dependency.

    Example:
        >>> _build_prompt("torch", "torch>=2.0")
        'list all apt-get dependencies, nix, and brew dependencies for the torch>=2.0 pip module...'
    """
    key_list = " ".join(json.dumps(key) for key in REQUIRED_KEYS)
    dep_keys = " ".join(json.dumps(key) for key in DEP_LIST_KEYS)
    return (
        f"list all apt-get dependencies, nix, and brew dependencies for the {requirement} "
        f"pip module. The result should be a json object with the following {key_list} "
        f'and optionally "description", "notes". These ({dep_keys}) should be list of '
        f"strings. Store that resulting json inside ./installer/dep_database/{name}.json"
    )


async def _gather_prompts(
    project_dir: Path, dependencies: list[str]
) -> tuple[list[tuple[str, str]], list[str]]:
    """
    Build prompts for deps that are missing or incomplete.

    Example:
        >>> asyncio.run(_gather_prompts(Path("."), ["torch>=2.0"]))
        ([(..., ...)], [...])
    """
    prompts: list[tuple[str, str]] = []
    missing: list[str] = []

    for requirement in dependencies:
        name = _normalize_requirement(requirement)
        if name.startswith("types-") or name.endswith("-stubs") or name.startswith("pytest-"):
            continue

        dest_path = project_dir / "installer" / "dep_database" / f"{name}.json"
        if not dest_path.exists():
            missing.append(name)
            prompts.append((name, _build_prompt(name, requirement)))
            continue

        if await _existing_entry_is_complete(dest_path):
            continue

        prompts.append((name, _build_prompt(name, requirement)))

    return prompts, missing


async def main(argv: list[str]) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dry-run", action="store_true", help="List work without calling claude.")
    parser.add_argument(
        "--max-concurrent", type=int, default=5, help="Number of simultaneous claude prompts."
    )
    parser.add_argument("--log-dir", default="./.claude", help="Where to store claude logs.")
    parser.add_argument("extra", nargs="*", help=argparse.SUPPRESS)
    args = parser.parse_args(argv)

    project_dir = await _project_root()
    dependencies = await _load_dependencies()
    prompts, missing = await _gather_prompts(project_dir, dependencies)

    list_only = args.dry_run or bool(args.extra)
    if list_only:
        for name, _ in prompts:
            status = "missing" if name in missing else "needs modification"
            print(f"{status:18}: {name}.json")
        total = len(prompts)
        print(f"total: {total}")
        print(f"missing: {len(missing)}")
        print(f"need modification: {total - len(missing)}")
        return

    if not prompts:
        print("No prompts to run; dep database already complete.")
        return

    await _run_claude_named_prompts(
        prompts,
        max_concurrent=max(args.max_concurrent, 1),
        log_dir=Path(args.log_dir),
    )


if __name__ == "__main__":
    try:
        asyncio.run(main(sys.argv[1:]))
    except KeyboardInterrupt:
        pass
