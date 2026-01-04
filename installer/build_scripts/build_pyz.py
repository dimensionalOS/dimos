#!/usr/bin/env python3
"""Build the installer zipapp and refresh bundled dependency data."""

from __future__ import annotations

import asyncio
import json
import os
import shutil
import sys
from pathlib import Path
from typing import Iterable

# python is kinda verbose here's the TLDR of what's going on:
    # rm -rf "$BUILD"
    # mkdir -p "$BUILD/app"
    # python3 "build_pip_dependency_database.py" # generates pip_dependency_database.json
    # # Copy code in
    # rsync -a "$ROOT/pyz_app/" "$BUILD/app/pyz_app/"
    # # Install dependencies into the app directory
    # python3 -m pip install -r "$ROOT/requirements.txt" -t "$BUILD/app" >/dev/null
    # # Build the zipapp. main points to pyz_app.__main__:main
    # python3 -m zipapp "$BUILD/app" \
    #   -o "$OUT" \
    #   -m "pyz_app.__main__:main"


SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
OUT_PATH = ROOT / "installer.pyz"
BUILD_DIR = ROOT / ".build_pyz"
APP_SRC = ROOT / "pyz_app"
APP_DEST = BUILD_DIR / "app" / "pyz_app"
REQUIREMENTS = ROOT / "requirements.txt"

DEP_DIR = ROOT / "dep_database"
DEPENDENCY_OUT = APP_SRC / "bundled_files" / "pip_dependency_database.json"
TOML_SOURCE = ROOT.parent / "pyproject.toml"
TOML_LINK = APP_SRC / "bundled_files" / "pyproject.toml"


async def _run_cmd(*args: str, cwd: Path | None = None, inherit_io: bool = False) -> None:
    """Run a subprocess and raise on failure."""
    kwargs = {}
    if cwd:
        kwargs["cwd"] = str(cwd)
    if inherit_io:
        kwargs.update(stdin=sys.stdin, stdout=sys.stdout, stderr=sys.stderr)
    proc = await asyncio.create_subprocess_exec(*args, **kwargs)
    code = await proc.wait()
    if code != 0:
        raise RuntimeError(f"Command {' '.join(args)} failed with exit code {code}")


async def _reset_build_dir() -> None:
    if BUILD_DIR.exists():
        await asyncio.to_thread(shutil.rmtree, BUILD_DIR)
    await asyncio.to_thread((BUILD_DIR / "app").mkdir, parents=True, exist_ok=True)


def _read_dep_json(dep_dir: Path) -> dict:
    aggregated: dict[str, object] = {}
    for path in sorted(dep_dir.glob("*.json")):
        name = path.stem.lower()
        try:
            aggregated[name] = json.loads(path.read_text())
        except Exception as exc:  # pragma: no cover - build-time guard
            print(f"{name} had an error: {exc}", file=sys.stderr)
    return aggregated


def _aggregate_dep_database_files() -> None:
    """Aggregate dep_database JSON and hardlink pyproject into bundled_files."""
    aggregated = _read_dep_json(DEP_DIR)
    DEPENDENCY_OUT.parent.mkdir(parents=True, exist_ok=True)
    DEPENDENCY_OUT.write_text(json.dumps(aggregated, indent=2, sort_keys=True)+"\n")

    TOML_LINK.parent.mkdir(parents=True, exist_ok=True)
    try:
        if TOML_LINK.exists():
            try:
                if TOML_LINK.samefile(TOML_SOURCE):
                    return
            except FileNotFoundError:
                pass
            TOML_LINK.unlink()
        os.link(TOML_SOURCE, TOML_LINK)
    except Exception as exc:
        print(f"Failed to hardlink pyproject.toml: {exc}", file=sys.stderr)


async def _copy_app_sources() -> None:
    """Copy the pyz_app sources into the build directory."""
    if shutil.which("rsync"):
        await _run_cmd("rsync", "-a", f"{APP_SRC}/", str(APP_DEST))
    else:  # pragma: no cover - fallback path
        await asyncio.to_thread(shutil.copytree, APP_SRC, APP_DEST, dirs_exist_ok=True)


async def _install_dependencies() -> None:
    """Install Python dependencies into the build app directory."""
    await _run_cmd(
        sys.executable,
        "-m",
        "pip",
        "install",
        "-r",
        str(REQUIREMENTS),
        "-t",
        str(BUILD_DIR / "app"),
    )


async def _build_zipapp() -> None:
    """Create the zipapp from the prepared build directory."""
    await _run_cmd(
        sys.executable,
        "-m",
        "zipapp",
        str(BUILD_DIR / "app"),
        "-o",
        str(OUT_PATH),
        "-m",
        "pyz_app.__main__:main",
        inherit_io=True,
    )


async def main(argv: Iterable[str] | None = None) -> None:
    """Orchestrate build steps with concurrency for system commands."""
    _ = argv  # unused, reserved for future args
    _aggregate_dep_database_files()
    await _reset_build_dir()

    # Run copy and pip install concurrently to speed up the build.
    await asyncio.gather(
        _copy_app_sources(),
        _install_dependencies(),
    )

    await _build_zipapp()
    print(f"Built: {OUT_PATH}")
    print(f"Run:   python3 {OUT_PATH}")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
