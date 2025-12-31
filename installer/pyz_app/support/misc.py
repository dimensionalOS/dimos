#!/usr/bin/env python3
from __future__ import annotations

import os
import re
import urllib.request
from functools import lru_cache
from pathlib import Path
from typing import Any

from . import prompt_tools as p
from . import pip_dependency_database as dep_db
from .dax import command_exists, run_command

try:
    import tomllib
except ModuleNotFoundError:  # pragma: no cover - fallback for older interpreters
    import tomli as tomllib  # type: ignore

_project_directory: Path | None = None
_already_called_apt_get_update = False


@lru_cache(maxsize=None)
def get_project_toml(branch: str = "main") -> dict[str, Any]:
    # FIXME: switch to main once code is public
    # url = "https://raw.githubusercontent.com/dimensionalOS/dimos/refs/heads/main/pyproject.toml"
    url = "https://raw.githubusercontent.com/dimensionalOS/dimos/refs/heads/dev/pyproject.toml?token=GHSAT0AAAAAADJ4QAIOIUHMTP3WT2Z32AEI2KURXLQ"
    try:
        with urllib.request.urlopen(url) as resp:  # nosec: trusted host, same as TS helper
            toml_text = resp.read().decode("utf-8")
        return tomllib.loads(toml_text)
    except Exception as exc:  # pragma: no cover - network dependent
        raise RuntimeError(f"Unable to download/parse pyproject.toml for dimos: {exc}") from exc


def get_system_deps(feature: str | None):
    toml_data = get_project_toml()
    apt_deps: set[str] = set()
    nix_deps: set[str] = set()
    brew_deps: set[str] = set()

    if feature is None:
        pip_deps = list(toml_data["project"]["dependencies"])
    elif isinstance(feature, (list, tuple, set)):
        pip_deps = []
        for feat in feature:
            pip_deps.extend(toml_data["project"]["optional-dependencies"].get(feat, []))
    else:
        pip_deps = list(toml_data["project"]["optional-dependencies"].get(feature, []))

    pip_deps = [re.sub(r"[<=>,;].+", "", dep) for dep in pip_deps]
    missing: list[str] = []

    for pip_dep in pip_deps:
        pip_dep_no_feature = re.sub(r"\[.+", "", pip_dep)
        system_dep_info = dep_db.DATA.get(pip_dep) or dep_db.DATA.get(pip_dep_no_feature)
        if not system_dep_info:
            missing.append(pip_dep)
            continue

        for key, value in system_dep_info.items():
            if key == "apt_dependencies":
                apt_deps.update(value)
            elif key == "nix_dependencies":
                nix_deps.update(value)
            elif key == "brew_dependencies":
                brew_deps.update(value)

    return {
        "aptDeps": sorted(apt_deps),
        "nixDeps": sorted(nix_deps),
        "brewDeps": sorted(brew_deps),
        "pipDeps": sorted(pip_deps),
        "missing": missing,
    }


def parse_version(text: str) -> str | None:
    match = re.search(r"\b(\d+(?:\.\d+)+)\b", text)
    return match.group(1) if match else None


def is_version_at_least(found: str, required: str) -> bool:
    found_parts = [int(x) for x in found.split(".")]
    required_parts = [int(x) for x in required.split(".")]
    length = max(len(found_parts), len(required_parts))
    for i in range(length):
        f = found_parts[i] if i < len(found_parts) else 0
        r = required_parts[i] if i < len(required_parts) else 0
        if f > r:
            return True
        if f < r:
            return False
    return True


def detect_python_command() -> str | None:
    if command_exists("python3"):
        return "python3"
    if command_exists("python"):
        return "python"
    return None


def ensure_git_and_lfs() -> None:
    if not command_exists("git"):
        raise RuntimeError("- ❌ git is required. Please install git and rerun.")
    git_lfs_res = run_command(["git", "lfs", "version"])
    if git_lfs_res.code != 0:
        raise RuntimeError("- ❌ git-lfs is required. Please install git-lfs and rerun.")


def ensure_port_audio() -> None:
    p.boring_log("Checking if portaudio is available")
    port_audio_res = run_command(
        ["pkg-config", "--modversion", "portaudio-2.0"], print_command=True
    )
    if port_audio_res.code != 0:
        raise RuntimeError("- ❌ portaudio is required. Please install portaudio and rerun.")


def ensure_python() -> str:
    python_cmd = detect_python_command()
    if not python_cmd:
        raise RuntimeError("- ❌ Python 3.10+ is required but was not found.")
    version_res = run_command([python_cmd, "--version"])
    version_text = (version_res.stdout or version_res.stderr or "").strip()
    parsed = parse_version(version_text)
    if not parsed or not is_version_at_least(parsed, "3.10.0"):
        raise RuntimeError(f"- ❌ Python 3.10+ required. Detected: {parsed or 'unknown'}")
    return python_cmd


def get_project_directory() -> Path:
    global _project_directory
    if _project_directory is None:
        p.console.print("Dimos needs to be installed to a project (not just a global install)")
        if p.ask_yes_no("Are you currently in a project directory?"):
            _project_directory = Path.cwd()
        else:
            raise RuntimeError(
                "- ❌ Please create a project directory and rerun this command from there."
            )
    return _project_directory


def apt_install(package_names: list[str]) -> None:
    global _already_called_apt_get_update
    if not package_names:
        return

    if not _already_called_apt_get_update:
        update_res = run_command(["sudo", "apt-get", "update"], print_command=True)
        if update_res.code != 0:
            raise RuntimeError(f"sudo apt-get update failed: {update_res.code}")
        _already_called_apt_get_update = True

    failed_packages: list[str] = []
    for each_pkg in package_names:
        res = run_command(["dpkg", "-s", each_pkg])
        if res.code == 0:
            p.console.print(f"- ✅ looks like {p.highlight(each_pkg)} is already installed")
            continue

        p.sub_header(f"- installing {p.highlight(each_pkg)}")
        install_res = run_command(
            ["sudo", "apt-get", "install", "-y", each_pkg], print_command=True
        )
        if install_res.code != 0:
            failed_packages.append(each_pkg)

    if failed_packages:
        cmds = "\n".join(f"    sudo apt-get install -y {pkg}" for pkg in failed_packages)
        raise RuntimeError(
            f"apt-get install failed for: {' '.join(failed_packages)}\n"
            f"Try to install them yourself with\n{cmds}"
        )


def add_git_ignore_patterns(
    project_path: str | Path,
    patterns: list[str],
    opts: dict[str, str] | None = None,
) -> dict[str, object]:
    project_path = Path(project_path)
    gitignore_path = project_path.joinpath(".gitignore")
    opts = opts or {}

    if gitignore_path.exists() and not gitignore_path.is_file():
        return {
            "updated": False,
            "added": [],
            "alreadyPresent": [],
            "ignoreDidNotExist": True,
        }
    if not gitignore_path.exists():
        return {
            "updated": False,
            "added": [],
            "alreadyPresent": list(patterns),
            "ignoreDidNotExist": False,
        }

    original = gitignore_path.read_text()
    has_trailing_newline = original.endswith("\n") or original.endswith("\r\n")
    text = original.replace("\r\n", "\n")

    existing_lines = text.split("\n")
    existing_set = {l.strip() for l in existing_lines if l.strip()}

    cleaned_patterns = [p.strip() for p in patterns if p.strip()]
    added: list[str] = []
    already_present: list[str] = []

    for pattern in cleaned_patterns:
        if pattern in existing_set:
            already_present.append(pattern)
        else:
            added.append(pattern)

    if not added:
        return {
            "updated": False,
            "added": [],
            "alreadyPresent": already_present,
            "ignoreDidNotExist": False,
        }

    new_lines: list[str] = []
    needs_newline = not has_trailing_newline and len(text) > 0
    if needs_newline:
        new_lines.append("")

    ends_with_blank_line = (
        len(existing_lines) > 0 and existing_lines[-1].strip() == ""
    )
    if has_trailing_newline and not ends_with_blank_line:
        new_lines.append("")

    comment = opts.get("comment", "").strip()
    if added and comment:
        header = comment if comment.startswith("#") else f"# {comment}"
        new_lines.append(header)

    new_lines.extend(added)

    updated_text = text + "\n".join(new_lines) + "\n"
    gitignore_path.write_text(updated_text)

    return {
        "updated": True,
        "added": added,
        "alreadyPresent": already_present,
        "ignoreDidNotExist": False,
    }


__all__ = [
    "add_git_ignore_patterns",
    "apt_install",
    "detect_python_command",
    "ensure_git_and_lfs",
    "ensure_port_audio",
    "ensure_python",
    "get_project_directory",
    "get_project_toml",
    "get_system_deps",
    "is_version_at_least",
    "parse_version",
]
