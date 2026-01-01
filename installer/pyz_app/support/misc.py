#!/usr/bin/env python3
# Copyright 2025 Dimensional Inc.
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

from functools import cache, lru_cache
from collections import deque
import os
from pathlib import Path
import re
import shutil
import sys
from typing import Any, Callable, Deque, Iterable, List
import requests

from . import pip_dependency_database as dep_db, prompt_tools as p
from .shell_tooling import command_exists, run_command
from .constants import dependency_human_names_set, dependency_apt_packages_set_minimal, dependency_nix_packages_set_minimal, dependency_brew_set_minimal
from .installer_status import installer_status

try:
    import tomllib
except ModuleNotFoundError:  # pragma: no cover - fallback for older interpreters
    import tomli as tomllib  # type: ignore

_project_directory: Path | None = None
_already_called_apt_get_update = False
_already_called_brew_update = False


class ProgressRenderer:
    """Minimal multi-line progress display with rolling output buffer."""

    def __init__(self, total: int, *, buffer_lines: int = 5) -> None:
        # Temporarily disable interactive progress rendering due to display issues.
        self.total = max(total, 1)
        self.buffer: Deque[str] = deque(maxlen=buffer_lines)
        self.current_index = 0
        self.current_name = ""
        self.rendered_lines = 0
        self.enabled = False
        self.term_width = shutil.get_terminal_size(fallback=(80, 24)).columns
        self.color_bar = "\x1b[32m"  # green
        self.color_label = "\x1b[36m"  # cyan
        self.reset = "\x1b[0m"

    def set_current(self, index: int, name: str) -> None:
        self.current_index = index
        self.current_name = name
        self.render()

    def add_output(self, line: str) -> None:
        if not self.enabled:
            return
        # strip ANSI escape sequences to avoid breaking layout
        cleaned = re.sub(r"\x1b\[[0-9;]*[A-Za-z]", "", line)
        cleaned = cleaned.rstrip("\n").rstrip("\r")
        self.buffer.append(cleaned)
        self.render()

    def render(self) -> None:
        if not self.enabled:
            return

        self.term_width = shutil.get_terminal_size(fallback=(80, 24)).columns
        bar_width = max(10, min(40, self.term_width - 30))
        completed = max(0, self.current_index - 1)
        fill = int(bar_width * (completed / self.total))
        bar_body = f"{'━' * fill}{' ' * (bar_width - fill)}"
        bar = f"{self.color_bar}┏{bar_body}┓{self.reset}"
        label = f"{self.color_label}{self.current_name}{self.reset}"
        progress_line = f"{bar} ({self.current_index}/{self.total}) installing {label}"

        def _visible_len(text: str) -> int:
            # crude ANSI stripper for length calculations
            return len(re.sub(r"\x1b\[[0-9;]*[A-Za-z]", "", text))

        def clip(text: str) -> str:
            if _visible_len(text) <= self.term_width:
                return text + " " * max(0, self.term_width - _visible_len(text))
            trimmed = text
            while _visible_len(trimmed) > self.term_width:
                trimmed = trimmed[:-1]
            return trimmed + " " * max(0, self.term_width - _visible_len(trimmed))

        max_lines = 1 + self.buffer.maxlen
        lines: List[str] = [clip(progress_line)]
        lines.extend(clip(l) for l in list(self.buffer)[-self.buffer.maxlen :])
        lines = lines[:max_lines]

        # Move cursor up to the start of the previous block
        if self.rendered_lines:
            sys.stdout.write(f"\x1b[{self.rendered_lines}F")

        sys.stdout.write("\x1b[?25l")  # hide cursor
        total_lines = len(lines)
        for idx, line in enumerate(lines):
            sys.stdout.write("\r")
            sys.stdout.write(line.ljust(self.term_width))
            sys.stdout.write("\x1b[K")
            if idx != total_lines - 1:
                sys.stdout.write("\n")
        # Clear any leftover lines from previous render
        if self.rendered_lines > total_lines:
            for _ in range(self.rendered_lines - total_lines):
                sys.stdout.write("\n")
                sys.stdout.write(" " * self.term_width)
        sys.stdout.flush()
        self.rendered_lines = total_lines

    def finish(self) -> None:
        if not self.enabled:
            return
        # Ensure cursor lands after the rendered block and becomes visible
        sys.stdout.write("\x1b[?25h\n")
        sys.stdout.flush()


@cache
def get_project_toml(branch: str = "main") -> dict[str, Any]:
    return tomllib.loads(dep_db.PROJECT_TOML)

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
        pip_dep = pip_dep.lower()
        
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
    apt_deps = apt_deps | dependency_apt_packages_set_minimal
    nix_deps = nix_deps | dependency_nix_packages_set_minimal
    brew_deps = brew_deps | dependency_brew_set_minimal
    combined_deps = set(apt_deps) | set(nix_deps) | set(brew_deps)
    return {
        "apt_deps": sorted(apt_deps),
        "nix_deps": sorted(nix_deps),
        "brew_deps": sorted(brew_deps),
        "pip_deps": sorted(pip_deps),
        "human_names_all": sorted(dependency_human_names_set | { dep_db.DEP_2_HUMAN_NAME.get(dep, dep) for dep in combined_deps }),
        "human_names_from_apt": sorted(dependency_human_names_set | { dep_db.DEP_2_HUMAN_NAME.get(dep, dep) for dep in apt_deps }),
        "human_names_from_brew": sorted(dependency_human_names_set | { dep_db.DEP_2_HUMAN_NAME.get(dep, dep) for dep in brew_deps }),
        "human_names_from_nix": sorted(dependency_human_names_set | { dep_db.DEP_2_HUMAN_NAME.get(dep, dep) for dep in nix_deps }),
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
    p.boring_log("- checking git")
    if not command_exists("git"):
        raise RuntimeError("- ❌ git is required. Please install git and rerun.")
    p.boring_log("- checking git-lfs")
    git_lfs_res = run_command(["git", "lfs", "version"])  # intentionally not part of dry_run
    if git_lfs_res.code != 0:
        raise RuntimeError("- ❌ git-lfs is required. Please install git-lfs and rerun.")


def ensure_port_audio() -> None:
    # TODO: only run this check if pyaudio is in the pip dependency list. Its in the pyprojct.toml core right now but it might not be forever
    p.boring_log("- checking if portaudio is available")
    port_audio_res = run_command(  # intentionally not part of dry_run
        ["pkg-config", "--modversion", "portaudio-2.0"], print_command=True, capture_output=True
    )
    if port_audio_res.code != 0:
        raise RuntimeError("- ❌ portaudio is required. Please install portaudio and rerun.")


def ensure_python() -> str:
    p.boring_log("- checking python version")
    python_cmd = detect_python_command()
    if not python_cmd:
        raise RuntimeError("- ❌ Python 3.10+ is required but was not found.")
    version_res = run_command([python_cmd, "--version"], capture_output=True)  # intentionally not part of dry_run
    version_text = (version_res.stdout or version_res.stderr or "").strip()
    parsed = parse_version(version_text)
    if not parsed or not is_version_at_least(parsed, "3.10.0"):
        raise RuntimeError(f"- ❌ Python 3.10+ required. Detected: {parsed or 'unknown'}")
    return python_cmd


def get_project_directory() -> Path:
    global _project_directory
    if _project_directory is None:
        if installer_status.get("template_repo"):
            _project_directory = Path.cwd()
        else:
            print("Dimos needs to be installed to a project (not just a global install)")
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

    progress = ProgressRenderer(len(package_names)) if len(package_names) > 1 else None

    if not _already_called_apt_get_update:
        update_res = run_command(
            ["sudo", "apt-get", "update"],
            print_command=True,
            dry_run=installer_status["dry_run"],
        )
        if update_res.code != 0:
            raise RuntimeError(f"sudo apt-get update failed: {update_res.code}")
        _already_called_apt_get_update = True

    failed_packages: list[str] = []
    for idx, each_pkg in enumerate(package_names, start=1):
        if progress and progress.enabled:
            progress.set_current(idx, each_pkg)
        res = run_command(["dpkg", "-s", each_pkg], dry_run=installer_status["dry_run"])
        if res.code == 0:
            p.sub_header(f"- ✅ looks like {p.highlight(each_pkg)} is already installed")
            continue

        p.sub_header(f"\n- installing {p.highlight(each_pkg)}")
        if progress and progress.enabled:
            output_lines: list[str] = []

            def _on_line(line: str) -> None:
                output_lines.append(line.rstrip("\n"))
                progress.add_output(line)

            install_res = run_command(
                ["sudo", "apt-get", "install", "-y", each_pkg],
                print_command=True,
                dry_run=installer_status["dry_run"],
                stream_callback=_on_line,
            )
            if install_res.code != 0:
                progress.finish()
                if output_lines:
                    print("\n".join(output_lines))
        else:
            install_res = run_command(
                ["sudo", "apt-get", "install", "-y", each_pkg],
                print_command=True,
                dry_run=installer_status["dry_run"],
            )
        if install_res.code != 0:
            failed_packages.append(each_pkg)

    if progress and progress.enabled:
        progress.finish()

    if failed_packages:
        cmds = "\n".join(f"    sudo apt-get install -y {pkg}" for pkg in failed_packages)
        raise RuntimeError(
            f"apt-get install failed for: {' '.join(failed_packages)}\n"
            f"Try to install them yourself with\n{cmds}"
        )


def ensure_xcode_cli_tools() -> None:
    p.boring_log("- checking Xcode Command Line Tools")
    try:
        run_command(
            ["xcode-select", "-p"], check=True, capture_output=True
        )  # intentionally not part of dry_run
    except Exception:
        if p.ask_yes_no("Install Xcode Command Line Tools now?"):
            res = run_command(
                ["xcode-select", "--install"], check=True, dry_run=installer_status["dry_run"]
            )
            if res.code != 0:
                raise RuntimeError("Failed to trigger Xcode Command Line Tools installation.")


def ensure_homebrew() -> None:
    if command_exists("brew"):
        p.boring_log("- homebrew found")
        return
    ensure_xcode_cli_tools()
    p.boring_log("- homebrew not found")
    if not p.ask_yes_no("Install Homebrew now? (will run the official install script)"):
        raise RuntimeError("Homebrew is required for automatic dependency install.")

    url = "https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh"
    dest = Path("/tmp/brew_install.sh")
    p.boring_log(f"- downloading Homebrew installer to {dest}")
    resp = requests.get(url, timeout=30)
    if resp.status_code != 200:
        raise RuntimeError(f"Failed to download Homebrew installer: HTTP {resp.status_code}")
    dest.write_bytes(resp.content)
    dest.chmod(0o755)

    res = run_command(
        ["/bin/bash", str(dest)],
        check=True,
        print_command=True,
        dry_run=installer_status["dry_run"],
    )
    if res.code != 0:
        raise RuntimeError("Homebrew installation failed.")


def brew_install(package_names: list[str]) -> None:
    global _already_called_brew_update
    if not package_names:
        return

    progress = ProgressRenderer(len(package_names)) if len(package_names) > 1 else None

    ensure_homebrew()
    if not _already_called_brew_update:
        p.boring_log("Running brew update")
        res = run_command(["brew", "update"], print_command=True, dry_run=installer_status["dry_run"])
        if res.code != 0:
            raise RuntimeError(f"brew update failed: {res.code}")
        _already_called_brew_update = True

    failed: list[str] = []
    for idx, pkg in enumerate(package_names, start=1):
        if progress and progress.enabled:
            progress.set_current(idx, pkg)
        res = run_command(["brew", "list", pkg], capture_output=True) # intentionally not dry_run
        if res.code == 0:
            p.sub_header(f"- ✅ looks like {p.highlight(pkg)} is already installed")
            continue
        p.sub_header(f"\n- installing {p.highlight(pkg)}")
        if progress and progress.enabled:
            output_lines: list[str] = []

            def _on_line(line: str) -> None:
                output_lines.append(line.rstrip("\n"))
                progress.add_output(line)

            install_res = run_command(
                ["brew", "install", pkg],
                print_command=True,
                dry_run=installer_status["dry_run"],
                stream_callback=_on_line,
            )
            if install_res.code != 0:
                progress.finish()
                if output_lines:
                    print("\n".join(output_lines))
        else:
            install_res = run_command(
                ["brew", "install", pkg], print_command=True, dry_run=installer_status["dry_run"]
            )
        if install_res.code != 0:
            failed.append(pkg)

    if progress and progress.enabled:
        progress.finish()

    if failed:
        raise RuntimeError(f"brew install failed for: {' '.join(failed)}")


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
            "already_present": [],
            "ignore_did_not_exist": True,
        }
    if not gitignore_path.exists():
        return {
            "updated": False,
            "added": [],
            "already_present": list(patterns),
            "ignore_did_not_exist": False,
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
            "already_present": already_present,
            "ignore_did_not_exist": False,
        }

    new_lines: list[str] = []
    needs_newline = not has_trailing_newline and len(text) > 0
    if needs_newline:
        new_lines.append("")

    ends_with_blank_line = len(existing_lines) > 0 and existing_lines[-1].strip() == ""
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
        "already_present": already_present,
        "ignore_did_not_exist": False,
    }


if __name__ == "__main__":
    # Self-test: exercise ProgressRenderer with dummy commands.
    pkgs = ["alpha", "beta", "gamma"]
    progress = ProgressRenderer(len(pkgs))
    for idx, name in enumerate(pkgs, start=1):
        progress.set_current(idx, name)
        # Simulate streaming output
        for line in [f"{name}: step 1", f"{name}: step 2", f"{name}: done"]:
            progress.add_output(line)
    progress.finish()
    print("ProgressRenderer self-test completed")


__all__ = [
    "add_git_ignore_patterns",
    "apt_install",
    "brew_install",
    "detect_python_command",
    "ensure_git_and_lfs",
    "ensure_homebrew",
    "ensure_port_audio",
    "ensure_python",
    "ensure_xcode_cli_tools",
    "get_project_directory",
    "get_project_toml",
    "get_system_deps",
    "is_version_at_least",
    "parse_version",
]
