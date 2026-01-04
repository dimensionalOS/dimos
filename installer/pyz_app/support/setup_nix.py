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

# Helper for generating a sample Nix flake for Dimos.
from __future__ import annotations

from collections.abc import Iterable
import json
from pathlib import Path

from . import prompt_tools as p
from .bundled_data import FLAKE_TEMPLATE
from .constants import minimum_nix_version
from .installer_status import installer_status
from .misc import ProgressRenderer, is_version_at_least, parse_version
from .shell_tooling import command_exists, run_command


def setup_nix_flake(project_dir: str | Path) -> Path:
    """Write flake.example.nix with the installer flake contents."""
    project_dir = Path(project_dir)
    example_path = project_dir / "flake.example.nix"
    if example_path.exists():
        if not p.ask_yes_no(f"{example_path.name} exists. Overwrite?"):
            return example_path
    example_path.write_text(FLAKE_TEMPLATE)
    return example_path


def ensure_nix_exists(min_version: str = minimum_nix_version) -> None:
    """Ensure nix is installed and meets the minimum version, offering install/upgrade."""
    if not command_exists("nix"):
        p.sub_header("- nix not detected")
        if not p.ask_yes_no("Install nix now?"):
            raise RuntimeError("nix is required for this option.")
        install_cmd = "curl -L https://nixos.org/nix/install | sh"
        res = run_command(
            ["sh", "-c", install_cmd],
            print_command=True,
            dry_run=installer_status["dry_run"],
        )
        if res.code != 0:
            raise RuntimeError("Failed to install nix.")

    ver_res = run_command(["nix", "--version"], capture_output=True)
    version_text = (ver_res.stdout or ver_res.stderr or "").strip()
    parsed = parse_version(version_text) or ""
    if not parsed:
        raise RuntimeError("Could not determine nix version.")

    if not is_version_at_least(parsed, min_version):
        p.sub_header(f"- nix version {parsed} is below required {min_version}")
        if not p.ask_yes_no("Update nix now?"):
            raise RuntimeError("nix is too old; please update to continue.")
        update_res = run_command(
            ["nix", "upgrade-nix"],
            print_command=True,
            dry_run=installer_status["dry_run"],
        )
        if update_res.code != 0:
            raise RuntimeError("Failed to update nix.")
        # Re-check
        ver_res = run_command(["nix", "--version"], capture_output=True)
        version_text = (ver_res.stdout or ver_res.stderr or "").strip()
        parsed = parse_version(version_text) or ""
        if not parsed or not is_version_at_least(parsed, min_version):
            raise RuntimeError("nix update did not succeed; version still too old.")


def ensure_flakes_enabled() -> None:
    """Ensure the user's nix.conf has flakes enabled."""
    config_path = Path.home() / ".config" / "nix" / "nix.conf"
    flakes_enabled = False
    if config_path.exists():
        text = config_path.read_text(encoding="utf-8")
        if "experimental-features" in text and "nix-command" in text and "flakes" in text:
            flakes_enabled = True

    if flakes_enabled:
        return

    p.sub_header("- nix flakes not detected in configuration")
    if not p.ask_yes_no("Enable nix flakes now? (required to proceed)"):
        raise RuntimeError("Cannot continue without nix flakes enabled.")

    config_path.parent.mkdir(parents=True, exist_ok=True)
    line = "experimental-features = nix-command flakes\n"
    with config_path.open("a", encoding="utf-8") as f:
        f.write(line)


def nix_install(package_names: list[str]) -> None:
    """Install packages via nix profile install with basic progress and reentrancy."""
    if not package_names:
        return

    ensure_nix_exists()
    ensure_flakes_enabled()

    progress = ProgressRenderer(len(package_names)) if len(package_names) > 1 else None

    failed_packages: list[str] = []
    for idx, each_pkg in enumerate(package_names, start=1):
        if progress and progress.enabled:
            progress.set_current(idx, each_pkg)

        installed = False
        list_res = run_command(
            ["nix", "profile", "list", "--json"],
            capture_output=True,
            dry_run=installer_status["dry_run"],
        )
        if list_res.code == 0 and list_res.stdout:
            try:
                profiles = json.loads(list_res.stdout)
                for entry in profiles:
                    name = entry.get("name") or entry.get("packageName") or ""
                    if name.endswith(f"#{each_pkg}") or name == each_pkg:
                        installed = True
                        break
            except Exception:
                pass

        if installed:
            p.sub_header(f"- ✅ looks like {p.highlight(each_pkg)} is already installed")
            continue

        p.sub_header(f"\n- installing {p.highlight(each_pkg)}")
        # remove pkgs prefix
        if each_pkg.startswith("pkgs."):
            each_pkg = each_pkg.split(".", 1)[1]

        install_cmd = ["nix", "profile", "install", f"nixpkgs#{each_pkg}"]

        if progress and progress.enabled:
            output_lines: list[str] = []

            def _on_line(line: str) -> None:
                output_lines.append(line.rstrip("\n"))
                progress.add_output(line)

            install_res = run_command(
                install_cmd,
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
                install_cmd,
                print_command=True,
                dry_run=installer_status["dry_run"],
            )

        if install_res.code != 0:
            failed_packages.append(each_pkg)

    if progress and progress.enabled:
        progress.finish()

    if failed_packages:
        cmds = "\n".join(f"    nix profile install nixpkgs#{pkg}" for pkg in failed_packages)
        raise RuntimeError(
            f"nix install failed for: {' '.join(failed_packages)}\n"
            f"Try to install them yourself with\n{cmds}"
        )


__all__ = [
    "FLAKE_TEMPLATE",
    "ensure_flakes_enabled",
    "ensure_nix_exists",
    "nix_install",
    "setup_nix_flake",
]
