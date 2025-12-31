#!/usr/bin/env python3
from __future__ import annotations

from ..constants import dependencyListAptPackages
from ..dax import command_exists, run_command
from ..misc import apt_install
from .. import prompt_tools as p

APT_PACKAGES = [
    "build-essential",
    "python3-dev",
    "python3-pip",
    "python3-setuptools",
    "python3-wheel",
    "libxml2-dev",
    "libxslt1-dev",
]

EXTRA_PACKAGES = [pkg for pkg in APT_PACKAGES if pkg not in dependencyListAptPackages]


def _maybe_install_apt_deps(packages: list[str]) -> bool:
    if not packages:
        return False
    if not command_exists("apt-get"):
        p.boring_log("- apt-get not available; please install these system dependencies manually")
        return False
    install_deps = p.confirm(
        "Detected apt-get. Install required system dependencies automatically? (sudo may prompt for a password)"
    )
    if not install_deps:
        return False
    try:
        apt_install(packages)
        return True
    except Exception as error:  # pragma: no cover - interactive helper
        p.error(str(error) or "Failed to install some system dependencies.")
        return False


def setup_drone_feature(*, assume_sys_deps_installed: bool = False) -> None:
    p.clear_screen()
    p.header("Optional Feature: Drone / MAVLink")

    if not assume_sys_deps_installed:
        print("- Likely system dependencies needed for pymavlink:")
        for pkg in EXTRA_PACKAGES:
            print(f"  • {pkg}")
        if EXTRA_PACKAGES:
            installed = _maybe_install_apt_deps(EXTRA_PACKAGES)
            proceed = installed or p.confirm(
                "Proceed to pip installation (continue even if some system deps may be missing)?"
            )
            if not proceed:
                p.error("Please install the listed system dependencies, then rerun this feature installer.")
                return
        else:
            p.boring_log("- No additional system dependencies beyond the core set.")

    res = run_command(["pip", "install", "dimos[drone]"])
    if res.code != 0:
        p.error("pip install dimos[drone] failed. Please ensure system deps are installed and try again.")
        p.error("If issues persist, reinstall system deps or install pymavlink with verbose logs.")
        return
    p.boring_log("- dimos[drone] installed successfully")


__all__ = ["setup_drone_feature"]
