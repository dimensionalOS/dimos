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

from .. import prompt_tools as p
from ..constants import dependency_list_apt_packages
from ..dax import command_exists, run_command
from ..misc import apt_install

APT_PACKAGES = [
    "build-essential",
    "python3-dev",
    "python3-pip",
    "python3-setuptools",
    "python3-wheel",
    "pkg-config",
    "libopenblas-dev",
    "liblapack-dev",
    "gfortran",
]

EXTRA_PACKAGES = [pkg for pkg in APT_PACKAGES if pkg not in dependency_list_apt_packages]


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


def setup_cpu_feature(*, assume_sys_deps_installed: bool = False) -> None:
    p.clear_screen()
    p.header("Optional Feature: CPU inference stack")

    if not assume_sys_deps_installed:
        print("- Likely system dependencies needed for CPU inference (onnxruntime, ctransformers):")
        for pkg in EXTRA_PACKAGES:
            print(f"  • {pkg}")
        if EXTRA_PACKAGES:
            installed = _maybe_install_apt_deps(EXTRA_PACKAGES)
            proceed = installed or p.confirm(
                "Proceed to pip installation (continue even if some system deps may be missing)?"
            )
            if not proceed:
                p.error(
                    "Please install the listed system dependencies, then rerun this feature installer."
                )
                return
        else:
            p.boring_log("- No additional system dependencies beyond the core set.")

    res = run_command(["pip", "install", "dimos[cpu]"])
    if res.code != 0:
        p.error(
            "pip install dimos[cpu] failed. Please ensure system deps are installed and try again."
        )
        p.error("If issues persist, reinstall system deps or build from source with verbose logs.")
        return
    p.boring_log("- dimos[cpu] installed successfully")


__all__ = ["setup_cpu_feature"]
