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

import sys
import os

from ..support import prompt_tools as p
from ..support.shell_tooling import command_exists
from ..support.get_system_analysis import get_system_analysis
from ..support.misc import (
    apt_install,
    brew_install,
    ensure_xcode_cli_tools,
    get_system_deps,
)
from ..support.dependency_minimizer import minimize_deps_based_on_prerequisites


def phase1(system_analysis, selected_features):
    p.header("Next Phase: System Dependency Install")
    if system_analysis is None:
        system_analysis = get_system_analysis()

    deps = get_system_deps(selected_features or None)
    
    is_nixos = os.path.exists('/etc/NIXOS')
    if sys.platform == "darwin":
        mention_system_dependencies(deps["human_names_from_brew"])
    else:
        if is_nixos:
            mention_system_dependencies(deps["human_names_from_nix"])
        else:
            mention_system_dependencies(deps["human_names_from_apt"])
    
    print()
    print()

    tools_were_auto_installed = False
    os_info = system_analysis.get("os", {})
    if os_info.get("name") == "debian_based":
        p.boring_log("Detected Debian-based OS")
        install_deps = p.ask_yes_no(
            "Install these system dependencies for you via apt-get? (NOTE: sudo may prompt for a password)"
        )
        if install_deps:
            p.boring_log("- this may take a few minutes...")
            try:
                apt_install(deps["apt_deps"])
                tools_were_auto_installed = True
            except Exception as error:
                p.error(getattr(error, "message", None) or str(error))
        # else:
        #     print("- skipping automatic installation.")
        #     proceed = p.confirm("Proceed to the next step without installing system dependencies?")
        #     if not proceed:
        #         print("- ❌ Please install the listed dependencies and rerun.")
        #         raise SystemExit(1)
    elif os_info.get("name") == "macos":
        p.boring_log("Detected macOS")
        try:
            ensure_xcode_cli_tools()
        except Exception as err:
            p.error(str(err))
            p.error(
                "The xcode cli tools are absolutely needed, please install them then rerun this script"
            )
            exit(1)
        if p.ask_yes_no("Install these system dependencies for you via Homebrew?"):
            try:
                dependencies = deps["brew_deps"]
                brew_install(deps["brew_deps"])
                tools_were_auto_installed = True
            except Exception as err:
                p.error(str(err))
        # else:
        #     proceed = p.confirm("Proceed to the next step without installing system dependencies?")
        #     if not proceed:
        #         print("- ❌ Please install the listed dependencies and rerun.")
        #         raise SystemExit(1)
    
    print()
    print()
    print()
    if not tools_were_auto_installed:
        p.confirm(
            "I can't always confirm that all those tools are installed\nPress enter if you've think you've got most of them\nor CTRL+C to cancel and install them yourself"
        )
    else:
        p.boring_log("- all system dependencies appear to be installed")
        p.confirm("Press enter to continue to next phase")


def mention_system_dependencies(human_names_deps):
    print("- Dimos will likely need the following system dependencies:")
    minimized_deps = minimize_deps_based_on_prerequisites(human_names_deps)
    missing_deps = [dep for dep in minimized_deps if not command_exists(dep)]
    for dep in missing_deps:
        print(f"  • {p.highlight(dep)}")
