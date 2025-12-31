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

from ..support import prompt_tools as p
from ..support.constants import dependency_list_human_names
from ..support.dax import command_exists
from ..support.get_tool_check_results import get_tool_check_results
from ..support.misc import (
    apt_install,
    brew_install,
    ensure_homebrew,
    ensure_xcode_cli_tools,
    get_system_deps,
)


def phase1(system_analysis, selected_features):
    p.clear_screen()
    p.header("Next Phase: System Dependency Install")
    if system_analysis is None:
        system_analysis = get_tool_check_results()

    deps = get_system_deps(selected_features or None)
    mention_system_dependencies()
    print()
    print()

    tools_were_auto_installed = False
    os_info = system_analysis.get("os", {})
    if os_info.get("name") == "debian_based":
        p.boring_log("Detected Debian-based OS")
        install_deps = p.confirm(
            "Install these system dependencies for you via apt-get? (NOTE: sudo may prompt for a password)"
        )
        if install_deps:
            p.boring_log("- this may take a few minutes...")
            try:
                apt_install(deps["apt_deps"])
                tools_were_auto_installed = True
            except Exception as error:
                p.error(getattr(error, "message", None) or str(error))
        else:
            print("- skipping automatic installation.")
            proceed = p.confirm("Proceed to the next step without installing system dependencies?")
            if not proceed:
                print("- ❌ Please install the listed dependencies and rerun.")
                raise SystemExit(1)
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
        if p.confirm("Install these system dependencies for you via Homebrew?"):
            dependencies = deps["brew_deps"]
            print(f"""dependencies = {dependencies}""")
            brew_install(deps["brew_deps"])
            tools_were_auto_installed = True
            # try:
            # except Exception as err:
            #     p.error(str(err))
        else:
            proceed = p.confirm("Proceed to the next step without installing system dependencies?")
            if not proceed:
                print("- ❌ Please install the listed dependencies and rerun.")
                raise SystemExit(1)

    if not tools_were_auto_installed:
        p.confirm(
            "I can't confirm that all those tools are installed\nPress enter to continue anyway, or CTRL+C to cancel and install them yourself"
        )

    p.confirm("Press enter to continue to next phase")


def mention_system_dependencies():
    print("- Dimos will likely need the following system dependencies:")
    missing_deps = [dep for dep in dependency_list_human_names if not command_exists(dep)]
    for dep in missing_deps:
        print(f"  • {p.highlight(dep)}")
