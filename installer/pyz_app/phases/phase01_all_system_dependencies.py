#!/usr/bin/env python3
from __future__ import annotations

from ..support.constants import dependencyListHumanNames
from ..support.dax import command_exists
from ..support.get_tool_check_results import get_tool_check_results
from ..support.misc import apt_install, get_system_deps
from ..support import prompt_tools as p


def phase1(system_analysis, selected_features):
    p.clear_screen()
    p.header("Next Phase: System Dependency Install")
    if system_analysis is None:
        system_analysis = get_tool_check_results()

    deps = get_system_deps(selected_features or None)
    mention_system_dependencies()

    tools_were_auto_installed = False
    os_info = system_analysis.get("os", {})
    if os_info.get("name") == "debianBased":
        p.boring_log("Detected Debian-based OS")
        install_deps = p.confirm(
            "Install these system dependencies for you via apt-get? (NOTE: sudo may prompt for a password)"
        )
        if install_deps:
            p.boring_log("- this may take a few minutes...")
            try:
                apt_install(deps["aptDeps"])
                tools_were_auto_installed = True
            except Exception as error:
                p.error(getattr(error, "message", None) or str(error))
        else:
            print("- skipping automatic installation.")
            proceed = p.confirm("Proceed to the next step without installing system dependencies?")
            if not proceed:
                print("- ❌ Please install the listed dependencies and rerun.")
                raise SystemExit(1)

    if not tools_were_auto_installed:
        p.confirm(
            "I can't confirm that all those tools are installed\nPress enter to continue anyway, or CTRL+C to cancel and install them yourself"
        )


def mention_system_dependencies():
    print("- we will need the following system dependencies:")
    missing_deps = [dep for dep in dependencyListHumanNames if not command_exists(dep)]
    for dep in missing_deps:
        print(f"  • {p.highlight(dep)}")
