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

import argparse
import sys
import textwrap

print("- importing install phases")
from .phases.phase00_logo_and_basic_checks import phase0
from .phases.phase01_all_system_dependencies import phase1
from .phases.phase02_check_absolutely_necessary_tools import phase2
from .phases.phase03_pip_install_dimos import phase3
from .phases.phase04_dimos_check import phase4
from .phases.phase05_env_setup import phase5
from .support.get_system_analysis import get_system_analysis
from .support.installer_status import installer_status
from .support.misc import get_project_toml

# Simple ANSI helpers for help text
RESET = "\x1b[0m"
BOLD = "\x1b[1m"
FG_CYAN = "\x1b[36m"
FG_GREEN = "\x1b[32m"


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    epilog = textwrap.dedent(
        f"""
        {FG_GREEN}Examples:{RESET}
          {FG_CYAN}dimos_installer --list-features{RESET}
          {FG_CYAN}dimos_installer --non-interactive --features sim,cuda{RESET}
          {FG_CYAN}dimos_installer --no-system-install --no-check --no-env-setup{RESET}
          {FG_CYAN}dimos_installer --just-system-install{RESET}
        
        v0.1.15
        """
    )
    parser = argparse.ArgumentParser(
        description="Dimos installer",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=epilog,
    )
    parser.add_argument(
        "--non-interactive",
        action="store_true",
        help="Run without prompts; skip phase0 and auto-detect system info.",
    )
    parser.add_argument(
        "--no-system-install",
        action="store_true",
        help="Skip phase1 (system dependency installation).",
    )
    parser.add_argument(
        "--no-check",
        action="store_true",
        help="Skip phase4 (dimos check).",
    )
    parser.add_argument(
        "--no-env-setup",
        action="store_true",
        help="Skip phase5 (environment setup).",
    )
    parser.add_argument(
        "--just-system-install",
        action="store_true",
        help="Only run system dependency installation (phase1) and exit.",
    )
    parser.add_argument(
        "--features",
        type=str,
        help="Comma-separated list of features to enable (skips interactive selection).",
    )
    parser.add_argument(
        "--list-features",
        action="store_true",
        help="List available features and exit.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print commands without executing installer side-effects (best-effort).",
    )
    parser.add_argument(
        "--from-dimos-template",
        action="store_true",
        help=argparse.SUPPRESS,
    )
    return parser.parse_args(argv)


def main():
    args = parse_args()
    non_interactive = args.non_interactive or args.just_system_install or not sys.stdin.isatty()
    installer_status["non_interactive"] = non_interactive
    installer_status["dry_run"] = bool(args.dry_run)
    installer_status["template_repo"] = bool(getattr(args, "from_dimos_template", False))

    cli_features = []
    if args.features:
        cli_features = [f.strip() for f in args.features.split(",") if f.strip()]

    if non_interactive:
        system_analysis = get_system_analysis()
        selected_features = cli_features
    else:
        system_analysis, selected_features = phase0()
        if cli_features:
            selected_features = cli_features

    if args.list_features:
        toml_data = get_project_toml()
        optional = toml_data["project"].get("optional-dependencies", {})
        available = [f for f in optional.keys() if f != "cpu"]
        print("Available features:")
        for feat in available:
            print(f"  - {feat}")
        return

    if not args.no_system_install:
        phase1(system_analysis, selected_features)
    if args.just_system_install:
        return
    phase2(system_analysis, selected_features)
    phase3(system_analysis, selected_features)
    if not args.no_check:
        phase4()
    if not args.no_env_setup:
        phase5()


if __name__ == "__main__":
    main()
