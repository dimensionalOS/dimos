#!/usr/bin/env python3
from __future__ import annotations

import sys

from .phases.phase00_logo_and_basic_checks import phase0
from .phases.phase01_all_system_dependencies import phase1
from .phases.phase02_check_absolutely_necessary_tools import phase2
from .phases.phase03_pip_install_dimos import phase3
from .phases.phase04_dimos_check import phase4
from .phases.phase05_env_setup import phase5
from .support import prompt_tools as p


def main() -> int:
    try:
        system_analysis, selected_features = phase0()
        phase1(system_analysis, selected_features)
        phase2(system_analysis, selected_features)
        phase3(system_analysis, selected_features)
        phase4()
        phase5()
        return 0
    except KeyboardInterrupt:
        print("\nCancelled by user.")
        return 130
    except SystemExit as exc:
        try:
            return int(exc.code) if exc.code is not None else 1
        except Exception:
            return 1
    except Exception as exc:
        p.error(f"Installer failed: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
