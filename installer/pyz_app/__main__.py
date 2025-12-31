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

from .phases.phase00_logo_and_basic_checks import phase0
from .phases.phase01_all_system_dependencies import phase1
from .phases.phase02_check_absolutely_necessary_tools import phase2
from .phases.phase03_pip_install_dimos import phase3
from .phases.phase04_dimos_check import phase4
from .phases.phase05_env_setup import phase5
from .support import prompt_tools as p


def main():
    system_analysis, selected_features = phase0()
    phase1(system_analysis, selected_features)
    phase2(system_analysis, selected_features)
    phase3(system_analysis, selected_features)
    phase4()
    phase5()


if __name__ == "__main__":
    main()
