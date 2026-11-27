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

"""Arm tests never read the calibrations saved on the machine running them."""

from __future__ import annotations

from pathlib import Path

import pytest


@pytest.fixture(autouse=True)
def calibration_root(
    tmp_path_factory: pytest.TempPathFactory, monkeypatch: pytest.MonkeyPatch
) -> Path:
    """An empty directory Damiao connections look in for saved calibrations."""
    root = tmp_path_factory.mktemp("arms")
    try:
        from dimos.robot.manipulators.damiao import connection
    except ImportError:
        # Without the Damiao SDK no Damiao connection runs, so none reads one.
        return root
    monkeypatch.setattr(connection, "CALIBRATION_ROOT", root)
    return root
