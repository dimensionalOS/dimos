# Copyright 2025-2026 Dimensional Inc.
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

from pathlib import Path

from dimos.robot.galaxea.r1pro.blueprints.basic.r1pro_coordinator import (
    r1pro_control,
    r1pro_coordinator,
)
from dimos.robot.galaxea.r1pro.blueprints.navigation.r1pro_nav import r1pro_nav
from dimos.robot.galaxea.r1pro.vendor_stack import (
    VENDOR_PROFILE,
    R1ProVendorStack,
    R1ProVendorStackConfig,
    boot_command,
)


def _config(tmp_path: Path, **overrides: object) -> R1ProVendorStackConfig:
    script = tmp_path / "robot_startup.sh"
    script.write_text("#!/bin/bash\n")
    return R1ProVendorStackConfig(startup_script=str(script), **overrides)


def test_boots_the_vendor_profile_when_its_sessions_are_missing(tmp_path: Path) -> None:
    config = _config(tmp_path)
    assert boot_command(config, {"build"}) == [
        "bash",
        str(tmp_path / "robot_startup.sh"),
        "boot",
        VENDOR_PROFILE,
    ]
    assert boot_command(config, {"hdas"}) is not None


def test_leaves_a_running_stack_alone(tmp_path: Path) -> None:
    assert boot_command(_config(tmp_path), {"hdas", "mobiman", "tools"}) is None


def test_disabled_or_not_installed_boots_nothing(tmp_path: Path) -> None:
    assert boot_command(_config(tmp_path, enabled=False), set()) is None
    missing = R1ProVendorStackConfig(startup_script=str(tmp_path / "absent.sh"))
    assert boot_command(missing, set()) is None


def _has_vendor_stack(blueprint) -> bool:
    return any(atom.module is R1ProVendorStack for atom in blueprint.active_blueprints)


def test_blueprints_on_our_mid360_driver_opt_in_and_stop_the_vendor_lidar() -> None:
    assert not _has_vendor_stack(r1pro_control())
    for blueprint in (r1pro_coordinator, r1pro_nav):
        atom = next(a for a in blueprint.active_blueprints if a.module is R1ProVendorStack)
        assert atom.kwargs["stop_vendor_lidar"] is True
    assert R1ProVendorStackConfig().stop_vendor_lidar is False
