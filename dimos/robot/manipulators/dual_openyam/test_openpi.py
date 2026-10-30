# Copyright 2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0.

from dimos.robot.get_all_blueprints import get_by_name
from dimos.robot.manipulators.dual_openyam.blueprints.openpi import openpi_dual_openyam


def test_openpi_blueprint_is_available_to_dimos_run():
    assert get_by_name("openpi-dual-openyam") is openpi_dual_openyam
