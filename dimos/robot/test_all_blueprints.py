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

import pytest

from dimos.core.blueprints import Blueprint
from dimos.robot.all_blueprints import all_blueprints
from dimos.robot.get_all_blueprints import get_blueprint_by_name

# Optional dependencies that are allowed to be missing
OPTIONAL_DEPENDENCIES = {"pyrealsense2", "pyzed", "geometry_msgs", "turbojpeg"}
OPTIONAL_ERROR_SUBSTRINGS = {
    "Unable to locate turbojpeg library automatically",
}


@pytest.mark.integration
@pytest.mark.parametrize("blueprint_name", all_blueprints.keys())
def test_all_blueprints_are_valid(blueprint_name: str) -> None:
    """Test that all blueprints in all_blueprints are valid Blueprint instances."""
    try:
        blueprint = get_blueprint_by_name(blueprint_name)
    except ModuleNotFoundError as e:
        if e.name in OPTIONAL_DEPENDENCIES:
            pytest.skip(f"Skipping due to missing optional dependency: {e.name}")
        raise
    except Exception as e:
        message = str(e)
        if any(substring in message for substring in OPTIONAL_ERROR_SUBSTRINGS):
            pytest.skip(f"Skipping due to missing optional dependency: {message}")
        raise
    assert isinstance(blueprint, Blueprint), (
        f"Blueprint '{blueprint_name}' is not a Blueprint, got {type(blueprint)}"
    )
