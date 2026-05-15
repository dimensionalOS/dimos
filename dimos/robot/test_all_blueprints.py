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

from dimos.core.coordination.blueprints import Blueprint
from dimos.robot.all_blueprints import all_blueprints
from dimos.robot.get_all_blueprints import get_blueprint_by_name

# Optional dependencies that are allowed to be missing
OPTIONAL_DEPENDENCIES = {"pyrealsense2", "pyzed", "geometry_msgs", "turbojpeg", "unitree_sdk2py"}
OPTIONAL_ERROR_SUBSTRINGS = {
    "Unable to locate turbojpeg library automatically",
    "ZED SDK not installed",
    "Descriptors cannot be created directly",
}

# These need git LFS, so can't be run on the ubuntu runners.
SELF_HOSTED_BLUEPRINTS = frozenset(
    {
        "coordinator-basic",
        "coordinator-cartesian-ik-mock",
        "coordinator-cartesian-ik-piper",
        "coordinator-combined-xarm6",
        "coordinator-mock",
        "coordinator-piper",
        "coordinator-servo-xarm6",
        "coordinator-teleop-dual",
        "coordinator-teleop-piper",
        "coordinator-teleop-xarm6",
        "coordinator-teleop-xarm7",
        "coordinator-velocity-xarm6",
        "coordinator-xarm6",
        "coordinator-xarm7",
        "dual-xarm6-planner",
        "teleop-quest-dual",
        "teleop-quest-piper",
        "teleop-quest-rerun",
        "teleop-quest-xarm6",
        "teleop-quest-xarm7",
        "unitree-g1-nav-sim",
        "xarm-perception",
        "xarm-perception-agent",
        "xarm-perception-sim",
        "xarm-perception-sim-agent",
        "xarm6-planner-only",
        "xarm7-planner-coordinator",
        "xarm7-planner-coordinator-agent",
    }
)

UBUNTU_BLUEPRINTS = sorted(set(all_blueprints) - SELF_HOSTED_BLUEPRINTS)
SELF_HOSTED_BLUEPRINTS = sorted(SELF_HOSTED_BLUEPRINTS)


def _check_blueprint(blueprint_name: str) -> None:
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


def test_old_self_hosted_blueprints() -> None:
    """Validate no non-existent name in SELF_HOSTED_BLUEPRINTS."""
    unused_names = set(SELF_HOSTED_BLUEPRINTS) - set(all_blueprints)
    assert not unused_names


@pytest.mark.parametrize("blueprint_name", UBUNTU_BLUEPRINTS)
def test_blueprint_is_valid(blueprint_name: str) -> None:
    """Validate blueprints that should import on the ubuntu-latest runner."""
    _check_blueprint(blueprint_name)


@pytest.mark.self_hosted
@pytest.mark.parametrize("blueprint_name", SELF_HOSTED_BLUEPRINTS)
def test_self_hosted_blueprint_is_valid(blueprint_name: str) -> None:
    """Validate blueprints that need heavy deps or LFS — self-hosted runner only."""
    _check_blueprint(blueprint_name)
