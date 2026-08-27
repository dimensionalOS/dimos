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

import subprocess
import sys

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
    # cockpit() blueprints without the [web] extra installed.
    "needs the web extra",
}

# These need self-hosted dependencies or external robot assets.
SELF_HOSTED_BLUEPRINTS = frozenset(
    {
        "coordinator-basic",
        "coordinator-cartesian-ik-mock",
        "coordinator-cartesian-ik-piper",
        "coordinator-combined-xarm6",
        "coordinator-flowbase",
        "coordinator-flowbase-keyboard-teleop",
        "coordinator-mobile-manip-mock",
        "coordinator-mock",
        "coordinator-mock-twist-base",
        "coordinator-openarm",
        "coordinator-piper",
        "coordinator-trajectory-xarm6",
        "coordinator-teleop-dual",
        "coordinator-teleop-piper",
        "coordinator-teleop-xarm6",
        "coordinator-teleop-xarm7",
        "coordinator-velocity-xarm6",
        "coordinator-xarm6",
        "coordinator-xarm7",
        "dual-xarm6-planner-coordinator",
        "learning-collect-webxr-xarm7",
        "openarm-planner-coordinator",
        "teleop-hosted-go2-multicam",
        "teleop-hosted-go2-transport",
        "teleop-hosted-xarm6",
        "teleop-hosted-xarm7",
        "teleop-webxr-dual",
        "teleop-webxr-go2",
        "teleop-webxr-hand-xarm7",
        "teleop-webxr-piper",
        "teleop-webxr-rerun",
        "teleop-webxr-xarm6",
        "teleop-webxr-xarm7",
        "teleop-webxr-xarm7-video",
        "unitree-g1-sonic-wbc",
        "xarm-perception",
        "xarm-perception-agent",
        "xarm-perception-sim",
        "xarm-perception-sim-agent",
        "xarm7-planner-coordinator",
        "xarm7-planner-coordinator-agent",
    }
)

SIMULATION_ONLY_SELF_HOSTED_BLUEPRINTS = frozenset(
    {
        "unitree-g1-sonic-webxr-teleop",
    }
)

UBUNTU_BLUEPRINTS = sorted(
    set(all_blueprints) - SELF_HOSTED_BLUEPRINTS - SIMULATION_ONLY_SELF_HOSTED_BLUEPRINTS
)
SELF_HOSTED_BLUEPRINTS = sorted(SELF_HOSTED_BLUEPRINTS)
SIMULATION_ONLY_SELF_HOSTED_BLUEPRINTS = sorted(SIMULATION_ONLY_SELF_HOSTED_BLUEPRINTS)


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
    expected = set(SELF_HOSTED_BLUEPRINTS) | set(SIMULATION_ONLY_SELF_HOSTED_BLUEPRINTS)
    unused_names = expected - set(all_blueprints)
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


@pytest.mark.self_hosted
@pytest.mark.parametrize("blueprint_name", SIMULATION_ONLY_SELF_HOSTED_BLUEPRINTS)
def test_simulation_only_self_hosted_blueprint_is_valid(blueprint_name: str) -> None:
    """Import simulation-only blueprints in a fresh MuJoCo-configured process."""
    code = (
        "from dimos.core.global_config import global_config; "
        'global_config.update(simulation="mujoco", viewer="none"); '
        "from dimos.robot.get_all_blueprints import get_blueprint_by_name; "
        "from dimos.core.coordination.blueprints import Blueprint; "
        f'blueprint = get_blueprint_by_name("{blueprint_name}"); '
        "assert isinstance(blueprint, Blueprint)"
    )

    subprocess.run([sys.executable, "-c", code], check=True)
