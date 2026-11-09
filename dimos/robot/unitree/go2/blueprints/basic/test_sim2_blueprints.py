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

"""A sim blueprint's manager has to hold every part of the robot, not just the robot.

A manager with no mounts answers for the robot alone, so the sensors bolted to it are nowhere in
the stage a viewer draws: the lidar publishes and nothing on screen shows what is doing the
sensing. The failure is silent at startup and only visible as an absence, which is why it is worth
a test that reads the wiring rather than waiting to notice.
"""

from typing import Any

import pytest

from dimos.robot.usd_manager import Mount, UsdManager

_BLUEPRINTS = ["unitree_go2_sim2_mid360", "unitree_go2_sim2_l1"]


def _blueprint(name: str) -> Any:
    from importlib import import_module

    return getattr(import_module(f"{__package__}.{name}"), name)


def _manager(blueprint: Any) -> Any:
    return next(atom for atom in blueprint.blueprints if atom.module is UsdManager)


@pytest.mark.parametrize("name", _BLUEPRINTS)
def test_the_manager_mounts_every_sensor_the_blueprint_deploys(name: str) -> None:
    blueprint = _blueprint(name)
    manager = _manager(blueprint)

    mounts = manager.kwargs.get("mounts") or {}
    assert mounts, f"{name}'s UsdManager has no mounts, so it answers for the robot alone"

    deployed = {atom.name for atom in blueprint.blueprints}
    for frame_id, mount in mounts.items():
        # Mount.of is the normalisation the module itself applies, so this checks the name the
        # manager will really call rather than a guess at it.
        remote = Mount.of(mount).remote_name
        assert remote in deployed, (
            f"{name} mounts {remote!r} at {frame_id!r}, which nothing in the blueprint deploys"
        )


@pytest.mark.parametrize("name", _BLUEPRINTS)
def test_every_frame_the_robot_offers_a_sensor_is_mounted_at(name: str) -> None:
    """Whichever mount frames this blueprint's sensors were pointed at are the ones grafted on."""
    blueprint = _blueprint(name)
    mounts = _manager(blueprint).kwargs.get("mounts") or {}

    sensed = {
        atom.kwargs["frame_id"]
        for atom in blueprint.blueprints
        if isinstance(atom.kwargs.get("frame_id"), str)
    }

    assert sensed, f"{name} deploys no sensor with a frame_id, so this test proves nothing"
    assert sensed <= set(mounts), f"{name} leaves {sorted(sensed - set(mounts))} unmounted"
