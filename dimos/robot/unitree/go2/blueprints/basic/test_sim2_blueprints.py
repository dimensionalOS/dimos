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

"""A sim blueprint's managers have to compose the whole world, not just the robot.

Two managers, nested. The inner one answers for the robot -- its layer with each sensor grafted
onto the frame it is bolted to -- and the outer one for the world, the room with the robot
grafted into it. What the outer one composes is what `get_scene` hands out, which is what the
physics runs and what every sensor renders from.

Each link in that chain fails silently. A manager with no mounts answers for the robot alone, so
the sensors are nowhere in the stage a viewer draws: the lidar publishes and nothing on screen
shows what is doing the sensing. A sensor pointed at no manager has no world to render. Both are
visible only as an absence, which is why they are worth a test that reads the wiring rather than
waiting to notice.
"""

from typing import Any

import pytest

from dimos.robot.usd_manager import Mount, UsdManager

_BLUEPRINTS = ["unitree_go2_sim2_mid360", "unitree_go2_sim2_l1"]


def _blueprint(name: str) -> Any:
    from importlib import import_module

    return getattr(import_module(f"{__package__}.{name}"), name)


def _managers(blueprint: Any) -> list[Any]:
    return [atom for atom in blueprint.blueprints if atom.module is UsdManager]


def _scene_manager(blueprint: Any) -> Any:
    """The outer one: the manager whose mounts hold another manager rather than a sensor."""
    managers = {atom.name for atom in _managers(blueprint)}
    return next(
        atom
        for atom in _managers(blueprint)
        if any(Mount.of(mount).remote_name in managers for mount in _mounts(atom).values())
    )


def _robot_manager(blueprint: Any) -> Any:
    """The inner one: the manager the scene's manager mounts."""
    mounted = {Mount.of(mount).remote_name for mount in _mounts(_scene_manager(blueprint)).values()}
    return next(atom for atom in _managers(blueprint) if atom.name in mounted)


def _mounts(atom: Any) -> dict[str, Any]:
    return atom.kwargs.get("mounts") or {}


@pytest.mark.parametrize("name", _BLUEPRINTS)
def test_the_world_is_composed_of_the_room_and_the_robot(name: str) -> None:
    blueprint = _blueprint(name)
    scene = _scene_manager(blueprint)

    assert scene.kwargs.get("model") is not None, "the scene's manager answers for the room"
    # Its own namespace, so it is not a second manager answering on the robot's topics.
    assert "/" in scene.name, f"{name}'s scene manager is not namespaced: {scene.name}"
    deployed = {atom.name for atom in blueprint.blueprints}
    for frame_id, mount in _mounts(scene).items():
        assert Mount.of(mount).remote_name in deployed, (
            f"{name} mounts {frame_id!r} on something nothing in the blueprint deploys"
        )


@pytest.mark.parametrize("name", _BLUEPRINTS)
def test_the_manager_mounts_every_sensor_the_blueprint_deploys(name: str) -> None:
    blueprint = _blueprint(name)
    mounts = _mounts(_robot_manager(blueprint))

    assert mounts, f"{name}'s robot manager has no mounts, so it answers for the robot alone"

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
    mounts = _mounts(_robot_manager(blueprint))

    sensed = {
        atom.kwargs["frame_id"]
        for atom in blueprint.blueprints
        if isinstance(atom.kwargs.get("frame_id"), str)
    }

    assert sensed, f"{name} deploys no sensor with a frame_id, so this test proves nothing"
    assert sensed <= set(mounts), f"{name} leaves {sorted(sensed - set(mounts))} unmounted"


@pytest.mark.parametrize("name", _BLUEPRINTS)
def test_every_sensor_is_pointed_at_the_manager_that_composes_the_world(name: str) -> None:
    """A sensor with no scene manager has no world to render, and says so at startup."""
    blueprint = _blueprint(name)
    scene = _scene_manager(blueprint).name
    sensors = [
        atom
        for atom in blueprint.blueprints
        if isinstance(atom.kwargs.get("frame_id"), str)
        and atom.name
        in {Mount.of(mount).remote_name for mount in _mounts(_robot_manager(blueprint)).values()}
    ]

    assert sensors, f"{name} deploys no sensors"
    for sensor in sensors:
        assert sensor.kwargs.get("scene_manager") == scene, (
            f"{name}'s {sensor.name} renders from {sensor.kwargs.get('scene_manager')!r}, "
            f"not the manager that composes the world ({scene!r})"
        )
