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


from collections.abc import Iterator
import json
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from dimos.robot.assets.model import UsdModel
from dimos.sim2.actions import CborKeyValueCommand
from dimos.sim2.dimsim.apartment import DimSimApartment
from dimos.sim2.dimsim.backend import ACTION, ACTIONS, DimSimBackend
from dimos.sim2.dimsim.dimsim_scene import DIMSIM_SCENES, STATE
from dimos.sim2.dimsim.kinematic_base import SimKinematicBase
from dimos.sim2.mujoco.sensor_world import open_view, robot_ids
from dimos.sim2.mujoco.world import SensorMount
from dimos.sim2.sim import compose_world
from dimos.sim2.usd_layer import layer_file
from dimos.sim2.world_observations import REFUSED, WorldMirror, geometry

pytest.importorskip("pxr")

_SCENE = DIMSIM_SCENES / "apartment"
_GO2 = Path(__file__).parents[2] / "robot" / "unitree" / "go2" / "assets" / "go2.usda"
_Apartment = tuple[DimSimBackend, WorldMirror]
_MANIFEST = json.loads((_SCENE / "objects" / "manifest.json").read_text())

# The apartment's models are Git LFS data, which the scene fetches as it loads.
pytestmark = pytest.mark.self_hosted


@pytest.fixture(scope="module")
def apartment() -> Iterator[_Apartment]:
    scene = DimSimApartment()
    go2 = SimKinematicBase(
        usd=UsdModel.from_file(_GO2), mount="go2", instance_name="go2/SimKinematicBase"
    )
    try:
        world, robot = compose_world([scene, go2]), go2.simulated_robot()
    finally:
        scene.stop()
        go2.stop()
    backend = DimSimBackend(world, [robot], timestep=0.02, reach=1.5)
    mirror = WorldMirror()
    mirror.apply(backend.frame())
    yield backend, mirror
    backend.close()


def _act(apartment: _Apartment, entity: str, action: str) -> WorldMirror:
    backend, mirror = apartment
    backend.apply([CborKeyValueCommand(entity, ACTION, action)])
    _at, frame = backend.advance()
    mirror.apply(frame)
    return mirror


def _cbor(mirror: WorldMirror, name: str) -> dict[str, Any]:
    entity = mirror.entity(name)
    assert entity is not None
    return entity.cbor


def test_opening_the_fridge_shows_up_in_the_world(apartment: _Apartment) -> None:
    _backend, mirror = apartment
    assert _cbor(mirror, "refrigerator")[STATE] == "closed"
    assert _cbor(mirror, "refrigerator")[ACTIONS] == ["to Open"]

    _act(apartment, "refrigerator", "to Open")

    assert (mirror.ids["refrigerator"], STATE, "Open") in mirror.received
    assert _cbor(mirror, "refrigerator")[ACTIONS] == ["to closed"]


def test_an_action_the_object_does_not_have_now_is_refused(apartment: _Apartment) -> None:
    state = _cbor(apartment[1], "refrigerator")[STATE]
    mirror = _act(apartment, "refrigerator", "fly")
    refused = [value for _, key, value in mirror.received if key == REFUSED]
    assert len(refused) == 1 and "no action 'fly'" in refused[0]["reason"]
    assert _cbor(mirror, "refrigerator")[STATE] == state


def test_every_action_in_the_scene_runs(apartment: _Apartment) -> None:
    _backend, mirror = apartment
    by_title: dict[str, list[dict[str, Any]]] = {}
    for entry in _MANIFEST:
        if entry["actions"]:
            by_title.setdefault(entry["title"], []).append(entry)
    objects = {e.path: e.cbor for e in mirror.entities.values() if ACTIONS in e.cbor}
    assert sum(len(entries) for entries in by_title.values()) == len(objects)
    for name, cbor in objects.items():
        (entry, *_) = by_title[cbor["title"]]
        names = {state["id"]: state["name"] for state in entry["states"]}
        untried = list(entry["actions"])
        while untried:
            here = _cbor(mirror, name)[STATE]
            action = next(
                (a for a in untried if names[a["from"]] == here),
                next(a for a in entry["actions"] if names[a["from"]] == here),
            )
            if action in untried:
                untried.remove(action)
            _act(apartment, name, action["id"])
            assert _cbor(mirror, name)[STATE] == names[action["to"]], (name, action)


def _rays_around(usd: str, mirror: WorldMirror, name: str) -> np.ndarray:
    """How far a lidar's view of the world ``usd`` sees, out from the object ``name``, all ways round."""
    entity = mirror.entity(name)
    assert entity is not None and entity.pose is not None
    view = open_view(
        layer_file(usd, "world"), robot_ids(mirror), SensorMount("lidar", "world", "lidar")
    )
    try:
        turns = np.linspace(0, 2 * np.pi, 72, endpoint=False)
        directions = np.stack([np.cos(turns), np.sin(turns), np.zeros_like(turns)], axis=1)
        raycaster = view.raycaster(range(6))
        return np.array(
            [
                raycaster.cast(entity.pose[:3] + np.array([0, 0, height]), directions, 10.0)[0]
                for height in (0.3, 0.8, 1.3)
            ]
        )
    finally:
        view.close()


def test_the_sensors_see_the_fridge_open(apartment: _Apartment) -> None:
    backend, mirror = apartment
    shapes, seen = geometry(mirror), _rays_around(backend.usd, mirror, "refrigerator")
    state = _cbor(mirror, "refrigerator")[STATE]
    _act(apartment, "refrigerator", "to Open" if state == "closed" else "to closed")
    assert geometry(mirror) == shapes + 1, "a sensor's view would never learn to rebuild"
    assert not np.allclose(_rays_around(backend.usd, mirror, "refrigerator"), seen)
