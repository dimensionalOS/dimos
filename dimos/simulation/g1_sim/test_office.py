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


"""The G1 standing in the converted office_555, and its lidar scanning that room."""

from __future__ import annotations

import mujoco
import numpy as np
from numpy.typing import NDArray
import pytest

from dimos.simulation.g1_sim.body import LeggedG1
from dimos.simulation.g1_sim.office import (
    LIDAR_GROUPS,
    PREFIX,
    office_lidar,
    sensor_pose,
    standing,
)
from dimos.simulation.g1_sim.world import MID360, STAND_HEIGHT
from dimos.simulation.scenes.mjcf import SCENE_GROUP
from dimos.simulation.scenes.office_555 import CLEAR_FLOOR
from dimos.simulation.sensors.mujoco_raycaster import MujocoRaycaster
from dimos.simulation.sim_world.body import COLLISION_GROUP, ROBOT_VISUAL_GROUP

pytestmark = pytest.mark.self_hosted

# a full Mid-360 frame is 20k points at 0.1 s
FRAME_POINTS = 20000


@pytest.fixture(scope="module")
def office() -> tuple[LeggedG1, mujoco.MjModel]:
    """The G1 reset on the office floor. One compile for the file: it is 479 bodies."""
    return standing()


def _mask(*groups: int) -> NDArray[np.uint8]:
    return np.isin(np.arange(6), groups).astype(np.uint8)


def test_the_room_and_the_robot_compose_into_one_model(
    office: tuple[LeggedG1, mujoco.MjModel],
) -> None:
    """Two USD layers, no MJCF between them, and the robot keeps all 29 of its joints."""
    body, model = office
    assert model.nbody > 400, "the room's 450 bodies are not in the model"
    assert len(body.joint_names) == 29
    assert model.nu == 29


def test_the_robot_is_namespaced_apart_from_the_room(
    office: tuple[LeggedG1, mujoco.MjModel],
) -> None:
    """Every G1 element takes the prefix, and the body class looks its names up through it.

    Both halves matter and they fail differently. Attaching without a prefix lets the room's
    names and the robot's collide -- both have geoms called `box_*`. Building the body without
    one makes every `mj_name2id` return -1, which MuJoCo does not raise on: the robot simply
    reads the wrong body forever.
    """
    body, model = office
    assert body.trunk >= 0, "the body class looked up unprefixed names"
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis") == -1
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{PREFIX}pelvis") == body.trunk
    assert all(name.startswith(PREFIX) for name in body.joint_names)
    assert len(body.feet) == 4


def test_it_stands_on_the_office_floor(office: tuple[LeggedG1, mujoco.MjModel]) -> None:
    """The room's own floor holds it up: nothing here adds a ground plane the way g1_spec's does.

    The G1 free-falls through a scene with no collidable floor, so a pelvis still at standing
    height after two seconds is the room's slab taking its weight.
    """
    body, _ = office
    body.reset(CLEAR_FLOOR[0], CLEAR_FLOOR[1], 0.0, 0.0)
    for _ in range(100):  # 2 s at 50 Hz
        body.tick(np.zeros(3))

    position, _ = body.base_pose()
    assert abs(position[2] - STAND_HEIGHT) < 0.1, "it sank, fell or floated"
    assert body.upright() > 0.9
    assert np.linalg.norm(position[:2] - np.array(CLEAR_FLOOR)) < 0.5, "it wandered off its spot"


def test_the_mount_turns_with_the_body(office: tuple[LeggedG1, mujoco.MjModel]) -> None:
    """The Mid-360 rides on the base, so a quarter turn of the robot turns the lens with it.

    Leaving the mount in the world frame instead is invisible at yaw 0, where the two frames
    agree, which is why this compares two headings. Only the rotation is checked: the mount is
    0.46 m up and 3.7 mm forward of the pelvis's centre, so no tolerance worth writing can tell
    a turned translation from an unturned one. The scan depends on the rotation either way.
    """
    body, _ = office
    body.reset(CLEAR_FLOOR[0], CLEAR_FLOOR[1], 0.0, 0.0)
    facing_x, lens_x = sensor_pose(body)
    body.reset(CLEAR_FLOOR[0], CLEAR_FLOOR[1], 0.0, float(np.pi / 2))
    facing_y, lens_y = sensor_pose(body)

    quarter_turn = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    assert np.allclose(lens_y, quarter_turn @ lens_x, atol=0.02), "the lens did not turn"
    base, _ = body.base_pose()
    assert abs(float(np.linalg.norm(facing_y - base)) - 0.4602) < 0.002, "it left the mount"
    assert abs(facing_x[2] - facing_y[2]) < 0.02, "the mount height turned with it"
    assert facing_x[2] > 1.1, "the Mid-360 is not above the pelvis"


def test_the_lidar_scans_the_room(office: tuple[LeggedG1, mujoco.MjModel]) -> None:
    """A frame off the real pattern, cast at the real room: the floor under it and walls 20 m off.

    office_555 is about 30 m across, so the far returns are the thing a generated box scene
    cannot produce -- its rooms are a few metres wide and its walls are all the same box.
    """
    body, _ = office
    body.reset(CLEAR_FLOOR[0], CLEAR_FLOOR[1], 0.0, 0.0)
    origin, rotation = sensor_pose(body)
    points = office_lidar(body).cast(origin, rotation, FRAME_POINTS)

    assert len(points) > FRAME_POINTS / 2, "most of the frame returned nothing"
    ranges = np.linalg.norm(points, axis=1)
    assert ranges.max() > 20.0, "it saw no far wall"
    world = origin + points @ rotation.T
    assert world[:, 2].min() < 0.05, "it did not see the floor"
    assert world[:, 2].max() > 3.0, "it did not see the upper slab"
    assert (world[:, 2] < 0.1).mean() > 0.2, "little of the frame landed on the floor"


def test_the_lidar_sees_the_furniture_and_not_its_collision_pieces(
    office: tuple[LeggedG1, mujoco.MjModel],
) -> None:
    """The mask's two halves, each checked by what breaks without it.

    Dropping the furniture group leaves the desks and chairs invisible -- a robot would drive
    through them -- because the converted room draws each as one group-1 mesh. Adding the
    collision group instead puts the robot's own group-3 shells in front of the lens, and a
    frame comes back empty.
    """
    body, model = office
    body.reset(CLEAR_FLOOR[0], CLEAR_FLOOR[1], 0.0, 0.0)
    origin, rotation = sensor_pose(body)

    furniture = _nearest_furniture(model, body.data, origin)
    direction = (furniture - origin) / np.linalg.norm(furniture - origin)
    reach = float(np.linalg.norm(furniture - origin)) + 0.5
    seen, _ = MujocoRaycaster(model, body.data, LIDAR_GROUPS).cast(origin, direction[None], reach)
    shell_only, _ = MujocoRaycaster(model, body.data, _mask(SCENE_GROUP)).cast(
        origin, direction[None], reach
    )
    assert 0 < seen[0] <= reach, "the furniture was invisible to the lidar"
    assert shell_only[0] < 0 or shell_only[0] > seen[0], "the shell alone already stopped the ray"

    with_collision = _mask(SCENE_GROUP, ROBOT_VISUAL_GROUP, COLLISION_GROUP)
    blinded = MID360.lidar(MujocoRaycaster(model, body.data, with_collision), 0)
    assert len(blinded.cast(origin, rotation, FRAME_POINTS)) == 0


def _nearest_furniture(
    model: mujoco.MjModel, data: mujoco.MjData, origin: NDArray[np.float64]
) -> NDArray[np.float64]:
    """The closest group-1 geom's centre, which is what the lidar ought to be stopped by."""
    centers = [
        data.geom_xpos[g]
        for g in range(model.ngeom)
        if int(model.geom_group[g]) == 1 and not (model.geom(g).name or "").startswith(PREFIX)
    ]
    assert centers, "the room brought no furniture"
    nearest = min(centers, key=lambda c: float(np.linalg.norm(c - origin)))
    return np.asarray(nearest, dtype=np.float64).copy()
