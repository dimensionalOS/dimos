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

"""The nav clearances have to keep covering the robot they describe."""

from __future__ import annotations

import numpy as np
import pytest

from dimos.robot.microduck import assets_fetch
from dimos.robot.microduck.config import MICRODUCK
from dimos.robot.microduck.places import FOUR_ROOM_XML
from dimos.robot.microduck.sim_module import MicroduckSimModule


def _duck_extent() -> tuple[float, float, float]:
    """(width, height, footprint circle) of the robot, in metres.

    AABB over the body subtree hanging off the robot's free joint, in the rest
    pose. Deliberately not `geom_rbound`: that is a bounding-SPHERE radius, so
    for this mesh-heavy model it reads ~1.3 m for a 25 cm duck.
    """
    import mujoco

    robot_xml = assets_fetch.variant_mjcf_path("default")
    if not robot_xml.exists():
        pytest.skip(f"Microduck asset cache not present ({assets_fetch.assets_root()})")
    module = MicroduckSimModule(
        scene_xml=FOUR_ROOM_XML,
        robot_mjcf=str(robot_xml),
        headless=True,
        spawn_xy=(0.0, 0.0),
    )
    try:
        model = module._compose_model()
    finally:
        module.stop()  # the module spins up threads even for a bare compose
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    root = int(model.jnt_bodyid[0])  # joint 0 is the trunk free joint
    subtree = {root}
    for b in range(model.nbody):
        p = b
        while p != 0:
            if p == root:
                subtree.add(b)
                break
            p = int(model.body_parentid[p])

    lo = np.full(3, np.inf)
    hi = np.full(3, -np.inf)
    for gid in range(model.ngeom):
        if int(model.geom_bodyid[gid]) not in subtree:
            continue
        pos = data.geom_xpos[gid]
        mat = data.geom_xmat[gid].reshape(3, 3)
        if int(model.geom_type[gid]) == int(mujoco.mjtGeom.mjGEOM_MESH):
            mid = int(model.geom_dataid[gid])
            start = int(model.mesh_vertadr[mid])
            verts = model.mesh_vert[start : start + int(model.mesh_vertnum[mid])].reshape(-1, 3)
            world = verts @ mat.T + pos
            lo = np.minimum(lo, world.min(axis=0))
            hi = np.maximum(hi, world.max(axis=0))
        else:
            half = np.abs(mat) @ np.asarray(model.geom_size[gid], dtype=float)
            lo = np.minimum(lo, pos - half)
            hi = np.maximum(hi, pos + half)
    size = hi - lo
    return float(max(size[0], size[1])), float(size[2]), float(np.hypot(size[0], size[1]))


@pytest.mark.mujoco
def test_clearances_cover_the_actual_robot() -> None:
    """Every clearance must exceed what it clears.

    These were loose constants copied into two blueprints, with a comment
    claiming they were the duck's size - they are not, they carry margin. The
    margins are the point, so assert both directions: big enough to be safe,
    small enough that nobody has quietly turned a 25 cm duck into a metre-wide
    one the planner refuses to route through a doorway.
    """
    pytest.importorskip("mujoco")
    width, height, circle = _duck_extent()

    assert MICRODUCK.width_clearance > width, (
        f"width_clearance {MICRODUCK.width_clearance} does not cover the duck's {width:.3f} m"
    )
    assert MICRODUCK.height_clearance > height, (
        f"height_clearance {MICRODUCK.height_clearance} does not cover the duck's {height:.3f} m"
    )
    assert MICRODUCK.rotation_diameter > circle, (
        f"rotation_diameter {MICRODUCK.rotation_diameter} does not cover the duck's "
        f"{circle:.3f} m turning circle"
    )
    # Margin, not a different robot. The four-room doorways are ~0.7 m.
    assert MICRODUCK.width_clearance < width + 0.15
    assert MICRODUCK.rotation_diameter < circle + 0.25


def test_blueprints_take_their_clearances_from_the_descriptor() -> None:
    """The duplication this file exists to prevent: the same three numbers
    were spelled out in both blueprints, so tuning one silently desynced the
    other."""
    from dimos.robot.microduck.blueprints import microduck_cockpit_sim, microduck_sim

    for module in (microduck_cockpit_sim, microduck_sim):
        source = module.__file__
        assert source is not None
        text = open(source).read()
        assert "MICRODUCK.width_clearance" in text, f"{source} does not use the descriptor"
        assert "_DUCK_WIDTH" not in text, f"{source} still carries a local copy"
