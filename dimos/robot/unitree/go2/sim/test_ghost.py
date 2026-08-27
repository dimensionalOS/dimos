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

"""The ghost body is visual only: attaching it never moves the physics."""

from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("mujoco")


# The go2 assets are vendored (data/go2_menagerie): a missing scene is a real
# failure now, never a skip.
pytestmark = [pytest.mark.go2sim]


def test_the_ghost_geom_can_touch_nothing():
    import mujoco

    from dimos.robot.unitree.go2.sim.engines.model import GHOST_BODY, load, mocap_index

    model, _data = load(ghost=True)
    gi = mocap_index(model, GHOST_BODY)
    assert gi >= 0
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, GHOST_BODY)
    gids = [g for g in range(model.ngeom) if model.geom_bodyid[g] == bid]
    assert gids, "the ghost draws something"
    for g in gids:
        assert model.geom_contype[g] == 0 and model.geom_conaffinity[g] == 0


def test_stepping_with_and_without_the_ghost_is_bit_identical():
    """The viewer invariant, mechanically: what you watch is what is scored."""
    import mujoco

    from dimos.robot.unitree.go2.sim.engines.model import GHOST_BODY, load, mocap_index

    def run(ghost: bool) -> np.ndarray:
        model, data = load(ghost=ghost)
        kid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")  # type: ignore[attr-defined]
        if kid >= 0:
            mujoco.mj_resetDataKeyframe(model, data, kid)
        if ghost:
            gi = mocap_index(model, GHOST_BODY)
            data.mocap_pos[gi] = [5.0, 5.0, 0.3]  # parked nearby, moving every step
        for step in range(250):
            if ghost:
                data.mocap_pos[gi][0] = 5.0 + 0.01 * step
            data.ctrl[:] = 0.3 * np.sin(step / 20.0)
            mujoco.mj_step(model, data)
        return np.array(data.qpos[:19])

    assert np.array_equal(run(False), run(True))
