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


"""The vendored office_555 layer: what has to still be true of the file that is committed."""

from __future__ import annotations

import collections

import numpy as np
import pytest

from dimos.simulation.scenes.office_555 import SCENE
from dimos.simulation.usd_spec import load_usd_spec

pytestmark = pytest.mark.self_hosted

# MuJoCo's own default, which is what a geom gets when nothing says otherwise
DEFAULT_GREY = (0.5, 0.5, 0.5, 1.0)


@pytest.fixture(scope="module")
def room() -> object:
    return load_usd_spec(SCENE.resolve()).compile()


def test_the_room_is_not_one_flat_grey(room: object) -> None:
    """The colours are in the layer, and this is the file that is served, so this is where it counts.

    A mesh's colour rides in `primvars:displayColor`, and the converter stripped every primvar
    off each mesh prim to drop the baked normals -- so 3570 of 3925 geoms came out MuJoCo's
    default grey and the whole room rendered flat. Nothing failed; it just looked wrong.

    The 23 that are legitimately grey here are the ones whose colour comes from a material
    rather than from themselves.
    """
    rgba = [tuple(np.round(room.geom_rgba[g], 3)) for g in range(room.ngeom)]  # type: ignore[attr-defined]
    grey = [c for c in rgba if np.allclose(c, DEFAULT_GREY)]

    assert len(grey) < 50, f"{len(grey)} of {len(rgba)} geoms are MuJoCo's default grey"
    assert len(collections.Counter(rgba)) > 40, "the room came out of the converter near-monochrome"
