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

import math
from pathlib import Path

import numpy as np
import pytest

from dimos.sim2.mujoco.world import compile_world
from dimos.sim2.usd_layer import layer_file

# linux-aarch64 has no usd-core wheel, and every scene here is a USD layer.
pytest.importorskip("pxr")

SCENE = Path(__file__).parent / "assets" / "scene.usda"

LIGHTS = """
    def DomeLight "sky"
    {
        color3f inputs:color = (1, 0.5, 0.25)
        float inputs:intensity = 2
    }

    def DistantLight "sun" (
        prepend apiSchemas = ["ShadowAPI"]
    )
    {
        color3f inputs:color = (1, 1, 0.5)
        float inputs:intensity = 3
        float inputs:exposure = 1
        bool inputs:shadow:enable = 0
        float xformOp:rotateX = 90
        uniform token[] xformOpOrder = ["xformOp:rotateX"]
    }

    def DistantLight "warm" (
        prepend apiSchemas = ["ShadowAPI"]
    )
    {
        float inputs:intensity = 1
        bool inputs:enableColorTemperature = 1
        float inputs:colorTemperature = 3000
    }

    def DistantLight "off"
    {
        token visibility = "invisible"
    }

    def SphereLight "bulb"
    {
    }
"""

EXPOSURE = """

def Camera "exposed"
{
    float exposure = -1
}

def RenderSettings "settings"
{
    rel camera = </exposed>
}
"""


def _lit() -> Path:
    geometry = '    def Scope "Geometry"'
    scene = SCENE.read_text().replace(geometry, LIGHTS + geometry, 1)
    return layer_file(scene + EXPOSURE, called="lights")


def test_a_stage_with_lights_is_lit_by_them_in_blenders_units_and_not_by_a_headlight():
    from pxr import UsdLux

    model = compile_world(_lit()).compile()
    camera = 0.5
    sky, sun, warm = (model.light(f"/scene/{name}") for name in ("sky", "sun", "warm"))
    assert model.nlight == 3, "the invisible light and the sphere light are not carried across"
    assert model.vis.headlight.active == 0

    assert sky.ambient == pytest.approx(np.array([1, 0.5, 0.25]) * 2 * camera)
    assert sky.diffuse == pytest.approx([0, 0, 0])

    assert sun.diffuse == pytest.approx(np.array([1, 1, 0.5]) * 3 * 2 / math.pi * camera)
    assert sun.specular == pytest.approx(sun.diffuse)
    assert sun.ambient == pytest.approx([0, 0, 0])
    assert sun.castshadow == 0
    assert model.light_dir[sun.id] == pytest.approx([0, 1, 0], abs=1e-6)

    blackbody = np.array(UsdLux.BlackbodyTemperatureAsRgb(3000))
    assert warm.diffuse == pytest.approx(blackbody / math.pi * camera, rel=1e-5)
    assert warm.castshadow == 1, "a ShadowAPI light casts unless it says not to"
    assert model.light_dir[warm.id] == pytest.approx([0, 0, -1])


def test_a_stage_without_lights_keeps_the_headlight():
    model = compile_world(SCENE).compile()
    assert model.nlight == 0
    assert model.vis.headlight.active != 0
