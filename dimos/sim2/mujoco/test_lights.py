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

import cv2
import mujoco
import numpy as np
import pytest

# linux-aarch64 has no usd-core wheel, and every scene here is a USD layer.
pytest.importorskip("pxr")

from dimos.sim2.mujoco.lights import apply_lights
from dimos.sim2.mujoco.renders import Renders
from dimos.sim2.mujoco.world import compile_world
from dimos.sim2.scene import layer_file

SCENE = Path(__file__).parent / "assets" / "scene.usda"
GEOMETRY = '    def Scope "Geometry"'

LIGHTS = """
    def DomeLight "sky" {
        color3f inputs:color = (1, 0.5, 0.25)
        float inputs:intensity = 2
    }
    def DistantLight "sun" (prepend apiSchemas = ["ShadowAPI"]) {
        color3f inputs:color = (1, 1, 0.5)
        float inputs:intensity = 3
        float inputs:exposure = 1
        bool inputs:normalize = 1
        bool inputs:shadow:enable = 0
        float xformOp:rotateX = 90
        uniform token[] xformOpOrder = ["xformOp:rotateX"]
    }
    def DistantLight "warm" (prepend apiSchemas = ["ShadowAPI"]) {
        float inputs:intensity = 1
        bool inputs:enableColorTemperature = 1
        float inputs:colorTemperature = 3000
    }
    def DistantLight "off" {
        token visibility = "invisible"
    }
    def SphereLight "bulb" {}
"""

EXPOSURE = """
def Camera "exposed" {
    float exposure = -1
}
def RenderSettings "settings" {
    rel camera = </exposed>
}
"""


def _lit_world(lights: str = "", tail: str = "", called: str = "lights") -> mujoco.MjSpec:
    """The world as a view that draws it compiles it: with the stage's lights and sky."""
    scene = layer_file(SCENE.read_text().replace(GEOMETRY, lights + GEOMETRY, 1) + tail, called)
    spec = compile_world(scene)
    apply_lights(spec, scene)
    return spec


def test_a_stage_with_lights_and_a_dome_is_lit_by_them_in_blenders_units():
    from pxr import UsdLux

    model = _lit_world(LIGHTS, EXPOSURE).compile()
    camera = 0.5
    sky, sun, warm = (model.light(f"/scene/{name}") for name in ("sky", "sun", "warm"))
    assert model.nlight == 3, "the invisible light and the sphere light are not carried across"
    assert model.vis.headlight.active == 0

    assert sky.ambient == pytest.approx(np.array([1, 0.5, 0.25]) * 2 * camera)
    assert sky.diffuse == pytest.approx([0, 0, 0])
    # A dome without a texture is a one-texel skybox of its color in the light's units.
    texture = int(np.flatnonzero(model.tex_type == mujoco.mjtTexture.mjTEXTURE_SKYBOX)[0])
    start, width = int(model.tex_adr[texture]), int(model.tex_width[texture])
    assert width == 1
    skybox = model.tex_data[start : start + 18].reshape(6, 3)
    assert skybox == pytest.approx(np.tile([255, 128, 64], (6, 1)), abs=1)

    # Normalized, Blender's sun strength is 4·I; the surface sends back a·E/π.
    assert sun.diffuse == pytest.approx(np.array([1, 1, 0.5]) * 3 * 2 * 4 / math.pi * camera)
    assert sun.specular == pytest.approx(sun.diffuse)
    assert sun.ambient == pytest.approx([0, 0, 0])
    assert sun.castshadow == 0
    assert model.light_dir[sun.id] == pytest.approx([0, 1, 0], abs=1e-6)

    blackbody = np.array(UsdLux.BlackbodyTemperatureAsRgb(3000))
    # Not normalized, the strength is the radiance of a disc 2 x 0.53 degrees across (USD's default angle).
    disc = math.pi * math.sin(math.radians(0.53)) ** 2
    assert warm.diffuse == pytest.approx(blackbody * 4 * disc / math.pi * camera, rel=1e-5)
    assert warm.castshadow == 1, "a ShadowAPI light casts unless it says not to"
    assert model.light_dir[warm.id] == pytest.approx([0, 0, -1])


def test_a_stage_without_lights_keeps_the_headlight_and_mujocos_default_sky():
    model = _lit_world().compile()
    assert (model.nlight, model.ntex) == (0, 0)
    assert model.vis.headlight.active != 0


def test_a_latlong_dome_is_the_sky_as_hydra_maps_it(renders: None, tmp_path: Path):
    height = 32
    v, u = (np.indices((height, 2 * height)) + 0.5) / np.array([height, 2 * height])[:, None, None]
    # The image's own coordinates as its light (OpenCV writes BGR), so the sky says where it came from.
    sky = np.stack([u, v, np.full_like(u, 0.5)], -1).astype(np.float32)
    cv2.imwrite(str(tmp_path / "sky.hdr"), sky)
    dome = f"""
    def DomeLight "sky" {{
        asset inputs:texture:file = @{tmp_path / "sky.hdr"}@
        token inputs:texture:format = "latlong"
        float xformOp:rotateX = 90
        uniform token[] xformOpOrder = ["xformOp:rotateX"]
    }}
"""
    spec = _lit_world(dome, called="dome")
    # The rotation stands the dome's +Y pole up along +Z and its +Z (u = 0.5) along -Y.
    seen = {"horizon_x": ((1, 0, 0), 0.25, 0.5), "horizon_minus_y": ((0, -1, 0), 0.5, 0.5)}
    seen |= {"horizon_minus_x": ((-1, 0, 0), 0.75, 0.5), "below_x": ((1, 0, -1), 0.25, 0.75)}
    for name, (looking, _, _) in seen.items():
        forward = np.array(looking, float) / np.linalg.norm(looking)
        right = np.cross(forward, (0.0, 0.0, 1.0))
        spec.worldbody.add_camera(
            name=name, pos=(0, 0, 0), xyaxes=[*right, *np.cross(right, forward)]
        )
    model = spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_kinematics(model, data)
    mujoco.mj_camlight(model, data)
    draws = Renders(model)
    try:
        for name, (_, u_seen, v_seen) in seen.items():
            red, green, blue = draws.render(data, name, 9, 9)[4, 4].astype(float) / 255
            assert (red, green, blue) == pytest.approx((0.5, v_seen, u_seen), abs=0.03), name
    finally:
        draws.close()
