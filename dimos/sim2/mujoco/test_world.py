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

from pathlib import Path

import cv2
import mujoco
import numpy as np
import pytest

from dimos.sim2.mujoco.renders import Renders
from dimos.sim2.mujoco.world import compile_world
from dimos.sim2.usd_layer import layer_file

# linux-aarch64 has no usd-core wheel, and every scene here is a USD layer.
pytest.importorskip("pxr")


def _slab(name: str, x: float, bound: str) -> str:
    """A 2 x 2 x 0.1 m mesh slab at x, painted red by displayColor, its st spread over its top."""
    low, high = x - 1, x + 1
    corners = [(low, -1, -0.1), (high, -1, -0.1), (high, 1, -0.1), (low, 1, -0.1)]
    corners += [(cx, cy, 0) for cx, cy, _ in corners]
    return f"""
    def Mesh "{name}" (
        prepend apiSchemas = ["MaterialBindingAPI"]
    )
    {{
        int[] faceVertexCounts = [4, 4, 4, 4, 4, 4]
        int[] faceVertexIndices = [4, 5, 6, 7, 3, 2, 1, 0, 0, 1, 5, 4, 1, 2, 6, 5, 2, 3, 7, 6, 3, 0, 4, 7]
        point3f[] points = {corners}
        texCoord2f[] primvars:st = [(0, 0), (1, 0), (1, 1), (0, 1), (0, 0), (1, 0), (1, 1), (0, 1)] (
            interpolation = "vertex"
        )
        color3f[] primvars:displayColor = [(1, 0, 0)]
        {bound}
    }}
"""


def test_a_textured_material_shows_its_own_color_whatever_the_display_color(
    renders: None, tmp_path: Path
):
    # OpenCV writes BGR, so this is the sRGB color (51, 153, 102).
    cv2.imwrite(str(tmp_path / "green.png"), np.full((4, 4, 3), (102, 153, 51), np.uint8))
    stage = f"""#usda 1.0
(
    defaultPrim = "world"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "world"
{{
    def DomeLight "sky"
    {{
        float inputs:intensity = 1
    }}
{_slab("tile", 0, "rel material:binding = </world/painted>")}
{_slab("bare", 3, "")}
    def Material "painted"
    {{
        token outputs:surface.connect = </world/painted/surface.outputs:surface>

        def Shader "surface"
        {{
            uniform token info:id = "UsdPreviewSurface"
            color3f inputs:diffuseColor.connect = </world/painted/image.outputs:rgb>
            token outputs:surface
        }}

        def Shader "image"
        {{
            uniform token info:id = "UsdUVTexture"
            asset inputs:file = @{tmp_path / "green.png"}@
            float2 inputs:st.connect = </world/painted/uv.outputs:result>
            float3 outputs:rgb
        }}

        def Shader "uv"
        {{
            uniform token info:id = "UsdPrimvarReader_float2"
            string inputs:varname = "st"
            float2 outputs:result
        }}
    }}
}}
"""
    spec = compile_world(layer_file(stage, called="textured"))
    spec.worldbody.add_camera(name="tile", pos=(0, 0, 3))
    spec.worldbody.add_camera(name="bare", pos=(3, 0, 3))
    model = spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_kinematics(model, data)
    mujoco.mj_camlight(model, data)
    draws = Renders(model)
    try:
        tile, bare = (draws.render(data, name, 9, 9)[4, 4] for name in ("tile", "bare"))
    finally:
        draws.close()
    # A white sky of strength 1 sends back each surface's own albedo.
    assert tile == pytest.approx([51, 153, 102], abs=2), "the texture is not tinted by displayColor"
    assert bare == pytest.approx([255, 0, 0], abs=2), "with no material bound, displayColor shows"
