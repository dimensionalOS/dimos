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

import multiprocessing
from multiprocessing.connection import Connection
from pathlib import Path

import cv2
import mujoco
import numpy as np
import pytest

from dimos.sim2.mujoco.lights import apply_lights
from dimos.sim2.mujoco.renders import RenderProcess, RenderQuality, Renders
from dimos.sim2.mujoco.view import MujocoView, _textured_materials_win
from dimos.sim2.mujoco.world import compile_world
from dimos.sim2.usd_layer import layer_file

SHADOWED = """
<mujoco>
  <worldbody>
    <light directional="true" castshadow="true" pos="0 0 6" dir="0.3 0.2 -1"/>
    <geom type="plane" size="4 4 0.1" rgba="0.8 0.8 0.8 1"/>
    <body name="box"><freejoint/><geom type="box" size="0.3 0.3 0.3" rgba="0.9 0.2 0.2 1"/></body>
    <body name="marker" mocap="true" pos="1 1 0.2"><geom type="sphere" size="0.2"/></body>
    <camera name="eye" pos="0 -3 2.5" xyaxes="1 0 0 0 0.6 0.8"/>
  </worldbody>
</mujoco>
"""


def _posed() -> tuple[mujoco.MjModel, mujoco.MjData]:
    model = mujoco.MjModel.from_xml_string(SHADOWED)
    data = mujoco.MjData(model)
    data.qpos[:3] = [0.5, -0.4, 0.6]
    data.mocap_pos[0] = [-1.0, 0.5, 0.4]
    mujoco.mj_kinematics(model, data)
    mujoco.mj_comPos(model, data)
    mujoco.mj_camlight(model, data)
    return model, data


def test_a_render_process_draws_what_this_process_draws(renders: None) -> None:
    model, data = _posed()
    here, there = Renders(model), RenderProcess(model)
    try:
        expected = here.render(data, "eye", 64, 48)
        assert np.array_equal(there.render(data, "eye", 64, 48), expected)
        assert expected.std() > 0
    finally:
        here.close()
        there.close()


def test_shadows_off_leave_the_floor_unshadowed(renders: None) -> None:
    model, data = _posed()
    lit, flat = Renders(model), Renders(model, RenderQuality(shadows=False))
    try:
        shadowed, unshadowed = lit.render(data, "eye", 64, 48), flat.render(data, "eye", 64, 48)
    finally:
        lit.close()
        flat.close()
    # The box's shadow darkens part of the floor, so the shadowed image is the darker one.
    assert shadowed.astype(int).sum() < unshadowed.astype(int).sum()


def test_quality_sets_the_models_shadow_map_samples_and_headlight() -> None:
    model = mujoco.MjModel.from_xml_string(SHADOWED)
    RenderQuality().apply_to_model(model)
    assert (model.vis.quality.shadowsize, model.vis.quality.offsamples) == (4096, 4)
    RenderQuality(shadow_size=512, samples=0, headlight=False).apply_to_model(model)
    assert (model.vis.quality.shadowsize, model.vis.quality.offsamples) == (512, 0)
    assert model.vis.headlight.active == 0


def test_samples_off_leave_edges_unblended(renders: None) -> None:
    model, data = _posed()
    smooth = Renders(model)
    jagged_model = mujoco.MjModel.from_xml_string(SHADOWED)
    RenderQuality(samples=0).apply_to_model(jagged_model)
    jagged = Renders(jagged_model)
    try:
        blended = smooth.render(data, "eye", 64, 48)
        aliased = jagged.render(data, "eye", 64, 48)
    finally:
        smooth.close()
        jagged.close()

    # Multisampling blends edge pixels, so it shows more distinct colors than a render without it.
    def distinct(image):
        return len(np.unique(image.reshape(-1, 3), axis=0))

    assert distinct(blended) > distinct(aliased)


def test_a_render_process_raises_what_the_render_raised(renders: None) -> None:
    model, data = _posed()
    there = RenderProcess(model)
    try:
        with pytest.raises(ValueError):
            there.render(data, "no_such_camera", 64, 48)
        assert there.render(data, "eye", 64, 48).shape == (48, 64, 3)
    finally:
        there.close()


def _render_shadowed_view(scene: str, images: Connection) -> None:
    view = MujocoView(Path(scene))
    images.send(view.render("eye", 64, 48))
    view.close()


def test_a_view_renders_shadows_in_a_forked_worker(renders: None, tmp_path: Path) -> None:
    # A module worker is forked from a forkserver; on macOS a shadow drawn there once aborted it.
    scene = tmp_path / "shadowed.xml"
    scene.write_text(SHADOWED)
    context = multiprocessing.get_context("forkserver")
    received, sent = context.Pipe(duplex=False)
    worker = context.Process(target=_render_shadowed_view, args=(str(scene), sent))
    worker.start()
    # With the worker's end closed here too, a worker that dies leaves recv an EOFError.
    sent.close()
    image = received.recv()
    worker.join()
    assert worker.exitcode == 0
    assert image.shape == (48, 64, 3) and image.std() > 0


def test_a_view_places_its_cameras_where_it_renders_them(renders: None, tmp_path: Path) -> None:
    scene = tmp_path / "shadowed.xml"
    scene.write_text(SHADOWED)
    view = MujocoView(scene)
    try:
        view.render("eye", 8, 8)
        assert view.pose("eye")[0] == pytest.approx([0, -3, 2.5])
    finally:
        view.close()


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
    layer = layer_file(stage, called="textured")
    spec = compile_world(layer)
    apply_lights(spec, layer)
    _textured_materials_win(spec)
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
