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

import mujoco
import numpy as np
import pytest

from dimos.sim2.mujoco.look import (
    agx,
    ambient_headroom,
    display,
    light_headroom,
    merged_light,
    pbr_neutral,
    scale_light,
    srgb_bytes,
    stage_tone_map,
)
from dimos.sim2.usd_layer import layer_file

# linux-aarch64 has no usd-core wheel, and every scene here is a USD layer.
pytest.importorskip("pxr")

SCENE = Path(__file__).parent / "assets" / "scene.usda"


def test_pbr_neutral_matches_the_khronos_reference_curve():
    linear = np.array(
        [[0, 0, 0], [0.18, 0.18, 0.18], [0.5, 0.2, 0.1], [1, 1, 1], [4, 1, 0.5]], np.float32
    )
    # Worked through the reference GLSL in KhronosGroup/ToneMapping by hand.
    knee = 1 - 0.24**2 / (0.96 + 0.24 - 0.76)
    white = 1 - 1 / (0.15 * (0.96 - knee) + 1)
    assert pbr_neutral(linear) == pytest.approx(
        np.array(
            [
                [0, 0, 0],
                [0.14, 0.14, 0.14],
                [0.46, 0.16, 0.06],
                [knee * (1 - white) + knee * white] * 3,
                [0.9832558, 0.46829915, 0.38247305],
            ]
        ),
        abs=1e-5,
    )
    assert pbr_neutral(np.array([[1e6, 1e6, 1e6]], np.float32)).max() < 1.0


def test_agx_is_blenders_agx_view():
    linear = np.array(
        [
            [0, 0, 0],
            [0.01] * 3,
            [0.18] * 3,
            [1] * 3,
            [4] * 3,
            [100] * 3,
            [0.5, 0.2, 0.1],
            [0.05, 0.3, 0.9],
            [3, 1, 0.2],
        ],
        np.float32,
    )
    # Blender 5.1's own OCIO processor, Linear Rec.709 -> sRGB display through the "AgX" view.
    blender = [
        [0, 0, 0],
        [0.0725] * 3,
        [0.4613] * 3,
        [0.771] * 3,
        [0.9137] * 3,
        [1, 1, 1],
        [0.6833, 0.4793, 0.361],
        [0.3718, 0.5863, 0.782],
        [0.9249, 0.7555, 0.639],
    ]
    assert agx(linear) == pytest.approx(np.array(blender), abs=2e-4)


def test_srgb_encoding_is_the_standard_transfer_curve():
    linear = np.array([0.0, 0.0031308, 0.18, 0.5, 1.0, 2.0], np.float32)
    assert srgb_bytes(linear).tolist() == [0, 10, 118, 188, 255, 255]


def test_a_render_scaled_by_its_headroom_comes_back_as_the_light_it_was():
    render = np.full((1, 1, 3), 51, np.uint8)
    assert display(render, 5.0, "linear").tolist() == [[[255, 255, 255]]]
    assert display(render, 1.0, "linear").tolist() == srgb_bytes(np.full((1, 1, 3), 0.2)).tolist()


def test_the_shade_and_the_direct_light_merge_into_one_linear_image():
    shade = np.array([[[51, 51, 51], [51, 51, 51], [255, 255, 255]]], np.uint8)
    lit = np.array([[[5, 5, 5], [102, 102, 102], [0, 0, 0]]], np.uint8)
    merged = merged_light(shade, 1.0, lit, 10.0)
    # Unlit, the shade's finer steps; lit, the shade plus what the lit render adds; the sky, left out of the lit render, the shade's.
    assert merged[0, :, 0] == pytest.approx([0.2, 4.0, 1.0])


def test_the_shades_headroom_counts_ambient_light_alone():
    model = mujoco.MjModel.from_xml_string(
        '<mujoco><visual><headlight active="0"/></visual><worldbody>'
        '<light directional="true" ambient="2 2 2" diffuse="9 9 9" specular="1 1 1"/>'
        '<geom type="plane" size="1 1 1"/></worldbody></mujoco>'
    )
    assert ambient_headroom(model) == pytest.approx(2.0)
    assert light_headroom(model) == pytest.approx(12.0)


def test_the_headroom_scales_every_light_so_the_brightest_surface_fits():
    model = mujoco.MjModel.from_xml_string(
        '<mujoco><visual><headlight active="0"/></visual><worldbody>'
        '<light directional="true" ambient="2 2 2" diffuse="1 0.5 0.5" specular="0.5 0.5 0.5"/>'
        '<light directional="true" ambient="0 0 0" diffuse="0.5 0.5 0.5" specular="0 0 0"/>'
        '<geom type="plane" size="1 1 1"/></worldbody></mujoco>'
    )
    headroom = light_headroom(model)
    assert headroom == pytest.approx(4.0)
    scale_light(model, 1 / headroom)
    assert light_headroom(model) == pytest.approx(1.0)
    assert model.light_ambient[0] == pytest.approx([0.5, 0.5, 0.5])


def _with_settings(attributes: str) -> Path:
    return layer_file(
        SCENE.read_text() + f'\ndef RenderSettings "look"\n{{\n{attributes}\n}}\n', called="look"
    )


def test_the_tone_map_is_the_one_the_render_settings_name():
    assert stage_tone_map(_with_settings('    uniform token dimos:look:toneMap = "neutral"')) == (
        "neutral"
    )
    assert stage_tone_map(_with_settings("    int omni:rtx:post:tonemap:op = 1")) == "linear"
    assert stage_tone_map(_with_settings('    uniform token dimos:look:toneMap = "agx"')) == "agx"
    assert stage_tone_map(_with_settings("")) is None
    assert stage_tone_map(SCENE) is None


def test_a_view_renders_linear_light_only_for_a_stage_that_asks_for_a_tone_map():
    from dimos.sim2.mujoco.view import MujocoView

    sky = '\ndef DomeLight "sky"\n{\n    float inputs:intensity = 3\n}\n'
    plain = layer_file(SCENE.read_text() + sky, called="look")
    toned = layer_file(
        SCENE.read_text() + sky + 'def RenderSettings "look"\n{\n'
        '    uniform token dimos:look:toneMap = "neutral"\n}\n',
        called="look",
    )
    assert MujocoView(plain).model.light_ambient[0] == pytest.approx([3, 3, 3])
    # Scaled into MuJoCo's 8 bits, to be scaled back up after the read.
    assert MujocoView(toned).model.light_ambient[0] == pytest.approx([1, 1, 1])


def test_a_lidar_view_opens_on_a_dome_sky_given_as_pixels():
    from dimos.sim2.mujoco.view import MujocoView
    from dimos.sim2.mujoco.world import SensorMount

    sky = '\ndef DomeLight "sky"\n{\n    color3f inputs:color = (1, 0.5, 0.25)\n}\n'
    scene = layer_file(SCENE.read_text() + sky, called="look")
    lidar = MujocoView(scene, mount=SensorMount("lidar", "world", "lidar"))
    assert lidar.model.tex_type.tolist() == [mujoco.mjtTexture.mjTEXTURE_SKYBOX]


def test_a_tone_mapped_view_shows_the_domes_sky_as_blender_would():
    from dimos.sim2.mujoco.view import MujocoView

    toned = layer_file(
        SCENE.read_text() + '\ndef DomeLight "sky"\n{\n    color3f inputs:color = (1, 0.5, 0.25)\n'
        "    float inputs:intensity = 3\n}\n"
        'def RenderSettings "look"\n{\n    uniform token dimos:look:toneMap = "neutral"\n}\n',
        called="look",
    )
    view = MujocoView(toned)
    model = view.model
    sky = model.tex_data[model.tex_adr[0] : model.tex_adr[0] + 3].reshape(1, 1, 3)
    # MuJoCo draws a skybox's texels unlit, so the render of the sky is its texel, read back up.
    expected = srgb_bytes(pbr_neutral(np.array([[[3, 1.5, 0.75]]], np.float32)))
    # The sky (3) is the brightest light here, so it sets the headroom.
    assert display(sky, 3.0, "neutral").astype(int) == pytest.approx(expected, abs=2)
    assert sky.max() == 255
