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

"""Shadow maps on slow renderers: time one render, turn shadow mapping off.

Shadow mapping a mesh-heavy scene costs several times a plain render on
integrated GPUs and in software GL (Mesa llvmpipe), which pins a simulation
that renders inside its physics loop below real time.
"""

from __future__ import annotations

import time
from typing import Literal, TypeAlias

import mujoco

#: "auto" decides from a timed render; "on" and "off" force the setting.
AutoSwitch: TypeAlias = Literal["auto", "on", "off"]

#: A shadowed render that takes more than this share of its frame budget is too slow.
SLOW_RENDER_SHARE = 0.3


def render_ms(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    camera_id: int,
    *,
    width: int,
    height: int,
    samples: int = 5,
    warmup: int = 3,
) -> float:
    """Average milliseconds of one offscreen colour render from ``camera_id``.

    The first renders pay the asset upload, so ``warmup`` renders are not timed.
    It opens and closes its own renderer: time before building the renderers
    that ``disable_shadows`` should reach.
    """
    renderer = mujoco.Renderer(model, height=height, width=width)
    try:
        for _ in range(warmup):
            renderer.update_scene(data, camera=camera_id)
            renderer.render()
        start = time.perf_counter()
        for _ in range(samples):
            renderer.update_scene(data, camera=camera_id)
            renderer.render()
        return (time.perf_counter() - start) / samples * 1e3
    finally:
        renderer.close()


def disable_shadows(model: mujoco.MjModel) -> None:
    """Turn shadow mapping off for every viewer and renderer built after this call.

    The shadow framebuffer is allocated at context creation, so the viewer and
    renderers must be built after it. Without a shadow map MuJoCo darkens
    everything outside a narrow spot light's cone, so those lights go off too.
    """
    model.vis.quality.shadowsize = 0
    narrow_spots = (model.light_type == mujoco.mjtLightType.mjLIGHT_SPOT) & (
        model.light_cutoff < 90
    )
    model.light_active[narrow_spots] = 0
