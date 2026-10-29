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

"""Use MuJoCo's native offscreen context for robosuite on macOS worker threads.

robosuite 1.5.1 selects GLFW on macOS, even when MUJOCO_GL is cgl. GLFW's
Cocoa initialization requires the main thread; MuJoCo's native CGL context
does not. Only context creation differs here. Rendering, depth, segmentation,
buffer resizing, texture uploads and destruction remain upstream methods.
"""

from functools import cache
import sys
from typing import Any

import mujoco
from robosuite.environments.base import REGISTERED_ENVS, MujocoEnv, make
from robosuite.utils.binding_utils import MjRenderContextOffscreen, MjSim


class NativeOffscreenContext(MjRenderContextOffscreen):
    """Initialize the upstream renderer with the native MuJoCo GL context."""

    def __init__(self, sim: MjSim, device_id: int = -1) -> None:
        self.sim = sim
        self.offscreen = True
        self.device_id = device_id
        self.gl_ctx = mujoco.GLContext(640, 480)
        self.gl_ctx.make_current()
        sim.forward()
        self.model, self.data = sim.model, sim.data
        self.scn = mujoco.MjvScene(sim.model._model, maxgeom=10000)
        self.cam = mujoco.MjvCamera()
        self.cam.fixedcamid = 0
        self.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
        self.vopt = mujoco.MjvOption()
        self.pert = mujoco.MjvPerturb()
        self.pert.active = 0
        self.pert.select = 0
        self.pert.skinselect = -1
        self._set_mujoco_context_and_buffers()
        sim.add_render_context(self)


class NativeRendering(MujocoEnv):
    """Install the native context at the upstream simulation-creation hook."""

    def _initialize_sim(self, xml_string: str | None = None) -> None:
        super()._initialize_sim(xml_string)
        if sys.platform == "darwin" and self.has_offscreen_renderer:
            NativeOffscreenContext(self.sim, device_id=self.render_gpu_device_id)


@cache
def _native_environment(environment_type: type[MujocoEnv]) -> type[MujocoEnv]:
    return type(
        f"NativeRendering{environment_type.__name__}",
        (NativeRendering, environment_type),
        {"__module__": __name__},
    )


def make_environment(name: str, **kwargs: Any) -> MujocoEnv:
    """Construct an upstream task with native macOS offscreen rendering."""
    if not kwargs.get("has_renderer", False):
        # Upstream only destroys the previous simulation on hard reset in this mode.
        kwargs.setdefault("renderer", "mujoco")
    if sys.platform != "darwin":
        return make(name, **kwargs)
    if name not in REGISTERED_ENVS:
        raise ValueError(f"unknown robosuite environment {name!r}")
    return _native_environment(REGISTERED_ENVS[name])(**kwargs)
