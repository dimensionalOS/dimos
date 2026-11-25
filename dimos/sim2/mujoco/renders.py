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

"""MuJoCo renders here or from an exec'd process: on macOS Metal compiles no shader in a forked worker, so a first shadow aborts it."""

from collections.abc import Sequence
from dataclasses import asdict, dataclass
from multiprocessing.connection import Connection
import socket
import subprocess
import sys

import mujoco
import numpy as np
from numpy.typing import NDArray


@dataclass(frozen=True)
class RenderQuality:
    """How a camera draws, in MuJoCo's own render knobs; a None takes the model's (MuJoCo's default unless the scene sets it)."""

    # Lights cast shadows (mjRND_SHADOW).
    shadows: bool = True
    # Each light's shadow map is this many texels square (vis.quality.shadowsize).
    shadow_size: int | None = None
    # Multisampling of the offscreen buffer, for anti-aliasing (vis.quality.offsamples).
    samples: int | None = None
    # Reflective materials mirror the scene (mjRND_REFLECTION).
    reflections: bool = True
    # The skybox is drawn behind everything (mjRND_SKYBOX).
    skybox: bool = True
    # Materials show their textures (mjVIS_TEXTURE).
    textures: bool = True
    # The tone curve applied to a stage's linear light ("agx", "neutral", "linear", "none"); None: the stage's.
    tone_map: str | None = None
    # MuJoCo's headlight is on; None: on only when the stage brings no lights of its own.
    headlight: bool | None = None

    def apply_to_model(self, model: mujoco.MjModel) -> None:
        """Set the knobs a GL context reads from the model when it is made."""
        if self.shadow_size is not None:
            model.vis.quality.shadowsize = self.shadow_size
        if self.samples is not None:
            model.vis.quality.offsamples = self.samples
        if self.headlight is not None:
            model.vis.headlight.active = int(self.headlight)


class Renders:
    """Renders in this process, one GL context per image size."""

    def __init__(self, model: mujoco.MjModel, quality: RenderQuality = RenderQuality()) -> None:
        self.model = model
        self._quality = quality
        self._options = mujoco.MjvOption()
        self._options.flags[mujoco.mjtVisFlag.mjVIS_TEXTURE] = int(quality.textures)
        self._renderers: dict[tuple[int, int], mujoco.Renderer] = {}

    def render(
        self,
        data: mujoco.MjData,
        camera: str,
        width: int,
        height: int,
        dimmer: float = 1.0,
        direct: bool = True,
    ) -> NDArray[np.uint8]:
        """`camera`'s image of `data` (lights placed by mj_camlight); `dimmer` < 1 scales every light, sky left out; `direct` False: ambient and emission only."""
        if (width, height) not in self._renderers:
            renderer = mujoco.Renderer(self.model, height, width)
            flags = renderer.scene.flags
            flags[mujoco.mjtRndFlag.mjRND_SHADOW] = int(self._quality.shadows)
            flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = int(self._quality.reflections)
            flags[mujoco.mjtRndFlag.mjRND_SKYBOX] = int(self._quality.skybox)
            self._renderers[width, height] = renderer
        renderer = self._renderers[width, height]
        if dimmer == 1.0 and direct:
            renderer.update_scene(data, camera=camera, scene_option=self._options)
            return renderer.render().copy()
        model, sky = self.model, mujoco.mjtRndFlag.mjRND_SKYBOX
        lights = model.light_ambient.copy(), model.light_diffuse.copy(), model.light_specular.copy()
        emission, shown = model.mat_emission.copy(), renderer.scene.flags[sky]
        model.light_ambient *= dimmer
        model.light_diffuse *= dimmer * direct
        model.light_specular *= dimmer * direct
        model.mat_emission *= dimmer
        # The skybox's texels were scaled for the undimmed light, so a dimmed image leaves it out.
        renderer.scene.flags[sky] = int(shown and dimmer == 1.0)
        try:
            renderer.update_scene(data, camera=camera, scene_option=self._options)
            return renderer.render().copy()
        finally:
            model.light_ambient[:], model.light_diffuse[:], model.light_specular[:] = lights
            model.mat_emission[:] = emission
            renderer.scene.flags[sky] = shown

    def render_each(
        self,
        data: mujoco.MjData,
        camera: str,
        width: int,
        height: int,
        lightings: Sequence[tuple[float, bool]],
    ) -> list[NDArray[np.uint8]]:
        """`camera`'s image of `data` under each (dimmer, direct) of `lightings`, as `render` takes them."""
        return [self.render(data, camera, width, height, *lighting) for lighting in lightings]

    def close(self) -> None:
        for renderer in self._renderers.values():
            renderer.close()
        self._renderers.clear()


class RenderProcess:
    """Renders in a process exec'd for it, sent the model once and each pose after."""

    def __init__(self, model: mujoco.MjModel, quality: RenderQuality = RenderQuality()) -> None:
        ours, theirs = socket.socketpair()
        self._process = subprocess.Popen(
            # -I: the child imports MuJoCo and NumPy from the interpreter's own site-packages alone.
            [sys.executable, "-I", __file__, str(theirs.fileno())],
            pass_fds=(theirs.fileno(),),
        )
        theirs.close()
        self._connection = Connection(ours.detach())
        # As plain fields: the child runs this file as __main__, where the class has another name.
        self._connection.send((model, asdict(quality)))

    def render(
        self,
        data: mujoco.MjData,
        camera: str,
        width: int,
        height: int,
        dimmer: float = 1.0,
        direct: bool = True,
    ) -> NDArray[np.uint8]:
        """`camera`'s image of `data`, whose cameras and lights are placed (mj_camlight); `dimmer` and `direct` as Renders takes them."""
        return self.render_each(data, camera, width, height, [(dimmer, direct)])[0]

    def render_each(
        self,
        data: mujoco.MjData,
        camera: str,
        width: int,
        height: int,
        lightings: Sequence[tuple[float, bool]],
    ) -> list[NDArray[np.uint8]]:
        """`camera`'s image of `data` under each of `lightings`, in one exchange with the process."""
        self._connection.send(
            (data.qpos, data.mocap_pos, data.mocap_quat, camera, width, height, list(lightings))
        )
        reply = self._connection.recv()
        if isinstance(reply, BaseException):
            raise reply
        images: list[NDArray[np.uint8]] = reply
        return images

    def close(self) -> None:
        self._connection.close()
        self._process.wait()


def _serve(descriptor: int) -> None:
    connection = Connection(descriptor)
    model, quality = connection.recv()
    renders = Renders(model, RenderQuality(**quality))
    data = mujoco.MjData(renders.model)
    try:
        while True:
            try:
                qpos, mocap_pos, mocap_quat, camera, width, height, lightings = connection.recv()
            except EOFError:
                return
            data.qpos[:], data.mocap_pos[:], data.mocap_quat[:] = qpos, mocap_pos, mocap_quat
            mujoco.mj_kinematics(renders.model, data)
            mujoco.mj_comPos(renders.model, data)
            mujoco.mj_camlight(renders.model, data)
            try:
                connection.send(renders.render_each(data, camera, width, height, lightings))
            except Exception as error:
                connection.send(error)
    finally:
        renders.close()


if __name__ == "__main__":
    _serve(int(sys.argv[1]))
