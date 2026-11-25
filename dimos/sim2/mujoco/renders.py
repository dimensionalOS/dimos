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

from multiprocessing.connection import Connection
import socket
import subprocess
import sys

import mujoco
import numpy as np
from numpy.typing import NDArray


class Renders:
    """Renders in this process, one GL context per image size."""

    def __init__(self, model: mujoco.MjModel) -> None:
        self.model = model
        self._renderers: dict[tuple[int, int], mujoco.Renderer] = {}

    def render(
        self, data: mujoco.MjData, camera: str, width: int, height: int
    ) -> NDArray[np.uint8]:
        """`camera`'s image of `data`, whose cameras and lights are placed (mj_camlight)."""
        if (width, height) not in self._renderers:
            self._renderers[width, height] = mujoco.Renderer(self.model, height, width)
        renderer = self._renderers[width, height]
        renderer.update_scene(data, camera=camera)
        return renderer.render().copy()

    def close(self) -> None:
        for renderer in self._renderers.values():
            renderer.close()
        self._renderers.clear()


class RenderProcess:
    """Renders in a process exec'd for it, sent the model once and each pose after."""

    def __init__(self, model: mujoco.MjModel) -> None:
        ours, theirs = socket.socketpair()
        self._process = subprocess.Popen(
            # -I: the child imports MuJoCo and NumPy from the interpreter's own site-packages alone.
            [sys.executable, "-I", __file__, str(theirs.fileno())],
            pass_fds=(theirs.fileno(),),
        )
        theirs.close()
        self._connection = Connection(ours.detach())
        self._connection.send(model)

    def render(
        self, data: mujoco.MjData, camera: str, width: int, height: int
    ) -> NDArray[np.uint8]:
        """`camera`'s image of `data`, whose cameras and lights are placed (mj_camlight)."""
        self._connection.send((data.qpos, data.mocap_pos, data.mocap_quat, camera, width, height))
        reply = self._connection.recv()
        if isinstance(reply, BaseException):
            raise reply
        image: NDArray[np.uint8] = reply
        return image

    def close(self) -> None:
        self._connection.close()
        self._process.wait()


def _serve(descriptor: int) -> None:
    connection = Connection(descriptor)
    renders = Renders(connection.recv())
    data = mujoco.MjData(renders.model)
    try:
        while True:
            try:
                qpos, mocap_pos, mocap_quat, camera, width, height = connection.recv()
            except EOFError:
                return
            data.qpos[:], data.mocap_pos[:], data.mocap_quat[:] = qpos, mocap_pos, mocap_quat
            mujoco.mj_kinematics(renders.model, data)
            mujoco.mj_comPos(renders.model, data)
            mujoco.mj_camlight(renders.model, data)
            try:
                connection.send(renders.render(data, camera, width, height))
            except Exception as error:
                connection.send(error)
    finally:
        renders.close()


if __name__ == "__main__":
    _serve(int(sys.argv[1]))
