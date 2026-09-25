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

"""Viewer mirror: a separate process that shows a running TARS sim in the MuJoCo viewer.

The simulation stays wherever the TarsClient lives (a script, a dimos worker thread...).
This process rebuilds the identical model, copies qpos from shared memory at 60 Hz and
renders it. Running the viewer in its own process (mjpython on macOS) avoids the
"GUI must be on the main thread" problem. Started by TarsClient(viewer=True).
"""

from __future__ import annotations

import argparse
from multiprocessing import resource_tracker, shared_memory
from pathlib import Path
import subprocess
import sys
import time

import mujoco
import mujoco.viewer
import numpy as np

from tars_sdk.model.params import Params
from tars_sdk.sim import SimBackend, build_model


class MirrorPublisher:
    """Sim side: writes qpos into shared memory and owns the viewer subprocess."""

    def __init__(self, backend: SimBackend, scale: float) -> None:
        self._backend = backend
        nq = backend.model.nq
        self._shm = shared_memory.SharedMemory(create=True, size=8 * (nq + 1))
        self._buf = np.ndarray((nq + 1,), dtype=np.float64, buffer=self._shm.buf)
        self._buf[:] = 0.0
        self._last = 0.0
        exe = Path(sys.executable)
        mjpython = exe.with_name("mjpython")
        python = str(mjpython) if sys.platform == "darwin" and mjpython.exists() else str(exe)
        x, y, yaw = backend.spawn
        cmd = [python, "-m", "tars_sdk.mirror", "--shm", self._shm.name, "--scale", str(scale)]
        cmd += ["--spawn", str(x), str(y), str(yaw)]
        if backend.scene is not None:
            cmd += ["--scene", str(backend.scene)]
        self.publish(force=True)
        self._proc = subprocess.Popen(cmd)
        backend.add_step_hook(self.publish)

    def publish(self, force: bool = False) -> None:
        now = time.monotonic()
        if not force and now - self._last < 1 / 60:
            return
        self._last = now
        self._buf[1:] = self._backend.data.qpos
        self._buf[0] += 1.0  # sequence number, tells the viewer new data arrived

    def close(self) -> None:
        self._proc.terminate()
        try:
            self._proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            self._proc.kill()
        self._shm.close()
        self._shm.unlink()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--shm", required=True)
    ap.add_argument("--scale", type=float, required=True)
    ap.add_argument("--scene", type=Path, default=None)
    ap.add_argument("--spawn", type=float, nargs=3, default=(0.0, 0.0, 0.0))
    args = ap.parse_args()

    model, prefix = build_model(Params(scale=args.scale).resolved(), args.scene, tuple(args.spawn))
    data = mujoco.MjData(model)
    shm = shared_memory.SharedMemory(name=args.shm)
    # attaching registers the block with this process's resource tracker, which would
    # unlink it on exit; the sim side owns it
    resource_tracker.unregister(shm._name, "shared_memory")  # type: ignore[attr-defined]
    buf = np.ndarray((model.nq + 1,), dtype=np.float64, buffer=shm.buf)
    hub = model.body(prefix + "base_link").id
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 3.0, 135.0, -20.0
        seq = -1.0
        while viewer.is_running():
            if buf[0] != seq:
                seq = float(buf[0])
                data.qpos[:] = buf[1:]
                mujoco.mj_forward(model, data)
                viewer.cam.lookat[:] = data.xpos[hub]
                viewer.sync()
            time.sleep(1 / 120)
    shm.close()


if __name__ == "__main__":
    main()
