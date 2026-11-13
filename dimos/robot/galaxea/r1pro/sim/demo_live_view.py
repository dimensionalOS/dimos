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

"""Display the running R1 Pro sim2 state in a read-only native MuJoCo window.

On macOS: mjpython -m dimos.robot.galaxea.r1pro.sim.demo_live_view
On Linux: python -m dimos.robot.galaxea.r1pro.sim.demo_live_view

This only subscribes to observations. It neither advances physics nor sends
commands; closing this window leaves the simulator and operator input running.
"""

import threading
import time

from dimos_lcm.sim_msgs import WorldState
import mujoco
import mujoco.viewer

from dimos.core.transport import ZenohTransport
from dimos.robot.assets.model import UsdModel
from dimos.robot.galaxea.r1pro.sim.config import SCENE
from dimos.sim2.physics.mujoco import MujocoBackend


def main() -> None:
    backend = MujocoBackend()
    backend.load(UsdModel.from_file(SCENE), 5_000_000)
    model, data = backend.model, backend.data
    root_name = "r1pro/base_link"
    root_joint = model.body(root_name).jntadr[0]
    root_address = model.jnt_qposadr[root_joint]
    latest: WorldState | None = None
    lock = threading.Lock()
    ready = threading.Event()

    def receive(sample: WorldState) -> None:
        nonlocal latest
        with lock:
            latest = sample
        ready.set()

    transport: ZenohTransport[WorldState] = ZenohTransport("dimos/observations", WorldState)
    transport.start()
    unsubscribe = transport.subscribe(receive)
    try:
        if not ready.wait(10):
            raise RuntimeError(
                "No live sim2 observations received; start the R1 Pro blueprint first"
            )
        with mujoco.viewer.launch_passive(
            model, data, show_left_ui=False, show_right_ui=False
        ) as viewer:
            viewer.cam.lookat[:] = [0, 0, 0.9]
            viewer.cam.distance = 2.6
            viewer.cam.azimuth = 150
            viewer.cam.elevation = -15
            print("Live R1 Pro sim2 view ready (read-only)", flush=True)
            logged = False
            while viewer.is_running():
                with lock:
                    sample = latest
                if sample is not None:
                    joints = sample.joint_truth
                    with viewer.lock():  # type: ignore[attr-defined]  # Missing in MuJoCo's stub.
                        for name, position in zip(joints.name, joints.position, strict=True):
                            data.qpos[model.joint(name).qposadr[0]] = position
                        root = next(b.pose for b in sample.bodies if b.body_id == root_name)
                        p, q = root.position, root.orientation
                        data.qpos[root_address : root_address + 7] = [
                            p.x,
                            p.y,
                            p.z,
                            q.w,
                            q.x,
                            q.y,
                            q.z,
                        ]
                        data.time = sample.header.time_ns / 1e9
                        mujoco.mj_forward(model, data)
                    viewer.sync()
                    if not logged:
                        print(
                            f"Rendered {len(joints.name)} joints at sim time {data.time:.3f}",
                            flush=True,
                        )
                        logged = True
                time.sleep(1 / 60)
    finally:
        unsubscribe()
        transport.stop()
        backend.close()


if __name__ == "__main__":
    main()
