# Copyright 2025 Dimensional Inc.
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

import threading

import reactivex as rx
import reactivex.operators as ops

from dimos.robot.sim.minecraft.action import Action


class Engine:
    _act = None
    frequency: float

    def __init__(self, frequency=20.0):
        self.frequency = frequency
        self._stop_event = threading.Event()
        self.loop_thread = None
        import minedojo

        self.env = minedojo.make(
            task_id="creative:1",
            image_size=(800, 1280),
            world_seed="dimensionalxx",
            use_voxel=True,
            voxel_size=dict(xmin=-10, ymin=-2, zmin=-10, xmax=10, ymax=2, zmax=10),
        )

    def noop(self):
        """Return a no-op Action instance."""
        return Action()

    def act(self, act):
        self._act = act

    def start(self):
        self._act = self.noop().array
        self.env.reset()
        self._stop_event.clear()

    def get_stream(self):
        self.start()
        self._done = False

        def step_environment(_):
            if self._stop_event.is_set() or self._done:
                return None

            try:
                obs, reward, terminated, truncated, info = self.env.step(self._act)
                if terminated or truncated:
                    self._done = True
                return (obs, reward, terminated, truncated, info)
            except RuntimeError as e:
                if "done=True" in str(e):
                    self._done = True
                    return None
                raise

        def on_dispose():
            self.stop()

        return rx.interval(1.0 / self.frequency).pipe(
            ops.map(step_environment),
            ops.take_while(lambda x: x is not None),
            ops.finally_action(on_dispose),
        )

    def stop(self):
        self._stop_event.set()
        if hasattr(self, "env") and self.env:
            self.env.close()
