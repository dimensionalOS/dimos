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

"""Keyboard teleop in the MuJoCo viewer.

    python -m tars_sdk.teleop        # Linux
    mjpython -m tars_sdk.teleop      # macOS (passive viewer needs mjpython)
    mjpython -m tars_sdk.teleop --roll   # start in roll mode

Keys: arrow up/down = vx +/-, arrow left/right = wz +/-, space = stop, S = stand, X = sit,
R = toggle walk / roll mode.
"""

from __future__ import annotations

import argparse
import time

import mujoco.viewer

from tars_sdk.client import TarsClient

KEY_UP, KEY_DOWN, KEY_LEFT, KEY_RIGHT, KEY_SPACE, KEY_S, KEY_X, KEY_R = (
    265,
    264,
    263,
    262,
    32,
    83,
    88,
    82,
)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--roll", action="store_true", help="switch to roll mode once standing")
    ap.add_argument("--scene", default=None, help="environment MJCF to spawn TARS into")
    ap.add_argument("--scale", type=float, default=None, help="size vs the 1.52 m film TARS")
    args = ap.parse_args()
    # Single-threaded: physics steps inside the viewer loop (the passive viewer segfaults
    # on macOS if another thread mutates mjData while it renders).
    client = TarsClient(realtime=False, cmd_timeout=None, scale=args.scale, scene=args.scene)
    client.connect()
    backend = client.sim
    cmd = [0.0, 0.0]
    mode = ["walk"]

    def on_key(key: int) -> None:
        max_vx, max_wz = client.limits
        if key == KEY_UP:
            cmd[0] = min(cmd[0] + max_vx / 4, max_vx)
        elif key == KEY_DOWN:
            cmd[0] = max(cmd[0] - max_vx / 4, -max_vx)
        elif key == KEY_LEFT:
            cmd[1] = min(cmd[1] + max_wz / 3, max_wz)
        elif key == KEY_RIGHT:
            cmd[1] = max(cmd[1] - max_wz / 3, -max_wz)
        elif key == KEY_SPACE:
            cmd[:] = [0.0, 0.0]
        elif key == KEY_S:
            client.stand()
        elif key == KEY_X:
            cmd[:] = [0.0, 0.0]
            client.sit()
        elif key == KEY_R:
            cmd[:] = [0.0, 0.0]
            mode[0] = "roll" if mode[0] == "walk" else "walk"
            client.set_mode(mode[0])
        print(f"[{mode[0]}] cmd vx={cmd[0]:+.2f} m/s  wz={cmd[1]:+.2f} rad/s")

    client.stand()
    if args.roll:
        mode[0] = "roll"
        client.set_mode("roll")
    print(__doc__)
    last_phase = ""
    frame = 0.02  # multiple of the 100 Hz control step, so sim time matches wall time
    with mujoco.viewer.launch_passive(backend.model, backend.data, key_callback=on_key) as viewer:
        hub = backend.model.body(backend.prefix + "base_link").id
        viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 3.0, 135.0, -20.0
        while viewer.is_running():
            t0 = time.perf_counter()
            client.move(*cmd)
            with viewer.lock():
                client.step(frame)
                viewer.cam.lookat[:] = backend.data.xpos[hub]
            viewer.sync()
            phase = client.get_state().mode
            if phase != last_phase:
                print(f"[{client.locomotion}] {phase}")
                last_phase = phase
            time.sleep(max(0.0, frame - (time.perf_counter() - t0)))
    client.disconnect()


if __name__ == "__main__":
    main()
