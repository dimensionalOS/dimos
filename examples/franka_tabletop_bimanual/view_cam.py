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

"""Live 640x480 colour view of one RealSense, for aiming it. Press q (or Esc) in the window to quit.

~/dimos-franka/.venv/bin/python view_cam.py            # tripod scene camera
~/dimos-franka/.venv/bin/python view_cam.py 2855       # any camera by serial (or its last digits)
"""

import sys

import cv2
import numpy as np
import pyrealsense2 as rs

TRIPOD = "261922070328"

want = sys.argv[1] if len(sys.argv) > 1 else TRIPOD


def start() -> tuple[str, rs.pipeline]:
    """Open the camera, waiting for it to (re)appear on USB."""
    while True:
        found = [
            d
            for d in rs.context().query_devices()
            if d.get_info(rs.camera_info.serial_number).endswith(want)
        ]
        if found:
            serial = found[0].get_info(rs.camera_info.serial_number)
            print(
                f"camera {serial} on USB {found[0].get_info(rs.camera_info.usb_type_descriptor)}",
                flush=True,
            )
            cfg = rs.config()
            cfg.enable_device(serial)
            cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            pipe = rs.pipeline()
            try:
                pipe.start(cfg)
                return serial, pipe
            except RuntimeError as e:
                print(f"could not start: {e}", flush=True)
        cv2.waitKey(1000)


serial, pipe = start()
name = f"camera {serial} (q to quit)"
try:
    while True:
        ok, frames = pipe.try_wait_for_frames(3000)
        if not ok:
            # A cable bump drops the stream: reopen rather than quit.
            print("no frames for 3 s; reconnecting", flush=True)
            try:
                pipe.stop()
            except RuntimeError:
                pass
            serial, pipe = start()
            continue
        img = np.asarray(frames.get_color_frame().get_data()).copy()
        # Thirds and a centre cross, to help level and centre the shot.
        for x in (213, 426):
            cv2.line(img, (x, 0), (x, 479), (255, 255, 255), 1)
        for y in (160, 320):
            cv2.line(img, (0, y), (639, y), (255, 255, 255), 1)
        cv2.drawMarker(img, (320, 240), (0, 0, 255), cv2.MARKER_CROSS, 30, 2)
        cv2.imshow(name, img)
        if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
            break
finally:
    pipe.stop()
    cv2.destroyAllWindows()
