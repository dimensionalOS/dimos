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

"""Alfred building an open-vocabulary map as it drives, and answering questions about it.

    dimos run alfred-hyperspace
    # then, from anywhere that can reach the module:
    #   hyperspace.find_objects("a traffic cone")

Nothing is pre-computed. The db starts empty; ``HyperspacePatches`` gates the D455's
colour down to about five frames a second, embeds each survivor with all three
checkpoints, and writes the patch vectors, the depth thumbnail and the colour frame
itself into the store. ``Hyperspace`` holds OWLv2, the text towers and the transform
buffer from the moment it starts, and its ``find_objects`` RPC does the whole query
against the map as it stands that instant: the patch index ranks frames, the three
models' agreement picks the places, OWLv2 draws a box on the best frame of each, and
that box plus the frame's depth becomes a box in the world. The answer carries those
frames back with it, so the caller can see what the robot was looking at.

WHAT THIS COSTS, said plainly because the hardware decides whether it is possible:
three SigLIP2 checkpoints at 5 Hz and a resident OWLv2 is well past a Jetson, in both
compute and memory. It is written for a machine that has the headroom. The gate is the
throttle if it does not: ``--hyperspacepatches.min-frame-interval-s`` buys frames back.

Two departures from the rest of Alfred's blueprints, both deliberate:

* Colour is ON. ``_alfred_hardware`` turns it off because nothing consumed it and raw
  colour is 19.6 MB/s of wire traffic. Hyperspace is a consumer, so it comes back, and
  with it the depth alignment that only happens when there is colour to align to.
* Depth is aligned to colour. A patch is placed off the depth under it, and unaligned
  depth is under something else.
"""

from __future__ import annotations

from dimos.constants import RECORDINGS_DIR
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.mapping.hyperspace.module import Hyperspace, HyperspacePatches
from dimos.mapping.hyperspace.siglip_embedder import DEFAULT_TRIO
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.robot.diy.alfred.config import ALFRED
from dimos.robot.diy.alfred.effector_high_level import AlfredHighLevel
from dimos.robot.diy.alfred.mount_tf import AlfredMountTf
from dimos.visualization.rerun.bridge import RerunBridgeModule

# One file, written by the ingest and read by the query. They are separate modules and
# may be separate processes, so this is sqlite in WAL mode doing the handoff: the
# query re-reads the patch rows it has not seen before every question.
HYPERSPACE_DB = str(RECORDINGS_DIR / "alfred_hyperspace.db")

# librealsense leaks usbfs fds in this worker; keep socket-accepting modules out of its
# fd table so EMFILE cannot take the rest of the graph down with it.
RealSenseCamera.dedicated_worker = True

_alfred_seeing = autoconnect(
    RealSenseCamera.blueprint(
        fps=30,
        enable_color=True,
        align_depth_to_color=True,
        enable_infrared=False,
        emitter_enabled=True,
        enable_imu=True,
        frame_id="d455_link",
        serial_number=ALFRED.d455_serial,
    ),
    AlfredMountTf.blueprint(),
    AlfredHighLevel.blueprint().remappings([(AlfredHighLevel, "wheel_odometry", "odom_sources")]),
).global_config(robot_model="alfred")

alfred_hyperspace = autoconnect(
    _alfred_seeing,
    HyperspacePatches.blueprint(
        db_path=HYPERSPACE_DB,
        models=DEFAULT_TRIO,
        # The colour frame behind each embedding frame is kept, because live there is
        # no recording to go back to and the detector has to be shown a picture of
        # somewhere the robot has already left.
        keep_frames=True,
        flat=True,
        # Stereo returns nothing off glass and shiny floors, and a box placed off that
        # lands metres past the thing. Filling costs ~50 ms on the frames that are
        # kept, once, rather than being paid again at every question.
        depth2depth_model="default",
    ),
    Hyperspace.blueprint(
        db_path=HYPERSPACE_DB,
        owl_threshold=0.5,
        detect_models=[],
    ),
    RerunBridgeModule.blueprint(
        pubsubs=[LCM()],
        rerun_open=global_config.rerun_open,
        rerun_web=global_config.rerun_web,
    ),
).global_config(n_workers=8, robot_model="alfred")
