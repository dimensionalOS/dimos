# Copyright 2025-2026 Dimensional Inc.
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

"""Unitree G1 GR00T WBC + Quest teleop + episode recording.

The full ``unitree-g1-groot-wbc`` stack (locomotion policy, nav, viewer,
``--simulation mujoco`` / ``--scene-package`` support) plus the Quest WebXR
retargeting module and the dimos.learning data-collection stack. Put on the
headset, open ``https://<host>:8443/teleop``, and:

    left stick        walk forward/back (+ yaw or strafe, see
                      ``right_stick_mode``)
    right stick       yaw (press = zero-Twist e-stop)
    both triggers     hold to track your hands with the robot's arms
                      (release: arms hold in place)
    B                 start / save an episode
    Y                 discard the in-progress episode

Wrist targets route to the ``dual_arm_ik`` coordinator task declared in
the groot blueprint (frame_id "dual_arm_ik/left|right"); locomotion goes
out as ``cmd_vel``.

Recording runs continuously into a timestamped session DB under
``~/.local/state/dimos/recordings/``; B/Y only place episode markers
(EpisodeMonitorModule). Off-sim, a RealSense provides ``color_image`` —
recorded for training and pushed into the headset as the operator's view.
The groot MuJoCo sim publishes no color camera, so sim sessions record
joints/commands only (point DataPrep's sync anchor at joint state, or
enable a sim color camera, if you need images from sim).

Export afterwards with ``dimos dataprep build`` — measured joint state,
the commanded wrist poses, and episode status are all in the DB, so
action semantics (next-state vs commanded) are a DataPrep config choice.

Usage:
    dimos --simulation mujoco --scene-package office run unitree-g1-teleop
    dimos run unitree-g1-teleop                      # real hardware
"""

from __future__ import annotations

from datetime import datetime

from dimos.constants import DEFAULT_CAPACITY_COLOR_IMAGE, STATE_DIR
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.core.stream import In
from dimos.core.transport import pSHMTransport
from dimos.learning.collection.episode_monitor import EpisodeMonitorModule
from dimos.learning.collection.recorder import CollectionRecorder
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_groot_wbc import unitree_g1_groot_wbc
from dimos.robot.unitree.g1.quest_teleop import G1QuestTeleopModule


class G1CollectionRecorder(CollectionRecorder):
    """CollectionRecorder + the commanded wrist targets.

    The dual-arm IK runs inside the coordinator, so commanded *joint*
    targets never appear on a stream — but the operator's commanded wrist
    poses do (frame_id "dual_arm_ik/left|right" on the cartesian command
    stream). Recording them keeps the commanded-action option open at
    export time; without them a session can only ever yield next-state
    actions.
    """

    # Own process: sqlite/eMMC writes and the torch import must not share
    # a GIL with control modules.
    dedicated_worker = True

    coordinator_cartesian_command: In[PoseStamped]


def _session_db() -> str:
    return str(STATE_DIR / "recordings" / f"session_g1_{datetime.now():%Y%m%d_%H%M%S}.db")


if not global_config.simulation:
    from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera

    class DedicatedRealSenseCamera(RealSenseCamera):
        """Own process: 15 fps frame copies must not share a GIL with the
        coordinator's tick loop (measured arm latency when colocated)."""

        dedicated_worker = True


def _camera_if_real() -> tuple[Blueprint, ...]:
    """Real RealSense only off-sim: the groot MuJoCo sim exposes no color
    camera, and instantiating the module with no device would fail."""
    if global_config.simulation:
        return ()
    return (DedicatedRealSenseCamera.blueprint(enable_pointcloud=False),)


unitree_g1_teleop = (
    autoconnect(
        unitree_g1_groot_wbc,
        G1QuestTeleopModule.blueprint(),
        *_camera_if_real(),
        EpisodeMonitorModule.blueprint(),  # default button_map: toggle=B, discard=Y
        G1CollectionRecorder.blueprint(db_path=_session_db()),
    )
    .remappings(
        [
            (G1QuestTeleopModule, "left_controller_output", "coordinator_cartesian_command"),
            (G1QuestTeleopModule, "right_controller_output", "coordinator_cartesian_command"),
        ]
    )
    # Camera frames stay off the LCM bus: every consumer (quest module,
    # recorder, viewer bridge) is on-box, and raw images multicast over LCM
    # make each subscribing process pay receive+decode per frame — measured
    # at ~31 MB/s and a starved coordinator tick loop on the Orin. SHM is
    # zero-copy; an unconsumed stream costs only the producer's write.
    .transports(
        {
            ("color_image", Image): pSHMTransport(
                "/color_image", default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE
            ),
            ("depth_image", Image): pSHMTransport(
                "/depth_image", default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE
            ),
        }
    )
)
