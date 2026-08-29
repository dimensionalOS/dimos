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

"""Unitree G1 GR00T WBC + Quest teleop + manipulation + recording.

The GR00T locomotion/control core (without navigation, mapping, or the legacy
viewer) plus the Quest WebXR retargeting module, collision-aware arm
manipulation, and the dimos.imitation data-collection stack. ``--simulation
mujoco`` and ``--scene-package`` remain supported. Put on the headset, open
``https://<host>:8443/teleop``, and:

    left stick        walk forward/back (+ yaw in strafe mode)
    right stick       yaw (press = zero-Twist e-stop)
    X + A             hold to track both arms from a shared reference
    B                 start / save an episode
    Y                 discard the in-progress episode

Controller poses route to the shared ``teleop_g1`` coordinator task declared
in the groot blueprint. With no autonomous navigation in this stack, Quest
locomotion connects directly to the GR00T WBC task.

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
from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
from dimos.imitation.collection.recorder import CollectionRecorder
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_groot_wbc import (
    _G1GrootCoordinator,
    _unitree_g1_groot_wbc_core,
)
from dimos.robot.unitree.g1.manip_config import g1_manipulation_model_config
from dimos.teleop.quest.quest_extensions import MobileVideoArmTeleopModule


class G1CollectionRecorder(CollectionRecorder):
    """CollectionRecorder plus the operator's absolute controller poses.

    The shared teleop IK captures controller and robot references internally,
    so joint commands do not appear on a stream. Recording both controller
    streams preserves the operator input alongside measured joint state.
    """

    # Own process: sqlite/eMMC writes and the torch import must not share
    # a GIL with control modules.
    dedicated_worker = True

    left_cartesian_command: In[PoseStamped]
    right_cartesian_command: In[PoseStamped]


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


class G1ManipulationModule(ManipulationModule):
    """Plan arm motion against the live full-body G1 collision model."""


unitree_g1_teleop = (
    autoconnect(
        _unitree_g1_groot_wbc_core,
        MobileVideoArmTeleopModule.blueprint(),
        G1ManipulationModule.blueprint(
            instance_name="G1Manipulation",
            model=g1_manipulation_model_config(),
            visualization=ViserVisualizationConfig(host="0.0.0.0"),
        ),
        *_camera_if_real(),
        EpisodeMonitorModule.blueprint(),  # default button_map: toggle=B, discard=Y
        G1CollectionRecorder.blueprint(
            db_path=_session_db(),
            # Collection observations/actions are synchronized by timestamp,
            # not localized in the world frame. Declaring them poseless also
            # avoids attempting a world-to-camera lookup when nav localization
            # is disabled for an upper-body-only session.
            poseless_streams=[
                "color_image",
                "status",
                "left_cartesian_command",
                "right_cartesian_command",
                "coordinator_joint_state",
            ],
        ),
    )
    .remappings(
        [
            (MobileVideoArmTeleopModule, "left_controller_output", "left_cartesian_command"),
            (MobileVideoArmTeleopModule, "right_controller_output", "right_cartesian_command"),
            (MobileVideoArmTeleopModule, "cmd_vel", "cmd_vel"),
            (G1ManipulationModule, "_control_coordinator", _G1GrootCoordinator),
        ]
    )
    # Camera frames stay off the LCM bus: both consumers (quest module and
    # recorder) are on-box, and raw images multicast over LCM make each
    # subscribing process pay receive+decode per frame — measured at ~31 MB/s
    # and a starved coordinator tick loop on the Orin. SHM is zero-copy; an
    # unconsumed stream costs only the producer's write.
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
