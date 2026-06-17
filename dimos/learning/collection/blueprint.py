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

"""Recording blueprints.

`CollectionRecorder` (a memory2 Recorder) captures the obs/action/status
streams to a SQLite session DB during the run and flushes it durably on
shutdown. DataPrep reads that DB afterwards.
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport, pLCMTransport
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.learning.collection.episode_monitor import (
    EpisodeMonitorModule,
    EpisodeStatus,
)
from dimos.learning.collection.recorder import CollectionRecorder
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.teleop.quest.blueprints import (
    teleop_quest_piper,
    teleop_quest_xarm7,
)
from dimos.teleop.quest.quest_types import Buttons

_DEFAULT_BUTTON_MAP = {"start": "A", "save": "B", "discard": "X"}


def _camera_if_real() -> tuple[Blueprint, ...]:
    """Real RealSense only off-sim. In `--simulation` the teleop coordinator's
    MujocoSimModule already publishes color_image on /camera/color_image, so a
    real camera would be redundant (and fail with no device connected)."""
    if global_config.simulation:
        return ()
    return (RealSenseCamera.blueprint(enable_pointcloud=False),)


# Transports inline per blueprint so each recording config is self-contained.
# joint_state is declared explicitly (not left to autoconnect) so it keeps
# recording if the recorder moves to its own process.
learning_collect_quest_xarm7 = autoconnect(
    teleop_quest_xarm7,
    *_camera_if_real(),
    EpisodeMonitorModule.blueprint(button_map=_DEFAULT_BUTTON_MAP),
    CollectionRecorder.blueprint(),
).transports(
    {
        ("buttons", Buttons): LCMTransport("/teleop/buttons", Buttons),
        ("color_image", Image): LCMTransport("/camera/color_image", Image),
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("status", EpisodeStatus): pLCMTransport("/learning/episode_status"),
    }
)


learning_collect_quest_piper = autoconnect(
    teleop_quest_piper,
    *_camera_if_real(),
    EpisodeMonitorModule.blueprint(button_map=_DEFAULT_BUTTON_MAP),
    CollectionRecorder.blueprint(),
).transports(
    {
        ("buttons", Buttons): LCMTransport("/teleop/buttons", Buttons),
        ("color_image", Image): LCMTransport("/camera/color_image", Image),
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("status", EpisodeStatus): pLCMTransport("/learning/episode_status"),
    }
)


__all__ = [
    "learning_collect_quest_piper",
    "learning_collect_quest_xarm7",
]
