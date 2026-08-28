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

"""Experimental native OpenYAM data-collection blueprint."""

from __future__ import annotations

from datetime import datetime
from typing import cast

from dimos.constants import STATE_DIR
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import ZenohTransport
from dimos.experimental.imitation.collection.recorder import NativeCollectionRecorder
from dimos.experimental.memory.rust_recorder import RustSqliteStoreConfig
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.hardware.sensors.camera.webcam import WebcamConfig
from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
from dimos.msgs.imitation_msgs.EpisodeStatus import EpisodeStatus
from dimos.msgs.protocol import DimosMsg
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.protocol.pubsub.impl.zenohpubsub import QOS_NEVER_DROP, Topic as ZenohTopic
from dimos.robot.manipulators.openyam.blueprints.teleop import teleop_quest_openyam


def _session_db() -> str:
    return str(STATE_DIR / "recordings" / f"session_openyam_{datetime.now():%Y%m%d_%H%M%S}.db")


def _require_zenoh() -> str | None:
    if global_config.transport != "zenoh":
        return "learning-collect-quest-openyam-native requires --transport zenoh"
    return None


learning_collect_quest_openyam_native = (
    autoconnect(
        NativeCollectionRecorder.blueprint(
            store=RustSqliteStoreConfig(path=_session_db()),
        ),
        EpisodeMonitorModule.blueprint(),
        teleop_quest_openyam,
        CameraModule.blueprint(
            instance_name="WristCamera",
            webcam=WebcamConfig(
                camera_index=0,
                width=640,
                height=480,
                fps=30.0,
                frame_id_prefix="wrist",
            ),
            frame_id="wrist_camera_link",
        ),
    )
    .transports(
        {
            ("color_image", Image): ZenohTransport(
                ZenohTopic("dimos/color_image", Image, qos=QOS_NEVER_DROP)
            ),
            ("coordinator_joint_state", JointState): ZenohTransport(
                ZenohTopic(
                    "dimos/coordinator_joint_state",
                    JointState,
                    qos=QOS_NEVER_DROP,
                )
            ),
            ("applied_joint_position_command", JointState): ZenohTransport(
                ZenohTopic(
                    "dimos/applied_joint_position_command",
                    JointState,
                    qos=QOS_NEVER_DROP,
                )
            ),
            ("status", EpisodeStatus): ZenohTransport(
                ZenohTopic(
                    "dimos/status",
                    cast("type[DimosMsg]", EpisodeStatus),
                    qos=QOS_NEVER_DROP,
                )
            ),
        }
    )
    .global_config(transport="zenoh")
    .requirements(_require_zenoh)
)
