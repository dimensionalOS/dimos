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

from pytest import MonkeyPatch

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.global_config import global_config
from dimos.core.transport import ZenohTransport
from dimos.imitation.collection.native_recorder import NativeCollectionRecorder
from dimos.msgs.imitation_msgs.EpisodeStatus import EpisodeStatus
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.protocol.pubsub.impl.zenohpubsub import QOS_NEVER_DROP
from dimos.robot.manipulators.openyam.blueprints.learning_collection import (
    _require_zenoh,
    learning_collect_quest_openyam,
)


def test_openyam_collection_uses_the_native_recorder_and_zenoh() -> None:
    assert learning_collect_quest_openyam.active_blueprints[0].module is NativeCollectionRecorder
    assert learning_collect_quest_openyam.global_config_overrides["transport"] == "zenoh"


def test_native_openyam_recorded_streams_never_drop() -> None:
    payload_types = {
        "color_image": Image,
        "coordinator_joint_state": JointState,
        "applied_joint_position_command": JointState,
        "status": EpisodeStatus,
    }
    for stream, payload_type in payload_types.items():
        transport = learning_collect_quest_openyam.transport_map[(stream, payload_type)]

        assert isinstance(transport, ZenohTransport)
        assert transport.topic.qos == QOS_NEVER_DROP


def test_native_openyam_paths_are_configurable_from_cli() -> None:
    parsed = BlueprintConfigParser(learning_collect_quest_openyam).parse(
        [
            "--nativecollectionrecorder.store.path",
            "/tmp/native-openyam.mcap",
            "--WristCamera.webcam.camera-index",
            "/dev/v4l/by-id/usb-wrist-camera",
            "--task",
            "pick up the red block",
        ],
        environ={},
    )

    assert parsed.module_kwargs("nativecollectionrecorder")["store"]["path"] == (
        "/tmp/native-openyam.mcap"
    )
    assert parsed.module_kwargs("WristCamera")["webcam"]["camera_index"] == (
        "/dev/v4l/by-id/usb-wrist-camera"
    )
    assert parsed.module_kwargs("episodemonitormodule")["task"] == "pick up the red block"


def test_native_openyam_rejects_an_lcm_override(monkeypatch: MonkeyPatch) -> None:
    monkeypatch.setattr(global_config, "transport", "lcm")

    assert _require_zenoh() == ("learning-collect-quest-openyam requires --transport zenoh")
