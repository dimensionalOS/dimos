#!/usr/bin/env python3
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

"""Teleop blueprints for testing and deployment.

Single sim/real blueprints — pass `--simulation` to run inside MuJoCo, omit for
real hardware. The underlying coordinator blueprints branch on
`global_config.simulation`.
"""

from dimos.control.blueprints.teleop import (
    coordinator_teleop_dual,
    coordinator_teleop_piper,
    coordinator_teleop_xarm6,
    coordinator_teleop_xarm7,
    piper_teleop_robot_model_config,
    xarm7_teleop_robot_model_config,
)
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.manipulation.data_collection.episode_boundary import EpisodeBoundary
from dimos.manipulation.data_collection.piper_blueprint_config import (
    piper_data_collection_rerun_config,
)
from dimos.manipulation.data_collection.recorder import RerunDataRecorder
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.teleop.quest.quest_extensions import ArmTeleopModule
from dimos.teleop.quest.quest_types import Buttons
from dimos.visualization.vis_module import vis_module

# Arm teleop with press-and-hold engage (has rerun viz)
teleop_quest_rerun = autoconnect(
    ArmTeleopModule.blueprint(),
    vis_module("rerun"),
).transports(
    {
        ("left_controller_output", PoseStamped): LCMTransport("/teleop/left_delta", PoseStamped),
        ("right_controller_output", PoseStamped): LCMTransport("/teleop/right_delta", PoseStamped),
        ("buttons", Buttons): LCMTransport("/teleop/buttons", Buttons),
    }
)


# XArm7 teleop (mock Meshcat preview by default, real with IP, MuJoCo with --simulation):
# right controller -> xarm7.
teleop_quest_xarm7 = autoconnect(
    ArmTeleopModule.blueprint(task_names={"right": "teleop_xarm"}),
    coordinator_teleop_xarm7,
    ManipulationModule.blueprint(
        robots=[xarm7_teleop_robot_model_config()],
        enable_viz=True,
    ),
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("right_controller_output", PoseStamped): LCMTransport(
            "/coordinator/cartesian_command", PoseStamped
        ),
        ("buttons", Buttons): LCMTransport("/teleop/buttons", Buttons),
    }
)


# Piper teleop (ManipulationModule viz on hardware/mock, MuJoCo with --simulation):
# right controller -> piper arm.
teleop_quest_piper = autoconnect(
    ArmTeleopModule.blueprint(task_names={"right": "teleop_piper"}),
    coordinator_teleop_piper,
    ManipulationModule.blueprint(
        robots=[piper_teleop_robot_model_config()],
        enable_viz=True,
    ),
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("right_controller_output", PoseStamped): LCMTransport(
            "/coordinator/cartesian_command", PoseStamped
        ),
        ("buttons", Buttons): LCMTransport("/teleop/buttons", Buttons),
    }
)

# Piper teleop data collection: right controller -> piper arm, with USB camera
# observation plus measured state / desired action streams. A live Rerun viewer
# and a standalone on-disk `.rrd` recorder are wired as independent sinks that
# share a single `piper_data_collection_rerun_config()` instance — guaranteeing
# the viewer and the recorder cannot drift at wiring time. Rolling over to the
# next episode within a session is operator-driven via `EpisodeBoundary`
# (right-controller A by default), which calls `recorder.rotate_recording()`.
_data_collection_rerun_config = piper_data_collection_rerun_config()
# Subset of `_data_collection_rerun_config` accepted by RerunBridgeModule.Config.
_DATA_COLLECTION_BRIDGE_KEYS = (
    "visual_override",
    "entity_prefix",
    "topic_to_entity",
    "blueprint",
)
# Subset of `_data_collection_rerun_config` accepted by RerunDataRecorderConfig.
_DATA_COLLECTION_RECORDER_KEYS = (
    "visual_override",
    "entity_prefix",
    "topic_to_entity",
    "camera_entity_path",
    "record_path_factory",
    "recording_id_factory",
    "episode_metadata",
)
teleop_quest_piper_data_collection = autoconnect(
    ArmTeleopModule.blueprint(task_names={"right": "teleop_piper"}),
    coordinator_teleop_piper,
    CameraModule.blueprint(),
    RerunDataRecorder.blueprint(
        **{k: _data_collection_rerun_config[k] for k in _DATA_COLLECTION_RECORDER_KEYS}
    ),
    EpisodeBoundary.blueprint(),
    ManipulationModule.blueprint(
        robots=[piper_teleop_robot_model_config()],
        enable_viz=True,
    ),
    vis_module(
        "rerun",
        rerun_config={k: _data_collection_rerun_config[k] for k in _DATA_COLLECTION_BRIDGE_KEYS},
    ),
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("desired_joint_action", JointState): LCMTransport(
            "/coordinator/desired_joint_action", JointState
        ),
        ("right_controller_output", PoseStamped): LCMTransport(
            "/coordinator/cartesian_command", PoseStamped
        ),
        ("buttons", Buttons): LCMTransport("/teleop/buttons", Buttons),
        ("color_image", Image): LCMTransport("/piper_data_collection/color_image", Image),
    }
)

# XArm6 teleop (sim with --simulation, real otherwise): right controller -> xarm6
teleop_quest_xarm6 = autoconnect(
    ArmTeleopModule.blueprint(task_names={"right": "teleop_xarm"}),
    coordinator_teleop_xarm6,
).transports(
    {
        ("right_controller_output", PoseStamped): LCMTransport(
            "/coordinator/cartesian_command", PoseStamped
        ),
        ("buttons", Buttons): LCMTransport("/teleop/buttons", Buttons),
    }
)


# Dual arm teleop: right -> piper, left -> xarm6 (TeleopIK, real-only)
teleop_quest_dual = autoconnect(
    ArmTeleopModule.blueprint(task_names={"right": "teleop_piper", "left": "teleop_xarm"}),
    coordinator_teleop_dual,
).transports(
    {
        ("right_controller_output", PoseStamped): LCMTransport(
            "/coordinator/cartesian_command", PoseStamped
        ),
        ("left_controller_output", PoseStamped): LCMTransport(
            "/coordinator/cartesian_command", PoseStamped
        ),
        ("buttons", Buttons): LCMTransport("/teleop/buttons", Buttons),
    }
)


__all__ = [
    "teleop_quest_dual",
    "teleop_quest_piper",
    "teleop_quest_piper_data_collection",
    "teleop_quest_rerun",
    "teleop_quest_xarm6",
    "teleop_quest_xarm7",
]
