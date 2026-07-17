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

from datetime import datetime

from dimos.constants import STATE_DIR
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.learning.collection.episode_monitor import EpisodeMonitorModule
from dimos.learning.collection.recorder import CollectionRecorder, CollectionRecorderConfig
from dimos.teleop.quest.blueprints import (
    teleop_quest_piper,
    teleop_quest_xarm7,
)
from dimos.visualization.rerun.collection_status import collection_status_rerun_config
from dimos.visualization.vis_module import vis_module


def _session_db(robot: str) -> str:
    """Timestamped session DB path under the state dir, namespaced by robot."""
    return str(STATE_DIR / "recordings" / f"session_{robot}_{datetime.now():%Y%m%d_%H%M%S}.db")


def _camera_if_real() -> tuple[Blueprint, ...]:
    """Real RealSense only off-sim. In `--simulation` the teleop coordinator's
    MujocoSimModule already publishes color_image on /camera/color_image, so a
    real camera would be redundant (and fail with no device connected)."""
    if global_config.simulation:
        return ()
    return (RealSenseCamera.blueprint(enable_pointcloud=False),)


# buttons / color_image / joint_state / status are left to autoconnect — each
# name is unique across the composed blueprint, so it resolves to a stable
# /<name> topic shared by producer and recorder.
learning_collect_quest_xarm7 = autoconnect(
    teleop_quest_xarm7,
    *_camera_if_real(),
    EpisodeMonitorModule.blueprint(),  # default button_map: toggle=B, discard=Y
    CollectionRecorder.blueprint(db_path=_session_db("xarm7")),
)


learning_collect_quest_piper = autoconnect(
    teleop_quest_piper,
    *_camera_if_real(),
    EpisodeMonitorModule.blueprint(),  # default button_map: toggle=B, discard=Y
    CollectionRecorder.blueprint(db_path=_session_db("piper")),
)


# Separate physical collector: the existing Piper collector remains unchanged.
_piper_rerun_recorder_config = CollectionRecorderConfig(
    db_path=_session_db("piper_rerun"),
    task_label="pick_and_place",
)

learning_collect_quest_piper_rerun = autoconnect(
    teleop_quest_piper,
    RealSenseCamera.blueprint(enable_pointcloud=False),
    EpisodeMonitorModule.blueprint(
        default_task_label=_piper_rerun_recorder_config.task_label,
    ),
    CollectionRecorder.blueprint(
        db_path=_piper_rerun_recorder_config.db_path,
        task_label=_piper_rerun_recorder_config.task_label,
    ),
    vis_module("rerun", rerun_config=dict(collection_status_rerun_config())),
)
