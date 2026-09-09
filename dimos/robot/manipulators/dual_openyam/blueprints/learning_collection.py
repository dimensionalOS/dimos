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

"""Dual OpenYAM Quest collection with profile-defined camera inputs."""

from pathlib import Path

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.imitation.cameras import CameraDevice, profile_cameras
from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
from dimos.imitation.collection.native_recorder import collection_recorder
from dimos.imitation.collection.profile import CollectionProfile
from dimos.robot.manipulators.dual_openyam.learning import DUAL_OPENYAM_COLLECTION


def build_dual_openyam_quest_collection(
    *,
    recording: Path,
    task: str,
    cameras: dict[str, CameraDevice],
    profile: CollectionProfile = DUAL_OPENYAM_COLLECTION,
    left_can_port: str | None = None,
    right_can_port: str | None = None,
) -> Blueprint:
    """Build bimanual collection; camera count and feature names come from the profile."""
    camera_blueprints, remappings = profile_cameras(profile, cameras)
    # Pink and the dual robot model are optional until this stack is selected.
    from dimos.robot.manipulators.dual_openyam.blueprints.teleop import (
        build_dual_openyam_quest_teleop,
    )

    return autoconnect(
        collection_recorder(profile=profile, recording=recording),
        EpisodeMonitorModule.blueprint(task=task),
        build_dual_openyam_quest_teleop(
            left_can_port=left_can_port,
            right_can_port=right_can_port,
        ),
        *camera_blueprints,
    ).remappings(remappings)
