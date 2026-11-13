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

"""R1 data contracts for application-independent episode collection."""

from collections.abc import Sequence
from pathlib import Path

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
from dimos.imitation.collection.profile import CollectionProfile
from dimos.imitation.collection.recorder import CollectionModule
from dimos.imitation.dataprep.schema import FeatureSpec, QualityConfig, RecordingSchema, SyncConfig
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.String import String
from dimos.robot.galaxea.r1pro.config import R1PRO_BODY_AXES
from dimos.robot.galaxea.r1pro.joylo import JAWS, UPPER

# Match the spatial camera outputs; capture keeps their existing compressed bytes.
CAMERAS = {
    "head": ("color_image", (384, 480, 3)),
    "left_wrist": ("left_wrist", (720, 1280, 3)),
    "right_wrist": ("right_wrist", (720, 1280, 3)),
}
POSITION_JOINTS = (*UPPER, *JAWS)


def collection_profile(cameras: Sequence[str] = tuple(CAMERAS)) -> CollectionProfile:
    if not cameras or len(set(cameras)) != len(cameras):
        raise ValueError("select at least one distinct camera")
    unknown = set(cameras) - CAMERAS.keys()
    if unknown:
        raise ValueError(f"unknown R1 cameras: {sorted(unknown)}")
    return CollectionProfile(
        observations={
            "observation.state": FeatureSpec(
                stream="coordinator_joint_state",
                message_type=JointState,
                field="position",
                dtype="float32",
                shape=(len(POSITION_JOINTS),),
                names=list(POSITION_JOINTS),
            ),
            **{
                f"observation.images.{camera}": FeatureSpec(
                    stream=CAMERAS[camera][0],
                    message_type=CompressedImage,
                    dtype="video",
                    shape=CAMERAS[camera][1],
                    names=["height", "width", "channels"],
                )
                for camera in cameras
            },
        },
        actions={
            "position": FeatureSpec(
                stream="coordinator_joint_command",
                message_type=JointState,
                field="position",
                dtype="float32",
                shape=(len(POSITION_JOINTS),),
                names=list(POSITION_JOINTS),
                source_kind="joint_position_updates",
                max_age_ms=200,
            ),
            "body_velocity": FeatureSpec(
                stream="coordinator_joint_velocity",
                message_type=JointState,
                field="velocity",
                dtype="float32",
                shape=(3,),
                names=list(R1PRO_BODY_AXES),
                source_kind="joint_velocity_updates",
                max_age_ms=200,
            ),
        },
        metadata={
            "name": "r1pro-teleop",
            "robot_type": "galaxea-r1pro",
            "action_semantics": "dispatched absolute joint positions and body-frame velocities; not hardware acknowledgements",
            "position_units": ["rad"] * len(UPPER) + ["m"] * len(JAWS),
            "body_velocity_units": ["m/s", "m/s", "rad/s"],
            "motion_settings_stream": "motion_settings",
            "cameras": list(cameras),
        },
    )


def collection_schema(profile: CollectionProfile) -> RecordingSchema:
    schema = profile.to_schema()
    schema.action_vector = ("position", "body_velocity")
    # Spatial RGBD runs at 5 Hz. Do not label duplicated frames as a 30 Hz dataset.
    # Independent 5 Hz cameras can differ by half a frame; gaps > two frames fail.
    schema.sync = SyncConfig(
        anchor=next(k for k, f in profile.observations.items() if f.dtype == "video"),
        rate_hz=5,
        tolerance_ms=110,
    )
    schema.quality = QualityConfig(mode="strict", max_alignment_error_ms=110, max_camera_gap_ms=450)
    return schema


def r1pro_collection(
    *, cameras: Sequence[str] = tuple(CAMERAS), recording: Path | None = None
) -> Blueprint:
    """Place before producers on the robot host. Supply --recorder.recording and --episodes.task."""
    profile = collection_profile(cameras)
    return autoconnect(
        CollectionModule.blueprint(
            instance_name="recorder",
            input_types={**profile.input_types(), "motion_settings": String},
            recording_schema=collection_schema(profile),
            **({"recording": recording} if recording else {}),
        ),
        EpisodeMonitorModule.blueprint(instance_name="episodes", button_map={}),
    )
