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

from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
from dimos.imitation.collection.profile import CollectionProfile
from dimos.imitation.collection.recorder import CollectionModule
from dimos.imitation.dataprep.schema import FeatureSpec, QualityConfig, RecordingSchema, SyncConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.msgs.std_msgs.String import String
from dimos.robot.galaxea.r1pro.config import R1PRO_BODY_AXES
from dimos.robot.galaxea.r1pro.joylo import JAWS, UPPER

# The cameras' own color streams, at the camera rate; capture keeps their compressed
# bytes. The 5 Hz RGB-D of the spatial sensors is for the mesh view, not for a dataset.
# The wrist size is the simulation's; the robot's wrists stream HARDWARE_WRIST_SHAPE.
CAMERAS = {
    "head": ("head_left_color", (1536, 1920, 3)),
    "left_wrist": ("wrist_left_color", (720, 1280, 3)),
    "right_wrist": ("wrist_right_color", (720, 1280, 3)),
}
HARDWARE_WRIST_SHAPE = (480, 640, 3)
# The rate of every camera above.
CAMERA_HZ = 30
POSITION_JOINTS = (*UPPER, *JAWS)
# What the operator does with the leader arms and Joy-Cons, and what JoyLo makes of it.
OPERATOR_INPUTS: dict[str, type[Any]] = {
    "leader_readings": JointState,
    "joycon_sample": Joy,
    "leader_joint_command": JointState,
}


def collection_profile(
    cameras: Sequence[str] = tuple(CAMERAS),
    *,
    wrist_shape: tuple[int, int, int] = CAMERAS["left_wrist"][1],
) -> CollectionProfile:
    """The dataset layout: joint state, camera images and commanded actions.

    Args:
        cameras: Which of CAMERAS to record.
        wrist_shape: Wrist image size as (height, width, channels), in pixels.
            Dataset export rejects frames of any other size.
    """
    if not cameras or len(set(cameras)) != len(cameras):
        raise ValueError("select at least one distinct camera")
    unknown = set(cameras) - CAMERAS.keys()
    if unknown:
        raise ValueError(f"unknown R1 cameras: {sorted(unknown)}")
    shapes = {
        camera: wrist_shape if camera.endswith("_wrist") else shape
        for camera, (_, shape) in CAMERAS.items()
    }
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
                    shape=shapes[camera],
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
            "arm_calibration_stream": "arm_calibration_state",
            "retarget_diagnostics_stream": "arm_retarget_state",
            "operator_input_streams": [
                "webxr_pose_observed",
                "webxr_head_observed",
                "webxr_joy_observed",
                "webxr_gamepad_observed",
                *OPERATOR_INPUTS,
            ],
            "mapped_hand_streams": [
                "hand_left_pose",
                "hand_right_pose",
                "left_cartesian_command",
                "right_cartesian_command",
            ],
            "cameras": list(cameras),
        },
    )


def collection_schema(profile: CollectionProfile) -> RecordingSchema:
    schema = profile.to_schema()
    schema.action_vector = ("position", "body_velocity")
    # The cameras run independently: two can differ by half a frame, so that and a
    # little jitter is the tolerance. A gap of more than two frames fails.
    frame_ms = 1000 / CAMERA_HZ
    schema.sync = SyncConfig(
        anchor=next(k for k, f in profile.observations.items() if f.dtype == "video"),
        rate_hz=CAMERA_HZ,
        tolerance_ms=0.55 * frame_ms,
    )
    schema.quality = QualityConfig(
        mode="strict", max_alignment_error_ms=0.55 * frame_ms, max_camera_gap_ms=2.25 * frame_ms
    )
    return schema


def r1pro_collection(
    *,
    cameras: Sequence[str] = tuple(CAMERAS),
    recording: Path | None = None,
    wrist_shape: tuple[int, int, int] = CAMERAS["left_wrist"][1],
    extra_inputs: Mapping[str, type[Any]] | None = None,
) -> Blueprint:
    """The recorder and episode monitor. Supply --recorder.recording and --episodes.task.

    Place it first, on the robot host, so it starts before the streams it records.

    Args:
        cameras: Which of CAMERAS go into the dataset.
        recording: Folder for the new recording; the launch option sets it otherwise.
        wrist_shape: Wrist image size as (height, width, channels), in pixels.
        extra_inputs: More streams to record, by name and message type, outside
            the dataset layout (for example a camera filming the robot).
    """
    profile = collection_profile(cameras, wrist_shape=wrist_shape)
    schema = collection_schema(profile)
    extra = dict(extra_inputs or {})
    schema.metadata["extra_streams"] = list(extra)
    return autoconnect(
        CollectionModule.blueprint(
            instance_name="recorder",
            input_types={
                **profile.input_types(),
                **OPERATOR_INPUTS,
                **extra,
                "motion_settings": String,
                "arm_calibration_state": String,
                "arm_retarget_state": String,
                "webxr_pose_observed": PoseStamped,
                "webxr_head_observed": PoseStamped,
                "webxr_joy_observed": Joy,
                "webxr_gamepad_observed": Joy,
                "left_cartesian_command": PoseStamped,
                "right_cartesian_command": PoseStamped,
                "hand_left_pose": PoseStamped,
                "hand_right_pose": PoseStamped,
            },
            recording_schema=schema,
            **({"recording": recording} if recording else {}),
        ),
        EpisodeMonitorModule.blueprint(instance_name="episodes", button_map={}),
    )
