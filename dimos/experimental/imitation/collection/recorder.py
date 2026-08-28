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

"""Experimental native recording profile for imitation data collection."""

from __future__ import annotations

from pydantic import Field

from dimos.core.stream import In
from dimos.experimental.memory.rust_recorder import (
    RustRecorder,
    RustRecorderConfig,
    RustRecordingStoreConfig,
    RustSqliteStoreConfig,
)
from dimos.msgs.imitation_msgs.EpisodeStatus import EpisodeStatus
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState


class NativeCollectionRecorderConfig(RustRecorderConfig):
    """Native recorder configuration for offline DataPrep."""

    store: RustRecordingStoreConfig = Field(default_factory=RustSqliteStoreConfig)
    record_tf: bool = Field(default=False, exclude=True)


class NativeCollectionRecorder(RustRecorder):
    """Declare the streams captured by the native collection recorder."""

    config: NativeCollectionRecorderConfig

    color_image: In[Image]
    coordinator_joint_state: In[JointState]
    applied_joint_position_command: In[JointState]
    status: In[EpisodeStatus]
