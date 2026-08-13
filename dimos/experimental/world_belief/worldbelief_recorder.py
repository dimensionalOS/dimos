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

from __future__ import annotations

from datetime import datetime
from pathlib import Path
from typing import Any, Protocol

from dimos.constants import STATE_DIR
from dimos.core.core import rpc
from dimos.core.stream import In
from dimos.memory2.module import Recorder, RecorderConfig, pose_setter_for
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.spec.utils import Spec
from dimos.utils.logging_config import setup_logger

_STATE_DIR = STATE_DIR / "worldbelief"
_RECORDING_BASE_PATH = _STATE_DIR / "recordings" / "worldbelief.db"

logger = setup_logger()


class WorldBeliefRecorderSpec(Spec, Protocol):
    """RPC boundary exposed to the WorldBelief query module."""

    @rpc
    def recording_path(self) -> str: ...


def _timestamped_recording_path(base: str | Path) -> Path:
    base_path = Path(base)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    candidate = base_path.with_name(f"{base_path.stem}_{timestamp}{base_path.suffix}")
    suffix = 1
    while candidate.exists():
        candidate = base_path.with_name(f"{base_path.stem}_{timestamp}_{suffix}{base_path.suffix}")
        suffix += 1
    return candidate


class WorldBeliefRecorderConfig(RecorderConfig):
    db_path: str | Path = _RECORDING_BASE_PATH
    default_frame_id: str = "world"


class WorldBeliefRecorder(Recorder):
    """Record camera, TF, and joint-state evidence."""

    color_image: In[Image]
    depth_image: In[Image]
    camera_info: In[CameraInfo]
    depth_camera_info: In[CameraInfo]
    coordinator_joint_state: In[JointState]
    config: WorldBeliefRecorderConfig

    @rpc
    def start(self) -> None:
        if not self.config.g.replay:
            db_path = _timestamped_recording_path(self.config.db_path)
            db_path.parent.mkdir(parents=True, exist_ok=True)
            self.config.db_path = db_path
            logger.info("WorldBelief recording DB: %s", db_path)
        super().start()

    @rpc
    def recording_path(self) -> str:
        """Return the active recording path."""
        return str(self.config.db_path)

    def _prepare_streams(self) -> None:
        super()._prepare_streams()
        depth = self.config.stream_remapping.get("depth_image", "depth_image")
        self.store.stream(depth, Image, codec="lz4+lcm")

    @pose_setter_for("coordinator_joint_state")
    async def _proprio_pose(self, msg: Any) -> Any:
        """Use an identity pose for proprioceptive joint-state records."""
        return Transform.identity().to_pose()
