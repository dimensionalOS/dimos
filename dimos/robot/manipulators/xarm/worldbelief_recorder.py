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

"""Memory2 recorder for the xArm6 WorldBelief hardware blueprint."""

from __future__ import annotations

from datetime import datetime
import inspect
from pathlib import Path
from typing import Any

from dimos.constants import STATE_DIR
from dimos.core.core import rpc
from dimos.core.stream import In
from dimos.memory2.module import Recorder, RecorderConfig
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.utils.logging_config import setup_logger

_STATE_DIR = STATE_DIR / "worldbelief" / "xarm6"
_HISTORY_PATH = _STATE_DIR / "worldbelief_history.db"
_RECORDING_BASE_PATH = _STATE_DIR / "recordings" / "xarm6_worldbelief.db"

logger = setup_logger()


def xarm6_worldbelief_history_path() -> str:
    return str(_HISTORY_PATH)


def _timestamped_recording_path(base: str | Path, now: datetime | None = None) -> Path:
    base_path = Path(base)
    timestamp = (now or datetime.now()).strftime("%Y%m%d_%H%M%S_%f")
    candidate = base_path.with_name(f"{base_path.stem}_{timestamp}{base_path.suffix}")
    suffix = 1
    while candidate.exists():
        candidate = base_path.with_name(f"{base_path.stem}_{timestamp}_{suffix}{base_path.suffix}")
        suffix += 1
    return candidate


class XArm6WorldBeliefRecorderConfig(RecorderConfig):
    db_path: str | Path = _RECORDING_BASE_PATH
    default_frame_id: str = "world"
    tf_tolerance: float = 0.5
    record_tf: bool = False


class XArm6WorldBeliefRecorder(Recorder):
    """Record replay-critical xArm6 WorldBelief streams for offline QA."""

    color_image: In[Image]
    depth_image: In[Image]
    camera_info: In[CameraInfo]
    depth_camera_info: In[CameraInfo]
    annotated_image: In[Image]
    detections_2d: In[Detection2DArray]
    detections_3d: In[Detection3DArray]
    pointcloud: In[PointCloud2]
    worldbelief_audit: In[dict]
    config: XArm6WorldBeliefRecorderConfig

    @rpc
    def start(self) -> None:
        if not self.config.g.replay:
            db_path = _timestamped_recording_path(self.config.db_path)
            db_path.parent.mkdir(parents=True, exist_ok=True)
            self.config.db_path = db_path
            logger.info("xArm6 WorldBelief recording DB: %s", db_path)
        super().start()

    @staticmethod
    def _stream_kwargs(name: str) -> dict[str, Any]:
        return {"codec": "lcm"} if name == "depth_image" else {}

    async def _resolve_pose(self, name: str, msg: Any, ts: float) -> Any:
        frame_id = getattr(msg, "frame_id", None) or self.config.default_frame_id
        if frame_id == "world":
            return Transform.identity().to_pose()
        pose = super()._resolve_pose(name, msg, ts)
        return await pose if inspect.isawaitable(pose) else pose
