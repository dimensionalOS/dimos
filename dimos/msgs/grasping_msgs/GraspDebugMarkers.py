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

"""Debug Rerun markers for agentic grasp execution."""

from __future__ import annotations

import json
import time
from types import ModuleType
from typing import BinaryIO, cast

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.types.timestamped import Timestamped


class GraspDebugMarkers(Timestamped):
    """Rerun-only grasp debug overlay encoded as a lightweight JSON message."""

    msg_name = "grasping_msgs.GraspDebugMarkers"

    def __init__(
        self,
        *,
        frame_id: str = "world",
        ts: float = 0.0,
        bbox_center: Vector3 | None = None,
        bbox_size: Vector3 | None = None,
        candidate_poses: list[Pose] | None = None,
        selected_candidate_index: int | None = None,
        pregrasp_pose: Pose | None = None,
        final_pose: Pose | None = None,
        label: str = "agentic grasp debug",
    ) -> None:
        self.frame_id = frame_id
        self.ts = ts if ts != 0.0 else time.time()
        self.bbox_center = bbox_center
        self.bbox_size = bbox_size
        self.candidate_poses = candidate_poses if candidate_poses is not None else []
        self.selected_candidate_index = selected_candidate_index
        self.pregrasp_pose = pregrasp_pose
        self.final_pose = final_pose
        self.label = label

    def lcm_encode(self) -> bytes:
        payload = {
            "frame_id": self.frame_id,
            "ts": self.ts,
            "bbox_center": _vector_to_list(self.bbox_center),
            "bbox_size": _vector_to_list(self.bbox_size),
            "candidate_poses": [_pose_to_payload(pose) for pose in self.candidate_poses],
            "selected_candidate_index": self.selected_candidate_index,
            "pregrasp_pose": _pose_to_payload(self.pregrasp_pose),
            "final_pose": _pose_to_payload(self.final_pose),
            "label": self.label,
        }
        return json.dumps(payload, separators=(",", ":")).encode("utf-8")

    encode = lcm_encode

    @classmethod
    def lcm_decode(cls, data: bytes | BinaryIO) -> GraspDebugMarkers:
        raw = data if isinstance(data, bytes) else data.read()
        payload = cast("dict[str, object]", json.loads(raw.decode("utf-8")))
        return cls(
            frame_id=str(payload.get("frame_id", "world")),
            ts=_float_from_payload(payload.get("ts", 0.0)),
            bbox_center=_vector_from_payload(payload.get("bbox_center")),
            bbox_size=_vector_from_payload(payload.get("bbox_size")),
            candidate_poses=[
                _pose_from_payload(item)
                for item in cast("list[object]", payload["candidate_poses"])
            ],
            selected_candidate_index=_optional_int(payload.get("selected_candidate_index")),
            pregrasp_pose=_pose_from_payload(payload.get("pregrasp_pose")),
            final_pose=_pose_from_payload(payload.get("final_pose")),
            label=str(payload.get("label", "agentic grasp debug")),
        )

    decode = lcm_decode

    def to_rerun(self) -> list[tuple[str, object]]:
        """Render bbox, candidates, selected target, and pregrasp/final path."""

        import rerun as rr

        base = "world/debug/grasp"
        entries: list[tuple[str, object]] = []
        if self.bbox_center is not None and self.bbox_size is not None:
            entries.append(
                (
                    f"{base}/target_bbox",
                    rr.Boxes3D(
                        centers=[_vector_to_list(self.bbox_center)],
                        half_sizes=[_vector_to_list(self.bbox_size * 0.5)],
                        colors=[[0, 180, 255, 80]],
                        labels=[self.label],
                        show_labels=True,
                        fill_mode="majorwireframe",
                    ),
                )
            )
        if self.candidate_poses:
            positions = [_vector_to_list(pose.position) for pose in self.candidate_poses]
            labels = [f"gpd[{index}]" for index in range(len(self.candidate_poses))]
            colors = [
                [0, 255, 80] if index == self.selected_candidate_index else [255, 220, 0]
                for index in range(len(self.candidate_poses))
            ]
            entries.append(
                (
                    f"{base}/candidate_points",
                    rr.Points3D(
                        positions=positions,
                        colors=colors,
                        radii=[0.012] * len(positions),
                        labels=labels,
                        show_labels=True,
                    ),
                )
            )
        if self.pregrasp_pose is not None and self.final_pose is not None:
            entries.append(
                (
                    f"{base}/pregrasp_to_final",
                    rr.LineStrips3D(
                        strips=[
                            [
                                _vector_to_list(self.pregrasp_pose.position),
                                _vector_to_list(self.final_pose.position),
                            ]
                        ],
                        colors=[[0, 255, 255]],
                        radii=[0.006],
                        labels=["pregrasp → final"],
                        show_labels=True,
                    ),
                )
            )
        for name, pose, length in (
            ("pregrasp_pose", self.pregrasp_pose, 0.055),
            ("final_grasp_pose", self.final_pose, 0.070),
        ):
            if pose is not None:
                entries.append((f"{base}/{name}", _pose_axes(rr, pose, length)))
        return entries


def _pose_axes(rr: ModuleType, pose: Pose, length: float) -> object:
    x_axis = pose.orientation.rotate_vector(Vector3(length, 0.0, 0.0))
    y_axis = pose.orientation.rotate_vector(Vector3(0.0, length, 0.0))
    z_axis = pose.orientation.rotate_vector(Vector3(0.0, 0.0, length))
    origin = _vector_to_list(pose.position)
    return rr.Arrows3D(
        origins=[origin, origin, origin],
        vectors=[_vector_to_list(x_axis), _vector_to_list(y_axis), _vector_to_list(z_axis)],
        colors=[[255, 60, 60], [60, 255, 60], [80, 140, 255]],
        labels=["+X", "+Y", "+Z"],
        show_labels=True,
    )


def _pose_to_payload(pose: Pose | None) -> dict[str, list[float]] | None:
    if pose is None:
        return None
    return {
        "position": _vector_to_xyz(pose.position),
        "orientation": [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ],
    }


def _pose_from_payload(value: object) -> Pose | None:
    if value is None:
        return None
    payload = cast("dict[str, object]", value)
    position = _vector_from_payload(payload["position"])
    orientation = _quat_from_payload(payload["orientation"])
    if position is None or orientation is None:
        return None
    return Pose((position, orientation))


def _vector_to_list(vector: Vector3 | None) -> list[float] | None:
    if vector is None:
        return None
    return [float(vector.x), float(vector.y), float(vector.z)]


def _vector_to_xyz(vector: Vector3) -> list[float]:
    return [float(vector.x), float(vector.y), float(vector.z)]


def _vector_from_payload(value: object) -> Vector3 | None:
    if value is None:
        return None
    items = cast("list[float]", value)
    return Vector3(float(items[0]), float(items[1]), float(items[2]))


def _quat_from_payload(value: object) -> Quaternion | None:
    if value is None:
        return None
    items = cast("list[float]", value)
    return Quaternion([float(items[0]), float(items[1]), float(items[2]), float(items[3])])


def _optional_int(value: object) -> int | None:
    if value is None:
        return None
    return int(cast("int | float", value))


def _float_from_payload(value: object) -> float:
    return float(cast("int | float | str", value))
