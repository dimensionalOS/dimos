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

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass
import json
import time
from typing import BinaryIO, cast

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseArray import PoseArray
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.grasping_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.std_msgs.Header import Header
from dimos.types.timestamped import Timestamped


@dataclass(frozen=True, slots=True)
class GraspVisConfig:
    """Rerun wireframe defaults for parallel-jaw grasp candidates."""

    max_grasps: int = 50
    top_k_highlight: int = 5
    finger_length_m: float = 0.055
    palm_depth_m: float = 0.035
    finger_thickness_m: float = 0.004
    default_width_m: float = 0.08
    min_score: float = 0.0
    top_color: tuple[int, int, int] = (0, 255, 80)
    good_color: tuple[int, int, int] = (255, 220, 0)
    low_color: tuple[int, int, int] = (255, 80, 0)


class GraspCandidateArray(Timestamped):
    """An ordered array of scored grasp candidates."""

    msg_name = "grasping_msgs.GraspCandidateArray"

    def __init__(
        self,
        header: Header | None = None,
        candidates: list[GraspCandidate] | None = None,
    ) -> None:
        self.header = header if header is not None else _make_header(time.time(), "world")
        self.ts = self.header.timestamp
        self.candidates = candidates if candidates is not None else []

    @property
    def frame_id(self) -> str:
        return str(self.header.frame_id)

    def __len__(self) -> int:
        return len(self.candidates)

    def __iter__(self) -> Iterator[GraspCandidate]:
        return iter(self.candidates)

    def __getitem__(self, index: int) -> GraspCandidate:
        return self.candidates[index]

    def to_pose_array(self) -> PoseArray:
        """Return PoseArray compatibility view preserving candidate order."""
        return PoseArray(
            header=self.header, poses=[candidate.pose for candidate in self.candidates]
        )

    def lcm_encode(self) -> bytes:
        payload = {
            "header": {"ts": self.header.timestamp, "frame_id": self.header.frame_id},
            "candidates": [candidate.to_dict() for candidate in self.candidates],
        }
        return json.dumps(payload, separators=(",", ":")).encode("utf-8")

    encode = lcm_encode

    @classmethod
    def lcm_decode(cls, data: bytes | BinaryIO) -> GraspCandidateArray:
        raw = _read_bytes(data)
        payload = _as_dict(json.loads(raw.decode("utf-8")))
        header_payload = _as_dict(payload["header"])
        candidates_payload = _as_list(payload["candidates"])
        return cls(
            header=_make_header(_as_float(header_payload["ts"]), str(header_payload["frame_id"])),
            candidates=[GraspCandidate.from_dict(candidate) for candidate in candidates_payload],
        )

    decode = lcm_decode

    def to_rerun(self, config: GraspVisConfig | None = None) -> object:
        """Render candidates as simplified gripper wireframes."""
        import rerun as rr  # type: ignore[import-not-found]

        cfg = config if config is not None else GraspVisConfig()
        visible = [candidate for candidate in self.candidates if candidate.score >= cfg.min_score]
        visible.sort(key=lambda candidate: candidate.score, reverse=True)
        visible = visible[: cfg.max_grasps]
        if not visible:
            return rr.LineStrips3D([])

        strips: list[list[list[float]]] = []
        colors: list[tuple[int, int, int]] = []
        radii: list[float] = []
        for rank, candidate in enumerate(visible):
            color = (
                cfg.top_color if rank < cfg.top_k_highlight else _score_color(candidate.score, cfg)
            )
            width = candidate.jaw_width if candidate.jaw_width > 0.0 else cfg.default_width_m
            for strip in _candidate_wireframe(candidate.pose, width, cfg):
                strips.append(strip)
                colors.append(color)
                radii.append(cfg.finger_thickness_m)

        return rr.LineStrips3D(strips=strips, colors=colors, radii=radii)


def _candidate_wireframe(
    pose: Pose, width: float, config: GraspVisConfig
) -> list[list[list[float]]]:
    half_width = width / 2.0
    local_strips = [
        [Vector3(0.0, -half_width, 0.0), Vector3(config.finger_length_m, -half_width, 0.0)],
        [Vector3(0.0, half_width, 0.0), Vector3(config.finger_length_m, half_width, 0.0)],
        [Vector3(0.0, -half_width, 0.0), Vector3(0.0, half_width, 0.0)],
        [Vector3(-config.palm_depth_m, -half_width, 0.0), Vector3(0.0, -half_width, 0.0)],
        [Vector3(-config.palm_depth_m, half_width, 0.0), Vector3(0.0, half_width, 0.0)],
    ]
    return [[_transform_point(pose, point) for point in strip] for strip in local_strips]


def _transform_point(pose: Pose, point: Vector3) -> list[float]:
    rotated = pose.orientation.rotate_vector(point)
    world = pose.position + rotated
    return [world.x, world.y, world.z]


def _score_color(score: float, config: GraspVisConfig) -> tuple[int, int, int]:
    if score >= 0.5:
        return config.good_color
    return config.low_color


def _make_header(ts: float, frame_id: str) -> Header:
    header = Header(frame_id)
    sec = int(ts)
    header.stamp.sec = sec
    header.stamp.nsec = int((ts - sec) * 1_000_000_000)
    header.frame_id = frame_id
    return header


def _read_bytes(data: bytes | BinaryIO) -> bytes:
    if isinstance(data, bytes):
        return data
    return data.read()


def _as_dict(value: object) -> dict[str, object]:
    if not isinstance(value, dict):
        raise ValueError("Expected JSON object")
    return cast("dict[str, object]", value)


def _as_list(value: object) -> list[object]:
    if not isinstance(value, list):
        raise ValueError("Expected JSON list")
    return value


def _as_float(value: object) -> float:
    if not isinstance(value, int | float | str):
        raise ValueError(f"Expected a float-compatible value, got {type(value).__name__}")
    return float(value)
