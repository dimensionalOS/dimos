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

"""URDF/MJCF frame loading.

The model file is the ground truth for a robot's frames. `UrdfLoader` parses it
lazily and derives the frame information dimos needs (the structural body frame
and the fixed-joint static transforms) without pulling in any control,
manipulation, or hardware machinery.
"""

from __future__ import annotations

from functools import cached_property
from pathlib import Path

from pydantic import BaseModel, Field, PrivateAttr

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.model_parser import JointDescription, ModelDescription, parse_model


class UrdfLoader(BaseModel):
    """Lazily parses a URDF/MJCF model and exposes its derived frame info.

    Parsing is deferred until first access so importing a config that references
    a model never triggers an LFS download or disk read.
    """

    name: str
    model_path: Path | None = None
    package_paths: dict[str, Path] = Field(default_factory=dict)
    xacro_args: dict[str, str] = Field(default_factory=dict)

    _parsed: ModelDescription | None = PrivateAttr(default=None)

    def _ensure_parsed(self) -> ModelDescription:
        if self._parsed is None:
            if self.model_path is None:
                raise ValueError(
                    f"UrdfLoader {self.name!r} has no model_path — frame info is unavailable. "
                    "Set model_path to a URDF/MJCF."
                )
            self._parsed = parse_model(self.model_path, self.package_paths, self.xacro_args)
        return self._parsed

    @property
    def model_description(self) -> ModelDescription:
        return self._ensure_parsed()

    @cached_property
    def all_frame_ids(self) -> list[str]:
        return list(self._ensure_parsed().links)

    @cached_property
    def body_frame(self) -> str:
        """The robot's structural root link (usually ``base_link``).

        Skips past ``type="floating"`` joints and returns the first structural
        link. (``world`` can be the true root frame, but it is detached from the
        robot, so it is not the body frame.)
        """
        model = self._ensure_parsed()
        outgoing: dict[str, list[JointDescription]] = {}
        for joint in model.joints:
            outgoing.setdefault(joint.parent_link, []).append(joint)
        current = model.root_link
        while True:
            floating_joint = next(
                (joint for joint in outgoing.get(current, []) if joint.type == "floating"),
                None,
            )
            if floating_joint is None:
                return current
            current = floating_joint.child_link

    @cached_property
    def static_transforms(self) -> dict[str, Transform]:
        """Fixed-joint transforms keyed by child frame (parent → child).

        Example::
            print(UrdfLoader(name="go2", model_path="go2.urdf").static_transforms)
            # {
            #     "camera_link": Transform(translation=(0.3, 0, 0), rotation=identity,
            #                              frame_id="base_link", child_frame_id="camera_link"),
            #     "camera_optical": Transform(translation=(0, 0, 0), rotation=(-0.5, 0.5, -0.5, 0.5),
            #                                 frame_id="camera_link", child_frame_id="camera_optical"),
            # }
        """
        result: dict[str, Transform] = {}
        for joint in self._ensure_parsed().joints:
            if joint.type != "fixed" or not joint.child_link:
                continue
            roll, pitch, yaw = joint.origin_rpy
            tx, ty, tz = joint.origin_xyz
            result[joint.child_link] = Transform(
                translation=Vector3(tx, ty, tz),
                rotation=Quaternion.from_euler(Vector3(roll, pitch, yaw)),
                frame_id=joint.parent_link,
                child_frame_id=joint.child_link,
            )
        return result
