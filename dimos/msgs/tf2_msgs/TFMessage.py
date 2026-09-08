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

from typing import TYPE_CHECKING, BinaryIO

from dimos_lcm.tf2_msgs import TFMessage as LCMTFMessage

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3

if TYPE_CHECKING:
    from collections.abc import Iterable, Iterator

    from dimos.visualization.rerun.bridge import RerunMulti

DEFAULT_LINKS_ROOT = "tf_links"
DEFAULT_TF_ROOT = "world/tf"
DEFAULT_AXIS_LENGTH = 0.5
DEPTH_SCALE = 0.8
AXIS_WIDTH_UI_POINTS = 2.0
AXIS_COLORS = [[255, 0, 0], [0, 255, 0], [0, 0, 255]]


def _triad(length: float):  # type: ignore[no-untyped-def]
    """XYZ arrows, red green blue."""
    import rerun as rr

    return rr.Arrows3D(
        origins=[[0.0, 0.0, 0.0]] * 3,
        vectors=[[length, 0.0, 0.0], [0.0, length, 0.0], [0.0, 0.0, length]],
        colors=AXIS_COLORS,
        radii=rr.components.Radius.ui_points(AXIS_WIDTH_UI_POINTS),
    )


class TfFrameTree:
    """Store each frame under its parents. This lets us view the tree in the left panel."""

    def __init__(
        self, axis_length: float = DEFAULT_AXIS_LENGTH, root: str = DEFAULT_TF_ROOT
    ) -> None:
        self.axis_length = axis_length
        self.root = root
        self._parents: dict[str, str] = {}
        self._placed: dict[str, tuple[str, int]] = {}  # frame -> (path, depth)

    def placements(self) -> dict[str, str]:
        return {frame: path for frame, (path, _depth) in self._placed.items()}

    def update(self, transforms: Iterable[Transform]) -> None:
        learned = False
        for transform in transforms:
            if self._parents.get(transform.child_frame_id) != transform.frame_id:
                self._parents[transform.child_frame_id] = transform.frame_id
                learned = True
        if learned:
            self._redraw()

    def _place(self, frame: str, walked: frozenset[str]) -> tuple[str, int]:
        import rerun as rr

        part = rr.escape_entity_path_part(frame)
        parent = self._parents.get(frame)
        if parent is None or parent in walked:
            return f"{self.root}/{part}", 0
        parent_path, parent_depth = self._place(parent, walked | {frame})
        return f"{parent_path}/{part}", parent_depth + 1

    def _redraw(self) -> None:
        import rerun as rr

        frames = {*self._parents, *self._parents.values()}
        placed = {frame: self._place(frame, frozenset()) for frame in frames}

        for frame, (old_path, _depth) in self._placed.items():
            new = placed.get(frame)
            if new is None or new[0] != old_path:
                rr.log(old_path, rr.Arrows3D(origins=[], vectors=[]), static=True)

        for frame, (path, depth) in placed.items():
            if self._placed.get(frame) != (path, depth):
                rr.log(
                    path,
                    rr.CoordinateFrame(f"tf#/{frame}"),
                    _triad(self.axis_length * DEPTH_SCALE**depth),
                    static=True,
                )

        self._placed = placed


class TFMessage:
    """TFMessage that accepts Transform objects and encodes to LCM format."""

    transforms: list[Transform]
    msg_name = "tf2_msgs.TFMessage"

    def __init__(self, *transforms: Transform) -> None:
        self.transforms = list(transforms)

    def add_transform(self, transform: Transform, child_frame_id: str = "base_link") -> None:
        """Add a transform to the message."""
        self.transforms.append(transform)
        self.transforms_length = len(self.transforms)

    def lcm_encode(self) -> bytes:
        """Encode as LCM TFMessage."""
        res = [t.lcm_transform() for t in self.transforms]

        lcm_msg = LCMTFMessage(
            transforms_length=len(self.transforms),
            transforms=res,
        )

        return lcm_msg.lcm_encode()  # type: ignore[no-any-return]

    @classmethod
    def lcm_decode(cls, data: bytes | BinaryIO) -> TFMessage:
        """Decode from LCM TFMessage bytes."""
        lcm_msg = LCMTFMessage.lcm_decode(data)

        transforms = []
        for lcm_transform_stamped in lcm_msg.transforms:
            ts = lcm_transform_stamped.header.stamp.sec + (
                lcm_transform_stamped.header.stamp.nsec / 1_000_000_000
            )

            lcm_trans = lcm_transform_stamped.transform.translation
            lcm_rot = lcm_transform_stamped.transform.rotation

            transform = Transform(
                translation=Vector3(lcm_trans.x, lcm_trans.y, lcm_trans.z),
                rotation=Quaternion(lcm_rot.x, lcm_rot.y, lcm_rot.z, lcm_rot.w),
                frame_id=lcm_transform_stamped.header.frame_id,
                child_frame_id=lcm_transform_stamped.child_frame_id,
                ts=ts,
            )
            transforms.append(transform)

        return cls(*transforms)

    def __len__(self) -> int:
        """Return number of transforms."""
        return len(self.transforms)

    def __getitem__(self, index: int) -> Transform:
        """Get transform by index."""
        return self.transforms[index]

    def __iter__(self) -> Iterator:  # type: ignore[type-arg]
        """Iterate over transforms."""
        return iter(self.transforms)

    def __repr__(self) -> str:
        return f"TFMessage({len(self.transforms)} transforms)"

    def __str__(self) -> str:
        lines = [f"TFMessage with {len(self.transforms)} transforms:"]
        for i, transform in enumerate(self.transforms):
            lines.append(
                f"  [{i}] {transform.frame_id} → {transform.child_frame_id} @ {transform.ts:.3f}"
            )
        return "\n".join(lines)

    def to_rerun(self, tree: TfFrameTree | None = None) -> RerunMulti:
        """Convert to (entity_path, archetype) pairs to log to rerun.

        Pass a TfFrameTree to also nest a triad per frame under its ancestors,
        matching the tf tree's shape in the entity panel.
        """
        import rerun as rr

        results: RerunMulti = []
        for transform in self.transforms:
            path = f"{DEFAULT_LINKS_ROOT}/{rr.escape_entity_path_part(transform.child_frame_id)}"
            results.append((path, transform.to_rerun()))

        if tree is not None:
            tree.update(self.transforms)
        return results
