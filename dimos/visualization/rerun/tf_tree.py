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

from collections.abc import Iterable

from dimos_generated.geometry_msgs.msg import TransformStamped
import rerun as rr

DEFAULT_LINKS_ROOT = "tf_links"
DEFAULT_TF_ROOT = "world/tf"
DEFAULT_AXIS_LENGTH = 0.5
DEPTH_SCALE = 0.8
AXIS_WIDTH_UI_POINTS = 2.0
AXIS_COLORS = [[255, 0, 0], [0, 255, 0], [0, 0, 255]]


def _triad(length: float) -> rr.Arrows3D:
    """XYZ arrows, red green blue."""
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

    def update(self, transforms: Iterable[TransformStamped]) -> None:
        learned = False
        for transform in transforms:
            if self._parents.get(transform.child_frame_id) != transform.header.frame_id:
                self._parents[transform.child_frame_id] = transform.header.frame_id
                learned = True
        if learned:
            self._redraw()

    def _place(self, frame: str, walked: frozenset[str]) -> tuple[str, int]:
        part = rr.escape_entity_path_part(frame)
        parent = self._parents.get(frame)
        if parent is None or parent in walked:
            return f"{self.root}/{part}", 0
        parent_path, parent_depth = self._place(parent, walked | {frame})
        return f"{parent_path}/{part}", parent_depth + 1

    def _redraw(self) -> None:
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
