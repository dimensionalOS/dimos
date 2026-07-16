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

"""Browser point-picking for case curation, served by viser.

Opens a dark-themed local web viewer with the final map and walked path.
Shift+click picks points in start/goal pairs. Every pair gets its own panel
entry with the coordinates, a name field, geometry-suggested tag checkboxes,
custom tags, and a negative toggle. Pairs save individually or all at once,
and stay editable after saving: rename or retag and press the pair's button
again to update the manifest. Plain clicks and drags only move the camera.
"""

from __future__ import annotations

import threading
from typing import TYPE_CHECKING

import numpy as np

from dimos.navigation.nav_3d.evaluator.generate import (
    LONG_STAIRS_DZ_M,
    LONG_STAIRS_WALKED_M,
    STAIRS_DZ_M,
)

if TYPE_CHECKING:
    from collections.abc import Callable

    from numpy.typing import NDArray
    import viser

    # (start, goal, negative, extra_tags, case_id) -> (ok, message, saved_id)
    SavePair = Callable[
        [tuple[float, float, float], tuple[float, float, float], bool, list[str], str | None],
        tuple[bool, str, str | None],
    ]
    # (saved_id, new_id, negative, extra_tags) -> (ok, message, saved_id)
    UpdateCase = Callable[[str, str, bool, list[str]], tuple[bool, str, str | None]]

# Selection cone half-angle around the click ray. Wide enough to hit a voxel
# point from across a room, narrow enough to stay on the intended surface.
PICK_CONE_RAD = 0.008
START_COLOR = (0, 255, 255)
GOAL_COLOR = (255, 140, 0)
PAIR_COLOR = (255, 255, 0)
SUGGESTED_TAGS = ("stairs", "flat", "up", "down", "long", "doorway")

INSTRUCTIONS = """**shift+click** picks START then GOAL, repeated per case.
Plain drag orbits, scroll zooms, right-drag pans.
"""


def pick_along_ray(
    points: NDArray[np.float32],
    origin: NDArray[np.float64],
    direction: NDArray[np.float64],
    cone_rad: float = PICK_CONE_RAD,
) -> NDArray[np.float32] | None:
    """Nearest cloud point inside a small cone around the click ray."""
    rel = points.astype(np.float64) - origin
    t = rel @ direction
    ahead = t > 0.05
    if not ahead.any():
        return None
    t = t[ahead]
    perp = np.linalg.norm(rel[ahead] - t[:, None] * direction, axis=1)
    angle = perp / t
    for widen in (1.0, 4.0):
        hit = angle < cone_rad * widen
        if hit.any():
            idx = np.flatnonzero(ahead)[hit]
            return np.asarray(points[idx[np.argmin(t[hit])]])
    return None


def suggested_tags(start: NDArray[np.float32], goal: NDArray[np.float32]) -> set[str]:
    """Geometry-derived tag suggestions, mirroring auto-generation's rules."""
    dz = float(goal[2] - start[2])
    euclid = float(np.linalg.norm(goal - start))
    tags: set[str] = set()
    if abs(dz) >= STAIRS_DZ_M:
        tags |= {"stairs", "up" if dz > 0 else "down"}
    else:
        tags.add("flat")
    if abs(dz) >= LONG_STAIRS_DZ_M or euclid >= LONG_STAIRS_WALKED_M:
        tags.add("long")
    return tags


class _PairEntry:
    """One picked start/goal pair and its editable panel widgets."""

    def __init__(
        self,
        server: viser.ViserServer,
        n: int,
        start: NDArray[np.float32],
        goal: NDArray[np.float32],
        save_pair: SavePair,
        update_case: UpdateCase,
        lock: threading.Lock,
    ) -> None:
        self._server = server
        self._n = n
        self.start = start
        self.goal = goal
        self._save_pair = save_pair
        self._update_case = update_case
        self._lock = lock
        self.saved_id: str | None = None
        self._name = ""
        self._checked = suggested_tags(start, goal)
        self._custom = ""
        self._negative = False
        self._status = "unsaved"
        self._build(expanded=True, order=None)

    def _build(self, *, expanded: bool, order: float | None) -> None:
        server = self._server
        start, goal = self.start, self.goal
        self.folder = server.gui.add_folder(
            f"pair {self._n}", order=order, expand_by_default=expanded
        )
        with self.folder:
            server.gui.add_markdown(
                f"({start[0]:.1f}, {start[1]:.1f}, {start[2]:.1f}) → "
                f"({goal[0]:.1f}, {goal[1]:.1f}, {goal[2]:.1f})"
            )
            self.id_text = server.gui.add_text(
                "name", initial_value=self._name, hint="empty = auto id"
            )
            with server.gui.add_folder("tags", expand_by_default=True):
                self.tag_boxes = {
                    tag: server.gui.add_checkbox(tag, tag in self._checked)
                    for tag in SUGGESTED_TAGS
                }
                self.custom_text = server.gui.add_text(
                    "custom", initial_value=self._custom, hint="comma-separated"
                )
            self.negative_box = server.gui.add_checkbox("negative (must refuse)", self._negative)
            self.message = server.gui.add_markdown(self._status)
            self.button = server.gui.add_button("save / update")

            @self.button.on_click
            def _(_event: object) -> None:
                # save_unsaved calls save_or_update already holding the lock;
                # the button path runs on a bare viser callback thread and must
                # take it to serialize suite/manifest mutation.
                with self._lock:
                    self.save_or_update()

    def remove(self) -> None:
        self.folder.remove()

    def _snapshot(self) -> None:
        self._name = self.id_text.value
        self._checked = {tag for tag, box in self.tag_boxes.items() if box.value}
        self._custom = self.custom_text.value
        self._negative = self.negative_box.value

    def extra_tags(self) -> list[str]:
        tags = [tag for tag, box in self.tag_boxes.items() if box.value]
        tags += [t.strip() for t in self.custom_text.value.split(",") if t.strip()]
        return tags

    def save_or_update(self) -> None:
        name = self.id_text.value.strip()
        if self.saved_id is None:
            ok, msg, saved = self._save_pair(
                (float(self.start[0]), float(self.start[1]), float(self.start[2])),
                (float(self.goal[0]), float(self.goal[1]), float(self.goal[2])),
                self.negative_box.value,
                self.extra_tags(),
                name or None,
            )
        else:
            ok, msg, saved = self._update_case(
                self.saved_id, name or self.saved_id, self.negative_box.value, self.extra_tags()
            )
        print(msg)
        if not (ok and saved is not None):
            self.message.content = f"**FAILED**: {msg}"
            return
        # Folders cannot be collapsed live in viser, expand_by_default is
        # only read when the folder is first created. Rebuild it collapsed
        # in place instead.
        self.saved_id = saved
        self._snapshot()
        self._name = saved
        self._status = msg
        order = self.folder.order
        self.folder.remove()
        self._build(expanded=False, order=order)


def pick_cases(
    dataset: str,
    map_points: NDArray[np.float32],
    map_colors: NDArray[np.uint8],
    walked: NDArray[np.float32],
    save_pair: SavePair,
    update_case: UpdateCase,
) -> None:
    """Serve the picker until the user exits from the panel or hits ctrl-c."""
    import viser

    server = viser.ViserServer(label=f"Pair Picker - {dataset}", verbose=False)
    server.gui.configure_theme(dark_mode=True)
    server.scene.set_background_image(np.full((1, 1, 3), 14, dtype=np.uint8))
    server.scene.set_up_direction("+z")
    server.scene.add_point_cloud(
        "/map",
        map_points,
        map_colors,
        point_size=0.035,
        point_shape="circle",
        precision="float32",
    )
    if len(walked) >= 2:
        segments = np.stack([walked[:-1], walked[1:]], axis=1)
        server.scene.add_line_segments(
            "/walked_path", segments, colors=(255, 255, 255), line_width=2.0
        )

    center = map_points.mean(axis=0)
    span = float(np.ptp(map_points[:, :2]))

    @server.on_client_connect
    def _(client: viser.ClientHandle) -> None:
        client.camera.position = tuple(center + np.array([0.6 * span, 0.6 * span, 0.45 * span]))
        client.camera.look_at = tuple(center)

    server.gui.add_markdown(INSTRUCTIONS)
    undo_button = server.gui.add_button("undo last pick")
    save_all_button = server.gui.add_button("save all unsaved")
    exit_button = server.gui.add_button("save all & exit")

    lock = threading.Lock()
    stop = threading.Event()
    picks: list[NDArray[np.float32]] = []
    pairs: list[_PairEntry] = []
    markers: list[viser.SceneNodeHandle] = []
    pair_count = 0

    @server.scene.on_click(modifier="shift")
    def _(event: viser.SceneClickEvent) -> None:
        nonlocal pair_count
        point = pick_along_ray(
            map_points, np.asarray(event.ray_origin), np.asarray(event.ray_direction)
        )
        if point is None:
            return
        with lock:
            is_goal = len(picks) % 2 == 1
            picks.append(point)
            n = len(picks)
            markers.append(
                server.scene.add_icosphere(
                    f"/picks/p{n}",
                    radius=0.09,
                    color=GOAL_COLOR if is_goal else START_COLOR,
                    position=(float(point[0]), float(point[1]), float(point[2]) + 0.05),
                )
            )
            if is_goal:
                start, goal = picks[-2], picks[-1]
                markers.append(
                    server.scene.add_line_segments(
                        f"/picks/l{n}",
                        np.stack([start, goal])[None],
                        colors=PAIR_COLOR,
                        line_width=2.5,
                    )
                )
                pair_count += 1
                pairs.append(
                    _PairEntry(server, pair_count, start, goal, save_pair, update_case, lock)
                )

    @undo_button.on_click
    def _(_event: object) -> None:
        with lock:
            if not picks:
                return
            if len(picks) % 2 == 0:
                # Completing pick of the last pair. Saved cases stay in the
                # manifest, only the panel entry and markers go away.
                pair = pairs.pop()
                pair.remove()
                markers.pop().remove()  # pair line
            picks.pop()
            markers.pop().remove()

    def save_unsaved() -> None:
        with lock:
            for pair in pairs:
                if pair.saved_id is None:
                    pair.save_or_update()

    @save_all_button.on_click
    def _(_event: object) -> None:
        save_unsaved()

    @exit_button.on_click
    def _(_event: object) -> None:
        save_unsaved()
        stop.set()

    print("picker running; ctrl-c to exit (unsaved pairs are discarded)")
    try:
        stop.wait()
    except KeyboardInterrupt:
        unsaved = sum(1 for p in pairs if p.saved_id is None)
        if unsaved:
            print(f"discarded {unsaved} unsaved pair(s)")
    finally:
        server.stop()
