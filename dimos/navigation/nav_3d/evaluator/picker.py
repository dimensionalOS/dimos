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

Serves the final map, the walked path, and every existing case as an editable
panel entry. Shift+click picks new start/goal pairs. Each entry exposes the
coordinates, a name field, tag checkboxes, a negative toggle, and save/delete
buttons, so any case can be renamed, retagged, flipped, or removed.
"""

from __future__ import annotations

from dataclasses import dataclass
import threading
from typing import TYPE_CHECKING

import numpy as np

from dimos.navigation.nav_3d.evaluator.tagging import (
    LONG_STAIRS_DZ_M,
    LONG_STAIRS_WALKED_M,
    STAIRS_DZ_M,
)

if TYPE_CHECKING:
    from collections.abc import Callable, Sequence

    from numpy.typing import NDArray
    import viser

    from dimos.navigation.nav_3d.evaluator.cases import Case

    # (start, goal, negative, tags, case_id) -> (ok, message, saved_id, saved_tags)
    SavePair = Callable[
        [tuple[float, float, float], tuple[float, float, float], bool, list[str], str | None],
        tuple[bool, str, str | None, list[str] | None],
    ]
    # (saved_id, new_id, negative, tags) -> (ok, message, saved_id, saved_tags)
    UpdateCase = Callable[
        [str, str, bool, list[str]], tuple[bool, str, str | None, list[str] | None]
    ]
    # (saved_id) -> (ok, message)
    DeleteCase = Callable[[str], tuple[bool, str]]

# Selection cone half-angle around the click ray. Wide enough to hit a voxel
# point from across a room, narrow enough to stay on the intended surface.
PICK_CONE_RAD = 0.008
START_COLOR = (0, 255, 255)
GOAL_COLOR = (255, 140, 0)
PAIR_COLOR = (255, 255, 0)
HIGHLIGHT_LINE_COLOR = (255, 255, 255)
MARKER_RADIUS = 0.09
HIGHLIGHT_MARKER_RADIUS = 0.16
LINE_WIDTH = 2.5
HIGHLIGHT_LINE_WIDTH = 6.0
SUGGESTED_TAGS = ("stairs", "flat", "up", "down", "long", "doorway")

INSTRUCTIONS = """**shift+click** picks START then GOAL, repeated per case.
**click** an endpoint sphere to highlight and open its case.
Plain drag orbits, scroll zooms, right-drag pans.
"""

# three.js ACES filmic tone mapping, the fitted curve and its color
# matrices. Applied by the viewer to mesh materials but not to the point
# and line shaders.
_ACES_INPUT = np.array(
    [
        [0.59719, 0.35458, 0.04823],
        [0.07600, 0.90834, 0.13383],
        [0.02840, 0.01566, 0.83777],
    ]
)
_ACES_OUTPUT = np.array(
    [
        [1.60475, -0.53108, -0.07367],
        [-0.10208, 1.10813, -0.00605],
        [-0.00327, -0.07276, 1.07602],
    ]
)
# White scene lights, bright enough that inverse-tone-mapped albedos fit
# in [0, 1]. LIGHT_REFERENCE is the ambient plus directional total on a
# typical face. Faces above or below it shade brighter or darker.
_AMBIENT_INTENSITY = 3.5
_DIRECTIONAL_INTENSITY = 2.0
_LIGHT_REFERENCE = 4.6


def _prelit_albedo(srgb: NDArray[np.uint8]) -> NDArray[np.float64]:
    """Linear albedo that tone-maps back to the wanted sRGB color when lit.

    Voxel cubes are lit meshes, so the viewer runs them through ACES tone
    mapping and would desaturate the height colormap. Feeding the inverse
    curve through the material albedo cancels that out at the reference
    light level.
    """
    c = srgb.astype(np.float64) / 255.0
    lin = np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)
    t = np.clip(lin @ np.linalg.inv(_ACES_OUTPUT).T, 0.0, 0.99)
    a2 = 1.0 - 0.983729 * t
    a1 = 0.0245786 - 0.4329510 * t
    a0 = -0.000090537 - 0.238081 * t
    v = (-a1 + np.sqrt(a1 * a1 - 4.0 * a2 * a0)) / (2.0 * a2)
    x = 0.6 * (v @ np.linalg.inv(_ACES_INPUT).T)
    return np.clip(x / _LIGHT_REFERENCE, 0.0, 1.0)


def _cube_colors(srgb: NDArray[np.uint8]) -> NDArray[np.uint8]:
    """Per-instance cube colors. The viewer reads these as linear RGB."""
    return np.asarray((_prelit_albedo(srgb) * 255.0).round(), dtype=np.uint8)


def _marker_color(srgb: tuple[int, int, int]) -> tuple[int, int, int]:
    """Marker mesh color. The viewer reads this as sRGB."""
    albedo = _prelit_albedo(np.array(srgb, dtype=np.uint8))
    out = np.where(albedo <= 0.0031308, albedo * 12.92, 1.055 * albedo ** (1 / 2.4) - 0.055)
    r, g, b = (out * 255.0).round().astype(int)
    return int(r), int(g), int(b)


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


@dataclass
class _Hooks:
    """Manifest callbacks and shared state handed to every pair entry."""

    save_pair: SavePair
    update_case: UpdateCase
    delete_case: DeleteCase
    lock: threading.Lock
    unregister: Callable[[_PairEntry], None]
    announce: Callable[[str], None]
    highlight: Callable[[_PairEntry], None]


class _PairEntry:
    """One start/goal pair and its editable panel widgets and scene markers."""

    def __init__(
        self,
        server: viser.ViserServer,
        n: int,
        start: NDArray[np.float32],
        goal: NDArray[np.float32],
        hooks: _Hooks,
        markers: list[viser.SceneNodeHandle],
        case: Case | None = None,
    ) -> None:
        self._server = server
        self._n = n
        self.start = start
        self.goal = goal
        self._hooks = hooks
        self.markers = markers
        self.preloaded = case is not None
        if case is None:
            self.saved_id: str | None = None
            self._name = ""
            self._checked = suggested_tags(start, goal)
            self._custom = ""
            self._negative = False
            self._status = "unsaved"
        else:
            self.saved_id = case.id
            self._name = case.id
            self._sync_tags(case.tags)
            self._negative = case.expect_fail
            self._status = "in manifest"
        self.removed = False
        self._build(expanded=case is None, order=None)
        for marker in markers:
            if hasattr(marker, "on_click"):
                marker.on_click(self._on_marker_click)

    def _on_marker_click(self, _event: object) -> None:
        with self._hooks.lock:
            self.reveal()

    def reveal(self) -> None:
        """Announce and highlight this pair, and open its panel entry."""
        self._hooks.announce(self._label())
        self._hooks.highlight(self)
        self._snapshot()
        order = self.panel.order
        self.panel.remove()
        self._build(expanded=True, order=order, scroll=True)

    def set_highlight(self, on: bool) -> None:
        if self.removed:
            return
        for marker in self.markers:
            if hasattr(marker, "radius"):
                marker.radius = HIGHLIGHT_MARKER_RADIUS if on else MARKER_RADIUS
            elif hasattr(marker, "line_width"):
                marker.line_width = HIGHLIGHT_LINE_WIDTH if on else LINE_WIDTH
                marker.colors = np.array(HIGHLIGHT_LINE_COLOR if on else PAIR_COLOR, dtype=np.uint8)

    def _label(self) -> str:
        return self.saved_id or f"pair {self._n}"

    def _sync_tags(self, tags: list[str]) -> None:
        """Split a manifest tag list into checkbox and custom-text state.

        The negative tag is owned by the checkbox. Everything not in the
        suggested set (auto, manual, ...) lands in the custom text so it
        stays visible and round-trips verbatim.
        """
        self._checked = {t for t in tags if t in SUGGESTED_TAGS}
        self._custom = ", ".join(t for t in tags if t not in SUGGESTED_TAGS and t != "negative")

    def _build(self, *, expanded: bool, order: float | None, scroll: bool = False) -> None:
        server = self._server
        start, goal = self.start, self.goal
        self.panel = server.gui.add_folder(self._label(), order=order, expand_by_default=expanded)
        with self.panel:
            if scroll:
                # Autofocus makes the browser scroll the side panel here.
                server.gui.add_html(
                    '<button autofocus style="width:0;height:0;padding:0;border:0;opacity:0">'
                    "</button>"
                )
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
            self.show_button = server.gui.add_button("show in scene")
            self.button = server.gui.add_button("save / update")
            self.delete_button = server.gui.add_button("delete")

            @self.show_button.on_click
            def _(_event: object) -> None:
                with self._hooks.lock:
                    self._hooks.announce(self._label())
                    self._hooks.highlight(self)

            @self.button.on_click
            def _(_event: object) -> None:
                # save_unsaved calls save_or_update already holding the lock.
                # The button path runs on a bare viser callback thread and must
                # take it to serialize suite/manifest mutation.
                with self._hooks.lock:
                    self.save_or_update()

            @self.delete_button.on_click
            def _(_event: object) -> None:
                with self._hooks.lock:
                    self.delete()

    def remove(self) -> None:
        self.removed = True
        self.panel.remove()
        for marker in self.markers:
            marker.remove()

    def delete(self) -> None:
        if self.saved_id is not None:
            ok, msg = self._hooks.delete_case(self.saved_id)
            print(msg)
            if not ok:
                self.message.content = f"**FAILED**: {msg}"
                return
        self._hooks.unregister(self)
        self.remove()

    def _snapshot(self) -> None:
        self._name = self.id_text.value
        self._checked = {tag for tag, box in self.tag_boxes.items() if box.value}
        self._custom = self.custom_text.value
        self._negative = self.negative_box.value

    def extra_tags(self) -> list[str]:
        tags = [tag for tag, box in self.tag_boxes.items() if box.value]
        tags += [t.strip() for t in self.custom_text.value.split(",") if t.strip()]
        return tags

    def save_or_update(self) -> bool:
        name = self.id_text.value.strip()
        if self.saved_id is None:
            ok, msg, saved, tags = self._hooks.save_pair(
                (float(self.start[0]), float(self.start[1]), float(self.start[2])),
                (float(self.goal[0]), float(self.goal[1]), float(self.goal[2])),
                self.negative_box.value,
                self.extra_tags(),
                name or None,
            )
        else:
            ok, msg, saved, tags = self._hooks.update_case(
                self.saved_id, name or self.saved_id, self.negative_box.value, self.extra_tags()
            )
        print(msg)
        if not (ok and saved is not None):
            self.message.content = f"**FAILED**: {msg}"
            return False
        # Viser cannot collapse a live panel, so replace it with the
        # collapsed button form, synced from the authoritative save.
        self.saved_id = saved
        self._snapshot()
        self._name = saved
        if tags is not None:
            self._sync_tags(tags)
        self._status = msg
        order = self.panel.order
        self.panel.remove()
        self._build(expanded=False, order=order)
        return True


def pick_cases(
    dataset: str,
    map_points: NDArray[np.float32],
    map_colors: NDArray[np.uint8],
    voxel_size: float,
    walked: NDArray[np.float32],
    cases: Sequence[Case],
    save_pair: SavePair,
    update_case: UpdateCase,
    delete_case: DeleteCase,
) -> None:
    """Serve the picker until the user exits from the panel or hits ctrl-c."""
    import viser

    server = viser.ViserServer(label=f"Pair Picker - {dataset}", verbose=False)
    server.gui.configure_theme(dark_mode=True)
    server.scene.set_background_image(np.full((1, 1, 3), 14, dtype=np.uint8))
    server.scene.set_up_direction("+z")
    # Neutral white lighting instead of the default HDRI environment map,
    # which tints the height colormap.
    server.scene.configure_environment_map(None)
    server.scene.configure_default_lights(enabled=False)
    server.scene.add_light_ambient("/lights/ambient", intensity=_AMBIENT_INTENSITY)
    server.scene.add_light_directional(
        "/lights/sun", intensity=_DIRECTIONAL_INTENSITY, position=(1.0, 2.0, 3.0)
    )
    # Cubes sit slightly under the voxel size so neighbors show a seam
    # instead of z-fighting, keeping individual voxels distinguishable.
    half = 0.42 * voxel_size
    corners = half * np.array(
        [[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)], dtype=np.float32
    )
    cube_faces = np.array(
        [
            [0, 1, 3],
            [0, 3, 2],
            [4, 6, 7],
            [4, 7, 5],
            [0, 4, 5],
            [0, 5, 1],
            [2, 3, 7],
            [2, 7, 6],
            [0, 2, 6],
            [0, 6, 4],
            [1, 5, 7],
            [1, 7, 3],
        ]
    )
    identity_quats = np.zeros((len(map_points), 4), dtype=np.float32)
    identity_quats[:, 0] = 1.0
    server.scene.add_batched_meshes_simple(
        "/map",
        corners,
        cube_faces,
        batched_wxyzs=identity_quats,
        batched_positions=map_points,
        batched_colors=_cube_colors(map_colors),
        flat_shading=True,
        cast_shadow=False,
        receive_shadow=False,
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
    selected_line = server.gui.add_markdown("selected: —")
    undo_button = server.gui.add_button("undo last pick")
    save_all_button = server.gui.add_button("save all unsaved")
    exit_button = server.gui.add_button("save all & exit")

    lock = threading.Lock()
    stop = threading.Event()
    pairs: list[_PairEntry] = []

    def announce(label: str) -> None:
        selected_line.content = f"selected: **{label}**"

    highlighted: list[_PairEntry] = []

    def highlight(entry: _PairEntry) -> None:
        while highlighted:
            highlighted.pop().set_highlight(False)
        entry.set_highlight(True)
        highlighted.append(entry)

    hooks = _Hooks(
        save_pair,
        update_case,
        delete_case,
        lock,
        lambda entry: pairs.remove(entry),
        announce,
        highlight,
    )
    marker_seq = 0

    def sphere(point: NDArray[np.float32], color: tuple[int, int, int]) -> viser.SceneNodeHandle:
        nonlocal marker_seq
        marker_seq += 1
        return server.scene.add_icosphere(
            f"/picks/m{marker_seq}",
            radius=0.09,
            color=_marker_color(color),
            position=(float(point[0]), float(point[1]), float(point[2]) + 0.05),
        )

    def pair_line(start: NDArray[np.float32], goal: NDArray[np.float32]) -> viser.SceneNodeHandle:
        nonlocal marker_seq
        marker_seq += 1
        return server.scene.add_line_segments(
            f"/picks/m{marker_seq}",
            np.stack([start, goal])[None],
            colors=PAIR_COLOR,
            line_width=2.5,
        )

    def pair_markers(
        start: NDArray[np.float32], goal: NDArray[np.float32]
    ) -> list[viser.SceneNodeHandle]:
        return [sphere(start, START_COLOR), sphere(goal, GOAL_COLOR), pair_line(start, goal)]

    for case in cases:
        start = np.asarray(case.start, dtype=np.float32)
        goal = np.asarray(case.goal, dtype=np.float32)
        pairs.append(
            _PairEntry(server, 0, start, goal, hooks, pair_markers(start, goal), case=case)
        )

    pending: list[tuple[viser.SceneNodeHandle, NDArray[np.float32]]] = []
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
            if not pending:
                pending.append((sphere(point, START_COLOR), point))
                return
            start_marker, start = pending.pop()
            markers = [start_marker, sphere(point, GOAL_COLOR), pair_line(start, point)]
            pair_count += 1
            pairs.append(_PairEntry(server, pair_count, start, point, hooks, markers))

    @undo_button.on_click
    def _(_event: object) -> None:
        with lock:
            if pending:
                pending.pop()[0].remove()
            elif pairs and not pairs[-1].preloaded:
                # Saved cases stay in the manifest, only the panel entry and
                # markers go away. Deleting from the manifest is the per-pair
                # delete button.
                entry = pairs.pop()
                entry.remove()
                if entry.saved_id is not None:
                    print(f"{entry.saved_id} stays in the manifest; use delete to remove it")

    def save_unsaved() -> int:
        with lock:
            return sum(not pair.save_or_update() for pair in pairs if pair.saved_id is None)

    @save_all_button.on_click
    def _(_event: object) -> None:
        save_unsaved()

    @exit_button.on_click
    def _(_event: object) -> None:
        if save_unsaved() == 0:
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
