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

"""Browser point-picking for case curation, served by viser."""

from __future__ import annotations

from dataclasses import dataclass
import threading
from typing import TYPE_CHECKING

import numpy as np

from dimos.core.global_config import global_config
from dimos.navigation.nav_3d.evaluator.curation import CurationError
from dimos.navigation.nav_3d.evaluator.tagging import elevation_tags
from dimos.navigation.nav_3d.evaluator.viz import turbo_by_height

if TYPE_CHECKING:
    from collections.abc import Callable

    from numpy.typing import NDArray
    import viser

    from dimos.navigation.nav_3d.evaluator.cases import Case
    from dimos.navigation.nav_3d.evaluator.curation import CaseStore

START_COLOR = (0, 255, 255)
GOAL_COLOR = (255, 140, 0)
PAIR_COLOR = (255, 255, 0)
HIGHLIGHT_LINE_COLOR = (255, 255, 255)
MARKER_RADIUS = 0.14
HIGHLIGHT_MARKER_RADIUS = 0.22
# Pick markers float this far above their point so the voxel cube they sit on
# does not hide them.
MARKER_LIFT = 0.12
LINE_WIDTH = 2.5
HIGHLIGHT_LINE_WIDTH = 6.0
# A pair being picked but not yet saved wears distinct colors and a thicker line
# so it stands out from the cases already in the manifest, until it is saved.
NEW_START_COLOR = (0, 255, 128)
NEW_GOAL_COLOR = (255, 0, 200)
NEW_PAIR_COLOR = (0, 255, 128)
NEW_LINE_WIDTH = 5.0
# Cube meshes are drawn slightly inside their voxel so neighbors show a seam.
CUBE_SHRINK = 0.42
BACKGROUND_LEVEL = 14
# Opening camera placement, as a multiple of the map's horizontal span.
CAMERA_OFFSET = (0.6, 0.6, 0.45)
SUGGESTED_TAGS = ("stairs", "flat", "up", "down", "long", "doorway")

INSTRUCTIONS = """**shift+click** picks START then GOAL, repeated per case.
**click** an endpoint sphere to highlight and open its case.
Plain drag orbits, scroll zooms, right-drag pans.
"""


# three.js ACES filmic tone mapping. The viewer applies it to mesh materials
# but not to the point and line shaders.
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
# White scene lights, bright enough that inverse-tone-mapped albedos fit in
# [0, 1]. LIGHT_REFERENCE is the ambient plus directional total on a typical face.
_AMBIENT_INTENSITY = 3.5
_DIRECTIONAL_INTENSITY = 2.0
_LIGHT_REFERENCE = 4.6


def _prelit_albedo(srgb: NDArray[np.uint8]) -> NDArray[np.float64]:
    """Linear albedo that cancels the viewer's ACES pass."""
    c = srgb.astype(np.float64) / 255.0
    lin = np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)
    t = np.clip(lin @ np.linalg.inv(_ACES_OUTPUT).T, 0.0, 0.99)
    a2 = 1.0 - 0.983729 * t
    a1 = 0.0245786 - 0.4329510 * t
    a0 = -0.000090537 - 0.238081 * t
    v = (-a1 + np.sqrt(a1 * a1 - 4.0 * a2 * a0)) / (2.0 * a2)
    x = 0.6 * (v @ np.linalg.inv(_ACES_INPUT).T)
    albedo: NDArray[np.float64] = np.clip(x / _LIGHT_REFERENCE, 0.0, 1.0)
    return albedo


def _cube_colors(srgb: NDArray[np.uint8]) -> NDArray[np.uint8]:
    """Per-instance cube colors. The viewer reads these as linear RGB."""
    return np.asarray((_prelit_albedo(srgb) * 255.0).round(), dtype=np.uint8)


def _marker_color(srgb: tuple[int, int, int]) -> tuple[int, int, int]:
    """Marker mesh color. The viewer reads this as sRGB."""
    albedo = _prelit_albedo(np.array(srgb, dtype=np.uint8))
    out = np.where(albedo <= 0.0031308, albedo * 12.92, 1.055 * albedo ** (1 / 2.4) - 0.055)
    r, g, b = (out * 255.0).round().astype(int)
    return int(r), int(g), int(b)


def _point(p: NDArray[np.float32]) -> tuple[float, float, float]:
    return (float(p[0]), float(p[1]), float(p[2]))


def pick_along_ray(
    points: NDArray[np.float32],
    origin: NDArray[np.float64],
    direction: NDArray[np.float64],
    radius: float,
) -> NDArray[np.float32] | None:
    """Nearest cloud point inside a tube around the click ray."""
    rel = points.astype(np.float64) - origin
    t = rel @ direction
    ahead = t > 0.05
    if not ahead.any():
        return None
    t = t[ahead]
    perp = np.linalg.norm(rel[ahead] - t[:, None] * direction, axis=1)
    for r in (radius, 3.0 * radius):
        hit = perp < r
        if hit.any():
            idx = np.flatnonzero(ahead)[hit]
            return np.asarray(points[idx[np.argmin(t[hit])]])
    return None


@dataclass
class _PairMarkers:
    """The three scene nodes drawn for one start/goal pair."""

    start: viser.IcosphereHandle
    goal: viser.IcosphereHandle
    line: viser.LineSegmentsHandle


@dataclass
class _Hooks:
    """Manifest store and shared state handed to every pair entry."""

    store: CaseStore
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
        markers: _PairMarkers,
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
            self._checked = set(elevation_tags(_point(start), _point(goal)))
            self._custom = ""
            self._negative = False
            self._dynamic = False
            self._status = "unsaved"
        else:
            self.saved_id = case.id
            self._name = case.id
            self._sync_tags(case.tags)
            self._negative = case.expect_fail
            self._dynamic = case.expect_final_fail
            self._status = "in manifest"
        self.removed = False
        self._build(expanded=case is None, order=None)
        markers.start.on_click(self._on_marker_click)
        markers.goal.on_click(self._on_marker_click)

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
        for ball in (self.markers.start, self.markers.goal):
            ball.radius = HIGHLIGHT_MARKER_RADIUS if on else MARKER_RADIUS
        self.markers.line.line_width = HIGHLIGHT_LINE_WIDTH if on else LINE_WIDTH
        self.markers.line.colors = np.array(
            HIGHLIGHT_LINE_COLOR if on else PAIR_COLOR, dtype=np.uint8
        )

    def _mark_saved(self) -> None:
        """Repaint the pick markers to the saved look, leaving new picks distinct."""
        self.markers.start.color = _marker_color(START_COLOR)
        self.markers.goal.color = _marker_color(GOAL_COLOR)
        self.markers.line.line_width = LINE_WIDTH
        self.markers.line.colors = np.array(PAIR_COLOR, dtype=np.uint8)

    def _label(self) -> str:
        return self.saved_id or f"pair {self._n}"

    def _sync_tags(self, tags: list[str]) -> None:
        """Split a tag list into checkbox and custom-text state."""
        self._checked = {t for t in tags if t in SUGGESTED_TAGS}
        self._custom = ", ".join(
            t for t in tags if t not in SUGGESTED_TAGS and t not in ("negative", "dynamic")
        )

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
            self.negative_box = server.gui.add_checkbox(
                "negative (should fail planning)", self._negative
            )
            self.dynamic_box = server.gui.add_checkbox(
                "dynamic (final map expected to fail planning)", self._dynamic
            )
            self.message = server.gui.add_markdown(self._status)
            self.button = server.gui.add_button("save / update")
            self.delete_button = server.gui.add_button("delete")

            # Button callbacks run on bare viser threads, so they take the lock
            # that save_unsaved already holds on its own path.
            def _on_save(_event: object) -> None:
                with self._hooks.lock:
                    self.save_or_update()

            def _on_delete(_event: object) -> None:
                with self._hooks.lock:
                    self.delete()

            self.button.on_click(_on_save)
            self.delete_button.on_click(_on_delete)

    def remove(self) -> None:
        self.removed = True
        self.panel.remove()
        for marker in (self.markers.start, self.markers.goal, self.markers.line):
            marker.remove()

    def delete(self) -> None:
        if self.saved_id is not None:
            try:
                self._hooks.store.delete(self.saved_id)
            except CurationError as err:
                self.message.content = f"**FAILED**: {err}"
                print(err)
                return
            print(f"deleted {self.saved_id}")
        self._hooks.unregister(self)
        self.remove()

    def _snapshot(self) -> None:
        self._name = self.id_text.value
        self._checked = {tag for tag, box in self.tag_boxes.items() if box.value}
        self._custom = self.custom_text.value
        self._negative = self.negative_box.value
        self._dynamic = self.dynamic_box.value

    def extra_tags(self) -> list[str]:
        tags = [tag for tag, box in self.tag_boxes.items() if box.value]
        tags += [t.strip() for t in self.custom_text.value.split(",") if t.strip()]
        return tags

    def save_or_update(self) -> bool:
        name = self.id_text.value.strip()
        store = self._hooks.store
        negative = self.negative_box.value
        dynamic = self.dynamic_box.value
        try:
            if self.saved_id is None:
                case = store.add(
                    _point(self.start),
                    _point(self.goal),
                    self.extra_tags(),
                    case_id=name or None,
                    expect_fail=negative,
                    expect_final_fail=dynamic,
                )
            else:
                case = store.update(
                    self.saved_id,
                    name or self.saved_id,
                    self.extra_tags(),
                    expect_fail=negative,
                    expect_final_fail=dynamic,
                )
        except CurationError as err:
            print(err)
            self.message.content = f"**FAILED**: {err}"
            return False
        msg = f"saved {case.id} [{', '.join(case.tags)}]"
        print(msg)
        self._mark_saved()
        # Viser cannot collapse a live panel, so replace it with the
        # collapsed button form, synced from the authoritative save.
        self.saved_id = case.id
        self._snapshot()
        self._name = case.id
        self._sync_tags(case.tags)
        self._status = msg
        order = self.panel.order
        self.panel.remove()
        self._build(expanded=False, order=order)
        return True


def _add_map_scene(
    server: viser.ViserServer,
    map_points: NDArray[np.float32],
    map_colors: NDArray[np.uint8],
    voxel_size: float,
    walked: NDArray[np.float32],
) -> None:
    """Draw the voxel map, the walked path, and the lighting they are lit by."""
    server.gui.configure_theme(dark_mode=True)
    server.scene.set_background_image(np.full((1, 1, 3), BACKGROUND_LEVEL, dtype=np.uint8))
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
    half = CUBE_SHRINK * voxel_size
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

    def _on_client_connect(client: viser.ClientHandle) -> None:
        client.camera.position = tuple(center + span * np.array(CAMERA_OFFSET))
        client.camera.look_at = tuple(center)

    server.on_client_connect(_on_client_connect)


class _PickerSession:
    """Owns the scene markers and the live pair list for one picker run."""

    def __init__(
        self,
        server: viser.ViserServer,
        store: CaseStore,
        occupied: NDArray[np.float32],
        voxel_size: float,
    ) -> None:
        self.server = server
        self.store = store
        self.occupied = occupied
        self.voxel_size = voxel_size
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.pairs: list[_PairEntry] = []
        self.pending: list[tuple[viser.IcosphereHandle, NDArray[np.float32]]] = []
        self.highlighted: list[_PairEntry] = []
        self.pair_count = 0
        self._marker_seq = 0
        self.selected_line = server.gui.add_markdown("selected: none")
        self.hooks = _Hooks(store, self.lock, self.pairs.remove, self.announce, self.highlight)

    def announce(self, label: str) -> None:
        self.selected_line.content = f"selected: **{label}**"

    def highlight(self, entry: _PairEntry) -> None:
        while self.highlighted:
            self.highlighted.pop().set_highlight(False)
        entry.set_highlight(True)
        self.highlighted.append(entry)

    def _next_path(self) -> str:
        self._marker_seq += 1
        return f"/picks/m{self._marker_seq}"

    def sphere(
        self, point: NDArray[np.float32], color: tuple[int, int, int]
    ) -> viser.IcosphereHandle:
        return self.server.scene.add_icosphere(
            self._next_path(),
            radius=MARKER_RADIUS,
            color=_marker_color(color),
            position=(float(point[0]), float(point[1]), float(point[2]) + MARKER_LIFT),
        )

    def pair_line(
        self,
        start: NDArray[np.float32],
        goal: NDArray[np.float32],
        color: tuple[int, int, int] = PAIR_COLOR,
        width: float = LINE_WIDTH,
    ) -> viser.LineSegmentsHandle:
        return self.server.scene.add_line_segments(
            self._next_path(), np.stack([start, goal])[None], colors=color, line_width=width
        )

    def load_manifest_pairs(self) -> None:
        """Draw every case already in the manifest as an editable entry."""
        for case in self.store.suite.cases:
            start = np.asarray(case.start, dtype=np.float32)
            goal = np.asarray(case.goal, dtype=np.float32)
            markers = _PairMarkers(
                self.sphere(start, START_COLOR),
                self.sphere(goal, GOAL_COLOR),
                self.pair_line(start, goal),
            )
            self.pairs.append(
                _PairEntry(self.server, 0, start, goal, self.hooks, markers, case=case)
            )

    def on_click(self, event: viser.SceneClickEvent) -> None:
        """Shift+click picks a start, then a goal, then opens the new pair."""
        point = pick_along_ray(
            self.occupied,
            np.asarray(event.ray_origin),
            np.asarray(event.ray_direction),
            self.voxel_size,
        )
        if point is None:
            return
        with self.lock:
            if not self.pending:
                self.pending.append((self.sphere(point, NEW_START_COLOR), point))
                return
            start_marker, start = self.pending.pop()
            markers = _PairMarkers(
                start_marker,
                self.sphere(point, NEW_GOAL_COLOR),
                self.pair_line(start, point, NEW_PAIR_COLOR, NEW_LINE_WIDTH),
            )
            self.pair_count += 1
            self.pairs.append(
                _PairEntry(self.server, self.pair_count, start, point, self.hooks, markers)
            )

    def on_undo(self, _event: object) -> None:
        with self.lock:
            if self.pending:
                self.pending.pop()[0].remove()
            elif self.pairs and not self.pairs[-1].preloaded:
                # Only the panel entry and markers go away. The per-pair delete
                # button is what removes it from the manifest.
                entry = self.pairs.pop()
                entry.remove()
                if entry.saved_id is not None:
                    print(f"{entry.saved_id} stays in the manifest; use delete to remove it")

    def save_unsaved(self) -> int:
        """Save every pair not yet in the manifest, returning the failure count."""
        with self.lock:
            return sum(not pair.save_or_update() for pair in self.pairs if pair.saved_id is None)

    def on_save_all(self, _event: object) -> None:
        self.save_unsaved()

    def on_exit(self, _event: object) -> None:
        if self.save_unsaved() == 0:
            self.stop.set()

    def serve(self) -> None:
        """Block until the panel exits or the terminal interrupts."""
        print("picker running; ctrl-c to exit (unsaved pairs are discarded)")
        try:
            self.stop.wait()
        except KeyboardInterrupt:
            unsaved = sum(1 for p in self.pairs if p.saved_id is None)
            if unsaved:
                print(f"discarded {unsaved} unsaved pair(s)")
        finally:
            self.server.stop()


def pick_cases(
    store: CaseStore,
    walked: NDArray[np.float32],
    occupied: NDArray[np.float32],
    voxel_size: float,
) -> None:
    """Serve the picker until the user exits the panel or hits ctrl-c."""
    # Lazy: viser is an optional extra, only needed by this command.
    import viser

    server = viser.ViserServer(
        host=global_config.listen_host,
        label=f"Pair Picker - {store.suite.dataset}",
        verbose=False,
    )
    _add_map_scene(server, occupied, turbo_by_height(occupied), voxel_size, walked)
    server.gui.add_markdown(INSTRUCTIONS)
    session = _PickerSession(server, store, occupied, voxel_size)
    session.load_manifest_pairs()

    server.scene.on_click(modifier="shift")(session.on_click)
    server.gui.add_button("undo last pick").on_click(session.on_undo)
    server.gui.add_button("save all unsaved").on_click(session.on_save_all)
    server.gui.add_button("save all & exit").on_click(session.on_exit)
    session.serve()
