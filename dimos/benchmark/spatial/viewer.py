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

# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
"""Read-only Viser rendering boundary for spatial corpus inspection."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
import math
import threading
from typing import TYPE_CHECKING

import numpy as np
from numpy.typing import NDArray

from dimos.benchmark.spatial.config import SPATIAL_BENCHMARK_V1
from dimos.benchmark.spatial.corpus_loader import (
    SpatialCorpusInstance,
    SpatialCorpusLoader,
    SpatialCorpusSelection,
)
from dimos.benchmark.spatial.map_generation import load_snapshot_map
from dimos.benchmark.spatial.models import (
    BarrierSegment,
    DirectNeighborCountContract,
    DirectRoomConnectionContract,
    Geometry,
    MapVariant,
    Marker,
    MarkerGeometry,
    OpeningEdge,
    Point2D,
    Polygon2D,
    Pose2D,
    Predicate,
    RotationContract,
    RotationGeometry,
    SameRoomContract,
    Split,
    Topology,
    TranslationContract,
    TranslationGeometry,
)

PointArray = NDArray[np.float64]
RgbColor = tuple[int, int, int]
BoxDimensions = tuple[float, float, float]

if TYPE_CHECKING:
    from viser import GuiTextHandle


# These are semantic colors, not a data colormap. The observed map is deliberately
# neutral because the horizontal lidar data has no useful height variation to encode.
OBSERVED_MAP_COLOR: RgbColor = (135, 143, 151)
ROBOT_COLOR: RgbColor = (37, 99, 235)
QUERY_COLOR: RgbColor = (217, 70, 239)
ORACLE_WALL_COLOR: RgbColor = (166, 75, 68)
ORACLE_OPENING_COLOR: RgbColor = (66, 143, 94)
ORACLE_TOPOLOGY_COLOR: RgbColor = (124, 58, 237)

# Review-only presentation conventions. Geometry records are intentionally 2-D;
# the low relief preserves the observed scan in a manually rotated Viser view
# without claiming to reproduce source-building dimensions.
DERIVED_WALL_WIDTH_M = 0.08
DERIVED_WALL_HEIGHT_M = 1.0
DERIVED_WALL_OPACITY = 0.46
DERIVED_THRESHOLD_DEPTH_M = 0.04
DERIVED_THRESHOLD_HEIGHT_M = 0.035
DERIVED_THRESHOLD_OPACITY = 0.72
DERIVED_TOPOLOGY_CLEARANCE_M = 0.025
DERIVED_QUERY_CLEARANCE_M = 0.06
DERIVED_RELIEF_PRESENTATION = "derived-private-architectural-relief"

AGENT_VISIBLE_GROUP = "agent-visible"
PRIVATE_ORACLE_GROUP = "private-oracle"


@dataclass(frozen=True)
class DrawCommand:
    kind: str
    name: str
    points: tuple[tuple[float, float, float], ...] = ()
    text: str = ""
    color: RgbColor | None = None
    group: str = ""
    line_width: float | None = None
    point_size: float | None = None
    dimensions_m: BoxDimensions | None = None
    opacity: float | None = None
    material: str = ""
    yaw_rad: float = 0.0
    derived: bool = False
    source: str = ""
    presentation: str = ""
    base_z_m: float | None = None
    rows: tuple[InspectorRow, ...] = ()


@dataclass(frozen=True)
class InspectorRow:
    """One concise, read-only row in the reviewer inspector."""

    label: str
    value: str
    hint: str = ""


@dataclass(frozen=True)
class InspectorSection:
    """A named native Viser inspector section with read-only rows."""

    title: str
    group: str
    rows: tuple[InspectorRow, ...]


@dataclass(frozen=True)
class QASample:
    """One physical public question, represented by its clean map instance."""

    instance: SpatialCorpusInstance
    label: str


class SpatialQASelector:
    """Deterministic, read-only state for the cascading spatial QA controls."""

    def __init__(
        self, loader: SpatialCorpusLoader, selection: SpatialCorpusSelection | None = None
    ) -> None:
        self._loader = loader
        requested = (
            loader.require_one(selection)
            if selection is not None and selection.instance_id
            else None
        )
        selection = selection or SpatialCorpusSelection()
        sample_selection = SpatialCorpusSelection(
            scene_id=selection.scene_id,
            trajectory_id=selection.trajectory_id,
            question_id=requested.question.question_id
            if requested is not None
            else selection.question_id,
            predicate=selection.predicate,
            variant=MapVariant.CLEAN,
        )
        samples = _qa_samples(loader, sample_selection)
        if not samples:
            raise ValueError("no corpus QA samples match selection")
        self._samples = samples
        self._predicate = (
            requested.question.predicate
            if requested is not None
            else selection.predicate or samples[0].instance.question.predicate
        )
        self._sample_label = self._initial_sample_label(requested)
        self._variant = selection.variant or MapVariant.CLEAN

    @property
    def predicate_labels(self) -> tuple[str, ...]:
        return tuple(_predicate_label(predicate) for predicate in self._predicates())

    @property
    def selected_predicate_label(self) -> str:
        return _predicate_label(self._predicate)

    @property
    def sample_labels(self) -> tuple[str, ...]:
        return tuple(sample.label for sample in self._predicate_samples())

    @property
    def selected_sample_label(self) -> str:
        return self._sample_label

    @property
    def selected_variant(self) -> MapVariant:
        return self._variant

    def select_predicate_label(self, label: str) -> SpatialCorpusInstance:
        predicate = next(
            (candidate for candidate in self._predicates() if _predicate_label(candidate) == label),
            None,
        )
        if predicate is None:
            raise ValueError(f"unknown QA predicate label: {label}")
        self._predicate = predicate
        self._sample_label = self._predicate_samples()[0].label
        return self.current_instance()

    def select_sample_label(self, label: str) -> SpatialCorpusInstance:
        if label not in self.sample_labels:
            raise ValueError(f"unknown QA sample label: {label}")
        self._sample_label = label
        return self.current_instance()

    def select_variant(self, variant: MapVariant) -> SpatialCorpusInstance:
        self._variant = variant
        return self.current_instance()

    def current_instance(self) -> SpatialCorpusInstance:
        sample = next(
            sample for sample in self._predicate_samples() if sample.label == self._sample_label
        )
        return self._loader.variant(sample.instance, self._variant)

    def _predicates(self) -> tuple[Predicate, ...]:
        return tuple(
            predicate
            for predicate in Predicate
            if any(sample.instance.question.predicate is predicate for sample in self._samples)
        )

    def _predicate_samples(self) -> tuple[QASample, ...]:
        return tuple(
            sample
            for sample in self._samples
            if sample.instance.question.predicate is self._predicate
        )

    def _initial_sample_label(self, requested: SpatialCorpusInstance | None) -> str:
        if requested is not None:
            requested_key = _physical_question_key(requested)
            for sample in self._predicate_samples():
                if _physical_question_key(sample.instance) == requested_key:
                    return sample.label
        return self._predicate_samples()[0].label


@dataclass
class ViserReadOnlyBoundary:
    """Small fakeable boundary; real Viser adapters must not expose edit controls."""

    commands: list[DrawCommand] = field(default_factory=list)
    qa_selector: SpatialQASelector | None = None
    on_qa_selection: Callable[[SpatialCorpusInstance], None] | None = None

    def clear(self) -> None:
        self.commands.clear()

    def add_point_cloud(
        self,
        name: str,
        points: PointArray,
        *,
        color: RgbColor,
        group: str,
        point_size: float,
    ) -> None:
        self.commands.append(
            DrawCommand(
                "point-cloud",
                name,
                _tuples(points),
                color=color,
                group=group,
                point_size=point_size,
            )
        )

    def add_polyline(
        self,
        name: str,
        points: PointArray,
        *,
        color: RgbColor,
        group: str,
        line_width: float,
    ) -> None:
        self.commands.append(
            DrawCommand(
                "polyline",
                name,
                _tuples(points),
                color=color,
                group=group,
                line_width=line_width,
            )
        )

    def add_box(
        self,
        name: str,
        position: PointArray,
        *,
        dimensions_m: BoxDimensions,
        yaw_rad: float,
        color: RgbColor,
        group: str,
        opacity: float,
        material: str,
        derived: bool,
        source: str,
        presentation: str,
        base_z_m: float,
    ) -> None:
        """Record a shaded solid, including its review-only provenance."""

        self.commands.append(
            DrawCommand(
                "box",
                name,
                _tuples(position.reshape((1, 3))),
                color=color,
                group=group,
                dimensions_m=dimensions_m,
                opacity=opacity,
                material=material,
                yaw_rad=yaw_rad,
                derived=derived,
                source=source,
                presentation=presentation,
                base_z_m=base_z_m,
            )
        )

    def add_markers(
        self,
        name: str,
        points: PointArray,
        *,
        color: RgbColor,
        group: str,
        point_size: float,
    ) -> None:
        self.commands.append(
            DrawCommand(
                "markers",
                name,
                _tuples(points),
                color=color,
                group=group,
                point_size=point_size,
            )
        )

    def add_label(
        self,
        name: str,
        text: str,
        position: PointArray,
        *,
        group: str,
    ) -> None:
        self.commands.append(DrawCommand("label", name, _tuples(position), text=text, group=group))

    def add_inspector_section(self, name: str, section: InspectorSection) -> None:
        """Record a compact, read-only inspector section for fakeable tests."""

        self.commands.append(
            DrawCommand(
                "inspector-section",
                name,
                text=section.title,
                group=section.group,
                rows=section.rows,
            )
        )

    def add_qa_selector(
        self,
        selector: SpatialQASelector,
        on_selection: Callable[[SpatialCorpusInstance], None],
    ) -> None:
        """Record read-only selector state; live adapters attach Viser controls."""

        self.qa_selector = selector
        self.on_qa_selection = on_selection


class RealViserReadOnlyBoundary(ViserReadOnlyBoundary):
    """Read-only adapter from DrawCommand semantics to a live Viser server."""

    def __init__(self, *, host: str = "127.0.0.1", port: int = 8080) -> None:
        super().__init__()
        try:
            import viser
        except ImportError as error:
            raise ImportError(
                "Viser is required for interactive spatial benchmark viewing. "
                "Install the visualization extra, then rerun the view command."
            ) from error
        self.server = viser.ViserServer(host=host, port=port)
        self.server.gui.configure_theme(
            control_layout="fixed",
            control_width="large",
            dark_mode=True,
            show_logo=False,
            show_share_button=False,
            brand_color=ROBOT_COLOR,
        )
        self.server.scene.set_up_direction("+z")
        self._inspector_fields: dict[str, tuple[GuiTextHandle, ...]] = {}
        self._qa_controls_created = False
        self._qa_controls_syncing = False
        self.url = f"http://{host}:{port}"

    def clear(self) -> None:
        super().clear()
        self.server.scene.reset()
        self.server.scene.set_up_direction("+z")

    def add_point_cloud(
        self,
        name: str,
        points: PointArray,
        *,
        color: RgbColor,
        group: str,
        point_size: float,
    ) -> None:
        super().add_point_cloud(name, points, color=color, group=group, point_size=point_size)
        colors = np.tile(np.asarray(color, dtype=np.uint8), (len(points), 1))
        self.server.scene.add_point_cloud(
            name,
            points=points,
            colors=colors,
            point_size=point_size,
            point_shading="flat",
        )

    def add_polyline(
        self,
        name: str,
        points: PointArray,
        *,
        color: RgbColor,
        group: str,
        line_width: float,
    ) -> None:
        super().add_polyline(name, points, color=color, group=group, line_width=line_width)
        if len(points) < 2:
            return
        segments = np.stack((points[:-1], points[1:]), axis=1)
        self.server.scene.add_line_segments(
            name,
            points=segments,
            colors=np.asarray(color, dtype=np.uint8),
            line_width=line_width,
        )

    def add_box(
        self,
        name: str,
        position: PointArray,
        *,
        dimensions_m: BoxDimensions,
        yaw_rad: float,
        color: RgbColor,
        group: str,
        opacity: float,
        material: str,
        derived: bool,
        source: str,
        presentation: str,
        base_z_m: float,
    ) -> None:
        super().add_box(
            name,
            position,
            dimensions_m=dimensions_m,
            yaw_rad=yaw_rad,
            color=color,
            group=group,
            opacity=opacity,
            material=material,
            derived=derived,
            source=source,
            presentation=presentation,
            base_z_m=base_z_m,
        )
        self.server.scene.add_box(
            name,
            dimensions=dimensions_m,
            color=color,
            opacity=opacity,
            material=material,
            flat_shading=True,
            position=position,
            wxyz=np.array(
                (math.cos(yaw_rad / 2.0), 0.0, 0.0, math.sin(yaw_rad / 2.0)),
                dtype=np.float64,
            ),
        )

    def add_markers(
        self,
        name: str,
        points: PointArray,
        *,
        color: RgbColor,
        group: str,
        point_size: float,
    ) -> None:
        super().add_markers(name, points, color=color, group=group, point_size=point_size)
        colors = np.tile(np.asarray(color, dtype=np.uint8), (len(points), 1))
        self.server.scene.add_point_cloud(
            name,
            points=points,
            colors=colors,
            point_size=point_size,
            point_shading="flat",
        )

    def add_label(
        self,
        name: str,
        text: str,
        position: PointArray,
        *,
        group: str,
    ) -> None:
        super().add_label(name, text, position, group=group)
        self.server.scene.add_label(
            name,
            text=text,
            position=position[0],
            font_size_mode="screen",
            font_screen_scale=1.25,
            anchor="bottom-center",
        )

    def add_inspector_section(self, name: str, section: InspectorSection) -> None:
        super().add_inspector_section(name, section)
        fields = self._inspector_fields.get(name)
        if fields is None:
            folder = self.server.gui.add_folder(
                section.title,
                expand_by_default=name in {"/inspector/question", "/inspector/private-answer"},
            )
            with folder:
                fields = tuple(
                    self.server.gui.add_text(
                        row.label,
                        row.value,
                        multiline=row.label == "Question",
                        disabled=True,
                        hint=row.hint or None,
                    )
                    for row in section.rows
                )
            self._inspector_fields[name] = fields
        if len(fields) != len(section.rows):
            raise RuntimeError(f"inspector section shape changed: {name}")
        for text_handle, row in zip(fields, section.rows, strict=True):
            text_handle.value = row.value

    def add_qa_selector(
        self,
        selector: SpatialQASelector,
        on_selection: Callable[[SpatialCorpusInstance], None],
    ) -> None:
        super().add_qa_selector(selector, on_selection)
        if not self._qa_controls_created:
            self._create_qa_controls()
        self._sync_qa_controls()

    def _create_qa_controls(self) -> None:
        if self.qa_selector is None:
            raise RuntimeError("QA selector state must exist before controls are created")
        selector = self.qa_selector
        controls = self.server.gui.add_folder("QA selection", expand_by_default=True)
        with controls:
            self._predicate_control = self.server.gui.add_dropdown(
                "Predicate",
                options=selector.predicate_labels,
                initial_value=selector.selected_predicate_label,
                hint="Question family shown on the map.",
            )
            self._sample_control = self.server.gui.add_dropdown(
                "Sample",
                options=selector.sample_labels,
                initial_value=selector.selected_sample_label,
                hint="Public question within the selected predicate.",
            )
            self._variant_control = self.server.gui.add_dropdown(
                "Map variant",
                options=tuple(variant.value for variant in MapVariant),
                initial_value=selector.selected_variant.value,
                hint="Paired observed map for the same physical question.",
            )
        self._predicate_control.on_update(self._select_predicate)
        self._sample_control.on_update(self._select_sample)
        self._variant_control.on_update(self._select_variant)
        self._qa_controls_created = True

    def _sync_qa_controls(self) -> None:
        if self.qa_selector is None:
            return
        self._qa_controls_syncing = True
        try:
            self._predicate_control.options = self.qa_selector.predicate_labels
            self._predicate_control.value = self.qa_selector.selected_predicate_label
            self._sample_control.options = self.qa_selector.sample_labels
            self._sample_control.value = self.qa_selector.selected_sample_label
            self._variant_control.value = self.qa_selector.selected_variant.value
        finally:
            self._qa_controls_syncing = False

    def _select_predicate(self, event: object) -> None:
        if self._qa_controls_syncing or self.qa_selector is None or self.on_qa_selection is None:
            return
        instance = self.qa_selector.select_predicate_label(_event_value(event))
        self._sync_qa_controls()
        self.on_qa_selection(instance)

    def _select_sample(self, event: object) -> None:
        if self._qa_controls_syncing or self.qa_selector is None or self.on_qa_selection is None:
            return
        instance = self.qa_selector.select_sample_label(_event_value(event))
        self._sync_qa_controls()
        self.on_qa_selection(instance)

    def _select_variant(self, event: object) -> None:
        if self._qa_controls_syncing or self.qa_selector is None or self.on_qa_selection is None:
            return
        instance = self.qa_selector.select_variant(MapVariant(_event_value(event)))
        self._sync_qa_controls()
        self.on_qa_selection(instance)

    def block_forever(self) -> None:
        threading.Event().wait()


class SpatialCorpusViserView:
    """Read-only renderer and navigation state for one corpus instance."""

    def __init__(
        self, loader: SpatialCorpusLoader, boundary: ViserReadOnlyBoundary | None = None
    ) -> None:
        self.loader = loader
        self.boundary = boundary if boundary is not None else ViserReadOnlyBoundary()

    def render(
        self,
        instance: SpatialCorpusInstance,
        *,
        show_oracle_geometry: bool = False,
        show_oracle_topology: bool = True,
    ) -> None:
        """Draw a read-only plan view without GT wall relief by default."""

        self.boundary.clear()
        cloud = load_snapshot_map(instance.variant_root, instance.snapshot)
        points, _colors = cloud.as_numpy()
        observed_points = np.asarray(points[:, :3], dtype=np.float64)
        self._presentation_base_z_m = _presentation_base_z(observed_points)
        relief_visible = instance.oracle is not None and show_oracle_geometry
        relief_top_z_m = self._presentation_base_z_m + (
            DERIVED_WALL_HEIGHT_M if relief_visible else 0.0
        )
        self._query_z_m = relief_top_z_m + DERIVED_QUERY_CLEARANCE_M
        self._topology_z_m = relief_top_z_m + DERIVED_TOPOLOGY_CLEARANCE_M
        for name, section in _inspector_sections(
            instance, show_oracle_geometry, show_oracle_topology
        ):
            self.boundary.add_inspector_section(name, section)
        self.boundary.add_point_cloud(
            "/agent-visible/observed-map",
            observed_points,
            color=OBSERVED_MAP_COLOR,
            group=AGENT_VISIBLE_GROUP,
            point_size=0.025,
        )
        self._draw_query(instance)
        if instance.oracle is not None and show_oracle_geometry:
            self._draw_oracle_geometry(instance.oracle.geometry, self._presentation_base_z_m)
        if instance.oracle is not None and show_oracle_topology:
            self._draw_oracle_topology(instance, instance.oracle.topology, self._topology_z_m)

    def start_qa_review(
        self, selection: SpatialCorpusSelection | None = None
    ) -> SpatialCorpusInstance:
        """Attach the cascading selector and render its deterministic initial sample."""

        selector = SpatialQASelector(self.loader, selection)
        self.boundary.add_qa_selector(selector, self.render)
        instance = selector.current_instance()
        self.render(instance)
        return instance

    def next_instance(self, current: SpatialCorpusInstance) -> SpatialCorpusInstance:
        return self.loader.next_instance(current, 1)

    def previous_instance(self, current: SpatialCorpusInstance) -> SpatialCorpusInstance:
        return self.loader.next_instance(current, -1)

    def variant(self, current: SpatialCorpusInstance, variant_name: str) -> SpatialCorpusInstance:
        return self.loader.variant(current, MapVariant(variant_name))

    def _draw_query(self, instance: SpatialCorpusInstance) -> PointArray:
        geometry = instance.instance.query_geometry
        if isinstance(geometry, MarkerGeometry):
            points = np.array(
                [[m.position.x_m, m.position.y_m, self._query_z_m] for m in geometry.markers],
                dtype=np.float64,
            )
            self.boundary.add_markers(
                "/agent-visible/query/markers",
                points,
                color=QUERY_COLOR,
                group=AGENT_VISIBLE_GROUP,
                point_size=0.12,
            )
            for marker, label in _marker_labels(instance, geometry):
                self.boundary.add_label(
                    f"/agent-visible/query/markers/{label}",
                    label,
                    np.array(
                        [[marker.position.x_m, marker.position.y_m, self._query_z_m + 0.02]],
                        dtype=np.float64,
                    ),
                    group=AGENT_VISIBLE_GROUP,
                )
            return points
        elif isinstance(geometry, TranslationGeometry) and isinstance(
            instance.question.contract, TranslationContract
        ):
            footprint = _footprint(geometry.start_pose, self._query_z_m)
            self._draw_pose(
                "/agent-visible/query/translation-start",
                geometry.start_pose,
                color=QUERY_COLOR,
                group=AGENT_VISIBLE_GROUP,
                z_m=self._query_z_m,
                line_width=3.0,
            )
            end = _translated_pose(geometry.start_pose, instance.question.contract.distance_m)
            sweep = _line(geometry.start_pose, end, self._query_z_m + 0.01)
            self.boundary.add_polyline(
                "/agent-visible/query/translation-sweep",
                sweep,
                color=QUERY_COLOR,
                group=AGENT_VISIBLE_GROUP,
                line_width=3.0,
            )
            return np.vstack((footprint, sweep))
        elif isinstance(geometry, RotationGeometry) and isinstance(
            instance.question.contract, RotationContract
        ):
            footprint = _footprint(geometry.pose, self._query_z_m)
            arc = _arc(
                geometry.pose, instance.question.contract.yaw_delta_rad, self._query_z_m + 0.01
            )
            self._draw_pose(
                "/agent-visible/query/rotation-pose",
                geometry.pose,
                color=QUERY_COLOR,
                group=AGENT_VISIBLE_GROUP,
                z_m=self._query_z_m,
                line_width=3.0,
            )
            self.boundary.add_polyline(
                "/agent-visible/query/rotation-arc",
                arc,
                color=QUERY_COLOR,
                group=AGENT_VISIBLE_GROUP,
                line_width=3.0,
            )
            return np.vstack((footprint, arc))
        elif geometry.kind == "pose-occupancy":
            footprint = _footprint(geometry.pose, self._query_z_m)
            self._draw_pose(
                "/agent-visible/query/pose-occupancy",
                geometry.pose,
                color=QUERY_COLOR,
                group=AGENT_VISIBLE_GROUP,
                z_m=self._query_z_m,
                line_width=3.0,
            )
            return footprint
        return np.empty((0, 3), dtype=np.float64)

    def _draw_pose(
        self,
        name: str,
        pose: Pose2D,
        *,
        color: RgbColor,
        group: str,
        z_m: float,
        line_width: float,
    ) -> None:
        self.boundary.add_polyline(
            f"{name}/footprint",
            _footprint(pose, z_m),
            color=color,
            group=group,
            line_width=line_width,
        )
        heading = _translated_pose(pose, SPATIAL_BENCHMARK_V1.footprint.side_length_m * 0.8)
        self.boundary.add_polyline(
            f"{name}/heading",
            _line(pose, heading, z_m),
            color=color,
            group=group,
            line_width=line_width,
        )

    def _draw_oracle_geometry(self, geometry: Geometry, base_z_m: float) -> None:
        for index, segment in enumerate(geometry.barrier_segments):
            position, dimensions_m, yaw_rad = _barrier_relief(segment, base_z_m)
            self.boundary.add_box(
                f"/private-oracle/relief/walls/barrier/{index}",
                position,
                dimensions_m=dimensions_m,
                yaw_rad=yaw_rad,
                color=ORACLE_WALL_COLOR,
                group=PRIVATE_ORACLE_GROUP,
                opacity=DERIVED_WALL_OPACITY,
                material="standard",
                derived=True,
                source="private-barrier-segment",
                presentation=DERIVED_RELIEF_PRESENTATION,
                base_z_m=base_z_m,
            )
        for index, opening in enumerate(geometry.openings):
            position, dimensions_m, yaw_rad = _opening_threshold_relief(opening, base_z_m)
            self.boundary.add_box(
                f"/private-oracle/relief/openings/threshold/{index}",
                position,
                dimensions_m=dimensions_m,
                yaw_rad=yaw_rad,
                color=ORACLE_OPENING_COLOR,
                group=PRIVATE_ORACLE_GROUP,
                opacity=DERIVED_THRESHOLD_OPACITY,
                material="toon3",
                derived=True,
                source="private-opening-polygon",
                presentation=DERIVED_RELIEF_PRESENTATION,
                base_z_m=base_z_m,
            )

    def _draw_oracle_topology(
        self, instance: SpatialCorpusInstance, topology: Topology, z_m: float
    ) -> None:
        centers = {room.room_id: _centroid(room.boundary) for room in topology.rooms}
        room_ids, edges = _relevant_topology(instance, topology)
        for room_id in room_ids:
            center = centers[room_id]
            self.boundary.add_markers(
                f"/private-oracle/topology/room/{room_id}",
                np.array([[center.x_m, center.y_m, z_m]], dtype=np.float64),
                color=ORACLE_TOPOLOGY_COLOR,
                group=PRIVATE_ORACLE_GROUP,
                point_size=0.05,
            )
        for edge in edges:
            first = centers[edge.first_room_id]
            second = centers[edge.second_room_id]
            self.boundary.add_polyline(
                f"/private-oracle/topology/opening/{edge.opening_id}",
                np.array(
                    [[first.x_m, first.y_m, z_m], [second.x_m, second.y_m, z_m]], dtype=np.float64
                ),
                color=ORACLE_TOPOLOGY_COLOR,
                group=PRIVATE_ORACLE_GROUP,
                line_width=0.8,
            )


def _footprint(pose: Pose2D, z_m: float) -> PointArray:
    half = (
        SPATIAL_BENCHMARK_V1.footprint.side_length_m / 2.0
        + SPATIAL_BENCHMARK_V1.footprint.safety_margin_m
    )
    corners = ((-half, -half), (half, -half), (half, half), (-half, half), (-half, -half))
    cos_yaw = math.cos(pose.yaw_rad)
    sin_yaw = math.sin(pose.yaw_rad)
    return np.array(
        [
            [pose.x_m + x * cos_yaw - y * sin_yaw, pose.y_m + x * sin_yaw + y * cos_yaw, z_m]
            for x, y in corners
        ],
        dtype=np.float64,
    )


def _translated_pose(pose: Pose2D, distance_m: float) -> Pose2D:
    return Pose2D(
        x_m=pose.x_m + distance_m * math.cos(pose.yaw_rad),
        y_m=pose.y_m + distance_m * math.sin(pose.yaw_rad),
        yaw_rad=pose.yaw_rad,
    )


def _line(start: Pose2D, end: Pose2D, z_m: float) -> PointArray:
    return np.array([[start.x_m, start.y_m, z_m], [end.x_m, end.y_m, z_m]], dtype=np.float64)


def _arc(pose: Pose2D, yaw_delta_rad: float, z_m: float) -> PointArray:
    radius = SPATIAL_BENCHMARK_V1.footprint.side_length_m * 0.75
    steps = 24
    return np.array(
        [
            [
                pose.x_m + radius * math.cos(pose.yaw_rad + yaw_delta_rad * i / steps),
                pose.y_m + radius * math.sin(pose.yaw_rad + yaw_delta_rad * i / steps),
                z_m,
            ]
            for i in range(steps + 1)
        ],
        dtype=np.float64,
    )


def _polygon(polygon: Polygon2D, z_m: float) -> PointArray:
    vertices = [*list(polygon.vertices), polygon.vertices[0]]
    return np.array([[point.x_m, point.y_m, z_m] for point in vertices], dtype=np.float64)


def _barrier_relief(
    segment: BarrierSegment, base_z_m: float
) -> tuple[PointArray, BoxDimensions, float]:
    """Turn one private 2-D barrier into a low prism starting at the shared base."""

    dx_m = segment.end.x_m - segment.start.x_m
    dy_m = segment.end.y_m - segment.start.y_m
    length_m = math.hypot(dx_m, dy_m)
    return (
        np.array(
            [
                (segment.start.x_m + segment.end.x_m) / 2.0,
                (segment.start.y_m + segment.end.y_m) / 2.0,
                base_z_m + DERIVED_WALL_HEIGHT_M / 2.0,
            ],
            dtype=np.float64,
        ),
        (length_m, DERIVED_WALL_WIDTH_M, DERIVED_WALL_HEIGHT_M),
        math.atan2(dy_m, dx_m),
    )


def _opening_threshold_relief(
    opening: Polygon2D, base_z_m: float
) -> tuple[PointArray, BoxDimensions, float]:
    """Turn a private opening footprint into a thin threshold at the shared base."""

    vertices = opening.vertices
    first, second = max(
        zip(vertices, (*vertices[1:], vertices[0]), strict=True),
        key=lambda edge: (edge[0].x_m - edge[1].x_m) ** 2 + (edge[0].y_m - edge[1].y_m) ** 2,
    )
    dx_m = second.x_m - first.x_m
    dy_m = second.y_m - first.y_m
    length_m = math.hypot(dx_m, dy_m)
    center_x = sum(point.x_m for point in vertices) / len(vertices)
    center_y = sum(point.y_m for point in vertices) / len(vertices)
    return (
        np.array(
            [center_x, center_y, base_z_m + DERIVED_THRESHOLD_HEIGHT_M / 2.0], dtype=np.float64
        ),
        (length_m, DERIVED_THRESHOLD_DEPTH_M, DERIVED_THRESHOLD_HEIGHT_M),
        math.atan2(dy_m, dx_m),
    )


def _centroid(polygon: Polygon2D) -> Point2D:
    x_m = sum(point.x_m for point in polygon.vertices) / len(polygon.vertices)
    y_m = sum(point.y_m for point in polygon.vertices) / len(polygon.vertices)
    return Point2D(x_m=x_m, y_m=y_m)


def _tuples(points: PointArray) -> tuple[tuple[float, float, float], ...]:
    return tuple((float(row[0]), float(row[1]), float(row[2])) for row in points)


def _presentation_base_z(points: PointArray) -> float:
    """Return the shared visual base without changing the observed point cloud."""

    return float(np.min(points[:, 2])) if len(points) else 0.0


def _inspector_sections(
    instance: SpatialCorpusInstance,
    show_oracle_geometry: bool,
    show_oracle_topology: bool,
) -> tuple[tuple[str, InspectorSection], ...]:
    """Build the compact native inspector sections for one review instance."""

    oracle_available = instance.oracle is not None
    private_group = PRIVATE_ORACLE_GROUP if oracle_available else "reviewer-status"
    geometry_status = _private_layer_status(oracle_available, show_oracle_geometry)
    topology_status = _private_layer_status(oracle_available, show_oracle_topology)
    return (
        (
            "/inspector/question",
            InspectorSection(
                "Question",
                AGENT_VISIBLE_GROUP,
                (
                    InspectorRow("Question", instance.question.text),
                    InspectorRow("Predicate", instance.question.predicate.value),
                    InspectorRow("Map variant", instance.instance.variant.value),
                ),
            ),
        ),
        (
            "/inspector/private-answer",
            InspectorSection(
                "Private answer",
                private_group,
                (
                    InspectorRow(
                        "Oracle truth",
                        _private_answer_value(instance),
                        "Private oracle only; not agent-visible.",
                    ),
                ),
            ),
        ),
        (
            "/inspector/evidence-key",
            InspectorSection(
                "Evidence key",
                "legend",
                (
                    InspectorRow("Public", "Gray: observed scan · Magenta: active query"),
                    InspectorRow(
                        "Private",
                        "Muted red: walls · Muted green: openings · Violet: topology",
                        "Private rows are reviewer-only.",
                    ),
                ),
            ),
        ),
        (
            "/inspector/private-relief",
            InspectorSection(
                "Private relief",
                private_group,
                (
                    InspectorRow("Source", "Derived from private 2-D oracle geometry."),
                    InspectorRow("Walls and openings", geometry_status),
                    InspectorRow("Topology", topology_status),
                ),
            ),
        ),
    )


def _private_layer_status(oracle_available: bool, visible: bool) -> str:
    if not oracle_available:
        return "Unavailable: no private oracle was loaded."
    return "Shown in this review." if visible else "Hidden in this review."


def _private_answer_value(instance: SpatialCorpusInstance) -> str:
    """Return one private oracle answer without reading public bundle records for it."""

    if instance.oracle is None:
        return "Unavailable — no private oracle loaded."
    answer = next(
        (
            candidate
            for candidate in instance.oracle.answers
            if candidate.question_id == instance.question.question_id
        ),
        None,
    )
    if answer is None:
        return "Unavailable — matching private oracle answer not loaded."
    value = answer.value.value
    display_value = "Yes" if value is True else "No" if value is False else str(value)
    return display_value


def _marker_labels(
    instance: SpatialCorpusInstance, geometry: MarkerGeometry
) -> tuple[tuple[Marker, str], ...]:
    """Use public contract order to keep marker labels stable across renderings."""

    markers = {marker.marker_id: marker for marker in geometry.markers}
    contract = instance.question.contract
    if isinstance(contract, (SameRoomContract, DirectRoomConnectionContract)):
        ids = (contract.first_marker_id, contract.second_marker_id)
    elif isinstance(contract, DirectNeighborCountContract):
        ids = (contract.marker_id,)
    else:
        ids = tuple(marker.marker_id for marker in geometry.markers)
    return tuple(
        (markers[marker_id], chr(ord("A") + index))
        for index, marker_id in enumerate(ids)
        if marker_id in markers
    )


def _relevant_topology(
    instance: SpatialCorpusInstance, topology: Topology
) -> tuple[tuple[str, ...], tuple[OpeningEdge, ...]]:
    """Return only private room-graph evidence needed by a marker topology query."""

    geometry = instance.instance.query_geometry
    if not isinstance(geometry, MarkerGeometry):
        return (), ()
    markers = {marker.marker_id: marker for marker in geometry.markers}
    contract = instance.question.contract
    if isinstance(contract, (SameRoomContract, DirectRoomConnectionContract)):
        marker_ids = (contract.first_marker_id, contract.second_marker_id)
    elif isinstance(contract, DirectNeighborCountContract):
        marker_ids = (contract.marker_id,)
    else:
        return (), ()
    room_ids = tuple(
        room_id
        for marker_id in marker_ids
        if (marker := markers.get(marker_id)) is not None
        and (room_id := _room_id_at_point(topology, marker.position)) is not None
    )
    unique_room_ids = tuple(dict.fromkeys(room_ids))
    if isinstance(contract, SameRoomContract):
        return unique_room_ids, ()
    if isinstance(contract, DirectRoomConnectionContract):
        if len(unique_room_ids) != 2:
            return unique_room_ids, ()
        target = frozenset(unique_room_ids)
        edges = tuple(
            edge
            for edge in topology.direct_openings
            if frozenset((edge.first_room_id, edge.second_room_id)) == target
        )
        return unique_room_ids, edges
    if not unique_room_ids:
        return (), ()
    source_room_id = unique_room_ids[0]
    edges = tuple(
        edge
        for edge in topology.direct_openings
        if source_room_id in (edge.first_room_id, edge.second_room_id)
    )
    neighbor_ids = tuple(
        edge.second_room_id if edge.first_room_id == source_room_id else edge.first_room_id
        for edge in edges
    )
    return tuple(dict.fromkeys((source_room_id, *neighbor_ids))), edges


def _room_id_at_point(topology: Topology, point: Point2D) -> str | None:
    for room in topology.rooms:
        if _point_in_polygon(point, room.boundary):
            return room.room_id
    return None


def _point_in_polygon(point: Point2D, polygon: Polygon2D) -> bool:
    if not _point_in_ring(point, polygon.vertices):
        return False
    return not any(_point_in_ring(point, hole) for hole in polygon.holes)


def _point_in_ring(point: Point2D, vertices: tuple[Point2D, ...]) -> bool:
    inside = False
    for first, second in zip(vertices, (*vertices[1:], vertices[0]), strict=True):
        if _point_on_segment(point, first, second):
            return True
        intersects = (first.y_m > point.y_m) != (second.y_m > point.y_m)
        if intersects:
            x_at_y = (second.x_m - first.x_m) * (point.y_m - first.y_m) / (
                second.y_m - first.y_m
            ) + first.x_m
            if point.x_m < x_at_y:
                inside = not inside
    return inside


def _point_on_segment(point: Point2D, first: Point2D, second: Point2D) -> bool:
    cross = (point.y_m - first.y_m) * (second.x_m - first.x_m) - (point.x_m - first.x_m) * (
        second.y_m - first.y_m
    )
    if not math.isclose(cross, 0.0, abs_tol=1e-9):
        return False
    return min(first.x_m, second.x_m) <= point.x_m <= max(first.x_m, second.x_m) and min(
        first.y_m, second.y_m
    ) <= point.y_m <= max(first.y_m, second.y_m)


def _qa_samples(
    loader: SpatialCorpusLoader, selection: SpatialCorpusSelection
) -> tuple[QASample, ...]:
    """Collect one clean instance per physical question in manifest-stable order."""

    samples: list[QASample] = []
    seen: set[tuple[str, str, str]] = set()
    ordinals = _split_scene_ordinals(loader)
    instances = loader.instances(selection)
    development = tuple(
        instance for instance in instances if instance.scene.split is Split.DEVELOPMENT
    )
    for instance in development or instances:
        key = _physical_question_key(instance)
        if key in seen:
            continue
        seen.add(key)
        split_name, ordinal = ordinals[instance.scene.scene_id]
        samples.append(QASample(instance, f"{split_name} {ordinal:02d} · {instance.question.text}"))
    return tuple(samples)


def _split_scene_ordinals(loader: SpatialCorpusLoader) -> dict[str, tuple[str, int]]:
    counts = {Split.DEVELOPMENT: 0, Split.HELD_OUT: 0}
    ordinals: dict[str, tuple[str, int]] = {}
    for scene in loader.manifest.scenes:
        counts[scene.split] += 1
        name = "Development" if scene.split is Split.DEVELOPMENT else "Held-out"
        ordinals[scene.scene_id] = (name, counts[scene.split])
    return ordinals


def _physical_question_key(instance: SpatialCorpusInstance) -> tuple[str, str, str]:
    return instance.scene.scene_id, instance.trajectory.trajectory_id, instance.question.question_id


def _predicate_label(predicate: Predicate) -> str:
    return predicate.value.replace("-", " ").title()


def _event_value(event: object) -> str:
    target = getattr(event, "target", None)
    return str(getattr(target, "value", ""))
