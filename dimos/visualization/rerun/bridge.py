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

"""Rerun bridge for logging pubsub messages with to_rerun() methods."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import field
from functools import lru_cache
import subprocess
import time
from typing import (
    Any,
    Literal,
    Protocol,
    TypeAlias,
    TypeGuard,
    cast,
    runtime_checkable,
)

from reactivex.disposable import Disposable
import rerun as rr
from rerun._baseclasses import Archetype
import rerun.blueprint as rrb
from rerun.blueprint import Blueprint
from toolz import pipe  # type: ignore[import-untyped]
import typer

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.msgs.sensor_msgs.PointCloud2 import register_colormap_annotation
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.protocol.pubsub.patterns import Glob, pattern_matches
from dimos.protocol.pubsub.spec import SubscribeAllCapable
from dimos.utils.logging_config import setup_logger

RERUN_GRPC_PORT = 9876
RERUN_WEB_PORT = 9090

# TODO OUT visual annotations
#
# In the future it would be nice if modules can annotate their individual OUTs with (general or rerun specific)
# hints related to their visualization
#
# so stuff like color, update frequency etc (some Image needs to be rendered on the 3d floor like occupancy grid)
# some other image is an image to be streamed into a specific 2D view etc.
#
# To achieve this we'd feed a full blueprint into the rerun bridge.
#
# rerun bridge can then inspect all transports used, all modules with their outs,
# automatically spy an all the transports and read visualization hints
#
# Temporarily we are using these "sideloading" visual_override={} dict on the bridge
# to define custom visualizations for specific topics
#
# as well as pubsubs={} to specify which protocols to listen to.

# TODO better TF processing
#
# this is rerun bridge specific, rerun has a specific (better) way of handling TFs
# using entity path conventions, each of these nodes in a path are TF frames:
#
# /world/robot1/base_link/camera/optical
#
# While here since we are just listening on TFMessage messages which optionally contain
# just a subset of full TF tree we don't know the full tree structure to build full entity
# path for a transform being published
#
# This is easy to reconstruct but a service/tf.py already does this so should be integrated here
#
# we have decoupled entity paths and actual transforms (like ROS TF frames)
# https://rerun.io/docs/concepts/logging-and-ingestion/transforms
#
# tf#/world
# tf#/base_link
# tf#/camera
#
# In order to solve this, bridge needs to own it's own tf service
# and render it's tf tree into correct rerun entity paths

logger = setup_logger()

BlueprintFactory: TypeAlias = Callable[[], "Blueprint"]

# to_rerun() can return a single archetype or a list of (entity_path, archetype) tuples
RerunMulti: TypeAlias = "list[tuple[str, Archetype]]"
RerunData: TypeAlias = "Archetype | RerunMulti"


def is_rerun_multi(data: Any) -> TypeGuard[RerunMulti]:
    """Check if data is a list of (entity_path, archetype) tuples."""
    return (
        isinstance(data, list)
        and bool(data)
        and isinstance(data[0], tuple)
        and len(data[0]) == 2
        and isinstance(data[0][0], str)
        and isinstance(data[0][1], Archetype)
    )


@runtime_checkable
class RerunConvertible(Protocol):
    """Protocol for messages that can be converted to Rerun data."""

    def to_rerun(self) -> RerunData: ...


ViewerMode = Literal["native", "web", "connect", "none"]


def _hex_to_rgba(hex_color: str) -> int:
    """Convert '#RRGGBB' to a 0xRRGGBBAA int (fully opaque)."""
    h = hex_color.lstrip("#")
    return (int(h, 16) << 8) | 0xFF


def _with_graph_tab(bp: Blueprint) -> Blueprint:
    """Add a Graph tab alongside the existing viewer layout without changing it."""

    root = bp.root_container
    return rrb.Blueprint(
        rrb.Tabs(
            root,
            rrb.GraphView(origin="blueprint", name="Graph"),
        ),
        auto_layout=bp.auto_layout,
        auto_views=bp.auto_views,
        collapse_panels=bp.collapse_panels,
    )


def _default_blueprint() -> Blueprint:
    """Default blueprint with black background and raised grid."""
    return rrb.Blueprint(
        rrb.Spatial3DView(
            origin="world",
            background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
            line_grid=rrb.LineGrid3D(
                plane=rr.components.Plane3D.XY.with_distance(0.5),
            ),
        ),
    )


# Maps global_config.viewer -> bridge viewer_mode.
# Evaluated at blueprint construction time (main process), not in start() (worker process).
_BACKEND_TO_MODE: dict[str, ViewerMode] = {
    "rerun": "native",
    "rerun-web": "web",
    "rerun-connect": "connect",
    "none": "none",
}


def _resolve_viewer_mode() -> ViewerMode:
    from dimos.core.global_config import global_config

    return _BACKEND_TO_MODE.get(global_config.viewer, "native")


class Config(ModuleConfig):
    """Configuration for RerunBridgeModule."""

    pubsubs: list[SubscribeAllCapable[Any, Any]] = field(default_factory=lambda: [LCM()])

    visual_override: dict[Glob | str, Callable[[Any], Archetype]] = field(default_factory=dict)

    # Static items logged once after start. Maps entity_path -> callable(rr) returning Archetype
    static: dict[str, Callable[[Any], Archetype]] = field(default_factory=dict)

    # Per-entity max update rate (Hz). Entities not listed are unthrottled.
    # Use for heavy entities to prevent viewer backpressure.
    max_hz: dict[str, float] = field(default_factory=dict)

    entity_prefix: str = "world"
    topic_to_entity: Callable[[Any], str] | None = None
    viewer_mode: ViewerMode = field(default_factory=_resolve_viewer_mode)
    connect_url: str = "rerun+http://127.0.0.1:9877/proxy"
    memory_limit: str = "25%"

    # Blueprint factory: callable(rrb) -> Blueprint for viewer layout configuration
    # Set to None to disable default blueprint
    blueprint: BlueprintFactory | None = _default_blueprint


class RerunBridgeModule(Module):
    """Bridge that logs messages from pubsubs to Rerun.

    Spawns its own Rerun viewer and subscribes to all topics on each provided
    pubsub. Any message that has a to_rerun() method is automatically logged.

    Example:
        from dimos.protocol.pubsub.impl.lcmpubsub import LCM

        lcm = LCM()
        bridge = RerunBridgeModule(pubsubs=[lcm])
        bridge.start()
        # All messages with to_rerun() are now logged to Rerun
        bridge.stop()
    """

    config: Config

    GV_SCALE = 100.0  # graphviz inches to rerun screen units
    MODULE_RADIUS = 30.0
    CHANNEL_RADIUS = 20.0

    @lru_cache(maxsize=256)
    def _visual_override_for_entity_path(
        self, entity_path: str
    ) -> Callable[[Any], RerunData | None]:
        """Return a composed visual override for the entity path.

        Chains matching overrides from config, ending with final_convert
        which handles .to_rerun() or passes through Archetypes.
        """
        # find all matching converters for this entity path
        matches = [
            fn
            for pattern, fn in self.config.visual_override.items()
            if pattern_matches(pattern, entity_path)
        ]

        # None means "suppress this topic entirely"
        if any(fn is None for fn in matches):
            return lambda msg: None

        # final step (ensures we return Archetype or None)
        def final_convert(msg: Any) -> RerunData | None:
            if isinstance(msg, Archetype):
                return msg
            if is_rerun_multi(msg):
                return msg
            if isinstance(msg, RerunConvertible):
                return msg.to_rerun()
            return None

        # compose all converters
        return lambda msg: pipe(msg, *matches, final_convert)

    def _get_entity_path(self, topic: Any) -> str:
        """Convert a topic to a Rerun entity path."""
        if self.config.topic_to_entity:
            return self.config.topic_to_entity(topic)

        # Default: use topic.name if available (LCM Topic), else str
        topic_str = getattr(topic, "name", None) or str(topic)
        # Strip everything after # (LCM topic suffix)
        topic_str = topic_str.split("#")[0]
        return f"{self.config.entity_prefix}{topic_str}"

    def _on_message(self, msg: Any, topic: Any) -> None:
        """Handle incoming message - log to rerun."""

        entity_path: str = self._get_entity_path(topic)

        # Throttle entities with a max_hz limit
        if entity_path in self._min_intervals:
            now = time.monotonic()
            if now - self._last_log.get(entity_path, 0.0) < self._min_intervals[entity_path]:
                return
            self._last_log[entity_path] = now

        # apply visual overrides (including final_convert which handles .to_rerun())
        rerun_data: RerunData | None = self._visual_override_for_entity_path(entity_path)(msg)

        if not rerun_data:
            return

        # TFMessage for example returns list of (entity_path, archetype) tuples
        if is_rerun_multi(rerun_data):
            for path, archetype in rerun_data:
                rr.log(path, archetype)
        else:
            rr.log(entity_path, cast("Archetype", rerun_data))

    @rpc
    def start(self) -> None:
        super().start()

        logger.info("Rerun bridge starting", viewer_mode=self.config.viewer_mode)

        # Build throttle lookup: entity_path → min interval in seconds
        self._last_log: dict[str, float] = {}
        self._min_intervals: dict[str, float] = {
            entity: 1.0 / hz for entity, hz in self.config.max_hz.items() if hz > 0
        }

        # Initialize and spawn Rerun viewer
        rr.init("dimos")

        if self.config.viewer_mode == "native":
            try:
                import rerun_bindings

                rerun_bindings.spawn(
                    port=RERUN_GRPC_PORT,
                    executable_name="dimos-viewer",
                    memory_limit=self.config.memory_limit,
                )
            except ImportError:
                pass  # dimos-viewer not installed
            except Exception:
                logger.warning(
                    "dimos-viewer found but failed to spawn, falling back to stock rerun",
                    exc_info=True,
                )
            rr.spawn(connect=True, memory_limit=self.config.memory_limit)
        elif self.config.viewer_mode == "web":
            server_uri = rr.serve_grpc()
            rr.serve_web_viewer(connect_to=server_uri, open_browser=False)
        elif self.config.viewer_mode == "connect":
            rr.connect_grpc(self.config.connect_url)
        # "none" - just init, no viewer (connect externally)

        if self.config.blueprint:
            rr.send_blueprint(_with_graph_tab(self.config.blueprint()))

        # Register colormap for viewer-side color resolution (PointCloud2 class_ids)
        register_colormap_annotation("turbo")

        # Start pubsubs and subscribe to all messages
        for pubsub in self.config.pubsubs:
            logger.info(f"bridge listening on {pubsub.__class__.__name__}")
            if hasattr(pubsub, "start"):
                pubsub.start()
            unsub = pubsub.subscribe_all(self._on_message)
            self.register_disposable(Disposable(unsub))

        # Add pubsub stop as disposable
        for pubsub in self.config.pubsubs:
            if hasattr(pubsub, "stop"):
                self.register_disposable(Disposable(pubsub.stop))  # type: ignore[union-attr]

        self._log_static()

    def _log_static(self) -> None:
        for entity_path, factory in self.config.static.items():
            data = factory(rr)
            if isinstance(data, list):
                for archetype in data:
                    rr.log(entity_path, archetype, static=True)
            else:
                rr.log(entity_path, data, static=True)

    @rpc
    def log_blueprint_graph(self, dot_code: str, module_names: list[str]) -> None:
        """Log a blueprint module graph from a Graphviz DOT string.

        Runs ``dot -Tplain`` to compute positions, then logs
        ``rr.GraphNodes`` + ``rr.GraphEdges`` to the active recording.

        Args:
            dot_code: The DOT-format graph (from ``introspection.blueprint.dot.render``).
            module_names: List of module class names (to distinguish modules from channels).
        """

        try:
            result = subprocess.run(
                ["dot", "-Tplain"], input=dot_code, text=True, capture_output=True, timeout=30
            )
        except (FileNotFoundError, subprocess.TimeoutExpired):
            return
        if result.returncode != 0:
            return

        node_ids: list[str] = []
        node_labels: list[str] = []
        node_colors: list[int] = []
        positions: list[tuple[float, float]] = []
        radii: list[float] = []
        edges: list[tuple[str, str]] = []
        module_set = set(module_names)

        for line in result.stdout.splitlines():
            if line.startswith("node "):
                parts = line.split()
                node_id = parts[1].strip('"')
                x = float(parts[2]) * self.GV_SCALE
                y = -float(parts[3]) * self.GV_SCALE
                label = parts[6].strip('"')
                color = parts[9].strip('"')

                node_ids.append(node_id)
                node_labels.append(label)
                positions.append((x, y))
                node_colors.append(_hex_to_rgba(color))
                radii.append(self.MODULE_RADIUS if node_id in module_set else self.CHANNEL_RADIUS)

            elif line.startswith("edge "):
                parts = line.split()
                edges.append((parts[1].strip('"'), parts[2].strip('"')))

        if not node_ids:
            return

        rr.log(
            "blueprint",
            rr.GraphNodes(
                node_ids=node_ids,
                labels=node_labels,
                colors=node_colors,
                positions=positions,
                radii=radii,
                show_labels=True,
            ),
            rr.GraphEdges(edges=edges, graph_type="directed"),
            static=True,
        )

    @rpc
    def stop(self) -> None:
        super().stop()


def run_bridge(
    viewer_mode: str = "native",
    memory_limit: str = "25%",
) -> None:
    """Start a RerunBridgeModule with default LCM config and block until interrupted."""
    import signal

    from dimos.protocol.service.lcmservice import autoconf

    autoconf(check_only=True)

    bridge = RerunBridgeModule(
        viewer_mode=viewer_mode,
        memory_limit=memory_limit,
        # any pubsub that supports subscribe_all and topic that supports str(topic)
        # is acceptable here
        pubsubs=[LCM()],
    )

    bridge.start()

    signal.signal(signal.SIGINT, lambda *_: bridge.stop())
    signal.pause()


app = typer.Typer()


@app.command()
def cli(
    viewer_mode: str = typer.Option(
        "native", help="Viewer mode: native (desktop), web (browser), none (headless)"
    ),
    memory_limit: str = typer.Option(
        "25%", help="Memory limit for Rerun viewer (e.g., '4GB', '16GB', '25%')"
    ),
) -> None:
    """Rerun bridge for LCM messages."""
    run_bridge(viewer_mode=viewer_mode, memory_limit=memory_limit)


if __name__ == "__main__":
    app()
