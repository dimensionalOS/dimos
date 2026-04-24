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
import subprocess
import time
from typing import (
    TYPE_CHECKING,
    Any,
    Protocol,
    TypeAlias,
    TypeGuard,
    cast,
    get_args,
    runtime_checkable,
)

from reactivex.disposable import Disposable
from toolz import pipe  # type: ignore[import-untyped]

if TYPE_CHECKING:
    from rerun._baseclasses import Archetype
    from rerun.blueprint import Blueprint

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.msgs.sensor_msgs.PointCloud2 import register_colormap_annotation
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.protocol.pubsub.patterns import Glob, pattern_matches
from dimos.protocol.pubsub.spec import SubscribeAllCapable
from dimos.utils.logging_config import setup_logger
from dimos.visualization.rerun.constants import (
    RERUN_ENABLE_WEB,
    RERUN_GRPC_PORT,
    RERUN_OPEN_DEFAULT,
    RERUN_WEB_PORT,
    RerunOpenOption,
)

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
    from rerun._baseclasses import Archetype

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


def _hex_to_rgba(hex_color: str) -> int:
    """Convert '#RRGGBB' to a 0xRRGGBBAA int (fully opaque)."""
    h = hex_color.lstrip("#")
    if len(h) == 6:
        return int(h + "ff", 16)
    return int(h[:8], 16)


def _with_graph_tab(bp: Blueprint) -> Blueprint:
    """Add a Graph tab alongside the existing viewer layout without changing it."""
    import rerun.blueprint as rrb

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
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Spatial3DView(
            origin="world",
            background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
            line_grid=rrb.LineGrid3D(
                plane=rr.components.Plane3D.XY.with_distance(0.5),
            ),
        ),
    )


class Config(ModuleConfig):
    """Configuration for RerunBridgeModule."""

    pubsubs: list[SubscribeAllCapable[Any, Any]] = field(default_factory=lambda: [LCM()])

    visual_override: dict[Glob | str, Callable[[Any], Archetype]] = field(default_factory=dict)
    static: dict[str, Callable[[Any], Archetype]] = field(default_factory=dict)
    max_hz: dict[str, float] = field(default_factory=dict)

    entity_prefix: str = "world"
    topic_to_entity: Callable[[Any], str] | None = None
    connect_url: str = "rerun+http://127.0.0.1:9877/proxy"
    memory_limit: str = "25%"
    rerun_open: RerunOpenOption = RERUN_OPEN_DEFAULT
    rerun_web: bool = RERUN_ENABLE_WEB
    web_port: int = RERUN_WEB_PORT
    blueprint: BlueprintFactory | None = _default_blueprint


def _rebuild_config() -> None:
    from rerun._baseclasses import Archetype
    from rerun.blueprint import Blueprint

    Config.model_rebuild(_types_namespace={"Archetype": Archetype, "Blueprint": Blueprint})


_rebuild_config()


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
    _last_log: dict[str, float]

    # Graphviz layout scale and node radii for blueprint graph
    GV_SCALE = 100.0
    MODULE_RADIUS = 20.0
    CHANNEL_RADIUS = 12.0

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._last_log = {}
        self._override_cache: dict[str, Callable[[Any], RerunData | None]] = {}

    def _visual_override_for_entity_path(
        self, entity_path: str
    ) -> Callable[[Any], RerunData | None]:
        """Return a composed visual override for the entity path.

        Chains matching overrides from config, ending with final_convert
        which handles .to_rerun() or passes through Archetypes. Cached per
        instance (not via ``lru_cache`` on a method, which would leak ``self``).
        """
        cached = self._override_cache.get(entity_path)
        if cached is not None:
            return cached

        # find all matching converters for this entity path
        matches = [
            fn
            for pattern, fn in self.config.visual_override.items()
            if pattern_matches(pattern, entity_path)
        ]

        # None means "suppress this topic entirely"
        if any(fn is None for fn in matches):
            result: Callable[[Any], RerunData | None] = lambda msg: None  # noqa: E731
            self._override_cache[entity_path] = result
            return result

        # final step (ensures we return Archetype or None)
        def final_convert(msg: Any) -> RerunData | None:
            from rerun._baseclasses import Archetype

            if isinstance(msg, Archetype):
                return msg
            if is_rerun_multi(msg):
                return msg
            if isinstance(msg, RerunConvertible):
                return msg.to_rerun()
            return None

        # compose all converters
        composed: Callable[[Any], RerunData | None] = lambda msg: pipe(  # noqa: E731
            msg, *matches, final_convert
        )
        self._override_cache[entity_path] = composed
        return composed

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
        import rerun as rr

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
        import socket
        from urllib.parse import urlparse

        import rerun as rr

        super().start()

        logger.info("Rerun bridge starting")

        # Build throttle lookup: entity_path → min interval in seconds
        self._last_log = {}
        self._min_intervals: dict[str, float] = {
            entity: 1.0 / hz for entity, hz in self.config.max_hz.items() if hz > 0
        }

        # Initialize and spawn Rerun viewer
        rr.init("dimos")

        # start grpc if needed
        # If the port is already in use (another instance running), connect

        parsed = urlparse(self.config.connect_url.replace("rerun+", "", 1))
        grpc_port = parsed.port or RERUN_GRPC_PORT

        port_in_use = False
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            port_in_use = sock.connect_ex(("127.0.0.1", grpc_port)) == 0

        if port_in_use:
            logger.info(f"gRPC port {grpc_port} already in use, connecting to existing server")
            rr.connect_grpc(url=self.config.connect_url)
            server_uri = self.config.connect_url
        else:
            server_uri = rr.serve_grpc(
                grpc_port=grpc_port,
                server_memory_limit=self.config.memory_limit,
            )
            logger.info(f"Rerun gRPC server ready at {server_uri}")

        # Check open arg
        if self.config.rerun_open not in get_args(RerunOpenOption):
            logger.warning(
                f"rerun_open was {self.config.rerun_open} which is not one of "
                f"{get_args(RerunOpenOption)}"
            )

        # launch native viewer if desired
        spawned = False
        if self.config.rerun_open in ("native", "both"):
            try:
                import rerun_bindings

                # Use --connect so the viewer connects to the bridge's gRPC
                # server rather than starting its own (which would conflict).
                rerun_bindings.spawn(
                    executable_name="dimos-viewer",
                    memory_limit=self.config.memory_limit,
                    extra_args=["--connect", server_uri],
                )
                spawned = True
            except ImportError:
                pass  # dimos-viewer not installed
            except Exception:
                logger.warning(
                    "dimos-viewer found but failed to spawn, falling back to stock rerun",
                    exc_info=True,
                )

            # fallback on normal (non-dimos-viewer) rerun
            if not spawned:
                try:
                    rr.spawn(connect=True, memory_limit=self.config.memory_limit)
                    spawned = True
                except (RuntimeError, FileNotFoundError):
                    logger.warning(
                        "Rerun native viewer not available (headless?). "
                        "Bridge will continue without a viewer — data is still "
                        "accessible via rerun-connect or rerun-web.",
                        exc_info=True,
                    )

        # web
        open_web = self.config.rerun_open == "web" or self.config.rerun_open == "both"
        if open_web or self.config.rerun_web:
            rr.serve_web_viewer(
                connect_to=server_uri,
                open_browser=open_web,
                web_port=self.config.web_port,
            )

        # printout
        if self.config.rerun_open == "none" or (self.config.rerun_open == "native" and not spawned):
            self._log_connect_hints(grpc_port)

        # setup blueprint
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

    def _log_connect_hints(self, grpc_port: int) -> None:
        """Log CLI commands for connecting a viewer to this bridge."""
        import socket

        from dimos.utils.generic import get_local_ips

        local_ips = get_local_ips()
        hostname = socket.gethostname()
        connect_url = f"rerun+http://127.0.0.1:{grpc_port}/proxy"

        lines = [
            "",
            "=" * 60,
            "Rerun gRPC server running (no viewer opened)",
            "",
            "Connect a viewer:",
            f"  dimos-viewer --connect {connect_url}",
        ]
        for ip, iface in local_ips:
            lines.append(f"  dimos-viewer --connect rerun+http://{ip}:{grpc_port}/proxy  # {iface}")
        lines.append("")
        lines.append(f"  hostname: {hostname}")
        lines.append("=" * 60)
        lines.append("")

        logger.info("\n".join(lines))

    def _log_static(self) -> None:
        import rerun as rr

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
        import rerun as rr

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
        self._override_cache.clear()
        super().stop()
