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
import signal
import socket
import subprocess
import sys
import threading
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
from urllib.parse import urlparse

from reactivex.disposable import Disposable
from toolz import pipe  # type: ignore[import-untyped]

from dimos.core.core import rpc
from dimos.core.global_config import global_config
from dimos.core.module import Module, ModuleConfig
from dimos.core.transport_factory import transport_topic
from dimos.msgs.helpers import resolve_msg_type
from dimos.msgs.tf2_msgs.TFMessage import TfFrameTree, TFMessage
from dimos.protocol.pubsub.impl.lcmpubsub import LCM, Topic
from dimos.protocol.pubsub.impl.zenohpubsub import Zenoh
from dimos.protocol.pubsub.patterns import Glob, pattern_matches
from dimos.protocol.pubsub.spec import SubscribeAllCapable
from dimos.protocol.service.lcmservice import autoconf
from dimos.utils.generic import get_local_ips
from dimos.utils.logging_config import setup_logger
from dimos.visualization.rerun.constants import (
    RERUN_ENABLE_WEB,
    RERUN_GRPC_PORT,
    RERUN_OPEN_DEFAULT,
    RERUN_WEB_VIEWER_PORT,
    RerunOpenOption,
)
from dimos.visualization.rerun.init import rerun_init

if TYPE_CHECKING:
    from rerun._baseclasses import Archetype
    from rerun.blueprint import Blueprint

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

logger = setup_logger()

RerunMulti: TypeAlias = "list[tuple[str, Archetype]]"
RerunData: TypeAlias = "Archetype | RerunMulti"


class _LatestOnlyDispatcher:
    """Drain at most the newest pending message for each topic."""

    def __init__(
        self,
        callback: Callable[[Any, Any], None],
        min_interval: Callable[[Any], float] | None = None,
    ) -> None:
        self._callback = callback
        self._min_interval = min_interval or (lambda topic: 0.0)
        self._latest: dict[str, tuple[Any, Any]] = {}
        self._last_dispatch: dict[str, float] = {}
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._drain,
            name="rerun-latest-only",
            daemon=True,
        )
        self._thread.start()

    def submit(self, msg: Any, topic: Any) -> None:
        with self._lock:
            self._latest[str(topic)] = (msg, topic)
        self._wake.set()

    def stop(self) -> None:
        self._stop.set()
        self._wake.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=2.0)
        self._thread = None
        with self._lock:
            self._latest.clear()
            self._last_dispatch.clear()

    def _drain(self) -> None:
        while True:
            if self._stop.is_set():
                return
            with self._lock:
                now = time.monotonic()
                ready_keys = [
                    key
                    for key, (_, topic) in self._latest.items()
                    if now - self._last_dispatch.get(key, 0.0) >= self._min_interval(topic)
                ]
                batch = [(key, self._latest.pop(key)) for key in ready_keys]
                due_times = [
                    self._last_dispatch.get(key, 0.0) + self._min_interval(topic)
                    for key, (_, topic) in self._latest.items()
                ]

            if batch:
                for key, (msg, topic) in batch:
                    self._last_dispatch[key] = time.monotonic()
                    try:
                        self._callback(msg, topic)
                    except Exception:
                        logger.error("Error in latest-only Rerun callback", exc_info=True)
                continue

            timeout = None
            if due_times:
                timeout = max(0.0, min(due_times) - time.monotonic())
            self._wake.wait(timeout)
            self._wake.clear()
            if self._stop.is_set():
                return


def _pubsub_topic(
    pubsub: Any,
    name: str,
    msg_name: str,
    *,
    latest_only: bool,
) -> Topic:
    msg_type = resolve_msg_type(msg_name)
    if msg_type is None:
        raise ValueError(f"Unknown Rerun topic message type {msg_name!r} for {name!r}")

    pubsub_config = getattr(pubsub, "config", None)
    if pubsub_config is None:
        raise TypeError(f"Rerun pubsub for {name!r} has no transport config")
    backend = getattr(pubsub_config, "transport", None)
    topic_name = transport_topic(name, pubsub_config)
    queue_capacity = 1 if latest_only else 10000
    if backend == "zenoh":
        from dimos.protocol.pubsub.impl.zenohpubsub import Topic as ZenohTopic

        return ZenohTopic(
            topic=topic_name,
            lcm_type=msg_type,
            queue_capacity=queue_capacity,
        )
    return Topic(
        topic=topic_name,
        lcm_type=msg_type,
        queue_capacity=queue_capacity,
    )


def _subscribe_topics(
    pubsub: Any,
    topics: dict[str, str] | None,
    callback: Callable[[Any, Any], None],
    *,
    latest_only: bool,
) -> list[Callable[[], None]]:
    if topics is None:
        return [pubsub.subscribe_all(callback)]
    return [
        pubsub.subscribe(
            _pubsub_topic(pubsub, name, msg_name, latest_only=latest_only),
            callback,
        )
        for name, msg_name in topics.items()
    ]

if TYPE_CHECKING:
    BlueprintFactory: TypeAlias = Callable[[], "Blueprint"]
    VisualOverride: TypeAlias = Callable[[Any], "Archetype"]
else:
    # Pydantic evaluates Config's annotations at runtime, so keep rerun types
    # out of them - importing rerun here would defeat the lazy import below.
    BlueprintFactory = VisualOverride = Callable[..., Any]


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


def _default_pubsubs(config: Any = None) -> list[SubscribeAllCapable[Any, Any]]:
    """Select the pubsub backend matching the active transport.

    All channels including TF flow over the active transport, so the bridge
    listens only on that backend. To also bridge external LCM publishers while
    running Zenoh, pass an explicit ``pubsubs=[Zenoh(), LCM()]``.
    """
    transport = getattr(config, "transport", None) or global_config.transport
    if transport == "zenoh":
        return [Zenoh()]
    return [LCM()]


def _resolve_pubsubs(config: Any) -> list[SubscribeAllCapable[Any, Any]]:
    """Return explicit pubsubs when truly overridden, else transport defaults.

    Older blueprints commonly passed ``pubsubs=[LCM()]`` as the effective
    default. Preserve the newer transport-driven behavior for that legacy
    value, but honor explicit non-default overrides such as custom backends.
    """
    fields_set: set[str] = cast("set[str]", getattr(config, "model_fields_set", set()))
    pubsubs = cast(
        "list[SubscribeAllCapable[Any, Any]] | None",
        getattr(config, "pubsubs", None),
    )
    if "pubsubs" in fields_set and pubsubs is not None:
        is_legacy_default = len(pubsubs) == 1 and isinstance(pubsubs[0], LCM)
        if not is_legacy_default:
            return pubsubs
    return _default_pubsubs(getattr(config, "g", config))


class Config(ModuleConfig):
    """Configuration for RerunBridgeModule.

    ``topics`` maps logical channel names to DimOS message type names. When it
    is set, the bridge subscribes only to those typed channels; ``None`` keeps
    the general-purpose all-topic bridge behavior. ``latest_only`` bounds each
    configured channel to its newest pending message before Rerun conversion.

    The pubsubs field is accepted for backwards compatibility. The legacy
    ``[LCM()]`` value is treated as the old default and replaced by the
    transport-driven runtime default. Explicit non-default overrides are still
    honored.
    """

    pubsubs: list[SubscribeAllCapable[Any, Any]] = field(default_factory=lambda: [LCM()])
    topics: dict[str, str] | None = None
    latest_only: bool = False

    visual_override: dict[Glob | str, VisualOverride | None] = field(default_factory=dict)
    static: dict[str, Callable[[Any], Any]] = field(default_factory=dict)
    max_hz: dict[str, float] = field(default_factory=dict)

    entity_prefix: str = "world"
    # Length of the triads to draw
    tf_axes: float = 0.0
    topic_to_entity: Callable[[Any], str] | None = None
    connect_url: str | None = None
    memory_limit: str = "25%"
    newest_first: bool = False
    rerun_open: RerunOpenOption = RERUN_OPEN_DEFAULT
    rerun_web: bool = RERUN_ENABLE_WEB
    web_port: int = RERUN_WEB_VIEWER_PORT
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
    dedicated_worker = True
    _last_log: dict[str, float]

    # TODO this doesn't belong here, either hardcode it or put it to rerun bridge config
    GRAPH_VIZ_SCALE = 100.0
    MODULE_RADIUS = 20.0
    CHANNEL_RADIUS = 12.0

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._last_log = {}
        self._override_cache: dict[str, Callable[[Any], RerunData | None]] = {}
        self._frame_attached: dict[str, str] = {}
        self._tf_lock = threading.Lock()
        self._tf_tree = self._new_tf_tree()

    def _new_tf_tree(self) -> TfFrameTree | None:
        if self.config.tf_axes <= 0:
            return None
        return TfFrameTree(
            axis_length=self.config.tf_axes,
            root=f"{self.config.entity_prefix}/tf",
        )

    @property
    def host(self) -> str:
        return self.config.g.rerun_host or self.config.g.listen_host

    def _visual_override_for_entity_path(
        self, entity_path: str
    ) -> Callable[[Any], RerunData | None]:
        """Return a composed visual override for the entity path.

        Chains matching overrides from config, ending with final_convert
        which handles .to_rerun() or passes through Archetypes. Cached per
        instance (not via ``lru_cache`` on a method, which would leak ``self``).
        """
        from rerun._baseclasses import Archetype

        cached = self._override_cache.get(entity_path)
        if cached is not None:
            return cached

        matches = [
            fn
            for pattern, fn in self.config.visual_override.items()
            if pattern_matches(pattern, entity_path)
        ]

        # None means "suppress this topic entirely"
        if any(fn is None for fn in matches):

            def suppressed(msg: Any) -> RerunData | None:
                return None

            self._override_cache[entity_path] = suppressed
            return suppressed

        def final_convert(msg: Any) -> RerunData | None:
            if isinstance(msg, Archetype):
                return msg
            if is_rerun_multi(msg):
                return msg
            if isinstance(msg, RerunConvertible):
                return msg.to_rerun()
            return None

        # compose all converters
        def composed(msg: Any) -> RerunData | None:
            return cast("RerunData | None", pipe(msg, *matches, final_convert))

        self._override_cache[entity_path] = composed
        return composed

    def _get_entity_path(self, topic: Any) -> str:
        if self.config.topic_to_entity:
            return self.config.topic_to_entity(topic)

        topic_str = getattr(topic, "name", None) or str(topic)
        # Strip type suffix: LCM uses '#type', Zenoh embeds type as '/type' in key expr
        # but _key_expr_to_topic already parsed it into topic.topic, so use that.
        raw = getattr(topic, "topic", topic_str)
        if isinstance(raw, str):
            topic_str = raw
        topic_str = topic_str.split("#")[0]
        # Strip Zenoh key prefix (dimos/) to match LCM entity paths
        if topic_str.startswith("dimos/"):
            topic_str = "/" + topic_str.removeprefix("dimos/")
        return f"{self.config.entity_prefix}{topic_str}"

    def _on_message(self, msg: Any, topic: Any, *, throttle: bool = True) -> None:
        """Handle incoming message - log to rerun."""
        import rerun as rr

        entity_path: str = self._get_entity_path(topic)

        # Throttle entities with a max_hz limit
        if throttle and entity_path in self._min_intervals:
            now = time.monotonic()
            if now - self._last_log.get(entity_path, 0.0) < self._min_intervals[entity_path]:
                return
            self._last_log[entity_path] = now

        if self._tf_tree is not None and isinstance(msg, TFMessage):
            with self._tf_lock:
                for path, archetype in msg.to_rerun(self._tf_tree):
                    rr.log(path, archetype)
            return

        rerun_data: RerunData | None = self._visual_override_for_entity_path(entity_path)(msg)

        if not rerun_data:
            return

        # TFMessage for example returns list of (entity_path, archetype) tuples
        if is_rerun_multi(rerun_data):
            for path, archetype in rerun_data:
                rr.log(path, archetype)
        else:
            rr.log(entity_path, cast("Archetype", rerun_data))
            # if source msg carries a frame_id, attach the entity to that TF frame
            # should skip if archetype is a Transform3D
            if not isinstance(rerun_data, rr.Transform3D):
                frame_id = getattr(msg, "frame_id", None)
                if frame_id and self._frame_attached.get(entity_path) != frame_id:
                    rr.log(entity_path, rr.Transform3D(parent_frame=f"tf#/{frame_id}"))
                    self._frame_attached[entity_path] = frame_id

    @rpc
    def start(self) -> None:
        import rerun as rr

        super().start()

        logger.info("Rerun bridge starting")

        self._last_log = {}
        self._frame_attached = {}
        self._tf_tree = self._new_tf_tree()
        self._min_intervals: dict[str, float] = {
            entity: 1.0 / hz for entity, hz in self.config.max_hz.items() if hz > 0
        }

        connect_url = self.config.connect_url
        if connect_url is None:
            connect_url = f"rerun+http://{self.host}:{RERUN_GRPC_PORT}/proxy"

        server_uri = rerun_init(
            start_grpc=True,
            grpc_config={
                "connect_url": connect_url,
                "server_memory_limit": self.config.memory_limit,
                "newest_first": self.config.newest_first,
            },
        )
        assert server_uri is not None  # start_grpc=True guarantees a URI

        parsed = urlparse(connect_url.replace("rerun+", "", 1))
        grpc_port = parsed.port or RERUN_GRPC_PORT

        if self.config.rerun_open not in get_args(RerunOpenOption):
            logger.warning(
                f"rerun_open was {self.config.rerun_open} which is not one of "
                f"{get_args(RerunOpenOption)}"
            )

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
                        "accessible via --rerun-open web or by connecting a viewer to the gRPC server.",
                        exc_info=True,
                    )

        open_web = self.config.rerun_open == "web" or self.config.rerun_open == "both"
        if open_web or self.config.rerun_web:
            rr.serve_web_viewer(
                connect_to=server_uri,
                open_browser=open_web,
                web_port=self.config.web_port,
            )

        # TODO: `spawned` is supposed to be false when run on the G1 (because viewer doesn't have a display) somehow it returns true
        if (
            self.config.rerun_open == "none"
            or (self.config.rerun_open == "native" and not spawned)
            or self.host == "0.0.0.0"
        ):
            self._log_connect_hints(grpc_port)

        if self.config.blueprint:
            rr.send_blueprint(_with_graph_tab(self.config.blueprint()))

        # Resolve pubsubs lazily — the module-level global_config singleton in worker
        # processes doesn't have CLI overrides. Use self.config.g which is the parent's
        # updated config, passed via the worker kwargs.
        pubsubs = _resolve_pubsubs(self.config)

        dispatcher: _LatestOnlyDispatcher | None = None
        callback: Callable[[Any, Any], None] = self._on_message
        if self.config.latest_only:
            def log_latest(msg: Any, topic: Any) -> None:
                self._on_message(msg, topic, throttle=False)

            dispatcher = _LatestOnlyDispatcher(
                log_latest,
                min_interval=lambda topic: self._min_intervals.get(
                    self._get_entity_path(topic), 0.0
                ),
            )
            dispatcher.start()
            callback = dispatcher.submit

        # Start pubsubs and subscribe to either configured topics or all messages.
        for pubsub in pubsubs:
            logger.info(f"bridge listening on {pubsub.__class__.__name__}")
            if hasattr(pubsub, "start"):
                pubsub.start()
            for unsubscribe in _subscribe_topics(
                pubsub,
                self.config.topics,
                callback,
                latest_only=self.config.latest_only,
            ):
                self.register_disposable(Disposable(unsubscribe))

        # Add pubsub stop as disposable
        for pubsub in pubsubs:
            if hasattr(pubsub, "stop"):
                self.register_disposable(Disposable(pubsub.stop))  # type: ignore[union-attr]

        if dispatcher is not None:
            self.register_disposable(Disposable(dispatcher.stop))

        self._log_static()

    def _log_connect_hints(self, grpc_port: int) -> None:
        """Log CLI commands for connecting a viewer to this bridge."""
        local_ips = get_local_ips()
        local_grpc = f"rerun+http://{self.host}:{grpc_port}/proxy"
        local_ws = f"ws://{self.host}:{self.config.g.rerun_websocket_server_port}/ws"
        hostname = socket.gethostname()

        columns = 60
        lines = [
            "",
            "=" * columns,
            "Rerun gRPC server running (no viewer opened)",
            "",
            "Connect a viewer:",
            f"  dimos-viewer --connect {local_grpc} --ws-url {local_ws}",
        ]
        for ip, iface in local_ips:
            remote_grpc = f"rerun+http://{ip}:{grpc_port}/proxy"
            remote_ws = f"ws://{ip}:{self.config.g.rerun_websocket_server_port}/ws"
            lines.append(f"  dimos-viewer --connect {remote_grpc} --ws-url {remote_ws}  # {iface}")
        lines.append("")
        lines.append(f"  hostname: {hostname}")
        lines.append("=" * columns)
        lines.append("")

        logger.info("\n".join(lines))

    def _log_static(self) -> None:
        import rerun as rr

        for entity_path, factory in self.config.static.items():
            data = factory(rr)
            if is_rerun_multi(data):
                logger.info(
                    "Rerun static entity",
                    entity_path=entity_path,
                    archetypes=[type(archetype).__name__ for _, archetype in data],
                )
                for path, archetype in data:
                    rr.log(path, archetype, static=True)
                continue

            if isinstance(data, list):
                logger.info(
                    "Rerun static entity",
                    entity_path=entity_path,
                    archetypes=[type(archetype).__name__ for archetype in data],
                )
                for archetype in data:
                    rr.log(entity_path, archetype, static=True)
            else:
                logger.info(
                    "Rerun static entity",
                    entity_path=entity_path,
                    archetypes=[type(data).__name__],
                )
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
                x = float(parts[2]) * self.GRAPH_VIZ_SCALE
                y = -float(parts[3]) * self.GRAPH_VIZ_SCALE
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
        self._frame_attached.clear()
        self._tf_tree = None
        super().stop()


def run_bridge(
    memory_limit: str = "25%",
    rerun_open: RerunOpenOption = RERUN_OPEN_DEFAULT,
    rerun_web: bool = RERUN_ENABLE_WEB,
) -> None:
    """Start a RerunBridgeModule with default LCM config and block until interrupted."""
    autoconf(check_only=True)

    bridge = RerunBridgeModule(
        memory_limit=memory_limit,
        rerun_open=rerun_open,
        rerun_web=rerun_web,
    )
    bridge.start()

    def _shutdown(*_: object) -> None:
        bridge.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown)
    signal.pause()
