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

from dataclasses import dataclass, field
from functools import lru_cache
from typing import (
    TYPE_CHECKING,
    Any,
    Literal,
    Protocol,
    TypeAlias,
    TypeGuard,
    cast,
    runtime_checkable,
)

from reactivex.disposable import Disposable
from toolz import pipe  # type: ignore[import-untyped]
import typer

from dimos.core import Module, rpc
from dimos.core.module import ModuleConfig
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.protocol.pubsub.patterns import Glob, pattern_matches
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

if TYPE_CHECKING:
    from collections.abc import Callable

    from rerun._baseclasses import Archetype
    from rerun.blueprint import Blueprint

    from dimos.protocol.pubsub.spec import SubscribeAllCapable

BlueprintFactory: TypeAlias = "Callable[[], Blueprint]"

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


ViewerMode = Literal["native", "web", "none"]


def _default_blueprint() -> Blueprint:
    """Default blueprint with black background and raised grid."""
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(  # type: ignore[no-any-return]
        rrb.Spatial3DView(
            origin="world",
            background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
            line_grid=rrb.LineGrid3D(
                plane=rr.components.Plane3D.XY.with_distance(0.2),
            ),
        ),
    )


@dataclass
class Config(ModuleConfig):
    """Configuration for RerunBridgeModule."""

    pubsubs: list[SubscribeAllCapable[Any, Any]] = field(
        default_factory=lambda: [LCM(autoconf=True)]
    )

    visual_override: dict[Glob | str, Callable[[Any], Archetype]] = field(default_factory=dict)

    # Static items logged once after start. Maps entity_path -> callable(rr) returning Archetype
    static: dict[str, Callable[[Any], Archetype]] = field(default_factory=dict)

    entity_prefix: str = "world"
    topic_to_entity: Callable[[Any], str] | None = None
    viewer_mode: ViewerMode = "native"
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

        lcm = LCM(autoconf=True)
        bridge = RerunBridgeModule(pubsubs=[lcm])
        bridge.start()
        # All messages with to_rerun() are now logged to Rerun
        bridge.stop()
    """

    default_config = Config
    config: Config

    @lru_cache(maxsize=256)
    def _visual_override_for_entity_path(
        self, entity_path: str
    ) -> Callable[[Any], RerunData | None]:
        """Return a composed visual override for the entity path.

        Chains matching overrides from config, ending with final_convert
        which handles .to_rerun() or passes through Archetypes.
        """
        from rerun._baseclasses import Archetype

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
        import rerun as rr

        # convert a potentially complex topic object into an str rerun entity path
        entity_path: str = self._get_entity_path(topic)

        # apply visual overrides (including final_convert which handles .to_rerun())
        rerun_data: RerunData | None = self._visual_override_for_entity_path(entity_path)(msg)

        # converters can also suppress logging by returning None
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
        import rerun as rr

        super().start()

        # Initialize and spawn Rerun viewer
        rr.init("dimos")

        if self.config.viewer_mode == "native":
            rr.spawn(connect=True, memory_limit=self.config.memory_limit)
        elif self.config.viewer_mode == "web":
            server_uri = rr.serve_grpc()
            rr.serve_web_viewer(connect_to=server_uri, open_browser=False)
        # "none" - just init, no viewer (connect externally)

        if self.config.blueprint:
            rr.send_blueprint(self.config.blueprint())

        # Start pubsubs and subscribe to all messages
        for pubsub in self.config.pubsubs:
            logger.info(f"bridge listening on {pubsub.__class__.__name__}")
            if hasattr(pubsub, "start"):
                pubsub.start()  # type: ignore[union-attr]
            unsub = pubsub.subscribe_all(self._on_message)
            self._disposables.add(Disposable(unsub))

        # Add pubsub stop as disposable
        for pubsub in self.config.pubsubs:
            if hasattr(pubsub, "stop"):
                self._disposables.add(Disposable(pubsub.stop))  # type: ignore[union-attr]

        self._log_static()

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
    def stop(self) -> None:
        super().stop()


def run_bridge(
    viewer_mode: str = "native",
    memory_limit: str = "25%",
) -> None:
    """Start a RerunBridgeModule with default LCM config and block until interrupted."""
    import signal

    bridge = RerunBridgeModule(
        viewer_mode=viewer_mode,
        memory_limit=memory_limit,
        # any pubsub that supports subscribe_all and topic that supports str(topic)
        # is acceptable here
        pubsubs=[LCM(autoconf=True)],
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

# you don't need to include this in your blueprint if you are not creating a
# custom rerun configuration for your deployment, you can also run rerun-bridge standalone
rerun_bridge = RerunBridgeModule.blueprint
