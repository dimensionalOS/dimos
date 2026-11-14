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

"""Modules of a Dual OpenYAM blueprint run in one process, wired as the blueprint wires them."""

from collections.abc import Callable, Iterator, Mapping
from contextlib import contextmanager
from dataclasses import dataclass
import threading
from typing import Any

from dimos.core.coordination.blueprints import Blueprint
from dimos.core.coordination.module_coordinator import resolve_module_refs
from dimos.core.native_module import NativeModule
from dimos.core.stream import Stream, Transport
from dimos.core.transport_factory import make_transport


class WireTransport(Transport[Any]):
    """An in-memory transport that hands every subscriber its own decoded copy.

    A network transport does the same. Modules in one process otherwise share
    each message object, and a publisher that reuses one, as the planner does
    with its static transforms, changes what the others have already stored.
    Only the newest message is kept.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._newest: list[Any] = []
        self._subscribers: list[Callable[[Any], Any]] = []

    @property
    def latest(self) -> Any:
        with self._lock:
            assert self._newest, "No message arrived on this transport"
            return self._newest[0]

    def broadcast(self, selfstream: Stream[Any] | None, value: Any) -> None:
        encode = getattr(value, "lcm_encode", None)
        encoded = None if encode is None else encode()
        with self._lock:
            self._newest = [value]
            callbacks = tuple(self._subscribers)
        for callback in callbacks:
            callback(value if encoded is None else type(value).lcm_decode(encoded))

    def subscribe(
        self, callback: Callable[[Any], Any], selfstream: Stream[Any] | None = None
    ) -> Callable[[], None]:
        with self._lock:
            self._subscribers.append(callback)

        def dispose() -> None:
            with self._lock:
                if callback in self._subscribers:
                    self._subscribers.remove(callback)

        return dispose

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass


@dataclass
class Deployed:
    """Modules of a blueprint running in this process, and the transport of every topic."""

    modules: dict[type, Any]
    topics: dict[str, Any]

    def measured(self) -> dict[str, float]:
        """The rig's newest joint positions, by joint name."""
        reading = self.topics["joint_state"].latest
        return dict(zip(reading.name, reading.position, strict=True))

    def distance_from(self, targets: dict[str, float]) -> float:
        """How far the farthest of the named joints is from its target."""
        now = self.measured()
        return max(abs(now[name] - target) for name, target in targets.items())


@contextmanager
def deployed(
    blueprint: Blueprint, *wanted: type, settings: Mapping[type, Mapping[str, Any]] | None = None
) -> Iterator[Deployed]:
    """Run the blueprint's modules of the wanted types in this process.

    They are wired topic by topic and reference by reference as the blueprint
    wires them, and started in the order given. Topics are joined in memory. A
    native module runs as its own process and can only reach a network
    transport, so the topics it is on get the one the stack would use, on the
    test session's private bus.

    Args:
        blueprint: The stack the modules come from.
        *wanted: The module types to run; a subclass in the blueprint counts.
        settings: Module arguments that differ from the blueprint's, by type.
    """
    stack = Deployed({}, {})
    named: dict[str, Any] = {}
    atoms = [
        (kind, atom)
        for atom in blueprint.active_blueprints
        for kind in wanted
        if issubclass(atom.module, kind)
    ]

    def topic_of(atom: Any, stream: Any) -> str:
        topic = blueprint.remapping_map.get((atom.name, stream.name), stream.name)
        assert isinstance(topic, str)
        return topic

    networked = {
        topic_of(atom, stream)
        for _, atom in atoms
        if issubclass(atom.module, NativeModule)
        for stream in atom.streams
    }
    try:
        for kind, atom in atoms:
            module: Any = atom.module(**{**atom.kwargs, **(settings or {}).get(kind, {})})
            stack.modules[kind] = named[atom.name] = module
            for stream in atom.streams:
                topic = topic_of(atom, stream)
                if topic not in stack.topics:
                    stack.topics[topic] = (
                        make_transport(f"/{topic}", stream.type)
                        if topic in networked
                        else WireTransport()
                    )
                module.set_transport(stream.name, stack.topics[topic])
        # The blueprint hands a module its references; here they are the objects themselves.
        for (consumer, reference), provider in resolve_module_refs(blueprint).items():
            if consumer in named and provider in named:
                named[consumer].set_module_ref(reference, named[provider])
        for kind in wanted:
            stack.modules[kind].start()
        yield stack
    finally:
        for module in reversed(stack.modules.values()):
            # The framework leaves a stopped module's to_thread workers idle;
            # the test session's thread check would count them as leaked.
            loop = module._loop
            module.stop()
            if loop is not None and not loop.is_closed():
                loop.run_until_complete(loop.shutdown_default_executor())
