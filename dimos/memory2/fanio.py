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

"""Fan-I/O primitives: emit several results from one pipeline run.

A fan-out module yields a
:class:`Bundle` per tick - a mapping from ``Out`` port name to payload - and
:func:`scatter_to_ports` routes each field to its matching port. The whole
pipeline runs *once* per tick (memory2 streams are lazy: every subscribe re-runs
the upstream, so a second subscribe would recompute every detector), which is
why fan-out is structural here rather than one derived stream per ``Out``.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import TYPE_CHECKING, Any

from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from reactivex.abc import DisposableBase

    from dimos.core.stream import Out
    from dimos.memory2.stream import Stream
    from dimos.memory2.type.observation import Observation

logger = setup_logger()


@dataclass(frozen=True)
class Bundle:
    """Per-tick mapping from ``Out`` port name to that port's payload.

    A multi-output ``pipeline()`` ends in a transform that yields a ``Bundle``
    per observation; :func:`scatter_to_ports` publishes each value to the ``Out``
    whose name matches the key. Keys **must** equal declared ``Out`` port names.

    A key may be omitted (or mapped to ``None``) to publish nothing on that port
    this tick; an empty-but-present payload (e.g. an empty ``Detection2DArray``)
    is still published - "no detections this frame" is distinct from "this port
    is idle". ``with_`` returns a new ``Bundle`` rather than mutating in place:
    the mapping is copied into a read-only view at construction, so neither
    ``bundle.values = ...`` (frozen dataclass) nor ``bundle.values["a"] = ...``
    (mapping proxy) can alter an existing bundle. The payload objects themselves
    are shared by reference - only the key->payload structure is immutable.
    """

    values: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        # Shallow-copy, then read-only proxy; __setattr__ because frozen=True.
        object.__setattr__(self, "values", MappingProxyType(dict(self.values)))

    def __getitem__(self, key: str) -> Any:
        return self.values[key]

    def get(self, key: str, default: Any = None) -> Any:
        return self.values.get(key, default)

    def with_(self, **updates: Any) -> Bundle:
        """Return a new ``Bundle`` with *updates* layered over the current values."""
        return Bundle({**self.values, **updates})


def scatter_to_ports(stream: Stream[Any], ports: dict[str, Out[Any]]) -> DisposableBase:
    """Subscribe *stream* once and publish its output to one or many ``Out`` ports.

    With a single port the stream's payload type is the port's payload type, so
    each observation's ``data`` is published verbatim - identical to a 1:1
    module. With multiple ports each observation's ``data`` must be a
    :class:`Bundle`; every field whose key names a port is published to that
    port, missing keys and ``None`` values are skipped for that tick, and an
    empty-but-present payload is still published.

    Exactly one ``stream.observable().subscribe(...)`` happens regardless of port
    count, so the pipeline (and any detectors in it) runs once per tick rather
    than once per output.
    """

    def _on_error(e: Exception) -> None:
        logger.error("scatter_to_ports() pipeline error: %s", e, exc_info=True)

    if len(ports) == 1:
        (out,) = ports.values()
        return stream.observable().subscribe(
            on_next=lambda obs: out.publish(obs.data),
            on_error=_on_error,
        )

    def _emit(obs: Observation[Any]) -> None:
        bundle = obs.data
        if not isinstance(bundle, Bundle):
            raise TypeError(
                f"multi-output pipeline must yield Bundle, got {type(bundle).__name__}"
            )
        for name, port in ports.items():
            payload = bundle.get(name)
            if payload is not None:
                port.publish(payload)

    return stream.observable().subscribe(on_next=_emit, on_error=_on_error)
