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

Scatter is **bundle-only and M-agnostic**: it never branches on the number of
ports nor on whether a tick's payload is a ``Bundle`` versus a raw ``T``. One
``Out`` reads ``bundle[its_name]`` exactly as two ``Out``s each read their own
key. The single back-compat concession lives in :func:`normalize_to_bundle`,
which wraps a 1:1 pipeline's raw payload into a one-key ``Bundle`` at the start
boundary so the scatter contract stays uniform without rewriting every 1:1
pipeline.
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


def normalize_to_bundle(stream: Stream[Any], ports: dict[str, Out[Any]]) -> Stream[Any]:
    """Bridge a raw single-output pipeline into the bundle-only scatter contract.

    Scatter is bundle-only (:func:`scatter_to_ports`), but a 1:1 pipeline still
    yields its payload ``T`` directly (e.g. ``VoxelGridMapper`` ->
    ``PointCloud2``). With exactly one ``Out`` port this wraps each such payload
    in a one-key :class:`Bundle` keyed by that port name, so scatter has a
    uniform, M-agnostic input without rewriting every 1:1 pipeline.

    A pipeline that already yields a :class:`Bundle` is passed through untouched -
    including a single-``Out`` module whose tail is a ``Bundle`` (marker-style
    1->1): its keys still route by port name, so scatter publishes
    ``bundle[port_name]`` rather than the whole bundle. With multiple ``Out``
    ports there is no single key to wrap a raw payload under, so the stream is
    returned unchanged and the pipeline must yield a ``Bundle`` by contract
    (scatter raises otherwise).
    """
    if len(ports) != 1:
        return stream

    (out_name,) = ports

    def _wrap(obs: Observation[Any]) -> Observation[Any]:
        data = obs.data
        if isinstance(data, Bundle):
            return obs
        return obs.derive(data=Bundle({out_name: data}))

    return stream.map(_wrap)


def scatter_to_ports(stream: Stream[Any], ports: dict[str, Out[Any]]) -> DisposableBase:
    """Subscribe *stream* once and publish each tick's :class:`Bundle` to its ports.

    Bundle-only and M-agnostic: every observation's ``data`` is a :class:`Bundle`
    keyed by ``Out`` port name, and for *each* declared port the matching field is
    published when present and not ``None``. Port count never selects a code path
    - a single ``Out`` reads ``bundle[its_name]`` exactly as two ``Out``\\s each
    read their own key. A missing key or a ``None`` value leaves that port idle
    for the tick; an empty-but-present payload (e.g. an empty detection array)
    still publishes, so "nothing detected this frame" stays distinct from "this
    port is idle".

    Single-output 1:1 pipelines that yield a raw payload are wrapped into a
    one-key ``Bundle`` by :func:`normalize_to_bundle` at the start boundary, so by
    the time scatter runs the contract is uniform; a multi-output pipeline that
    fails to yield a ``Bundle`` raises here. Exactly one
    ``stream.observable().subscribe(...)`` happens regardless of port count, so
    the pipeline (and any detectors in it) runs once per tick rather than once per
    output.
    """

    def _on_error(e: Exception) -> None:
        logger.error("scatter_to_ports() pipeline error: %s", e, exc_info=True)

    def _emit(obs: Observation[Any]) -> None:
        bundle = obs.data
        if not isinstance(bundle, Bundle):
            raise TypeError(
                f"fan-out pipeline must yield a Bundle keyed by Out port name at the "
                f"scatter boundary, got {type(bundle).__name__}"
            )
        for name, port in ports.items():
            payload = bundle.get(name)
            if payload is not None:
                port.publish(payload)

    return stream.observable().subscribe(on_next=_emit, on_error=_on_error)
