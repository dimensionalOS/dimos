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

"""RelayBridgeModule subclasses with generated ports, built on the core module class factory."""

from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any, Literal

from dimos.core import dynamic_module
from dimos.web.relay_bridge.manifest import MAX_MANIFEST_ID_LEN, RESERVED_CHANNEL_PREFIX, Dir
from dimos.web.relay_bridge.relay_bridge_module import RelayBridgeModule


@dataclass(frozen=True)
class DynamicPortSpec:
    """One generated bridge port.

    Only fields that affect the class's annotations (and therefore
    autoconnection) belong here. Encoding, rates, params, delivery, and
    publish policy are instance configuration and must not create distinct
    Python classes.
    """

    stream: str
    message_type: type[Any]
    direction: Dir


_PORT_DIRECTIONS: dict[str, Literal["in", "out"]] = {"rx": "in", "tx": "out"}
# Instance attributes the class does not declare: ref is set by set_ref after deploy, rpc and encoded are instance state.
_RESERVED_NAMES = frozenset({"ref", "rpc", "encoded"})


def _port(spec: DynamicPortSpec) -> dynamic_module.PortSpec:
    if not isinstance(spec, DynamicPortSpec):
        raise TypeError(f"expected DynamicPortSpec, got {type(spec).__name__}")
    stream = spec.stream
    if not isinstance(stream, str):
        raise TypeError(f"stream id must be str, got {type(stream).__name__}")
    if stream.startswith(RESERVED_CHANNEL_PREFIX):
        raise ValueError(
            f"stream id {stream!r} is invalid: ids beginning with "
            f"{RESERVED_CHANNEL_PREFIX!r} are reserved for protocol control"
        )
    if len(stream) > MAX_MANIFEST_ID_LEN:
        raise ValueError(f"stream id {stream!r} is longer than {MAX_MANIFEST_ID_LEN} chars")
    direction = _PORT_DIRECTIONS.get(spec.direction)
    if direction is None:
        raise ValueError(
            f"stream {stream!r}: direction must be one of {tuple(_PORT_DIRECTIONS)}, "
            f"got {spec.direction!r}"
        )
    return dynamic_module.PortSpec(stream, spec.message_type, direction)


def make_relay_bridge_class(specs: Sequence[DynamicPortSpec]) -> type[RelayBridgeModule]:
    """Build or return the memoized RelayBridgeModule subclass for specs, in any order."""
    # Looked up at call time so the restart path's reload of dynamic_module is picked up.
    return dynamic_module.make_module_class(
        RelayBridgeModule, [_port(spec) for spec in specs], reserved=_RESERVED_NAMES
    )
