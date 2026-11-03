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

"""Dynamically generated RelayBridgeModule subclasses that survive the worker pipe.

DimOS discovers a module's streams from class annotations and ships the class
object itself into forkserver workers (DeployModuleRequest) over the CBOR
codec, which sends classes by importable name. A class built at runtime has no
such name, so `make_relay_bridge_class` builds classes through a dedicated
metaclass whose `cbor_encode`/`cbor_decode` serialize the canonical port specs
and reconstruct through the memoized factory on the far side.
"""

from abc import ABCMeta
from collections.abc import Sequence
from dataclasses import dataclass
import hashlib
import keyword
import sys
import threading
import types
from typing import Any, cast, get_args, get_type_hints

from dimos.core.stream import In, Out
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


class _DynamicRelayBridgeMeta(ABCMeta):
    """Marker metaclass for generated bridge classes.

    Derives from ABCMeta because that is Module's metaclass (a bare `type`
    metaclass would conflict). The CBOR codec looks for the hooks on a class
    object's metaclass, so ordinary ABCMeta classes are unaffected.
    """

    dynamic_port_specs: tuple[DynamicPortSpec, ...]

    def cbor_encode(cls) -> tuple[tuple[str, type[Any], str], ...]:
        # Plain field tuples, not DynamicPortSpec instances: instances would pin
        # the payload to one generation of the dataclass, so re-encoding a class
        # that survived an importlib.reload of this module would fail.
        return _spec_fields(cls.dynamic_port_specs)

    @classmethod
    def cbor_decode(cls, fields: Sequence[Sequence[Any]]) -> type[RelayBridgeModule]:
        return _rebuild_relay_bridge_class(tuple(tuple(f) for f in fields))


_CLASS_NAME_PREFIX = "DynamicRelayBridge_"
_DIGEST_LEN = 10
_VALID_DIRECTIONS: tuple[str, ...] = get_args(Dir)
# hasattr() would miss annotation-only names (config) and attributes assigned
# in __init__ (ref is clobbered by set_ref after deploy; rpc and encoded are
# instance state); the leading-underscore rule rejects the private rest.
_RESERVED_NAMES: frozenset[str] = (
    frozenset(dir(RelayBridgeModule))
    | frozenset(get_type_hints(RelayBridgeModule))
    | {"ref", "rpc", "encoded"}
)

# Process-global on purpose: repeated decodes of the same class must return
# the identical class object, because blueprints and the coordinator compare
# module classes with `is`.
_memo: dict[tuple[DynamicPortSpec, ...], type[RelayBridgeModule]] = {}
_memo_lock = threading.Lock()


def _resolves_by_reference(message_type: type[Any]) -> bool:
    """Mirror the CBOR codec's by-reference lookup for a class."""
    obj: Any = sys.modules.get(message_type.__module__)
    for part in message_type.__qualname__.split("."):
        obj = getattr(obj, part, None)
        if obj is None:
            return False
    return obj is message_type


def _validate_specs(specs: tuple[DynamicPortSpec, ...]) -> None:
    for spec in specs:
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
        if not stream.isidentifier() or keyword.iskeyword(stream):
            raise ValueError(f"stream id {stream!r} is not a valid Python identifier")
        if stream.startswith("_"):
            raise ValueError(
                f"stream id {stream!r} is invalid: leading underscores are reserved "
                "(underscored streams are invisible to Module.inputs/outputs)"
            )
        if len(stream) > MAX_MANIFEST_ID_LEN:
            raise ValueError(f"stream id {stream!r} is longer than {MAX_MANIFEST_ID_LEN} chars")
        if stream in _RESERVED_NAMES:
            raise ValueError(
                f"stream id {stream!r} collides with an existing RelayBridgeModule attribute"
            )
        if spec.direction not in _VALID_DIRECTIONS:
            raise ValueError(
                f"stream {stream!r}: direction must be one of {_VALID_DIRECTIONS}, "
                f"got {spec.direction!r}"
            )
        message_type = spec.message_type
        # TODO(PY311): drop the GenericAlias check — on 3.10,
        # isinstance(list[int], type) is True.
        if not isinstance(message_type, type) or isinstance(message_type, types.GenericAlias):
            raise TypeError(
                f"stream {stream!r}: message_type must be a class, got {message_type!r}"
            )
        if message_type.__module__ == "__main__":
            raise ValueError(
                f"stream {stream!r}: message type {message_type.__qualname__} is only "
                "defined in __main__; move it into an importable module so worker "
                "processes can reconstruct it"
            )
        if "<locals>" in message_type.__qualname__:
            raise ValueError(
                f"stream {stream!r}: message type {message_type.__qualname__} is defined "
                "inside a function; move it to module level so it can be sent by reference"
            )
        if not _resolves_by_reference(message_type):
            raise ValueError(
                f"stream {stream!r}: message type {message_type.__module__}."
                f"{message_type.__qualname__} cannot be sent by reference; it must be "
                "importable under its own module and qualified name"
            )
    seen: set[str] = set()
    for spec in specs:
        if spec.stream in seen:
            raise ValueError(f"duplicate stream id {spec.stream!r}")
        seen.add(spec.stream)


def _spec_sort_key(spec: DynamicPortSpec) -> str:
    return spec.stream


def _spec_fields(
    specs: tuple[DynamicPortSpec, ...],
) -> tuple[tuple[str, type[Any], str], ...]:
    return tuple((s.stream, s.message_type, s.direction) for s in specs)


def _canonical_digest(canonical: tuple[DynamicPortSpec, ...]) -> str:
    lines = [
        f"{s.stream}:{s.direction}:{s.message_type.__module__}.{s.message_type.__qualname__}"
        for s in canonical
    ]
    return hashlib.sha256("\n".join(lines).encode()).hexdigest()[:_DIGEST_LEN]


def _port_annotation(spec: DynamicPortSpec) -> Any:
    # In[x] with a variable x is invalid annotation syntax to mypy, but at
    # runtime it is an ordinary __class_getitem__ call.
    port: Any = In if spec.direction == "rx" else Out
    return port[spec.message_type]


def make_relay_bridge_class(specs: Sequence[DynamicPortSpec]) -> type[RelayBridgeModule]:
    """Build (or return the memoized) RelayBridgeModule subclass for *specs*.

    The canonical (stream-sorted) spec tuple is the class identity; the digest
    in the class name is diagnostics only.
    """
    ordered = tuple(specs)
    _validate_specs(ordered)
    canonical = tuple(sorted(ordered, key=_spec_sort_key))

    with _memo_lock:
        cached = _memo.get(canonical)
        if cached is not None:
            return cached

        name = _CLASS_NAME_PREFIX + _canonical_digest(canonical)
        existing = globals().get(name)
        if existing is not None:
            # After importlib.reload of this module (the coordinator's restart
            # path) the memo is fresh but published classes survive in the
            # namespace. Compare field tuples, not DynamicPortSpec equality: a
            # reload also replaces the dataclass, and dataclass __eq__ requires
            # both sides to be the same class.
            existing_specs = getattr(existing, "dynamic_port_specs", None)
            if existing_specs is not None and _spec_fields(tuple(existing_specs)) == _spec_fields(
                canonical
            ):
                klass = cast("type[RelayBridgeModule]", existing)
                _memo[canonical] = klass
                return klass
            raise ValueError(f"digest collision: {name} already exists for different port specs")

        annotations = {spec.stream: _port_annotation(spec) for spec in canonical}
        namespace: dict[str, Any] = {
            "__annotations__": annotations,
            "__module__": __name__,
            "dynamic_port_specs": canonical,
        }
        created = _DynamicRelayBridgeMeta(name, (RelayBridgeModule,), namespace)
        klass = cast("type[RelayBridgeModule]", created)
        _memo[canonical] = klass
        # Published so the coordinator's restart path (importlib.reload of this
        # module, then getattr by class name) can find the class again.
        globals()[name] = klass
        return klass


def _rebuild_relay_bridge_class(
    fields: tuple[tuple[str, type[Any], str], ...],
) -> type[RelayBridgeModule]:
    return make_relay_bridge_class(
        [
            DynamicPortSpec(stream, message_type, cast("Dir", direction))
            for stream, message_type, direction in fields
        ]
    )
