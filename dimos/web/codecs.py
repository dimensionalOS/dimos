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

"""Web codec registry: DimOS message <-> wire payload, keyed by encoding id.

An encoder turns one DimOS message into the bytes of a relay data frame; the
matching JavaScript decoder (web/sdk, DecoderRegistry) turns them back into a
browser value. Decoders here run the other direction (browser publish -> DimOS
message) and are registered now but only exercised by the publish ticket (W7).

    @web_encoder("path.points.v1")
    def encode_path(msg: Path) -> bytes: ...

    @web_decoder("goal.json.v1")
    def decode_goal(value: dict[str, Any], context: PublishContext) -> PoseStamped: ...

The registry is consulted in the parent process: the cockpit() blueprint
compiler resolves encoding ids to callables at blueprint definition time and
ships the resolved callable inside an immutable runtime channel spec, so a
worker never depends on the user's codec module having been imported there.
Codec functions must therefore live at module level (standard pickle ships
them by reference); built-ins register in
dimos/web/relay_bridge/builtin_codecs.py.

Encoders take the message (plus, optionally, the channel's params mapping) and
return `bytes`, an `EncodedPayload` when the frame needs header meta, or None
to skip the sample. The generic json.v1 encoder is the only codec applied
without registration, and only to JSON-shaped message types; anything
potentially large (DimOS/LCM messages, images, arrays, bytes) needs an
explicit @web_encoder.
"""

from collections.abc import Callable, Mapping
import dataclasses
from dataclasses import dataclass
import inspect
import json
import sys
import threading
import types
from typing import Any, TypeVar, Union, get_args, get_origin, get_type_hints

from dimos.web.relay_bridge.manifest import MAX_MANIFEST_ID_LEN, RESERVED_CHANNEL_PREFIX

_F = TypeVar("_F", bound=Callable[..., Any])

# Frame header meta rides the wire on every frame; keep it a small JSON object.
MAX_ENCODED_META_BYTES = 16 * 1024

# Message types the generic json.v1 encoder accepts (plus dataclasses). An
# exact-class whitelist on purpose: a subclass may carry arbitrarily large
# state, and a mistaken Image/PointCloud2 declaration must fail at authoring
# time instead of serializing a pixel buffer per frame.
_JSON_V1_TYPES = frozenset({str, int, float, bool, dict, list})


@dataclass(frozen=True)
class EncodedPayload:
    """Encoder result carrying frame payload bytes plus optional header meta."""

    payload: bytes
    meta: Mapping[str, Any] | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.payload, (bytes, bytearray, memoryview)):
            raise TypeError(f"payload must be bytes-like, got {type(self.payload).__name__}")
        object.__setattr__(self, "payload", bytes(self.payload))
        if self.meta is None:
            return
        if not isinstance(self.meta, Mapping):
            raise TypeError(f"meta must be a mapping or None, got {type(self.meta).__name__}")
        if any(not isinstance(key, str) for key in self.meta):
            raise ValueError("meta keys must be strings")
        try:
            # allow_nan=False: NaN/Infinity are not JSON; the frame header
            # would reject them later and the frame would vanish silently.
            dumped = json.dumps(dict(self.meta), separators=(",", ":"), allow_nan=False)
        except (TypeError, ValueError) as e:
            raise ValueError(f"meta must be JSON-serializable: {e}") from e
        if len(dumped.encode()) > MAX_ENCODED_META_BYTES:
            raise ValueError(
                f"meta is {len(dumped.encode())} B encoded (limit {MAX_ENCODED_META_BYTES}); "
                "meta rides the frame header, put bulk data in the payload"
            )
        object.__setattr__(self, "meta", dict(self.meta))


@dataclass(frozen=True)
class PublishContext:
    """Relay-authored provenance handed to tx decoders (publish ticket, W7)."""

    robot: str
    ch: str
    relay_ts: float
    principal: str | None = None
    gen: int | None = None


@dataclass(frozen=True)
class EncoderDef:
    encoding: str
    message_type: type[Any]
    encode: Callable[..., Any]
    takes_params: bool
    # Validates a channel's merged params before any frame is encoded, so a
    # bad authored value (jpeg quality) fails at blueprint/module start.
    check_params: Callable[[Mapping[str, Any]], None] | None = None


@dataclass(frozen=True)
class DecoderDef:
    encoding: str
    message_type: type[Any]
    decode: Callable[..., Any]
    takes_context: bool


# Append-only after import time in practice; the lock serializes registration
# (tests and user modules may register from any thread), lookups read a dict
# that only ever gains keys.
_registry_lock = threading.Lock()
_encoders: dict[str, EncoderDef] = {}
_decoders: dict[str, DecoderDef] = {}


def _check_encoding_id(encoding: Any) -> None:
    if not isinstance(encoding, str) or not 1 <= len(encoding) <= MAX_MANIFEST_ID_LEN:
        raise ValueError(f"encoding id must be 1..{MAX_MANIFEST_ID_LEN} chars, got {encoding!r}")
    if encoding.startswith(RESERVED_CHANNEL_PREFIX):
        raise ValueError(
            f"encoding id {encoding!r} is invalid: ids beginning with "
            f"{RESERVED_CHANNEL_PREFIX!r} are reserved for protocol control"
        )


def _describe(func: Callable[..., Any]) -> str:
    return f"{func.__module__}.{func.__qualname__}"


def _check_registrable(func: Callable[..., Any], label: str) -> None:
    """Decoration-time importability gate.

    The full pickle-by-reference walk (`_resolves_by_reference`) cannot run
    here: a decorator fires before the def statement binds the module
    attribute. resolve_encoder() completes the check once the module exists.
    """
    if not inspect.isfunction(func):
        raise TypeError(f"{label} must be a module-level function, got {func!r}")
    if func.__name__ == "<lambda>":
        raise ValueError(f"{label} cannot be a lambda; standard pickle ships codecs by reference")
    if "<locals>" in func.__qualname__:
        raise ValueError(
            f"{label} {_describe(func)} is defined inside a function; move it to "
            "module level so it can be pickled by reference"
        )
    if func.__module__ == "__main__":
        raise ValueError(
            f"{label} {func.__qualname__} is only defined in __main__; move it into "
            "an importable module so worker processes can reconstruct it"
        )


def _resolves_by_reference(func: Callable[..., Any]) -> bool:
    """Mirror pickle's save-by-reference lookup for a function."""
    obj: Any = sys.modules.get(func.__module__)
    for part in func.__qualname__.split("."):
        obj = getattr(obj, part, None)
        if obj is None:
            return False
    return obj is func


def _signature_params(func: Callable[..., Any], label: str) -> list[inspect.Parameter]:
    params = list(inspect.signature(func).parameters.values())
    for param in params:
        if param.kind not in (param.POSITIONAL_ONLY, param.POSITIONAL_OR_KEYWORD):
            raise ValueError(
                f"{label} {_describe(func)} must take 1 or 2 positional parameters "
                f"(no *args/**kwargs/keyword-only), got {param.name!r}"
            )
    if len(params) not in (1, 2):
        raise ValueError(
            f"{label} {_describe(func)} must take 1 or 2 positional parameters, got {len(params)}"
        )
    return params


def _union_members(hint: Any) -> tuple[Any, ...] | None:
    """Members of a `X | Y` / Optional[X] annotation, None for non-unions."""
    if get_origin(hint) in (Union, types.UnionType):
        return get_args(hint)
    return None


def _check_params_annotation(hint: Any, func: Callable[..., Any]) -> None:
    args = get_args(hint)
    if get_origin(hint) not in (dict, Mapping) or not args or args[0] is not str:
        raise ValueError(
            f"web encoder {_describe(func)}: the second parameter must be annotated "
            f"Mapping[str, Any] (the channel params), got {hint!r}"
        )


def _check_encoder_return(hints: Mapping[str, Any], func: Callable[..., Any]) -> None:
    if "return" not in hints:
        raise ValueError(
            f"web encoder {_describe(func)}: the return annotation must be bytes, "
            "EncodedPayload, or an optional union of them; none given"
        )
    hint = hints["return"]
    members = _union_members(hint)
    candidates = members if members is not None else (hint,)
    produces = {bytes, EncodedPayload}
    if not all(m in produces or m is type(None) for m in candidates) or not any(
        m in produces for m in candidates
    ):
        raise ValueError(
            f"web encoder {_describe(func)}: the return annotation must be bytes, "
            f"EncodedPayload, or an optional union of them, got {hint!r}"
        )


def _is_json_annotation(hint: Any) -> bool:
    """Whether an annotation describes a parsed-JSON value shape."""
    if hint is Any or hint in _JSON_V1_TYPES or hint is type(None):
        return True
    members = _union_members(hint)
    if members is not None:
        return all(_is_json_annotation(m) for m in members)
    return get_origin(hint) in (dict, list, Mapping)


def _register_encoder(definition: EncoderDef) -> None:
    with _registry_lock:
        existing = _encoders.get(definition.encoding)
        # Same module-level function re-decorated (importlib.reload of the
        # defining module) replaces silently; a different owner is a clash.
        if existing is not None and _describe(existing.encode) != _describe(definition.encode):
            raise ValueError(
                f"web encoder for encoding {definition.encoding!r} is already "
                f"registered ({_describe(existing.encode)})"
            )
        _encoders[definition.encoding] = definition


def _register_decoder(definition: DecoderDef) -> None:
    with _registry_lock:
        existing = _decoders.get(definition.encoding)
        if existing is not None and _describe(existing.decode) != _describe(definition.decode):
            raise ValueError(
                f"web decoder for encoding {definition.encoding!r} is already "
                f"registered ({_describe(existing.decode)})"
            )
        _decoders[definition.encoding] = definition


def web_encoder(
    encoding: str,
    *,
    check_params: Callable[[Mapping[str, Any]], None] | None = None,
) -> Callable[[_F], _F]:
    """Register a module-level function as the encoder for `encoding`.

    The first parameter annotation is the supported DimOS message type; an
    optional second parameter annotated `Mapping[str, Any]` receives the
    channel's params. Returns bytes, EncodedPayload, or None (skip sample).
    """
    _check_encoding_id(encoding)

    def decorate(func: _F) -> _F:
        _check_registrable(func, "web encoder")
        params = _signature_params(func, "web encoder")
        hints = get_type_hints(func)
        message_type = hints.get(params[0].name)
        if not isinstance(message_type, type):
            raise ValueError(
                f"web encoder {_describe(func)}: the first parameter must be annotated "
                f"with the supported message class, got {message_type!r}"
            )
        takes_params = len(params) == 2
        if takes_params:
            _check_params_annotation(hints.get(params[1].name), func)
        _check_encoder_return(hints, func)
        _register_encoder(EncoderDef(encoding, message_type, func, takes_params, check_params))
        return func

    return decorate


def web_decoder(encoding: str) -> Callable[[_F], _F]:
    """Register a module-level function as the decoder for `encoding`.

    The return annotation is the produced DimOS message type; an optional
    second parameter annotated `PublishContext` receives relay provenance.
    Registered now, exercised by generic tx publish (W7).
    """
    _check_encoding_id(encoding)

    def decorate(func: _F) -> _F:
        _check_registrable(func, "web decoder")
        params = _signature_params(func, "web decoder")
        hints = get_type_hints(func)
        value_hint = hints.get(params[0].name)
        if value_hint is None or not _is_json_annotation(value_hint):
            raise ValueError(
                f"web decoder {_describe(func)}: the first parameter must be annotated "
                f"with the JSON value shape it accepts (dict/list/scalars or a union "
                f"of them), got {value_hint!r}"
            )
        message_type = hints.get("return")
        if not isinstance(message_type, type) or message_type is type(None):
            raise ValueError(
                f"web decoder {_describe(func)}: the return annotation must be the "
                f"produced message class, got {message_type!r}"
            )
        takes_context = len(params) == 2
        if takes_context and hints.get(params[1].name) is not PublishContext:
            raise ValueError(
                f"web decoder {_describe(func)}: the second parameter must be annotated "
                f"PublishContext, got {hints.get(params[1].name)!r}"
            )
        _register_decoder(DecoderDef(encoding, message_type, func, takes_context))
        return func

    return decorate


def encoder_definition(encoding: str) -> EncoderDef | None:
    return _encoders.get(encoding)


def decoder_definition(encoding: str) -> DecoderDef | None:
    return _decoders.get(encoding)


def encode_json_v1(msg: Any) -> bytes:
    """Generic json.v1 encoder for JSON-shaped values and dataclasses.

    allow_nan=False: a non-finite number would serialize to `NaN`/`Infinity`,
    which is not JSON and which the browser's JSON.parse rejects - better to
    drop the sample at the codec boundary (ValueError) than ship a corrupt
    frame.
    """
    value: Any = msg
    if dataclasses.is_dataclass(msg) and not isinstance(msg, type):
        value = dataclasses.asdict(msg)
    return json.dumps(value, separators=(",", ":"), allow_nan=False).encode()


def resolve_encoder(encoding: str, message_type: type[Any]) -> EncoderDef:
    """The encoder a channel (encoding, message type) compiles to; ValueError
    when the pair is unsupported. Runs in the parent at blueprint authoring
    time, so this also finishes the pickle-by-reference check deferred from
    decoration."""
    definition = _encoders.get(encoding)
    if definition is not None:
        if message_type is not definition.message_type:
            raise ValueError(
                f"encoding {encoding!r} encodes {definition.message_type.__qualname__}, "
                f"not {message_type.__qualname__}"
            )
        if not _resolves_by_reference(definition.encode):
            raise ValueError(
                f"encoder {_describe(definition.encode)} for {encoding!r} cannot be "
                "pickled by reference; it must be importable under its own module "
                "and qualified name"
            )
        return definition
    if encoding == "json.v1":
        # DimosMsg detection by attribute: the protocol has a data member, so
        # issubclass() is unavailable, and importing dimos.msgs here would
        # defeat this module's import-lightness. Some wire messages (Image)
        # are dataclasses too - the marker must win.
        is_dimos_msg = callable(getattr(message_type, "lcm_encode", None))
        plain_dataclass = dataclasses.is_dataclass(message_type) and not is_dimos_msg
        if message_type not in _JSON_V1_TYPES and not plain_dataclass:
            raise ValueError(
                f"message type {message_type.__module__}.{message_type.__qualname__} is "
                "not supported by json.v1 (JSON scalars, lists, dicts and plain "
                "dataclasses only; DimOS wire messages may be huge); register an "
                "explicit codec with @web_encoder(...)"
            )
        return EncoderDef("json.v1", message_type, encode_json_v1, takes_params=False)
    raise ValueError(
        f"no encoder registered for encoding {encoding!r}; register one with "
        f"@web_encoder({encoding!r})"
    )
