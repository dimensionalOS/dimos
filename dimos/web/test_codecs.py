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

"""Codec registry tests: decorators, validation, lookup, json.v1 gate."""

from collections.abc import Mapping
from dataclasses import dataclass
import subprocess
import sys
from typing import Any

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.Image import Image
from dimos.web.codecs import (
    MAX_ENCODED_META_BYTES,
    DecoderDef,
    EncodedPayload,
    EncoderDef,
    PublishContext,
    decoder_definition,
    encode_json_v1,
    encoder_definition,
    resolve_encoder,
    web_decoder,
    web_encoder,
)


@dataclass(frozen=True)
class _Point:
    x: float
    y: float


# Undecorated fixtures for decoration-error tests: decorating them at module
# level would fail the import, and defining them inside a test trips the
# <locals> gate before the error under test.
def _enc_ok(msg: _Point) -> bytes:
    return encode_json_v1(msg)


def _enc_with_params(msg: _Point, params: Mapping[str, Any]) -> bytes:
    return encode_json_v1({"x": msg.x, "scale": params.get("scale", 1)})


def _enc_dict_params(msg: _Point, params: dict[str, Any]) -> bytes:
    return b""


def _enc_zero_params() -> bytes:
    return b""


def _enc_three_params(msg: _Point, params: Mapping[str, Any], extra: int) -> bytes:
    return b""


def _enc_varargs(*msgs: Any) -> bytes:
    return b""


def _enc_unannotated(msg) -> bytes:
    return b""


def _enc_generic_first(msg: list[int]) -> bytes:
    return b""


def _enc_bad_second(msg: _Point, params: int) -> bytes:
    return b""


def _enc_bare_params(msg: _Point, params: dict) -> bytes:
    return b""


def _enc_int_key_params(msg: _Point, params: Mapping[int, Any]) -> bytes:
    return b""


def _enc_no_return(msg: _Point):
    return b""


def _enc_bad_return(msg: _Point) -> int:
    return 1


def _enc_optional_union(msg: _Point) -> bytes | EncodedPayload | None:
    return None


def _dec_ok(value: dict[str, Any]) -> _Point:
    return _Point(value["x"], value["y"])


def _dec_with_context(value: dict[str, Any], context: PublishContext) -> _Point:
    return _Point(value["x"], value["y"])


def _dec_no_return(value: dict[str, Any]):
    return None


def _dec_bad_context(value: dict[str, Any], context: int) -> _Point:
    return _Point(0.0, 0.0)


def _dec_bytes_input(value: bytes) -> _Point:
    return _Point(0.0, 0.0)


def _dec_unannotated_input(value) -> _Point:
    return _Point(0.0, 0.0)


def _dec_none_return(value: dict[str, Any]) -> None:
    return None


def _dec_union_input(value: dict[str, Any] | list[Any] | None) -> _Point:
    return _Point(0.0, 0.0)


def _dec_any_input(value: Any) -> _Point:
    return _Point(0.0, 0.0)


def test_encoder_decorator_registers_and_returns_the_function() -> None:
    decorated = web_encoder("t.enc.reg.v1")(_enc_ok)
    assert decorated is _enc_ok
    definition = encoder_definition("t.enc.reg.v1")
    assert definition == EncoderDef("t.enc.reg.v1", _Point, _enc_ok, takes_params=False)


def test_encoder_second_param_variants() -> None:
    web_encoder("t.enc.params.v1")(_enc_with_params)
    web_encoder("t.enc.dictparams.v1")(_enc_dict_params)
    assert encoder_definition("t.enc.params.v1").takes_params is True
    assert encoder_definition("t.enc.dictparams.v1").takes_params is True


def test_encoder_check_params_is_kept() -> None:
    def check(params: Mapping[str, Any]) -> None:
        pass

    web_encoder("t.enc.check.v1", check_params=check)(_enc_ok)
    assert encoder_definition("t.enc.check.v1").check_params is check


def test_same_function_re_registration_is_idempotent() -> None:
    web_encoder("t.enc.idem.v1")(_enc_ok)
    web_encoder("t.enc.idem.v1")(_enc_ok)
    assert encoder_definition("t.enc.idem.v1").encode is _enc_ok


def test_conflicting_duplicate_registration_raises() -> None:
    web_encoder("t.enc.dup.v1")(_enc_ok)
    with pytest.raises(ValueError, match="already registered.*_enc_ok"):
        web_encoder("t.enc.dup.v1")(_enc_with_params)


@pytest.mark.parametrize("encoding", ["", "x" * 65, "@control"], ids=["empty", "long", "reserved"])
def test_encoding_id_bounds(encoding: str) -> None:
    with pytest.raises(ValueError):
        web_encoder(encoding)


@pytest.mark.parametrize(
    ("func", "match"),
    [
        (_enc_zero_params, "1 or 2 positional parameters"),
        (_enc_three_params, "1 or 2 positional parameters"),
        (_enc_varargs, "positional parameters"),
        (_enc_unannotated, "first parameter must be annotated"),
        (_enc_generic_first, "first parameter must be annotated"),
        (_enc_bad_second, "second parameter must be annotated Mapping"),
        (_enc_bare_params, "second parameter must be annotated Mapping"),
        (_enc_int_key_params, "second parameter must be annotated Mapping"),
        (_enc_no_return, "return annotation must be bytes"),
        (_enc_bad_return, "return annotation must be bytes"),
    ],
    ids=[
        "zero",
        "three",
        "varargs",
        "unannotated",
        "generic_first",
        "bad_second",
        "bare_params",
        "int_key_params",
        "no_return",
        "bad_return",
    ],
)
def test_encoder_signature_rejected(func, match: str) -> None:
    with pytest.raises(ValueError, match=match):
        web_encoder("t.enc.badsig.v1")(func)


def test_encoder_return_union_accepted() -> None:
    web_encoder("t.enc.union.v1")(_enc_optional_union)
    assert encoder_definition("t.enc.union.v1").encode is _enc_optional_union


def test_lambda_rejected() -> None:
    with pytest.raises(ValueError, match="lambda"):
        web_encoder("t.enc.lambda.v1")(lambda msg: b"")


def test_locals_function_rejected() -> None:
    def local_encoder(msg: _Point) -> bytes:
        return b""

    with pytest.raises(ValueError, match="defined inside a function"):
        web_encoder("t.enc.locals.v1")(local_encoder)


def test_main_module_function_rejected(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(_enc_ok, "__module__", "__main__")
    with pytest.raises(ValueError, match="only defined in __main__"):
        web_encoder("t.enc.main.v1")(_enc_ok)


def test_non_function_rejected() -> None:
    class CallableEncoder:
        def __call__(self, msg: _Point) -> bytes:
            return b""

    with pytest.raises(TypeError, match="module-level function"):
        web_encoder("t.enc.obj.v1")(CallableEncoder())


def test_decoder_contracts() -> None:
    web_decoder("t.dec.plain.v1")(_dec_ok)
    web_decoder("t.dec.ctx.v1")(_dec_with_context)
    web_decoder("t.dec.union.v1")(_dec_union_input)
    web_decoder("t.dec.any.v1")(_dec_any_input)
    plain = decoder_definition("t.dec.plain.v1")
    assert plain == DecoderDef("t.dec.plain.v1", _Point, _dec_ok, takes_context=False)
    assert decoder_definition("t.dec.ctx.v1").takes_context is True


def test_decoder_signature_rejected() -> None:
    with pytest.raises(ValueError, match="return annotation must be the produced message class"):
        web_decoder("t.dec.noret.v1")(_dec_no_return)
    with pytest.raises(ValueError, match="return annotation must be the produced message class"):
        web_decoder("t.dec.noneret.v1")(_dec_none_return)
    with pytest.raises(ValueError, match="second parameter must be annotated PublishContext"):
        web_decoder("t.dec.badctx.v1")(_dec_bad_context)
    with pytest.raises(ValueError, match="first parameter must be annotated with the JSON value"):
        web_decoder("t.dec.bytesin.v1")(_dec_bytes_input)
    with pytest.raises(ValueError, match="first parameter must be annotated with the JSON value"):
        web_decoder("t.dec.noin.v1")(_dec_unannotated_input)


def test_duplicate_decoder_registration_raises() -> None:
    web_decoder("t.dec.dup.v1")(_dec_ok)
    with pytest.raises(ValueError, match="already registered.*_dec_ok"):
        web_decoder("t.dec.dup.v1")(_dec_with_context)


def test_encode_json_v1_values() -> None:
    assert encode_json_v1(_Point(1.5, -2.5)) == b'{"x":1.5,"y":-2.5}'
    assert encode_json_v1({"a": [1, "x"], "b": None}) == b'{"a":[1,"x"],"b":null}'
    assert encode_json_v1("hello") == b'"hello"'


def test_encode_json_v1_rejects_non_finite_numbers() -> None:
    # `NaN`/`Infinity` are not JSON and JSON.parse in the browser rejects
    # them; the sample must die at the codec boundary, not ship corrupt.
    with pytest.raises(ValueError):
        encode_json_v1(float("nan"))
    with pytest.raises(ValueError):
        encode_json_v1({"x": float("inf")})
    with pytest.raises(ValueError):
        encode_json_v1(_Point(float("nan"), 0.0))


def test_resolve_json_v1_whitelist_accepts() -> None:
    for message_type in (dict, list, str, int, float, bool, _Point):
        definition = resolve_encoder("json.v1", message_type)
        assert definition.encode is encode_json_v1
        assert definition.takes_params is False


@pytest.mark.parametrize(
    # Image is the sharp case: a dataclass AND a DimOS wire message - the
    # DimosMsg marker must beat the dataclass whitelist.
    "message_type",
    [np.ndarray, bytes, PoseStamped, Path, Image],
    ids=lambda t: t.__name__,
)
def test_resolve_json_v1_whitelist_rejects(message_type: type) -> None:
    with pytest.raises(ValueError, match=r"not supported by json\.v1.*@web_encoder"):
        resolve_encoder("json.v1", message_type)


def test_resolve_registered_type_mismatch() -> None:
    web_encoder("t.enc.mismatch.v1")(_enc_ok)
    with pytest.raises(ValueError, match="encodes _Point, not dict"):
        resolve_encoder("t.enc.mismatch.v1", dict)


def test_resolve_unknown_encoding() -> None:
    with pytest.raises(ValueError, match="no encoder registered for encoding 'nope.v1'"):
        resolve_encoder("nope.v1", dict)


def test_resolve_rechecks_pickle_by_reference(monkeypatch: pytest.MonkeyPatch) -> None:
    # Decoration cannot verify the module binding (the def statement has not
    # bound the name yet); resolve does. Simulate a codec whose module-level
    # name was deleted after registration.
    web_encoder("t.enc.unref.v1")(_enc_ok)
    monkeypatch.delattr(sys.modules[__name__], "_enc_ok")
    with pytest.raises(ValueError, match="cannot be pickled by reference"):
        resolve_encoder("t.enc.unref.v1", _Point)


def test_encoded_payload_normalizes_bytes_like() -> None:
    assert EncodedPayload(bytearray(b"ab")).payload == b"ab"
    assert EncodedPayload(memoryview(b"cd"), {"n": 2}).meta == {"n": 2}


def test_encoded_payload_rejects_bad_payload_and_meta() -> None:
    with pytest.raises(TypeError, match="payload must be bytes-like"):
        EncodedPayload("text")
    with pytest.raises(TypeError, match="meta must be a mapping"):
        EncodedPayload(b"", meta=[1, 2])
    with pytest.raises(ValueError, match="meta keys must be strings"):
        EncodedPayload(b"", meta={1: "x"})
    with pytest.raises(ValueError, match="JSON-serializable"):
        EncodedPayload(b"", meta={"blob": b"\x00"})
    # Non-finite numbers are not JSON; the frame header would reject them
    # later and the frame would vanish silently.
    with pytest.raises(ValueError, match="JSON-serializable"):
        EncodedPayload(b"", meta={"n": float("nan")})
    with pytest.raises(ValueError, match="JSON-serializable"):
        EncodedPayload(b"", meta={"deep": [{"n": float("inf")}]})
    with pytest.raises(ValueError, match=str(MAX_ENCODED_META_BYTES)):
        EncodedPayload(b"", meta={"pad": "x" * MAX_ENCODED_META_BYTES})


def test_import_stays_light() -> None:
    # User modules import dimos.web.codecs for the decorators; that must not
    # drag in the bridge runtime, aioquic, or numpy.
    code = (
        "import sys; import dimos.web.codecs; "
        "assert 'dimos.web.relay_bridge.relay_bridge_module' not in sys.modules; "
        "assert 'aioquic' not in sys.modules; "
        "assert 'numpy' not in sys.modules"
    )
    subprocess.run([sys.executable, "-c", code], check=True)
