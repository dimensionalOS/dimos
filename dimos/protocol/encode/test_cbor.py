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

from __future__ import annotations

import collections.abc
from dataclasses import dataclass, field
import enum
import functools
import math
from pathlib import Path
import threading
from types import MappingProxyType
import typing
from typing import Any, Generic, NamedTuple, TypeVar

from cbor2 import CBORTag, dumps
import numpy as np
from pydantic import BaseModel, ConfigDict, Field, PrivateAttr
import pytest

from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.protocol.encode.cbor import (
    TAG_CLASS,
    TAG_DATACLASS,
    TAG_DIMOSMSG,
    TAG_ENUM,
    TAG_GENERIC_ALIAS,
    TAG_REGISTERED,
    TAG_UNION,
    CborDecodeError,
    decode,
    encode,
    register_codec,
)
from dimos.types.robot_location import RobotLocation
from dimos.utils.testing.cbor import roundtrip

T = TypeVar("T")


class Point(NamedTuple):
    x: int
    y: str


class Color(enum.Enum):
    RED = "r"
    BLUE = "b"


class Level(enum.IntEnum):
    LOW = 1
    HIGH = 2


class Mode(str, enum.Enum):
    FAST = "fast"


class Permission(enum.Flag):
    R = 1
    W = 2


class Bits(enum.IntFlag):
    A = 1
    B = 4


class Box(Generic[T]):
    pass


class Greeter(typing.Protocol):
    """Its metaclass lives in typing, but it is an ordinary class to the codec."""

    def hi(self) -> str: ...


@dataclass
class WithDerived:
    seed: int = 1

    def __post_init__(self) -> None:
        self._double = self.seed * 2


class WithPrivate(BaseModel):
    value: int = 1
    _derived: int = PrivateAttr(default=0)

    def model_post_init(self, _context: Any) -> None:
        self._derived = self.value * 2


class RaisingHook:
    def cbor_encode(self) -> None:
        raise ValueError("no owner")

    @classmethod
    def cbor_decode(cls, data: Any) -> RaisingHook:
        return cls()


@dataclass(frozen=True)
class Frozen:
    a: int
    b: tuple[int, ...] = ()
    cache: int = field(default=7, init=False)


@dataclass(slots=True)
class Slotted:
    x: float
    tags: list[str]


class Settings(BaseModel):
    model_config = ConfigDict(extra="allow", populate_by_name=True)

    name: str = Field(alias="Name")
    count: int = 1
    nested: Slotted | None = None


class Temperature:
    def __init__(self, kelvin: float) -> None:
        self.kelvin = kelvin

    def cbor_encode(self) -> float:
        return self.kelvin

    @classmethod
    def cbor_decode(cls, data: float) -> Temperature:
        return cls(data)


class RemoteHandle:
    def __init__(self, name: str) -> None:
        self.name = name


class LiveHandle:
    """Decodes to a stand-in of another class, like a stream decoding to its remote twin."""

    def __init__(self, name: str) -> None:
        self.name = name

    def cbor_encode(self) -> str:
        return self.name

    @classmethod
    def cbor_decode(cls, data: str) -> RemoteHandle:
        return RemoteHandle(data)


class TaggedMeta(type):
    """Metaclass hooks serialize classes that are not importable by name."""

    def cbor_encode(cls) -> str:
        return cls.tag  # type: ignore[attr-defined]

    @classmethod
    def cbor_decode(cls, data: str) -> type:
        return _CLASSES_BY_TAG[data]


class Tagged(metaclass=TaggedMeta):
    tag = "tagged"


_CLASSES_BY_TAG = {"tagged": Tagged}


class Celsius:
    def __init__(self, degrees: float) -> None:
        self.degrees = degrees


def _decode_celsius(degrees: float) -> Celsius:
    return Celsius(degrees)


register_codec(Celsius, lambda c: c.degrees, _decode_celsius)


class Proxy:
    """Fabricates every attribute; the codec must never consult the instance."""

    def __init__(self) -> None:
        self.calls: list[str] = []

    def __getattr__(self, name: str) -> Any:
        self.calls.append(name)
        return lambda *args: None


class Outer:
    class Inner:
        pass


def module_fn(a: int, k: int = 0) -> int:
    return a + k


class Plain:
    def method(self) -> None:
        pass


@pytest.mark.parametrize(
    "value",
    [
        None,
        True,
        0,
        -5,
        2**70,
        1.5,
        math.inf,
        "text",
        b"bytes",
        [1, "a", [2.5, None]],
        {"a": 1, "b": {"c": [1, 2]}, 3: "int key", None: "none key"},
        (),
        (1, "two", (3.0,)),
        {1, (2, 3)},
        frozenset({"a", "b"}),
        Path("/tmp/x/y.txt"),
        Color.RED,
        Level.HIGH,
        Mode.FAST,
        Permission.R | Permission.W,
        Bits.A | Bits.B,
        Point(1, "y"),
        Frozen(1, (2, 3)),
        Slotted(1.5, ["t"]),
        Vector3(1, 2, 3),
        PoseStamped(ts=1.5, frame_id="odom", position=[1, 2, 3]),
    ],
)
def test_roundtrip_preserves_value_and_type(value: Any) -> None:
    out = roundtrip(value)
    assert out == value
    assert type(out) is type(value)


def test_nan_roundtrip() -> None:
    assert math.isnan(roundtrip(math.nan))


def test_nested_container_types_survive() -> None:
    value = {"t": (1, [2, (3,)]), "s": {frozenset({1})}, "l": [Point(1, "a")]}
    out = roundtrip(value)
    assert out == value
    assert type(out["t"]) is tuple
    assert type(out["t"][1]) is list
    assert type(out["t"][1][1]) is tuple
    assert type(next(iter(out["s"]))) is frozenset
    assert type(out["l"][0]) is Point


def test_dict_with_non_primitive_keys() -> None:
    value = {(1, 2): "tuple", Color.BLUE: "enum", np.float64(1.5): "np"}
    out = roundtrip(value)
    assert out == value
    assert [type(k) for k in out] == [tuple, Color, np.float64]


@pytest.mark.parametrize("dtype", [np.bool_, np.uint8, np.uint16, np.int32, np.float32, np.float64])
@pytest.mark.parametrize("shape", [(), (3,), (2, 3, 4)])
def test_ndarray_roundtrip(dtype: Any, shape: tuple[int, ...]) -> None:
    value = np.asarray(np.arange(math.prod(shape)).reshape(shape) % 2).astype(dtype)
    out = roundtrip(value)
    assert isinstance(out, np.ndarray)
    assert out.dtype == value.dtype
    assert out.shape == value.shape
    assert np.array_equal(out, value)
    assert out.flags.writeable


def test_fortran_ordered_array() -> None:
    value = np.asfortranarray(np.arange(6).reshape(2, 3))
    assert np.array_equal(roundtrip(value), value)


@pytest.mark.parametrize("value", [np.float64(1.5), np.int64(-2), np.bool_(True), np.str_("hi")])
def test_numpy_scalar_keeps_its_numpy_type(value: Any) -> None:
    out = roundtrip(value)
    assert type(out) is type(value)
    assert out == value


def test_dataclass_post_init_is_not_rerun() -> None:
    # __post_init__ pads the 2-tuple to 3 on construction; the receiver must get the
    # normalized value as is, not run __post_init__ on it again.
    loc = RobotLocation("kitchen", (1.0, 2.0), (0.5,), metadata={"k": [1]})  # type: ignore[arg-type]
    assert loc.position == (1.0, 2.0, 0.0)
    out = roundtrip(loc)
    assert out == loc
    assert type(out.position) is tuple


def test_frozen_dataclass_keeps_init_false_field() -> None:
    value = Frozen(1)
    object.__setattr__(value, "cache", 99)
    assert roundtrip(value).cache == 99


def test_pydantic_model_roundtrip() -> None:
    model = Settings(Name="a", extra_field=3, nested=Slotted(2.0, ["t"]))  # type: ignore[call-arg]
    out = roundtrip(model)
    assert type(out) is Settings
    assert out.name == "a"
    assert out.count == 1
    assert out.nested == Slotted(2.0, ["t"])
    assert out.model_fields_set == model.model_fields_set
    assert out.model_extra == {"extra_field": 3}


def test_global_config_roundtrip() -> None:
    config = GlobalConfig(robot_ip="10.0.0.1", n_workers=3)
    out = roundtrip(config)
    assert type(out) is GlobalConfig
    assert out.model_dump() == config.model_dump()


@pytest.mark.parametrize(
    "image",
    [
        Image(data=np.arange(4 * 6 * 3, dtype=np.uint8).reshape(4, 6, 3), format=ImageFormat.RGB),
        Image(data=np.full((2, 3), 700, dtype=np.uint16), format=ImageFormat.DEPTH16),
        Image(data=np.full((2, 3), 0.25, dtype=np.float32), format=ImageFormat.DEPTH),
        # The LCM form would label this rgb8 and decode it as uint8.
        Image(data=np.full((2, 3, 3), 0.5, dtype=np.float32), format=ImageFormat.RGB),
    ],
)
def test_image_roundtrip(image: Image) -> None:
    image.frame_id = "cam"
    image.ts = 1.5
    out = roundtrip(image)
    assert type(out) is Image
    assert out.format == image.format
    assert out.frame_id == "cam"
    assert out.ts == 1.5
    assert out.data.dtype == image.data.dtype
    assert np.array_equal(out.data, image.data)
    assert out.data.flags.writeable


def test_class_and_function_references() -> None:
    assert roundtrip(Outer.Inner) is Outer.Inner
    assert roundtrip(Greeter) is Greeter
    assert roundtrip(list[float]) == list[float]
    assert roundtrip(dict[str, tuple[int, ...]]) == dict[str, tuple[int, ...]]
    assert roundtrip(int | None) == int | None
    assert roundtrip(module_fn) is module_fn
    assert roundtrip(Color) is Color
    partial = functools.partial(module_fn, 1, k=2)
    out = roundtrip(partial)
    assert isinstance(out, functools.partial)
    assert out.func is module_fn
    assert (out.args, out.keywords) == ((1,), {"k": 2})
    assert out() == 3


def test_custom_protocol() -> None:
    out = roundtrip({"t": Temperature(273.15)})
    assert type(out["t"]) is Temperature
    assert out["t"].kelvin == 273.15


def test_custom_protocol_may_return_a_stand_in() -> None:
    out = roundtrip(LiveHandle("cam"))
    assert type(out) is RemoteHandle
    assert out.name == "cam"


def test_metaclass_hooks_serialize_class_objects() -> None:
    assert roundtrip(Tagged) is Tagged
    assert roundtrip([Tagged, Outer]) == [Tagged, Outer]


def test_registered_codec() -> None:
    out = roundtrip(Celsius(21.0))
    assert type(out) is Celsius
    assert out.degrees == 21.0


def test_proxy_getattr_is_never_consulted() -> None:
    proxy = Proxy()
    with pytest.raises(TypeError, match="Cannot CBOR-encode"):
        encode(proxy)
    assert proxy.calls == []


def _local_function() -> None:
    pass


@pytest.mark.parametrize(
    "value",
    [
        lambda: None,
        Plain(),
        Plain().method,
        threading.Lock(),
        MappingProxyType({}),
        np.array([object()]),
        np.zeros(2, dtype=[("a", "i4")]),
        bytearray(b"x"),
    ],
)
def test_unsupported_values_raise_type_error(value: Any) -> None:
    with pytest.raises(TypeError, match="Cannot CBOR-encode"):
        encode(value)


def test_local_definitions_raise_type_error() -> None:
    def inner() -> None:
        pass

    class Local:
        pass

    for value in (inner, Local, Local()):
        with pytest.raises(TypeError, match="Cannot CBOR-encode"):
            encode(value)


def test_error_names_the_path() -> None:
    with pytest.raises(TypeError, match=r"value\[1\]\['k'\]"):
        encode([1, {"k": threading.Lock()}])


def test_cyclic_reference_raises() -> None:
    cycle: list[Any] = []
    cycle.append(cycle)
    with pytest.raises(TypeError, match="cyclic"):
        encode(cycle)


def test_repeated_reference_is_not_a_cycle() -> None:
    shared = [1]
    assert roundtrip([shared, shared]) == [[1], [1]]


def test_register_codec_requires_module_level_decoder() -> None:
    with pytest.raises(TypeError, match="Cannot CBOR-encode"):
        register_codec(Plain, lambda p: None, lambda data: Plain())


@pytest.mark.parametrize(
    "payload",
    [
        CBORTag(TAG_DATACLASS, ["builtins:int", {}]),
        CBORTag(TAG_ENUM, ["builtins:int", "x"]),
        CBORTag(TAG_CLASS, "math:pi"),
        CBORTag(TAG_CLASS, "no_such_module_anywhere:X"),
        CBORTag(TAG_DIMOSMSG, ["builtins:int", b""]),
        CBORTag(999, 1),
    ],
)
def test_bad_payloads_raise_decode_error(payload: Any) -> None:
    with pytest.raises(CborDecodeError):
        decode(dumps(payload))


def test_truncated_input_raises_decode_error() -> None:
    with pytest.raises(CborDecodeError):
        decode(encode([1, 2, 3])[:-1])


@pytest.mark.parametrize(
    "form, expected",
    [
        (typing.List[int], list[int]),  # noqa: UP006
        (typing.Dict[str, typing.Any], dict[str, typing.Any]),  # noqa: UP006
        (typing.Optional[int], int | None),  # noqa: UP045
        (typing.Union[int, str, None], int | str | None),
        (typing.Literal["a", 1], typing.Literal["a", 1]),
        (typing.Literal["a"] | None, typing.Literal["a"] | None),
        (typing.Any, typing.Any),
        (collections.abc.Callable[[int], str], collections.abc.Callable[[int], str]),
        (Box[int], Box[int]),
        (tuple[()], tuple[()]),
    ],
)
def test_typing_forms_roundtrip(form: Any, expected: Any) -> None:
    assert roundtrip(form) == expected


def test_registered_codec_payload_cannot_name_its_own_decoder() -> None:
    # Before the fix this ran eval("6*7"); now only a type this process registered decodes.
    for type_ref in ("builtins:eval", "builtins:exec"):
        with pytest.raises(CborDecodeError, match="is not a class"):
            decode(dumps(CBORTag(TAG_REGISTERED, [type_ref, "builtins", "6*7"])))
    with pytest.raises(CborDecodeError, match="no codec registered"):
        decode(dumps(CBORTag(TAG_REGISTERED, ["builtins:int", "builtins", "6*7"])))
    with pytest.raises(CborDecodeError, match="cannot import"):
        decode(dumps(CBORTag(TAG_REGISTERED, [f"{__name__}:Celsius", "no_such_module_x", 1.0])))


@pytest.mark.parametrize(
    "payload",
    [
        CBORTag(TAG_UNION, [1, 2]),
        CBORTag(TAG_UNION, []),
        CBORTag(TAG_GENERIC_ALIAS, [1, [2]]),
        CBORTag(TAG_GENERIC_ALIAS, ["not a class", []]),
    ],
)
def test_type_form_payloads_must_hold_types(payload: Any) -> None:
    with pytest.raises(CborDecodeError):
        decode(dumps(payload))


def test_missing_dataclass_field_names_class_and_field() -> None:
    with pytest.raises(CborDecodeError, match=r"Frozen.*'b'"):
        decode(dumps(CBORTag(TAG_DATACLASS, [f"{__name__}:Frozen", {"a": 1}])))


def test_encode_only_raises_type_error() -> None:
    with pytest.raises(TypeError, match="Cannot CBOR-encode"):
        encode("\ud800")  # a lone surrogate, which cbor2 itself rejects
    with pytest.raises(TypeError, match=r"RaisingHook at value\[0\]: no owner"):
        encode([RaisingHook()])


def test_dataclass_derived_state_stays_behind() -> None:
    # The documented rule: fields only, __post_init__ does not run again.
    out = roundtrip(WithDerived(5))
    assert out.seed == 5
    assert not hasattr(out, "_double")


def test_pydantic_private_state_is_process_local() -> None:
    model = WithPrivate(value=3)
    model._derived = 99
    out = roundtrip(model)
    assert out.value == 3
    assert out._derived == 6  # model_post_init ran on the receiving side
