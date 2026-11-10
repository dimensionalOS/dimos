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

import re
import subprocess
import sys

import pytest

from dimos.core import dynamic_module
from dimos.core.dynamic_module import PortSpec, make_module_class
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.protocol.encode.cbor import decode, encode


class _BaseA(Module):
    existing_feed: In[Vector3]


class _BaseB(Module):
    pass


class _MarkerA:
    pass


class _MarkerB:
    pass


_PORTS = (
    PortSpec("sensor_ping", Vector3, "in"),
    PortSpec("operator_note", str, "out"),
)


def test_ports_in_any_order_give_identical_class():
    generated = make_module_class(_BaseA, _PORTS)
    again = make_module_class(_BaseA, tuple(reversed(_PORTS)))
    assert again is generated
    assert re.fullmatch(r"Dynamic_BaseA_[0-9a-f]{10}", generated.__name__)
    assert generated.__module__ == "dimos.core.dynamic_module"
    assert generated.__qualname__ == generated.__name__
    assert issubclass(generated, _BaseA)


def test_generated_class_has_ports_and_base_streams():
    generated = make_module_class(_BaseA, _PORTS)
    module = generated()
    try:
        assert sorted(module.inputs) == ["existing_feed", "sensor_ping"]
        assert sorted(module.outputs) == ["operator_note"]
        assert isinstance(module.sensor_ping, In)
        assert module.sensor_ping.type is Vector3
        assert isinstance(module.operator_note, Out)
        assert module.operator_note.type is str
    finally:
        module.stop()


def test_different_bases_give_different_classes():
    on_a = make_module_class(_BaseA, _PORTS)
    on_b = make_module_class(_BaseB, _PORTS)
    assert on_a is not on_b
    assert issubclass(on_b, _BaseB)
    assert not issubclass(on_b, _BaseA)


def test_different_message_types_give_different_classes():
    shape_a = make_module_class(_BaseB, [PortSpec("marker_feed", _MarkerA, "in")])
    shape_b = make_module_class(_BaseB, [PortSpec("marker_feed", _MarkerB, "in")])
    assert shape_a is not shape_b
    assert shape_a.__name__ != shape_b.__name__


def test_cbor_roundtrip_returns_identical_class():
    on_a = make_module_class(_BaseA, _PORTS)
    on_b = make_module_class(_BaseB, _PORTS)
    assert decode(encode(on_a)) is on_a
    assert decode(encode(on_b)) is on_b


def test_generated_class_is_published_for_reload_lookup():
    generated = make_module_class(_BaseA, _PORTS)
    assert getattr(sys.modules["dimos.core.dynamic_module"], generated.__name__) is generated


def test_memo_reset_returns_published_class(monkeypatch):
    generated = make_module_class(_BaseA, [PortSpec("memo_reset_feed", Vector3, "in")])
    monkeypatch.setattr(dynamic_module, "_memo", {})
    again = make_module_class(_BaseA, [PortSpec("memo_reset_feed", Vector3, "in")])
    assert again is generated


def test_digest_collision_raises(monkeypatch):
    shape_a = make_module_class(_BaseA, [PortSpec("collide_a", Vector3, "in")])
    shape_b = make_module_class(_BaseA, [PortSpec("collide_b", Vector3, "in")])
    monkeypatch.setattr(dynamic_module, "_memo", {})
    monkeypatch.setattr(dynamic_module, shape_a.__name__, shape_b)
    with pytest.raises(ValueError, match="digest collision"):
        make_module_class(_BaseA, [PortSpec("collide_a", Vector3, "in")])


_RELOAD_SCRIPT = """
import importlib, sys
from dimos.core.test_dynamic_module import _BaseA
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.protocol.encode.cbor import decode, encode
import dimos.core.dynamic_module as dynamic_module

C = dynamic_module.make_module_class(_BaseA, [dynamic_module.PortSpec("reload_feed", Vector3, "in")])
data = encode(C)
mod = importlib.reload(sys.modules["dimos.core.dynamic_module"])
assert getattr(mod, C.__name__) is C, "restart getattr lost the class"
assert decode(data) is C, "pre-reload payload lost identity"
assert decode(encode(C)) is C, "re-encode after reload failed"
again = mod.make_module_class(_BaseA, [mod.PortSpec("reload_feed", Vector3, "in")])
assert again is C, "factory lost identity after reload"
"""


def test_reload_then_reencode_preserves_identity():
    result = subprocess.run(
        [sys.executable, "-c", _RELOAD_SCRIPT], capture_output=True, text=True, timeout=120
    )
    assert result.returncode == 0, result.stderr


@pytest.mark.parametrize(
    ("ports", "expected", "match"),
    [
        (["not_a_spec"], TypeError, "expected PortSpec"),
        ([PortSpec(42, Vector3, "in")], TypeError, "must be str"),
        ([PortSpec("not valid", Vector3, "in")], ValueError, "identifier"),
        ([PortSpec("class", Vector3, "in")], ValueError, "identifier"),
        ([PortSpec("_hidden", Vector3, "in")], ValueError, "underscore"),
        (
            [PortSpec("existing_feed", Vector3, "in")],
            ValueError,
            "collides with an existing _BaseA",
        ),
        ([PortSpec("peek_stream", Vector3, "in")], ValueError, "collides"),
        (
            [PortSpec("dup", Vector3, "in"), PortSpec("dup", str, "out")],
            ValueError,
            "duplicate",
        ),
        ([PortSpec("bad_dir", Vector3, "sideways")], ValueError, "direction"),
        ([PortSpec("not_a_class", 42, "in")], TypeError, "must be a class"),
        ([PortSpec("generic_alias", list[int], "in")], TypeError, "must be a class"),
    ],
)
def test_invalid_ports_rejected(ports, expected, match):
    with pytest.raises(expected, match=match):
        make_module_class(_BaseA, ports)


def test_caller_reserved_names_rejected():
    with pytest.raises(ValueError, match="collides"):
        make_module_class(_BaseA, [PortSpec("handle", Vector3, "in")], reserved={"handle"})


def test_main_module_message_type_rejected():
    main_only = type("MainOnly", (), {"__module__": "__main__"})
    with pytest.raises(ValueError, match="__main__"):
        make_module_class(_BaseA, [PortSpec("main_feed", main_only, "in")])


def test_function_local_message_type_rejected():
    class LocalType:
        pass

    with pytest.raises(ValueError, match="inside a function"):
        make_module_class(_BaseA, [PortSpec("local_feed", LocalType, "in")])


def test_unimportable_message_type_rejected():
    ghost = type("GhostType", (), {"__module__": __name__})
    with pytest.raises(ValueError, match="sent by reference"):
        make_module_class(_BaseA, [PortSpec("ghost_feed", ghost, "in")])
