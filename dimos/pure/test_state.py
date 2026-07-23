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

"""pm.State: frozen kw-only construction, public replace(), and Mealy threading."""

import dataclasses
import sys

import pytest

from dimos.pure import pm


class Point(pm.State):
    x: int = 0
    y: int = 0


def test_defaults_and_kw_only():
    p = Point()
    assert (p.x, p.y) == (0, 0)
    assert Point(x=1, y=2) == Point(y=2, x=1)
    with pytest.raises(TypeError):
        Point(1, 2)  # positional construction is rejected — kw-only


def test_frozen():
    p = Point(x=1)
    with pytest.raises(dataclasses.FrozenInstanceError):
        p.x = 9  # type: ignore[misc]


def test_replace_returns_new_leaves_original():
    p = Point(x=1, y=2)
    q = p.replace(x=5)
    assert q == Point(x=5, y=2)
    assert p == Point(x=1, y=2)  # original untouched
    assert q is not p


def test_replace_multi_field():
    assert Point(x=1, y=2).replace(x=7, y=8) == Point(x=7, y=8)


def test_replace_unknown_field_names_it():
    with pytest.raises(TypeError, match="bogus"):
        Point(x=1).replace(bogus=1)


def test_replace_dunder_is_the_protocol():
    p = Point(x=1)
    assert p.__replace__(y=3) == Point(x=1, y=3)


def test_equality_hash_repr():
    assert Point(x=1, y=2) == Point(x=1, y=2)
    assert Point(x=1) != Point(x=2)
    assert hash(Point(x=1, y=2)) == hash(Point(x=1, y=2))
    assert repr(Point(x=1, y=2)) == "Point(x=1, y=2)"


def test_reserved_name_guard():
    with pytest.raises(TypeError, match="'replace' is reserved"):

        class Bad(pm.State):
            replace: int = 0  # would shadow State.replace


def test_inheritance_extends_fields():
    class Point3(Point):
        z: int = 0

    p = Point3(x=1, y=2, z=3)
    assert (p.x, p.y, p.z) == (1, 2, 3)
    assert p.replace(z=9) == Point3(x=1, y=2, z=9)


@pytest.mark.skipif(sys.version_info < (3, 13), reason="copy.replace is 3.13+")
def test_copy_replace():
    import copy

    p = Point(x=1, y=2)
    assert copy.replace(p, x=5) == Point(x=5, y=2)


# ── e2e: a Mealy module threads pm.State through over() ───────────────────────


@dataclasses.dataclass(frozen=True)
class Sample:
    ts: float
    v: int


class Accum(pm.PureModule):
    class In(pm.In):
        sample: Sample = pm.tick()

    class Out(pm.Out):
        total: int = pm.contract(min_hz=1)

    class State(pm.State):
        total: int = 0  # running sum — plain data, default-constructible

    def step(self, s: State, i: In) -> tuple[State, Out]:
        s = s.replace(total=s.total + i.sample.v)
        return s, Accum.Out(total=s.total)


def test_mealy_state_classifies():
    spec = pm.step_spec(Accum)
    assert spec.kind is pm.StepKind.MEALY
    assert spec.state_type is Accum.State  # pm.State accepted as Mealy State


# State threading through a live over() run is covered where the driver lands.
