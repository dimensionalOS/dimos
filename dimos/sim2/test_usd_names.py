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

"""A description names a body; a model read from USD calls it something longer."""

import pytest

from dimos.sim2.usd_names import resolve, resolved, shorten

USD_NAMES = [
    "world",
    "/g1/Joints/pelvis",
    "/g1/Joints/left_knee_link",
    "/office_555/Geometry/box_1",
]


def test_a_short_name_finds_the_prim_path_it_refers_to():
    assert resolve(USD_NAMES, "pelvis") == "/g1/Joints/pelvis"
    assert resolve(USD_NAMES, "box_1") == "/office_555/Geometry/box_1"


def test_a_name_that_is_already_whole_is_taken_as_written():
    # a model that really does call something this means it, so it wins over any tail match
    assert resolve(["/a/pelvis", "pelvis"], "pelvis") == "pelvis"
    assert resolve(USD_NAMES, "/g1/Joints/pelvis") == "/g1/Joints/pelvis"


def test_mjcf_names_are_untouched():
    """The same lookup has to keep working for a model that never saw USD."""
    assert resolve(["world", "base", "neck"], "base") == "base"


def test_two_of_the_same_short_name_is_an_error_not_a_guess():
    with pytest.raises(KeyError, match="ambiguous"):
        resolve(["/left/wheel", "/right/wheel"], "wheel")


def test_a_miss_says_what_there_was():
    with pytest.raises(KeyError, match="pelvis"):
        resolve(["/a/base"], "pelvis")
    # and enough of the available names to see their shape, without printing a whole scene
    with pytest.raises(KeyError, match=r"\.\.\. \(5 more\)"):
        resolve([f"/scene/thing_{i}" for i in range(11)], "nothing")


def test_resolved_hands_back_none_where_resolve_would_raise():
    assert resolved(USD_NAMES, "pelvis") == "/g1/Joints/pelvis"
    assert resolved(USD_NAMES, "nothing") is None
    assert resolved(["/a/x", "/b/x"], "x") is None


class _Element:
    """Enough of a MjSpec element to rename: a name, and nothing else."""

    def __init__(self, name: str) -> None:
        self.name = name


class _Reference:
    def __init__(self, **fields: str) -> None:
        self.__dict__.update(fields)


class _Spec:
    """Enough of a MjSpec to shorten: the collections it renames and the ones that refer to them."""

    def __init__(self, **kinds: list) -> None:
        for kind in ("bodies", "joints", "sites", "cameras", "actuators", "sensors", "pairs"):
            setattr(self, kind, kinds.get(kind, []))


def test_shortening_renames_a_models_parts_to_the_names_a_description_uses():
    spec = _Spec(
        bodies=[_Element("world"), _Element("/toy/Geometry/base")],
        joints=[_Element("/toy/Geometry/base/head/neck")],
    )

    renames = shorten(spec)

    assert [body.name for body in spec.bodies] == ["world", "base"]
    assert [joint.name for joint in spec.joints] == ["neck"]
    assert renames == {"/toy/Geometry/base": "base", "/toy/Geometry/base/head/neck": "neck"}


def test_a_reference_follows_the_part_it_names():
    """An actuator names the joint it drives as a string, and renaming the joint does not touch it.

    Left behind, the compiled world comes out with no actuators at all and nothing says why.
    """
    spec = _Spec(
        joints=[_Element("/toy/Geometry/base/head/neck")],
        actuators=[_Reference(name="/toy/Physics/neck", target="/toy/Geometry/base/head/neck")],
        sensors=[_Reference(name="gyro", objname="/toy/Geometry/base", refname="")],
        bodies=[_Element("/toy/Geometry/base")],
    )

    shorten(spec)

    assert spec.actuators[0].target == "neck"
    assert spec.actuators[0].name == "neck"
    assert spec.sensors[0].objname == "base"


def test_a_hinge_and_the_motor_on_it_may_share_a_name():
    """MuJoCo's namespaces are per kind, and the toy robot's joint and actuator are both "neck".

    Judged together each was the other's ambiguity, so neither was shortened.
    """
    spec = _Spec(
        joints=[_Element("/toy/Geometry/base/head/neck")],
        actuators=[_Reference(name="/toy/Physics/neck", target="/toy/Geometry/base/head/neck")],
    )

    shorten(spec)

    assert spec.joints[0].name == "neck"
    assert spec.actuators[0].name == "neck"


def test_two_parts_of_one_kind_ending_the_same_way_keep_their_paths():
    spec = _Spec(bodies=[_Element("/robot/left/wheel"), _Element("/robot/right/wheel")])

    assert shorten(spec) == {}
    assert [body.name for body in spec.bodies] == ["/robot/left/wheel", "/robot/right/wheel"]


def test_a_short_name_something_else_already_holds_is_left_alone():
    spec = _Spec(bodies=[_Element("base"), _Element("/toy/Geometry/base")])

    assert shorten(spec) == {}
