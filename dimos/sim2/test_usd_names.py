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

from dimos.sim2.usd_names import resolve, resolved

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
