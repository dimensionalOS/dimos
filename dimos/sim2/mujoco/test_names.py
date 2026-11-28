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

from types import SimpleNamespace

import pytest

from dimos.sim2.mujoco.names import resolve, resolved, shorten

USD_NAMES = [
    "world",
    "/g1/Joints/pelvis",
    "/g1/Joints/left_knee_link",
    "/office_555/Geometry/box_1",
]


@pytest.mark.parametrize(
    "names, wanted, found",
    [
        (USD_NAMES, "pelvis", "/g1/Joints/pelvis"),
        (USD_NAMES, "box_1", "/office_555/Geometry/box_1"),
        # A name that is already whole is taken as written, and MJCF names are untouched.
        (["/a/pelvis", "pelvis"], "pelvis", "pelvis"),
        (USD_NAMES, "/g1/Joints/pelvis", "/g1/Joints/pelvis"),
        (["world", "base", "neck"], "base", "base"),
        # Shortened, each body keeps only its owner (scene/spot) in front.
        (
            [f"office_555/{spot}/pelvis" for spot in ("main", "left", "right")],
            "office_555/left/g1_29dof_rev_1_0/Geometry/pelvis",
            "office_555/left/pelvis",
        ),
        # A frame is told from its namesakes by every namespace it sits under.
        (
            ["/room/dog/imu/sim_imu/imu_link", "/room/dog/livox/mid360/imu_link"],
            "dog/imu/imu_link",
            "/room/dog/imu/sim_imu/imu_link",
        ),
        (["dog/imu_link"], "dog/imu/imu_link", "dog/imu_link"),
        # An ambiguous name or a miss is an error that says what there was, never a guess.
        (["/left/wheel", "/right/wheel"], "wheel", "ambiguous"),
        (["/a/base"], "pelvis", "pelvis"),
        ([f"/scene/thing_{i}" for i in range(11)], "nothing", r"\.\.\. \(5 more\)"),
    ],
)
def test_a_name_resolves_to_the_one_prim_path_it_refers_to(
    names: list[str], wanted: str, found: str
) -> None:
    # A name in the list is a resolution; anything else is the error it raises.
    if found in names:
        assert resolve(names, wanted) == resolved(names, wanted) == found
    else:
        with pytest.raises(KeyError, match=found):
            resolve(names, wanted)
        assert resolved(names, wanted) is None


def _spec(**kinds: list) -> SimpleNamespace:
    every = ("bodies", "joints", "sites", "cameras", "actuators", "sensors", "pairs")
    return SimpleNamespace(**{kind: kinds.get(kind, []) for kind in every})


def _named(*names: str) -> list[SimpleNamespace]:
    return [SimpleNamespace(name=name) for name in names]


@pytest.mark.parametrize(
    "bodies, ids, shortened",
    [
        (["world", "/toy/Geometry/base"], (), ["world", "base"]),
        # Two ending the same way, or a short name something else holds, keep their paths.
        (["/robot/left/wheel", "/robot/right/wheel"], (), None),
        (["base", "/toy/Geometry/base"], (), None),
        # Three of one robot in a composed world keep their ids; the world around them its names.
        (
            ["/office_555/left/g1/Geometry/pelvis", "/office_555/main/g1/Geometry/pelvis"],
            ("left", "main"),
            ["left/pelvis", "main/pelvis"],
        ),
        (["/office_555/Geometry/desk_1"], ("left",), None),
        # An id matches a whole path segment and not part of one.
        (["/scene/left_arm/Geometry/base"], ("left",), None),
    ],
)
def test_shortening_renames_a_body_only_where_its_short_name_is_its_own(
    bodies: list[str], ids: tuple[str, ...], shortened: list[str] | None
) -> None:
    spec = _spec(bodies=_named(*bodies))
    renames = shorten(spec, ids=ids)
    shortened = shortened or bodies
    assert [body.name for body in spec.bodies] == shortened
    assert renames == {old: new for old, new in zip(bodies, shortened, strict=True) if old != new}


def test_shortening_renames_joints_and_the_references_that_name_them() -> None:
    spec = _spec(
        bodies=_named("/toy/Geometry/base"),
        joints=_named("/toy/Geometry/base/head/neck"),
        actuators=[
            SimpleNamespace(name="/toy/Physics/neck", target="/toy/Geometry/base/head/neck")
        ],
        sensors=[SimpleNamespace(name="gyro", objname="/toy/Geometry/base", refname="")],
    )
    shorten(spec)
    # A hinge and the motor on it may share a name.
    assert [spec.joints[0].name, spec.actuators[0].name, spec.actuators[0].target] == ["neck"] * 3
    assert spec.sensors[0].objname == "base"
