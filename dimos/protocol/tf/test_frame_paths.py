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

from typing import Any

import pytest

from dimos.core.tf_module import TfModule, TfModuleConfig
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.protocol.tf.frame_paths import check_frame_path, frame_path_error, is_hardware_edge
from dimos.protocol.tf.tf import MultiTBuffer


@pytest.mark.parametrize(
    "path,complaint",
    [
        ("go2/base", None),
        ("_x/y_2", None),
        ("", "empty"),
        ("go2//base", "empty segment"),
        ("go2/2base", "'2base' starts with '2'"),
        ("go2/front-cam", "'front-cam' has at position 5 '-'"),
    ],
)
def test_a_frame_path_is_prim_names_joined_by_slashes(path: str, complaint: str | None) -> None:
    error = frame_path_error(path)
    if complaint is None:
        assert error is None
        check_frame_path(path, "the frame")
    else:
        assert error is not None and complaint in error
        with pytest.raises(ValueError, match="the frame"):
            check_frame_path(path, "the frame")


def test_a_hardware_edge_is_one_whose_both_ends_arrived_with_a_slash() -> None:
    assert is_hardware_edge("/go2_A/go2", "/go2_A/go2/base")
    assert not is_hardware_edge("odom", "/go2_A/go2/base")
    assert not is_hardware_edge("go2_A/go2", "go2_A/go2/base")


def test_a_buffer_looks_a_slashed_wire_frame_up_without_its_slash() -> None:
    buffer = MultiTBuffer()
    wire = Transform(
        translation=Vector3(1.0, 0.0, 0.0), frame_id="/a", child_frame_id="/a/b", ts=1.0
    )
    buffer.receive_transform(wire)

    found = buffer.get("a", "a/b", time_tolerance=10.0)
    assert found is not None and found.translation.x == pytest.approx(1.0)
    assert buffer.get_frames() == {"a", "a/b"}
    assert (wire.frame_id, wire.child_frame_id) == ("/a", "/a/b"), (
        "what other subscribers got is untouched"
    )


def test_an_odometry_edge_without_slashes_resolves_through_to_hardware_frames() -> None:
    buffer = MultiTBuffer()
    buffer.receive_transform(
        Transform(
            translation=Vector3(2.0, 0.0, 0.0), frame_id="odom", child_frame_id="go2/base", ts=1.0
        ),
        Transform(
            translation=Vector3(0.0, 0.0, 0.1),
            frame_id="/go2/base",
            child_frame_id="/go2/base/lidar",
            ts=1.0,
        ),
    )

    lidar = buffer.get("odom", "go2/base/lidar", time_tolerance=10.0)
    assert lidar is not None
    assert (lidar.translation.x, lidar.translation.z) == pytest.approx((2.0, 0.1))


def test_a_frame_with_a_hardware_and_an_imaginary_parent_is_found_both_ways() -> None:
    buffer = MultiTBuffer()
    buffer.receive_transform(
        Transform(
            translation=Vector3(0.0, 0.0, 0.3), frame_id="go2", child_frame_id="go2/base", ts=1.0
        ),
        Transform(
            translation=Vector3(2.0, 0.0, 0.0), frame_id="odom", child_frame_id="go2/base", ts=1.0
        ),
        Transform(
            translation=Vector3(0.1, 0.0, 0.0),
            frame_id="go2/base",
            child_frame_id="go2/base/lidar",
            ts=1.0,
        ),
    )

    lidar = buffer.get("odom", "go2/base/lidar", time_tolerance=10.0)
    assert lidar is not None
    assert lidar.translation.x == pytest.approx(2.1)
    top = buffer.get("odom", "go2", time_tolerance=10.0)
    assert top is not None
    assert (top.translation.x, top.translation.z) == pytest.approx((2.0, -0.3))


class _Edges(TfModuleConfig):
    pass


class _Edged(TfModule):
    config: _Edges


class _Slashed(_Edged):
    """Puts its "top" and "base" frames on the wire with a slash, as a HardwareModule does its own."""

    @classmethod
    def blueprint_wire_roles(cls, atom: Any) -> set[str]:
        return {"top", "base"}


def _atoms(
    *declared: tuple[dict[str, str], list[tuple[str, str]]], slashed: tuple[int, ...] = ()
) -> list[Any]:
    from dimos.core.coordination.blueprints import autoconnect, config_defaults

    blueprint = autoconnect(
        *(
            (_Slashed if index in slashed else _Edged)
            .blueprint(tf_frames_global=frames, tf_edges=edges)
            .namespace(f"m{index}")
            for index, (frames, edges) in enumerate(declared)
        )
    )
    from dataclasses import replace

    return [
        replace(atom, kwargs={**config_defaults(atom.module), **atom.kwargs})
        for atom in blueprint.solidify().active_blueprints
    ]


def test_a_hardware_and_an_imaginary_parent_are_allowed() -> None:
    TfModule.blueprint_checks(
        _atoms(
            ({"top": "go2", "base": "go2/base"}, [("top", "base")]),
            ({"odom": "odom", "base": "go2/base"}, [("odom", "base")]),
            slashed=(0,),
        )
    )


def test_two_imaginary_parents_are_refused() -> None:
    with pytest.raises(
        ValueError, match=r"'go2/base' has several parents: map .*odom .*\(all imaginary"
    ):
        TfModule.blueprint_checks(
            _atoms(
                ({"map": "map", "base": "go2/base"}, [("map", "base")]),
                ({"odom": "odom", "base": "go2/base"}, [("odom", "base")]),
            )
        )


def test_an_edge_closing_a_loop_is_refused_even_with_one_parent_each() -> None:
    # odom -> go2/base, go2 -> go2/base and odom -> go2: no frame has two parents of one kind, but they loop.
    with pytest.raises(ValueError, match=r"form a cycle: .*closes the loop"):
        TfModule.blueprint_checks(
            _atoms(
                ({"top": "go2", "base": "go2/base"}, [("top", "base")]),
                (
                    {"odom": "odom", "base": "go2/base", "top": "go2"},
                    [("odom", "base"), ("odom", "top")],
                ),
            )
        )


def test_a_namespace_is_a_prim_name_so_it_cannot_start_with_a_digit() -> None:
    with pytest.raises(ValueError, match="not starting with a digit"):
        _Edged.blueprint().namespace("2nd")
    assert _Edged.blueprint().namespace("_2nd").namespace("robot0")
