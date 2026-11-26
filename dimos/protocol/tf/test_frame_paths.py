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

from dimos.core.hardware_module import HardwareModule
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


def test_a_hardware_edge_is_one_a_hardware_module_names() -> None:
    hardware_ids = {("go2_A/go2", "go2_A/go2/base")}
    assert is_hardware_edge("go2_A/go2", "go2_A/go2/base", hardware_ids)
    assert not is_hardware_edge("odom", "go2_A/go2/base", hardware_ids)


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


class _Hardware(HardwareModule):
    """A HardwareModule's atom, made without a USD: only its class marks its edges as hardware."""


def _atoms(
    *declared: tuple[dict[str, str], list[tuple[str, str]]], hardware: tuple[int, ...] = ()
) -> list[Any]:
    from dimos.core.coordination.blueprints import Blueprint, autoconnect, config_defaults

    blueprint = autoconnect(
        *(
            Blueprint.create(
                _Hardware if index in hardware else _Edged, tf_frames_global=frames, tf_edges=edges
            ).namespace(f"m{index}")
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
            hardware=(0,),
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
