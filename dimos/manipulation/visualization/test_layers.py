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

"""Tests for backend-neutral visualization layers."""

from dataclasses import FrozenInstanceError

import numpy as np
import pytest

from dimos.manipulation.visualization.layers import (
    LineSetElement,
    PointCloudElement,
    VisualizationLayer,
)


def test_point_cloud_snapshots_positions_and_normalizes_colors() -> None:
    points = np.asarray([[1.0, 2.0, 3.0]], dtype=np.float64)
    colors = np.asarray([[0.0, 0.5, 1.0]], dtype=np.float32)

    element = PointCloudElement("object", points, colors, point_size=0.005)
    points[:] = 9.0
    colors[:] = 0.0

    np.testing.assert_array_equal(element.points, [[1.0, 2.0, 3.0]])
    np.testing.assert_array_equal(element.colors, [[0, 128, 255]])
    assert element.points.dtype == np.float32
    assert element.points.flags.writeable is False
    assert element.colors is not None and element.colors.flags.writeable is False
    with pytest.raises(ValueError, match="read-only"):
        element.points[0, 0] = 4.0
    with pytest.raises(FrozenInstanceError):
        element.point_size = 1.0  # type: ignore[misc]


def test_line_set_accepts_uniform_and_per_line_colors() -> None:
    vertices = np.asarray([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0]])
    edges = np.asarray([[0, 1], [1, 2]])

    uniform = LineSetElement("uniform", vertices, edges, colors=np.asarray([255, 0, 0]))
    per_line = LineSetElement(
        "rank-1",
        vertices,
        edges,
        colors=np.asarray([[0, 255, 0], [255, 128, 0]]),
        line_width=2.0,
    )

    np.testing.assert_array_equal(uniform.colors, [255, 0, 0])
    np.testing.assert_array_equal(per_line.colors, [[0, 255, 0], [255, 128, 0]])
    assert per_line.edges.dtype == np.int32
    assert per_line.edges.flags.writeable is False


@pytest.mark.parametrize("value", ["", "/grasp", "grasp/", "grasp//cloud", "grasp cloud"])
def test_layer_rejects_invalid_id(value: str) -> None:
    with pytest.raises(ValueError, match="layer ID|visualization ID"):
        VisualizationLayer(value, "world", ())


@pytest.mark.parametrize("value", ["", "rank/1", "rank 1"])
def test_element_rejects_invalid_id(value: str) -> None:
    with pytest.raises(ValueError, match="element ID|visualization ID"):
        PointCloudElement(value, np.empty((0, 3)))


@pytest.mark.parametrize(
    ("kwargs", "message"),
    [
        ({"points": np.zeros((3,))}, "shape"),
        ({"points": np.asarray([[np.nan, 0.0, 0.0]])}, "finite"),
        (
            {
                "points": np.zeros((2, 3)),
                "colors": np.zeros((1, 3)),
            },
            "colors",
        ),
        (
            {
                "points": np.zeros((1, 3)),
                "colors": np.asarray([[256, 0, 0]]),
            },
            "colors",
        ),
        ({"points": np.zeros((1, 3)), "point_size": 0.0}, "positive"),
    ],
)
def test_point_cloud_rejects_invalid_geometry(kwargs: dict[str, object], message: str) -> None:
    with pytest.raises(ValueError, match=message):
        PointCloudElement("cloud", **kwargs)  # type: ignore[arg-type]


@pytest.mark.parametrize(
    ("edges", "message"),
    [
        (np.zeros((2, 3)), "shape"),
        (np.asarray([[0.5, 1.0]]), "integer"),
        (np.asarray([[-1, 0]]), "out-of-range"),
        (np.asarray([[0, 2]]), "out-of-range"),
    ],
)
def test_line_set_rejects_invalid_edges(edges: np.ndarray, message: str) -> None:
    with pytest.raises(ValueError, match=message):
        LineSetElement(
            "lines",
            np.asarray([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]),
            edges,
        )


def test_line_set_rejects_invalid_appearance() -> None:
    with pytest.raises(ValueError, match="colors"):
        LineSetElement(
            "lines",
            np.asarray([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]),
            np.asarray([[0, 1]]),
            colors=np.asarray([[0, 0, 0], [255, 255, 255]]),
        )
    with pytest.raises(ValueError, match="positive"):
        LineSetElement(
            "lines",
            np.asarray([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]),
            np.asarray([[0, 1]]),
            line_width=float("nan"),
        )


def test_layer_rejects_empty_frame_and_duplicate_elements() -> None:
    element = PointCloudElement("object", np.empty((0, 3)))
    with pytest.raises(ValueError, match="frame_id"):
        VisualizationLayer("grasp/object-cloud", " ", (element,))
    with pytest.raises(ValueError, match="unique"):
        VisualizationLayer("grasp/object-cloud", "world", (element, element))


def test_layer_owns_element_tuple() -> None:
    source = [PointCloudElement("object", np.empty((0, 3)))]

    layer = VisualizationLayer("grasp/object-cloud", "world", source)  # type: ignore[arg-type]
    source.clear()

    assert [item.id for item in layer.elements] == ["object"]
