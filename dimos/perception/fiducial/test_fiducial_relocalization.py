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

import json
import math
import os
import tempfile
from typing import Any

import pytest
import yaml

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.perception.fiducial.fiducial_relocalization import load_marker_map


def _write_marker_json(payload: Any) -> str:
    """Serialize a survey payload to a temp JSON file; caller unlinks in finally."""
    fd, path = tempfile.mkstemp(suffix=".json")
    with os.fdopen(fd, "w") as f:
        json.dump(payload, f)
    return path


def _write_marker_yaml(payload: Any) -> str:
    """Serialize a survey payload to a temp YAML file; caller unlinks in finally."""
    fd, path = tempfile.mkstemp(suffix=".yaml")
    with os.fdopen(fd, "w") as f:
        yaml.safe_dump(payload, f)
    return path


def test_load_marker_map_parses_schema_and_ignores_meta() -> None:
    """Invariant: load_marker_map turns the survey JSON into id(int) -> map_T_marker,
    keying by parsed int id, ignoring the meta block entirely, and stamping each
    Transform frame_id='map' / child_frame_id='marker_<id>'. Translation is read
    [x,y,z] and rotation [qx,qy,qz,qw] exactly as written."""
    half_sqrt2 = math.sqrt(2.0) / 2.0  # 90 deg about +z as [qx,qy,qz,qw]
    payload = {
        "meta": {"surveyed_by": "referee", "date": "2026-07-20", "units": "m"},
        "markers": {
            "7": {"translation": [1.0, 2.0, 3.0], "rotation": [0.0, 0.0, 0.0, 1.0]},
            "12": {
                "translation": [-0.5, 4.0, 0.25],
                "rotation": [0.0, 0.0, half_sqrt2, half_sqrt2],
            },
        },
    }
    path = _write_marker_json(payload)
    try:
        markers = load_marker_map(path)

        assert set(markers) == {7, 12}
        assert all(isinstance(k, int) for k in markers)  # ids parsed to int, meta dropped

        t7 = markers[7]
        assert isinstance(t7, Transform)
        assert t7.frame_id == "map"
        assert t7.child_frame_id == "marker_7"
        assert (t7.translation.x, t7.translation.y, t7.translation.z) == (1.0, 2.0, 3.0)
        assert (t7.rotation.x, t7.rotation.y, t7.rotation.z, t7.rotation.w) == (0.0, 0.0, 0.0, 1.0)

        t12 = markers[12]
        assert t12.child_frame_id == "marker_12"
        assert (t12.translation.x, t12.translation.y, t12.translation.z) == (-0.5, 4.0, 0.25)
        assert (t12.rotation.z, t12.rotation.w) == (half_sqrt2, half_sqrt2)
    finally:
        os.unlink(path)


def test_load_marker_map_parses_yaml_survey() -> None:
    """Invariant: a ``.yaml`` survey loads identically to JSON -- the runtime loader
    accepts the format the derivation pipeline writes (RECORDING.markers.yaml) and the
    eval overlay reads, so an existing YAML marker survey handed to marker_map_file
    still builds map_T_marker. Same schema, same int keys, same [x,y,z]/[qx,qy,qz,qw]."""
    payload = {
        "markers": {
            "10": {
                "translation": [-4.1657, -30.985, 0.2677],
                "rotation": [0.1958, -0.6668, -0.6949, 0.1847],
            }
        }
    }
    path = _write_marker_yaml(payload)
    try:
        markers = load_marker_map(path)

        assert set(markers) == {10}
        assert isinstance(next(iter(markers)), int)  # id parsed to int
        t10 = markers[10]
        assert t10.frame_id == "map"
        assert t10.child_frame_id == "marker_10"
        assert (t10.translation.x, t10.translation.y, t10.translation.z) == (
            -4.1657,
            -30.985,
            0.2677,
        )
        assert (t10.rotation.x, t10.rotation.y, t10.rotation.z, t10.rotation.w) == (
            0.1958,
            -0.6668,
            -0.6949,
            0.1847,
        )
    finally:
        os.unlink(path)


def test_load_marker_map_yaml_survey_validates() -> None:
    """Invariant: the loud validation is format-agnostic -- a malformed YAML entry
    (short rotation) raises the same ValueError as JSON, so YAML surveys are not a
    silent bypass around the survey-quality gate."""
    payload = {"markers": {"9": {"translation": [0.0, 0.0, 0.0], "rotation": [0.0, 0.0, 1.0]}}}
    path = _write_marker_yaml(payload)
    try:
        with pytest.raises(ValueError, match=r"marker 9: rotation must be \[x, y, z, w\]"):
            load_marker_map(path)
    finally:
        os.unlink(path)


def test_load_marker_map_empty_and_missing_markers_block() -> None:
    """Invariant: an empty markers block yields {}, and a file with no markers key
    at all is treated as empty rather than crashing -- meta-only surveys load clean."""
    for payload in ({"markers": {}}, {"meta": {"note": "not surveyed yet"}}):
        path = _write_marker_json(payload)
        try:
            assert load_marker_map(path) == {}
        finally:
            os.unlink(path)


def test_load_marker_map_rejects_zero_norm_quaternion() -> None:
    """Invariant: an unnormalizable (zero-norm) quaternion is caught at load with a
    clear message, not deferred to a later Quaternion.inverse() blow-up."""
    payload = {"markers": {"3": {"translation": [0.0, 0.0, 0.0], "rotation": [0.0, 0.0, 0.0, 0.0]}}}
    path = _write_marker_json(payload)
    try:
        with pytest.raises(ValueError, match=r"marker 3: rotation quaternion norm is"):
            load_marker_map(path)
    finally:
        os.unlink(path)


def test_load_marker_map_rejects_non_finite_translation() -> None:
    """Invariant: a non-finite translation coordinate (NaN/Inf) is rejected at load,
    since it would otherwise poison the map->world composition downstream."""
    payload = {
        "markers": {"5": {"translation": [1.0, float("nan"), 0.0], "rotation": [0.0, 0.0, 0.0, 1.0]}}
    }
    # json.dump emits NaN as the bare token NaN, which json.load reads back as float('nan').
    path = _write_marker_json(payload)
    try:
        with pytest.raises(ValueError, match=r"marker 5: translation must be finite"):
            load_marker_map(path)
    finally:
        os.unlink(path)


@pytest.mark.parametrize("dropped", ["translation", "rotation"])
def test_load_marker_map_missing_key_names_the_key(dropped: str) -> None:
    """Invariant: whichever required key is absent, the KeyError names THAT key -- the
    operator must be told which field of the survey entry is missing, not left to guess
    (a bare 'half-built pose' would ship a silently-wrong tag instead)."""
    entry = {"translation": [0.0, 0.0, 0.0], "rotation": [0.0, 0.0, 0.0, 1.0]}
    del entry[dropped]
    path = _write_marker_json({"markers": {"8": entry}})
    try:
        # KeyError stringifies to the repr of the missing key, e.g. "'rotation'".
        with pytest.raises(KeyError, match=rf"'{dropped}'"):
            load_marker_map(path)
    finally:
        os.unlink(path)


@pytest.mark.parametrize(
    "translation, rotation, field, bad",
    [
        ([1.0, 2.0], [0.0, 0.0, 0.0, 1.0], "translation", [1.0, 2.0]),  # short list
        ([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], "rotation", [0.0, 0.0, 1.0]),  # short list
        ("1,2,3", [0.0, 0.0, 0.0, 1.0], "translation", "1,2,3"),  # not a list at all
        ([0.0, 0.0, 0.0], 5.0, "rotation", 5.0),  # not a list at all
    ],
)
def test_load_marker_map_malformed_message_echoes_got_value(
    translation: Any, rotation: Any, field: str, bad: Any
) -> None:
    """Invariant: a malformed translation/rotation raises ValueError that names the
    field AND echoes the offending value verbatim (the 'got' half of got-vs-want), so
    the survey line can be found and fixed. Covers both the wrong-length and the
    not-a-list (isinstance) rejection paths."""
    path = _write_marker_json(
        {"markers": {"6": {"translation": translation, "rotation": rotation}}}
    )
    try:
        with pytest.raises(ValueError) as exc:
            load_marker_map(path)
        message = str(exc.value)
        assert f"marker 6: {field}" in message  # field named
        assert repr(bad) in message  # offending value echoed verbatim
    finally:
        os.unlink(path)
