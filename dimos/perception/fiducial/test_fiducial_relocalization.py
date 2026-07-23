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
from pathlib import Path
from typing import Any

import pytest
import yaml

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.perception.fiducial.fiducial_relocalization import load_marker_map


def _survey(tmp_path: Path, payload: Any, suffix: str = "json") -> str:
    """Write a survey payload to a temp file of the given format; return its path."""
    p = tmp_path / f"survey.{suffix}"
    p.write_text(json.dumps(payload) if suffix == "json" else yaml.safe_dump(payload))
    return str(p)


@pytest.mark.parametrize("suffix", ["json", "yaml"])
def test_load_marker_map_parses_schema_and_ignores_meta(tmp_path: Path, suffix: str) -> None:
    """Invariant: load_marker_map turns a survey (JSON or the YAML the derivation pipeline
    writes) into id(int) -> map_T_marker, keying by parsed int id, dropping the meta block,
    and stamping each Transform frame_id='map' / child_frame_id='marker_<id>'. Translation
    is read [x,y,z] and rotation [qx,qy,qz,qw] exactly as written."""
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
    markers = load_marker_map(_survey(tmp_path, payload, suffix))

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


def test_load_marker_map_empty_and_missing_markers_block(tmp_path: Path) -> None:
    """Invariant: an empty markers block yields {}, and a file with no markers key
    at all is treated as empty rather than crashing -- meta-only surveys load clean."""
    for i, payload in enumerate(({"markers": {}}, {"meta": {"note": "not surveyed yet"}})):
        path = tmp_path / f"empty_{i}.json"
        path.write_text(json.dumps(payload))
        assert load_marker_map(str(path)) == {}


def test_load_marker_map_rejects_zero_norm_quaternion(tmp_path: Path) -> None:
    """Invariant: an unnormalizable (zero-norm) quaternion is caught at load with a
    clear message, not deferred to a later Quaternion.inverse() blow-up."""
    payload = {"markers": {"3": {"translation": [0.0, 0.0, 0.0], "rotation": [0.0, 0.0, 0.0, 0.0]}}}
    with pytest.raises(ValueError, match=r"marker 3: rotation quaternion norm is"):
        load_marker_map(_survey(tmp_path, payload))


def test_load_marker_map_rejects_non_finite_translation(tmp_path: Path) -> None:
    """Invariant: a non-finite translation coordinate (NaN/Inf) is rejected at load,
    since it would otherwise poison the map->world composition downstream."""
    payload = {
        "markers": {
            "5": {"translation": [1.0, float("nan"), 0.0], "rotation": [0.0, 0.0, 0.0, 1.0]}
        }
    }
    # json.dump emits NaN as the bare token NaN, which json.load reads back as float('nan').
    with pytest.raises(ValueError, match=r"marker 5: translation must be finite"):
        load_marker_map(_survey(tmp_path, payload))


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
    tmp_path: Path, translation: Any, rotation: Any, field: str, bad: Any
) -> None:
    """Invariant: a malformed translation/rotation raises ValueError that names the
    field AND echoes the offending value verbatim (the 'got' half of got-vs-want), so
    the survey line can be found and fixed. Covers both the wrong-length and the
    not-a-list (isinstance) rejection paths."""
    payload = {"markers": {"6": {"translation": translation, "rotation": rotation}}}
    with pytest.raises(ValueError) as exc:
        load_marker_map(_survey(tmp_path, payload))
    message = str(exc.value)
    assert f"marker 6: {field}" in message  # field named
    assert repr(bad) in message  # offending value echoed verbatim
