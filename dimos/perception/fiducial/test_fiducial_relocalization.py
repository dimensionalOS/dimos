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

"""Crucial-invariant test for the surveyed marker-map loader."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import pytest
import yaml

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.perception.fiducial.fiducial_relocalization import load_marker_map


def _survey(tmp_path: Path, payload: Any, suffix: str = "json") -> str:
    p = tmp_path / f"survey.{suffix}"
    p.write_text(json.dumps(payload) if suffix == "json" else yaml.safe_dump(payload))
    return str(p)


@pytest.mark.parametrize("suffix", ["json", "yaml"])
def test_load_marker_map_parses_schema_and_ignores_meta(tmp_path: Path, suffix: str) -> None:
    """Invariant 12 (parse): a survey (JSON or the YAML the derivation pipeline writes)
    becomes id(int) -> map_T_marker, keying by parsed int id, dropping meta, stamping
    frame_id='map' / child_frame_id='marker_<id>'. translation [x,y,z] and rotation
    [qx,qy,qz,qw] are read exactly. An empty markers block yields {}."""
    payload = {
        "meta": {"surveyed_by": "referee"},
        "markers": {"7": {"translation": [1.0, 2.0, 3.0], "rotation": [0.0, 0.0, 0.0, 1.0]}},
    }
    markers = load_marker_map(_survey(tmp_path, payload, suffix))

    assert set(markers) == {7} and isinstance(next(iter(markers)), int)  # int id, meta dropped
    t7 = markers[7]
    assert isinstance(t7, Transform)
    assert t7.frame_id == "map" and t7.child_frame_id == "marker_7"
    assert (t7.translation.x, t7.translation.y, t7.translation.z) == (1.0, 2.0, 3.0)
    assert (t7.rotation.x, t7.rotation.y, t7.rotation.z, t7.rotation.w) == (0.0, 0.0, 0.0, 1.0)
    assert load_marker_map(_survey(tmp_path, {"meta": {"note": "unsurveyed"}}, suffix)) == {}


@pytest.mark.parametrize(
    "translation, rotation, field, bad",
    [
        ([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 0.0], "rotation quaternion norm is", 0.0),  # zero-norm
        ([1.0, float("nan"), 0.0], [0.0, 0.0, 0.0, 1.0], "translation must be finite", None),
        ([1.0, 2.0], [0.0, 0.0, 0.0, 1.0], "translation", [1.0, 2.0]),  # short list
        ([0.0, 0.0, 0.0], "1,2,3,4", "rotation", "1,2,3,4"),  # not a list at all
    ],
)
def test_load_marker_map_rejects_malformed(
    tmp_path: Path, translation: Any, rotation: Any, field: str, bad: Any
) -> None:
    """Invariant 12 (reject): a malformed entry is caught AT LOAD -- an unnormalizable
    quaternion, a non-finite/short/non-list value -- with a ValueError naming marker 6
    and the field, so it can't silently zero-fill or blow up later in the composition.
    The message echoes the offending value verbatim (the 'got' half) where one exists."""
    payload = {"markers": {"6": {"translation": translation, "rotation": rotation}}}
    with pytest.raises(ValueError, match=r"marker 6") as exc:
        load_marker_map(_survey(tmp_path, payload))
    assert field in str(exc.value)
    if bad is not None:
        assert repr(bad) in str(exc.value)  # offending value echoed
