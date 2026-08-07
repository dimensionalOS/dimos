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

"""FREE v1 reader tests, on a synthetic blob so no data download is needed."""

from __future__ import annotations

import struct

import numpy as np
import pytest

from dimos.navigation.motion.simulation.policy import (
    BAND_0_1,
    BAND_1_5,
    BAND_ROTATE,
    FreePolicy,
)

OBS, ACT, HIST, ENC_VEL, ENC_LAT = 4, 3, 2, 3, 2


def _branch(shapes):
    out = struct.pack("<I", len(shapes))
    for nin, nout in shapes:
        w = np.arange(nin * nout, dtype="<f4") / (nin * nout)
        out += struct.pack("<II", nin, nout) + w.tobytes()
        out += np.full(nout, 0.1, dtype="<f4").tobytes()
    return out


def _blob(bands=(BAND_0_1, BAND_1_5, BAND_ROTATE)):
    b = b"FREE" + struct.pack("<IIIIII", 1, HIST, OBS, ACT, ENC_VEL, ENC_LAT)
    b += np.array([10.0, 5.0], dtype="<f4").tobytes()
    b += np.zeros(OBS, dtype="<f4").tobytes()  # ob_mean
    b += np.ones(OBS, dtype="<f4").tobytes()  # ob_scale
    for fill in (0.0, 1.0, 0.5, 40.0, 1.0):  # act_mean, scale, default, kp, kd
        b += np.full(ACT, fill, dtype="<f4").tobytes()
    b += np.zeros(6, dtype="<f4").tobytes()  # band thresholds
    b += struct.pack("<I", len(bands))
    for kind in bands:
        b += struct.pack("<I", kind)
        b += _branch([(OBS * HIST, 5), (5, ENC_VEL + ENC_LAT)])  # encoder
        b += _branch([(OBS + ENC_VEL + ENC_LAT, 6), (6, ACT)])  # actor
    return b


@pytest.fixture
def policy(tmp_path):
    p = tmp_path / "synth.bin"
    p.write_bytes(_blob())
    return FreePolicy.load(p)


def test_header_and_arrays(policy):
    assert (policy.hist, policy.obs_per_frame, policy.act_dim) == (HIST, OBS, ACT)
    assert policy.clip_obs == 10.0
    assert policy.clip_act == 5.0
    np.testing.assert_allclose(policy.kp, 40.0)
    np.testing.assert_allclose(policy.kd, 1.0)
    np.testing.assert_allclose(policy.default_pose, 0.5)
    assert sorted(policy.bands) == [BAND_0_1, BAND_1_5, BAND_ROTATE]


def test_rejects_bad_magic_and_version(tmp_path):
    bad = tmp_path / "bad.bin"
    bad.write_bytes(b"NOPE" + _blob()[4:])
    with pytest.raises(ValueError, match="not a FREE blob"):
        FreePolicy.load(bad)

    ver = tmp_path / "ver.bin"
    ver.write_bytes(b"FREE" + struct.pack("<I", 99) + _blob()[8:])
    with pytest.raises(ValueError, match="unsupported FREE version"):
        FreePolicy.load(ver)


@pytest.mark.parametrize(
    ("cmd", "expected"),
    [
        ((0.0, 0.0, 0.8), BAND_ROTATE),  # turning in place
        ((0.5, 0.0, 0.0), BAND_0_1),
        ((1.5, 0.0, 0.0), BAND_1_5),
        ((0.0, 1.2, 0.0), BAND_1_5),  # lateral counts toward speed
        ((0.0, 0.0, 0.0), BAND_0_1),  # standing still is not "rotate"
    ],
)
def test_band_selection(policy, cmd, expected):
    assert policy.select(*cmd).kind == expected


def test_band_selection_falls_back_when_band_absent(tmp_path):
    p = tmp_path / "one.bin"
    p.write_bytes(_blob(bands=(BAND_0_1,)))
    only = FreePolicy.load(p)
    assert only.select(0.0, 0.0, 0.8).kind == BAND_0_1
    assert only.select(3.0, 0.0, 0.0).kind == BAND_0_1


def test_forward_shape_and_latent_is_normalized(policy):
    p_obs = np.linspace(-1.0, 1.0, OBS * HIST)
    assert policy.forward(policy.bands[BAND_0_1], p_obs).shape == (ACT,)

    band = policy.bands[BAND_0_1]
    h = p_obs
    for w, b in band.encoder[:-1]:
        h = np.where(h @ w + b > 0, h @ w + b, np.expm1(np.minimum(h @ w + b, 0)))
    w, b = band.encoder[-1]
    lat = (h @ w + b)[ENC_VEL:]
    assert np.linalg.norm(lat) > 0  # non-degenerate before normalization


def test_act_clips_and_applies_affine(policy):
    action, target = policy.act(np.full(OBS * HIST, 50.0), np.zeros(3))
    assert np.all(np.abs(action) <= policy.clip_act)
    # act_scale=1, act_mean=0 in the synthetic blob
    np.testing.assert_allclose(target, action)


def test_normalize_clips(policy):
    out = policy.normalize(np.full(OBS, 1e6))
    np.testing.assert_allclose(out, policy.clip_obs)
