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

"""The FREE blob reader and forward pass, against a hand-built blob."""

from __future__ import annotations

from pathlib import Path
import struct

import numpy as np
import pytest

from dimos.robot.unitree.go2.sim.policy import (
    BAND_0_1,
    BAND_1_5,
    BAND_ROTATE,
    FreePolicy,
)

FREEWALK_BIN = Path(__file__).parents[5] / "data/ml-trajectory-research/freewalk_mcf.bin"

HIST, OBS, ACT, ENC_VEL, ENC_LAT = 2, 45, 12, 3, 4


def _blob(bands: tuple[int, ...] = (BAND_0_1,), seed: int = 0) -> bytes:
    """A minimal FREE v1 blob: single-linear-layer encoder and actor."""
    rng = np.random.default_rng(seed)
    out = bytearray(b"FREE")

    def u32(v: int) -> None:
        out.extend(struct.pack("<I", v))

    def f32(a: object) -> None:
        out.extend(np.asarray(a, "<f4").tobytes())

    u32(1)
    for v in (HIST, OBS, ACT, ENC_VEL, ENC_LAT):
        u32(v)
    f32([100.0, 10.0])  # clip_obs, clip_act
    f32(np.zeros(OBS))  # ob_mean
    f32(np.ones(OBS))  # ob_scale
    f32(np.zeros(ACT))  # act_mean
    f32(0.25 * np.ones(ACT))  # act_scale
    f32(np.linspace(-0.5, 0.5, ACT))  # default_pose
    f32(40.0 * np.ones(ACT))  # kp
    f32(np.ones(ACT))  # kd
    f32(np.zeros(6))  # band thresholds (unused by select)
    u32(len(bands))
    for kind in bands:
        u32(kind)
        u32(1)  # encoder: one linear layer
        u32(OBS * HIST)
        u32(ENC_VEL + ENC_LAT)
        f32(rng.normal(0, 0.1, (OBS * HIST) * (ENC_VEL + ENC_LAT)))
        f32(rng.normal(0, 0.1, ENC_VEL + ENC_LAT))
        u32(1)  # actor: one linear layer
        u32(OBS + ENC_VEL + ENC_LAT)
        u32(ACT)
        f32(rng.normal(0, 0.1, (OBS + ENC_VEL + ENC_LAT) * ACT))
        f32(rng.normal(0, 0.1, ACT))
    return bytes(out)


def test_the_reader_recovers_every_header_field():
    p = FreePolicy.loads(_blob())
    assert (p.hist, p.obs_per_frame, p.act_dim) == (HIST, OBS, ACT)
    assert (p.enc_vel, p.enc_lat) == (ENC_VEL, ENC_LAT)
    assert p.clip_obs == 100.0 and p.clip_act == 10.0
    assert p.kp == pytest.approx(40.0 * np.ones(ACT))
    assert len(p.bands) == 1 and BAND_0_1 in p.bands


def test_wrong_magic_and_version_are_refused():
    with pytest.raises(ValueError, match="not a FREE blob"):
        FreePolicy.loads(b"NOPE" + _blob()[4:])
    bad = bytearray(_blob())
    bad[4:8] = struct.pack("<I", 9)
    with pytest.raises(ValueError, match="version"):
        FreePolicy.loads(bytes(bad))


def test_forward_matches_the_hand_computed_pass():
    """Single-layer branches make the whole net checkable by hand, including
    the L2-normalised latent and the actor seeing only the CURRENT frame."""
    p = FreePolicy.loads(_blob())
    band = p.bands[BAND_0_1]
    rng = np.random.default_rng(1)
    p_obs = rng.normal(0, 1, OBS * HIST)

    w_e, b_e = band.encoder[0]
    e = p_obs @ w_e + b_e
    vel, lat = e[:ENC_VEL], e[ENC_VEL:]
    lat = lat / np.linalg.norm(lat)
    w_a, b_a = band.actor[0]
    expected = np.concatenate([p_obs[:OBS], vel, lat]) @ w_a + b_a

    assert p.forward(band, p_obs) == pytest.approx(expected)


def test_act_clips_the_action_and_maps_it_to_a_joint_target():
    p = FreePolicy.loads(_blob())
    p_obs = np.zeros(OBS * HIST)
    action, target = p.act(p_obs, np.zeros(3))
    assert np.all(np.abs(action) <= p.clip_act)
    assert target == pytest.approx(action * p.act_scale + p.act_mean)


def test_the_band_rule_routes_rotate_and_speed():
    p = FreePolicy.loads(_blob(bands=(BAND_0_1, BAND_1_5, BAND_ROTATE)))
    assert p.select(0.0, 0.0, 0.5).kind == BAND_ROTATE  # turning in place
    assert p.select(1.2, 0.0, 0.0).kind == BAND_1_5
    assert p.select(0.4, 0.0, 0.3).kind == BAND_0_1
    # without the rotate expert, turning falls back to the slow band
    p2 = FreePolicy.loads(_blob(bands=(BAND_0_1,)))
    assert p2.select(0.0, 0.0, 0.5).kind == BAND_0_1


@pytest.mark.skipif(not FREEWALK_BIN.is_file(), reason=f"no policy blob at {FREEWALK_BIN}")
def test_the_shipped_freewalk_blob_is_the_45_channel_himloco():
    p = FreePolicy.load(FREEWALK_BIN)
    assert (p.obs_per_frame, p.hist, p.act_dim) == (45, 6, 12)
    assert {BAND_0_1, BAND_1_5, BAND_ROTATE} <= set(p.bands)
