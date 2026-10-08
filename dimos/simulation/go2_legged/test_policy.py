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

from __future__ import annotations

import itertools
from pathlib import Path
import struct

import numpy as np
from numpy.typing import ArrayLike
import pytest

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.simulation.go2_legged.policy import (
    BAND_FAST,
    BAND_LIMITS,
    BAND_ROTATE,
    BAND_WALK,
    FREE_OBS,
    JOINT_COUNT,
    FreePolicy,
    OnnxGo2Policy,
    Proprioception,
    load_policy,
)

DEFAULT_POSE = [0.0, 0.9, -1.8] * 4


def _f32(values: ArrayLike) -> bytes:
    return np.asarray(values, "<f4").tobytes()


def _layer(rng: np.random.Generator, nin: int, nout: int) -> bytes:
    weights = rng.standard_normal(nin * nout).astype("<f4") * 0.1
    bias = rng.standard_normal(nout).astype("<f4") * 0.1
    return struct.pack("<II", nin, nout) + weights.tobytes() + bias.tobytes()


def _block(rng: np.random.Generator, sizes: list[int]) -> bytes:
    layers = [_layer(rng, a, b) for a, b in itertools.pairwise(sizes)]
    return struct.pack("<I", len(layers)) + b"".join(layers)


def _blob(
    hist: int = 2,
    enc_vel: int = 3,
    enc_lat: int = 4,
    kinds: tuple[int, ...] = (0, 3),
    version: int = 1,
    obs: int = FREE_OBS,
    encoder_out: int | None = None,
) -> bytes:
    rng = np.random.default_rng(0)
    head = b"FREE" + struct.pack("<IIIIII", version, hist, obs, JOINT_COUNT, enc_vel, enc_lat)
    head += _f32([100.0, 100.0])
    head += _f32(np.zeros(obs)) + _f32(np.ones(obs))
    head += _f32(DEFAULT_POSE) + _f32([0.25] * JOINT_COUNT)
    head += _f32(DEFAULT_POSE) + _f32([40.0] * JOINT_COUNT) + _f32([1.0] * JOINT_COUNT)
    head += _f32(np.zeros(BAND_LIMITS))
    bands = b""
    for kind in kinds:
        encoder = _block(rng, [hist * obs, 8, encoder_out or enc_vel + enc_lat])
        actor = _block(rng, [obs + enc_vel + enc_lat, 8, JOINT_COUNT])
        bands += struct.pack("<I", kind) + encoder + actor
    return head + struct.pack("<I", len(kinds)) + bands


def _obs(pose: ArrayLike = DEFAULT_POSE) -> Proprioception:
    return Proprioception(
        np.zeros(3),
        np.array([0.0, 0.0, -1.0]),
        np.asarray(pose, dtype=np.float64),
        np.zeros(JOINT_COUNT),
    )


def test_reads_the_blob_layout() -> None:
    policy = FreePolicy(_blob())
    assert policy.hist == 2
    assert set(policy.bands) == {BAND_WALK, BAND_ROTATE}
    assert policy.default_pose == pytest.approx(DEFAULT_POSE)
    assert policy.kp[0] == 40.0 and policy.kd[0] == 1.0
    assert policy.joint_names[0] == "FL_hip_joint" and policy.joint_names[3] == "FR_hip_joint"


def test_rejects_a_blob_without_the_magic() -> None:
    with pytest.raises(ValueError, match="FREE"):
        FreePolicy(b"BLND" + bytes(64))


@pytest.mark.parametrize(
    ("blob", "message"),
    [
        (_blob(version=2), "version"),
        (_blob(obs=44), "observation"),
        (_blob(kinds=()), "no band"),
        (_blob(encoder_out=5), "encoder"),
    ],
)
def test_rejects_blobs_with_the_wrong_layout(blob: bytes, message: str) -> None:
    with pytest.raises(ValueError, match=message):
        FreePolicy(blob)


def test_acts_deterministically_and_resumes_from_a_clean_history() -> None:
    a, b = FreePolicy(_blob()), FreePolicy(_blob())
    command = np.array([0.5, 0.0, 0.0])
    first = a.act(_obs(), command)
    assert first.shape == (JOINT_COUNT,)
    assert np.array_equal(first, b.act(_obs(), command))
    a.act(_obs(), command)
    a.reset()
    assert np.array_equal(a.act(_obs(), command), first)


def test_picks_the_rotate_band_for_a_pure_turn() -> None:
    policy = FreePolicy(_blob())
    assert policy.band_for(np.array([0.0, 0.0, 0.5])).kind == BAND_ROTATE
    assert policy.band_for(np.array([0.5, 0.0, 0.5])).kind == BAND_WALK
    assert policy.band_for(np.array([2.0, 0.0, 0.0])).kind == BAND_WALK


def test_falls_back_to_the_nearest_slower_band() -> None:
    turn, sprint = np.array([0.0, 0.0, 0.5]), np.array([6.0, 0.0, 0.0])
    assert FreePolicy(_blob(kinds=(0, 1, 2))).band_for(turn).kind == BAND_WALK
    assert FreePolicy(_blob(kinds=(0, 1))).band_for(sprint).kind == BAND_FAST
    assert FreePolicy(_blob(kinds=(0,))).band_for(sprint).kind == BAND_WALK


def test_refuses_weights_inside_the_repository() -> None:
    with pytest.raises(ValueError, match="outside the repository"):
        FreePolicy.load(DIMOS_PROJECT_ROOT / "data" / "weights.bin")


def test_load_policy_picks_the_format_by_suffix(tmp_path: Path) -> None:
    blob = tmp_path / "weights.bin"
    blob.write_bytes(_blob())
    assert isinstance(load_policy(blob), FreePolicy)
    with pytest.raises(ValueError, match="format"):
        load_policy(tmp_path / "policy.pt")


@pytest.mark.self_hosted
def test_bundled_policy_loads_and_acts() -> None:
    policy = load_policy(None)
    assert isinstance(policy, OnnxGo2Policy)
    targets = policy.act(_obs(policy.default_pose), np.zeros(3))
    assert targets.shape == (JOINT_COUNT,)
    assert np.all(np.isfinite(targets))
