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

"""The example's claim, on the LFS sample: the identified plant beats stock in both loops."""

from __future__ import annotations

import pytest

from dimos.navigation.sysid.engines.mujoco import MujocoBackend
from dimos.navigation.sysid.example import loop1, loop2
from dimos.navigation.sysid.ingest import read_streams, sample
from dimos.navigation.sysid.plant import TORQUE_ENVELOPES
from dimos.navigation.sysid.policy import FreePolicy
from dimos.navigation.sysid.ranges import Preset, load_preset

pytestmark = pytest.mark.go2sim


def mujoco(preset: Preset) -> MujocoBackend:
    return MujocoBackend(envelope=TORQUE_ENVELOPES[preset.envelope] if preset.envelope else None)


def test_measured_beats_stock() -> None:
    recording, policy_path = sample()
    st = read_streams(recording)
    policy = FreePolicy.load(policy_path)
    stock, measured = load_preset("stock"), load_preset("measured")

    assert loop1(st, measured, mujoco)["joint"] < loop1(st, stock, mujoco)["joint"]
    walk_stock, walk_measured = (
        loop2(st, policy, stock, mujoco),
        loop2(st, policy, measured, mujoco),
    )
    for term in ("along", "cross", "yaw"):
        assert walk_measured.median(term) < walk_stock.median(term)
