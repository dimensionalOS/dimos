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

"""Checkpoint ordering, wheel interfaces and recurrent history are explicit contracts."""

from pathlib import Path
from types import SimpleNamespace
from typing import Any

import numpy as np
from numpy.typing import NDArray
import pytest

from dimos.experimental.gta_sim2.m20_actor import M20Actor, Profile, home_pose


class CaptureSession:
    def __init__(self, size: int) -> None:
        self.size = size
        self.inputs: list[NDArray[np.float32]] = []

    def get_inputs(self) -> list[Any]:
        return [SimpleNamespace(name="obs", shape=[1, self.size])]

    def get_outputs(self) -> list[Any]:
        return [SimpleNamespace(name="actions", shape=[1, 16])]

    def run(
        self, outputs: Any, inputs: dict[str, NDArray[np.float32]]
    ) -> list[NDArray[np.float32]]:
        self.inputs.append(inputs["obs"].copy())
        return [np.arange(16, dtype=np.float32)[None]]


@pytest.mark.parametrize("profile,size", [("sdk57", 57), ("dreamwaq342", 342)])
def test_wheels_history_and_checkpoint_layout(
    monkeypatch: pytest.MonkeyPatch, profile: Profile, size: int
) -> None:
    session = CaptureSession(size)

    def load(*args: Any, **kwargs: Any) -> CaptureSession:
        return session

    monkeypatch.setattr("onnxruntime.InferenceSession", load)
    actor = M20Actor(Path("fixture.onnx"), profile)
    q = home_pose(profile)
    q[3] += 0.4  # front-right hip-x in canonical order
    q[12:] = 900  # wheel rotations must never enter position observations
    dq = np.arange(16, dtype=float)
    gyro, gravity, command = (
        np.array([4.0, 8.0, 12.0]),
        np.array([0.0, 0.0, -1.0]),
        np.array([1.0, 0.5, 0.4]),
    )
    positions, velocities = actor.act(q, dq, gyro, gravity, command)
    latest = session.inputs[-1][0, -57:]
    if profile == "dreamwaq342":
        np.testing.assert_allclose(latest[:9], [2, 1, 0.1, 1, 2, 3, 0, 0, -1])
        assert latest[13] == pytest.approx(0.4)
        np.testing.assert_array_equal(latest[[12, 16, 20, 24]], 0)
        np.testing.assert_allclose(latest[[28, 32, 36, 40]], [0.6, 0.65, 0.7, 0.75])
        np.testing.assert_allclose(velocities[12:], [15, 35, 55, 75])
        assert positions[3] == pytest.approx(1.0)
        np.testing.assert_array_equal(session.inputs[-1][0, :-57], 0)
    else:
        np.testing.assert_allclose(latest[:9], [1, 2, 3, 0, 0, -1, 1, 0.5, 0.4])
        np.testing.assert_array_equal(latest[21:25], 0)
        np.testing.assert_allclose(velocities[12:], [60, 65, 70, 75])
        assert positions[3] == pytest.approx(0.375)
    np.testing.assert_array_equal(positions[12:], 0)
    np.testing.assert_array_equal(velocities[:12], 0)
    actor.act(q, dq, gyro, gravity, command)
    np.testing.assert_array_equal(session.inputs[-1][0, -16:], np.arange(16))
    if profile == "dreamwaq342":
        np.testing.assert_array_equal(session.inputs[-1][0, -114:-57], latest)
    actor.reset()
    actor.act(q, dq, gyro, gravity, command)
    np.testing.assert_array_equal(session.inputs[-1], session.inputs[0])
    with pytest.raises(ValueError, match="Invalid M20 policy observation"):
        actor.act(q, dq, gyro, gravity, np.full(3, np.nan))


def test_profile_mismatch_fails_before_inference(monkeypatch: pytest.MonkeyPatch) -> None:
    def load(*args: Any, **kwargs: Any) -> CaptureSession:
        return CaptureSession(342)

    monkeypatch.setattr("onnxruntime.InferenceSession", load)
    with pytest.raises(ValueError, match="does not match sdk57"):
        M20Actor(Path("wrong-profile.onnx"), "sdk57")
