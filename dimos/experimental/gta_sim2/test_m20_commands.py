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

"""Brake transitions, timestamp identity and bounded targets retain physical PD torque."""

from pathlib import Path
from typing import Any

import cbor2
from dimos_lcm.common_msgs import SampleHeader
from dimos_lcm.sim_msgs import CborObservation, WorldState
import numpy as np
import pytest

from dimos.experimental.gta_sim2.m20 import M20PolicyModule
from dimos.experimental.gta_sim2.m20_actor import HOME, JOINTS, KP, M20Actor
from dimos.experimental.gta_sim2.test_m20_actor import CaptureSession
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState


def test_stop_duplicate_and_motor_contract(
    monkeypatch: pytest.MonkeyPatch, request: pytest.FixtureRequest
) -> None:
    session = CaptureSession(57)

    def load(*args: Any, **kwargs: Any) -> CaptureSession:
        return session

    monkeypatch.setattr("onnxruntime.InferenceSession", load)
    module = M20PolicyModule(
        model_path=Path("fixture.onnx"),
        command_velocity_limits=(3.0, 0.5, 0.7),
        turning_forward_speed=1.5,
    )
    request.addfinalizer(module.stop)
    module._actor = M20Actor(Path("fixture.onnx"))
    module._clock_id = "evaluation-clock"
    module._ids = JOINTS
    module._lower, module._upper = np.full(12, -1.0), np.full(12, 1.0)
    joints = JointState(ts=0.02, name=list(JOINTS), position=HOME.tolist(), velocity=[0.0] * 16)
    imu = Imu(ts=0.02, orientation_covariance=[0.0] * 9)
    imu.orientation.w = 1
    stopped = module.compute(joints, imu)
    assert session.inputs == []
    assert module.status()["mode"] == "stand"
    rows = {c.interface_name: np.asarray(c.values) for c in stopped.joint_commands}
    np.testing.assert_allclose(rows["position"], HOME)
    np.testing.assert_array_equal(rows["velocity"], 0)
    np.testing.assert_array_equal(rows["kp"][12:], 20)
    np.testing.assert_allclose(rows["kd"][12:], 2)
    assert stopped.header.clock_id == "evaluation-clock"
    assert stopped.based_on_time_ns == stopped.apply_at_ns == 20_000_000
    assert stopped.valid_until_ns == 120_000_000
    assert module.compute(joints, imu) is stopped
    module.move(Twist(linear=Vector3(1, 0, 0)), duration=10)
    joints.ts = imu.ts = 0.04
    moving = module.compute(joints, imu)
    assert len(session.inputs) == 1
    assert module.compute(joints, imu) is moving
    assert len(session.inputs) == 1
    rows = {c.interface_name: np.asarray(c.values) for c in moving.joint_commands}
    unbounded = HOME + np.arange(16) * np.array([0.125, 0.25, 0.25] * 4 + [5] * 4)
    np.testing.assert_allclose(rows["kp"] * rows["position"] + rows["effort"], KP * unbounded)
    assert abs(rows["position"][:12]).max() <= 1
    module.move(Twist(), duration=10)
    joints.ts = imu.ts = 0.06
    module.compute(joints, imu)
    assert module.status()["mode"] == "stand"
    module.move(Twist(linear=Vector3(1, 0, 0)), duration=10)
    joints.ts = imu.ts = 0.08
    module.compute(joints, imu)
    np.testing.assert_array_equal(session.inputs[-1][0, -16:], 0)
    module.move(Twist(), duration=10)
    joints.ts = imu.ts = 0.1
    joints.position[12:] = [1.0, 2.0, 3.0, 4.0]
    module.compute(joints, imu)
    joints.ts = imu.ts = 0.12
    joints.position[12:] = [1.1, 2.1, 3.1, 4.1]
    parked = module.compute(joints, imu)
    parking_position = next(
        c.values for c in parked.joint_commands if c.interface_name == "position"
    )
    np.testing.assert_allclose(parking_position[12:], [1, 2, 3, 4])
    imu.ts = 0.14
    with pytest.raises(ValueError, match="matching, monotonic"):
        module.compute(joints, imu)
    module.move(Twist(linear=Vector3(1.5, 0, 0)), duration=10)
    for index in range(40):
        joints.ts = imu.ts = 0.14 + index * 0.02
        module.compute(joints, imu)
    np.testing.assert_allclose(module.status()["policy_velocity"], [1.5, 0, 0])
    ramp = np.array([row[0, 6] for row in session.inputs[-40:]])
    assert np.max(np.diff(ramp)) <= 0.040001
    module.move(Twist(), duration=10)
    joints.ts = imu.ts = 0.94
    braking = module.compute(joints, imu)
    np.testing.assert_allclose(module.status()["policy_velocity"], [1.46, 0, 0])
    braking_kp = next(c.values for c in braking.joint_commands if c.interface_name == "kp")
    np.testing.assert_array_equal(braking_kp[12:], 0)
    for index in range(38):
        joints.ts = imu.ts = 0.96 + index * 0.02
        module.compute(joints, imu)
    assert module.status()["mode"] == "stand"
    np.testing.assert_allclose(module.status()["policy_velocity"], 0, atol=1e-8)
    # A faster forward ceiling must retain independent side/yaw limits and the
    # same acceleration ramp; the old hardcoded clamp silently saturated at 1.5.
    module.move(Twist(linear=Vector3(4, -1, 0)), duration=10)
    for index in range(80):
        joints.ts = imu.ts = 1.72 + index * 0.02
        module.compute(joints, imu)
    np.testing.assert_allclose(module.status()["policy_velocity"], [3.0, -0.5, 0])
    ramp = np.array([row[0, 6] for row in session.inputs[-80:]])
    assert np.max(np.diff(ramp)) <= 0.040001
    module.move(Twist(linear=Vector3(3, 0, 0), angular=Vector3(0, 0, 2)), duration=10)
    for index in range(40):
        joints.ts = imu.ts = 3.32 + index * 0.02
        module.compute(joints, imu)
    np.testing.assert_allclose(module.status()["policy_velocity"], [1.5, 0, 0.7])
    assert module.status()["turning_limited"]


@pytest.mark.asyncio
async def test_streaming_governor_uses_robot_yaw_and_stops_on_stale_bounds(
    monkeypatch: pytest.MonkeyPatch, request: pytest.FixtureRequest
) -> None:
    session = CaptureSession(57)
    monkeypatch.setattr("onnxruntime.InferenceSession", lambda *a, **k: session)
    module = M20PolicyModule(model_path=Path("fixture.onnx"), streaming_required=True)
    request.addfinalizer(module.stop)
    module._actor = M20Actor(Path("fixture.onnx"))
    module._clock_id, module._ids = "clock", JOINTS
    module._lower, module._upper = np.full(12, -1.0), np.full(12, 1.0)
    joints = JointState(ts=0.02, name=list(JOINTS), position=HOME.tolist(), velocity=[0.0] * 16)
    imu = Imu(ts=0.02, orientation_covariance=[0.0] * 9)
    imu.orientation.w = imu.orientation.z = np.sqrt(0.5)  # Body +X faces world +Y.
    module.move(Twist(linear=Vector3(1, 0, 0)), duration=10)
    module.compute(joints, imu)
    np.testing.assert_array_equal(module.status()["policy_velocity"], [0, 0, 0])
    payload = cbor2.dumps(
        {"position": [0, 9, 0], "safe_bounds": [[-9, -9], [9, 9]], "latency_s": 1}
    )
    sample = WorldState(
        header=SampleHeader(clock_id="clock", epoch=0, time_ns=40_000_000),
        cbor_observations=[
            CborObservation(
                entity_id="world", key="gta/streaming", value=payload, value_length=len(payload)
            )
        ],
    )
    await module.handle_observations(sample)
    joints.ts = imu.ts = 0.04
    module.compute(joints, imu)
    np.testing.assert_allclose(module.status()["policy_velocity"], 0, atol=1e-12)
    module.move(Twist(linear=Vector3(-1, 0, 0)), duration=10)
    joints.ts = imu.ts = 0.06
    module.compute(joints, imu)
    assert module.status()["policy_velocity"][0] < 0
    joints.ts = imu.ts = 0.2
    module.compute(joints, imu)
    assert module.status()["streaming_governed"]
    np.testing.assert_allclose(module.status()["policy_velocity"], 0, atol=1e-12)
