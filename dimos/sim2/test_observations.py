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

from contextlib import closing

import cbor2
from dimos_lcm.sim_msgs import Action, Actions, JointCommand, Observation, Observations
import numpy as np
import pytest

from dimos.sim2.control import joint_targets
from dimos.sim2.mujoco import MujocoBackend
from dimos.sim2.observations import observations, read_poses
from dimos.sim2.runtime import Runtime
from dimos.sim2.sensors import MujocoSensors


@pytest.fixture
def runtime(world):
    return Runtime(MujocoBackend(world))


def wire_sample(runtime):
    return Observations.lcm_decode(
        observations(runtime.run_id, runtime.backend.layout, runtime.snapshot()).lcm_encode()
    )


def test_full_sample_is_self_contained_after_skipped_publications(runtime, world):
    first = wire_sample(runtime)
    runtime.advance(100)
    latest = wire_sample(runtime)
    assert (
        latest.observations_length
        == first.observations_length
        == len(runtime.backend.layout.bodies)
    )
    with closing(MujocoSensors(world, runtime.backend.layout)) as reader:
        sample = reader.observe(latest)  # no announcement or earlier message
        np.testing.assert_allclose(sample.poses, runtime.snapshot().poses)
        assert reader.model.nq == reader.model.nv == reader.model.nu == 0
        assert reader.observe(first) is None
        assert reader.history[-1] is sample


def test_reset_rejects_delayed_old_epoch_and_clears_scan_history(runtime, world):
    with closing(MujocoSensors(world, runtime.backend.layout)) as reader:
        for _ in range(10):
            runtime.advance(10)
            reader.observe(wire_sample(runtime))
        assert reader.lidar("left/lidar") is not None
        old = wire_sample(runtime)
        runtime.reset()
        new = reader.observe(wire_sample(runtime))
        assert reader.observe(old) is None
        assert list(reader.history) == [new]
        assert reader.lidar("left/lidar") is None


@pytest.mark.parametrize(
    "damage", ["missing", "duplicate", "sparse", "revision", "quaternion", "truncated"]
)
def test_invalid_batch_cannot_partially_update_a_reader(runtime, world, damage):
    good = wire_sample(runtime)
    with closing(MujocoSensors(world, runtime.backend.layout)) as reader:
        accepted = reader.observe(good)
        bad = wire_sample(runtime)
        bad.tick += 1
        if damage == "missing":
            bad.observations.pop()
            bad.observations_length -= 1
        elif damage == "duplicate":
            bad.observations[1] = bad.observations[0]
        elif damage == "sparse":
            bad.complete = False
        elif damage == "revision":
            bad.model_revision = "other"
        elif damage == "truncated":
            bad.observations[0].data = bad.observations[0].data[:9]
            bad.observations[0].data_length = 9
        else:
            # Structurally valid nested pose, physically invalid zero quaternion.
            bad.observations[0].data = bad.observations[0].data[:32] + bytes(32)
        with pytest.raises(ValueError):
            reader.observe(bad)
        assert reader.history[-1] is accepted


def test_optional_cbor_and_unknown_observations_do_not_supply_required_poses(runtime):
    message = wire_sample(runtime)
    data = cbor2.dumps({"reflectance": 0.5})
    message.observations.extend(
        [
            Observation(
                entity_id="box",
                kind=Observation.CBOR,
                schema="example.material.v1",
                data_length=len(data),
                data=data,
            ),
            Observation(entity_id="box", kind=99, data_length=3, data=b"new"),
        ]
    )
    message.observations_length = len(message.observations)
    sample = read_poses(Observations.lcm_decode(message.lcm_encode()), runtime.backend.layout)
    np.testing.assert_allclose(sample.poses, runtime.snapshot().poses)


def test_command_batch_roundtrips_gains_and_rejects_unknown_action_atomically(runtime):
    command = JointCommand(
        n=1,
        names=["left/slide_x"],
        position=[0.3],
        velocity=[0.1],
        kp=[150.0],
        kd=[10.0],
        effort=[2.0],
    )
    payload = command.lcm_encode()
    message = Actions(
        run_id=runtime.run_id,
        model_revision=runtime.backend.layout.revision,
        epoch=runtime.epoch,
        tick=-1,
        ttl=0.25,
        actions_length=1,
        actions=[Action(kind=Action.JOINT_COMMAND, data_length=len(payload), data=payload)],
    )
    target = joint_targets(
        Actions.lcm_decode(message.lcm_encode()), runtime.run_id, runtime.backend.layout
    )
    assert (target.positions, target.velocities, target.kp, target.kd, target.feedforward) == (
        (0.3,),
        (0.1,),
        (150.0,),
        (10.0,),
        (2.0,),
    )
    message.actions.append(Action(kind=99, data_length=0, data=b""))
    message.actions_length += 1
    with pytest.raises(ValueError, match="unsupported action"):
        joint_targets(message, runtime.run_id, runtime.backend.layout)
    assert runtime.snapshot().position[0] == 0
