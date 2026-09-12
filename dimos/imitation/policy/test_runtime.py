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

"""Shared execution contract, independent of model framework dependencies."""

import time

import numpy as np
import pytest

from dimos.control.tasks.trajectory_task.trajectory_task import (
    TrajectoryCancellationResult,
    TrajectoryCancellationStatus,
    TrajectoryExecutionResult,
    TrajectoryExecutionStatus,
)
from dimos.experimental.isolated_python.bootstrap import load_class
from dimos.imitation.policy import runtime as policy_runtime
from dimos.imitation.policy.module import policy_module
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.protocol.rpc.pubsubrpc import LCMRPC
from dimos.utils.testing.waiting import wait_until


@pytest.fixture
def runtime(mocker):
    mocker.patch("dimos.core.module.get_loop", return_value=(mocker.MagicMock(), None))
    for method in ("__init__", "serve_module_rpc", "start", "stop"):
        mocker.patch.object(LCMRPC, method, return_value=None)
    mapping = {"left_camera": "left", "right_camera": "right", "overhead": "top"}
    atom = policy_module(image_mapping=mapping).blueprints[0]
    cls = load_class(atom.module.implementation)
    module = cls(
        _isolated_python_runtime=True,
        image_mapping=mapping,
        backend="abc",
        joint_names=["a", "b"],
        policy_path="unused",
        task="bottles",
    )
    backend = mocker.MagicMock(
        n_action_steps=15, chunk_size=30, fps=100.0, action_lower=None, action_upper=None
    )
    backend.predict.return_value = np.tile([0.2, 0.3], (30, 1)).astype(np.float32)
    factory = mocker.MagicMock(Backend=mocker.Mock(return_value=backend))
    mocker.patch.object(policy_runtime, "import_module", return_value=factory)
    control = mocker.MagicMock()
    control.list_tasks.return_value = ["policy_rollout"]
    control.execute_trajectory.return_value = TrajectoryExecutionResult(
        TrajectoryExecutionStatus.ACCEPTED
    )
    control.cancel_trajectory.return_value = TrajectoryCancellationResult(
        TrajectoryCancellationStatus.ALREADY_STOPPED
    )
    mocker.patch.object(module, "_control", control, create=True)
    try:
        yield module, backend, control
    finally:
        module.stop()


def provide_observations(module):
    timestamp = time.time()
    for index, port in enumerate(module.config.image_mapping):
        module._on_image(
            port,
            Image(
                data=np.full((4, 6, 3), index, dtype=np.uint8), format=ImageFormat.RGB, ts=timestamp
            ),
        )
    module._on_joint_state(JointState(name=["b", "a"], position=[0.3, 0.2], ts=timestamp))


def test_named_images_and_joints_reach_backend_and_only_execution_horizon_runs(runtime):
    module, backend, control = runtime
    provide_observations(module)
    assert module.preflight_rollout()["policy_ready"]
    control.execute_trajectory.assert_not_called()
    images, state = backend.predict.call_args.args
    assert set(images) == {"left", "right", "top"}
    np.testing.assert_array_equal(images["right"], np.ones((4, 6, 3), dtype=np.uint8))
    np.testing.assert_allclose(state, [0.2, 0.3])
    module.start_rollout()
    wait_until(lambda: control.execute_trajectory.called, timeout=1)
    module.stop_rollout()
    trajectory = control.execute_trajectory.call_args.args[0]
    assert trajectory.joint_names == ["a", "b"]
    assert len(trajectory.points) == 16
    assert trajectory.points[-1].time_from_start == 0.15
    assert not module.rollout_status()["active"]


@pytest.mark.parametrize("missing", ["left_camera", "right_camera", "overhead"])
def test_every_camera_is_required_for_preflight(runtime, missing):
    module, backend, control = runtime
    provide_observations(module)
    del module._latest_images[missing]
    status = module.preflight_rollout()
    assert not status["policy_ready"]
    assert missing in status["last_error"]
    backend.predict.assert_not_called()
    control.execute_trajectory.assert_not_called()


def test_stale_camera_prevents_rollout_even_when_other_inputs_are_fresh(runtime):
    module, backend, control = runtime
    provide_observations(module)
    image, timestamp = module._latest_images["overhead"]
    module._latest_images["overhead"] = (image, timestamp - 10)
    assert not module.preflight_rollout()["policy_ready"]
    assert "overhead" in module.rollout_status()["last_error"]
    backend.predict.assert_not_called()
    control.execute_trajectory.assert_not_called()
