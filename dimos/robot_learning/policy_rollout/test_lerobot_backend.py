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

from types import SimpleNamespace
from typing import Any, cast

import numpy as np
import pytest

from dimos.robot_learning.policy_rollout.backends.lerobot.backend import (
    LeRobotBackend,
    _tensorized_preprocessor_input,
)
from dimos.robot_learning.policy_rollout.models import BackendBatch


class FakeNoGrad:
    def __init__(self) -> None:
        self.entered = False

    def __enter__(self) -> None:
        self.entered = True

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        return None


class FakePolicy:
    def __init__(self) -> None:
        self.config = SimpleNamespace(device="cpu", n_action_steps=7, action_dim=7)
        self.eval_called = False
        self.reset_called = False
        self.selected_batches: list[object] = []
        self.chunk_batches: list[object] = []
        self.to_devices: list[str] = []

    def eval(self) -> None:
        self.eval_called = True

    def to(self, device: str) -> None:
        self.to_devices.append(device)

    def reset(self) -> None:
        self.reset_called = True

    def select_action(self, batch: object) -> tuple[float, ...]:
        self.selected_batches.append(batch)
        return (0.0, 0.1, -0.1, 0.2, -0.2, 0.3, 1.0)

    def predict_action_chunk(self, batch: object) -> tuple[tuple[float, ...], ...]:
        self.chunk_batches.append(batch)
        return ((0.0, 0.1, -0.1, 0.2, -0.2, 0.3, 1.0),)


class FakePolicyClass:
    policy = FakePolicy()
    checkpoint_ids: list[str] = []

    @classmethod
    def from_pretrained(cls, pretrained_name_or_path: str) -> FakePolicy:
        cls.checkpoint_ids.append(pretrained_name_or_path)
        return cls.policy


def test_lerobot_backend_import_boundary_without_lerobot() -> None:
    backend = LeRobotBackend()

    description = backend.describe()

    assert description.backend_type == "lerobot"
    assert description.checkpoint_id == "lerobot/VLA-JEPA-LIBERO"


def test_initialize_loads_policy_sets_device_eval_and_processors(mocker) -> None:
    policy = FakePolicy()
    FakePolicyClass.policy = policy
    FakePolicyClass.checkpoint_ids = []
    preprocessor_calls: list[object] = []
    postprocessor_calls: list[object] = []
    factory_calls: list[dict[str, object]] = []

    def make_pre_post_processors(**kwargs: object) -> tuple[object, object]:
        factory_calls.append(dict(kwargs))
        return (
            lambda batch: _record(preprocessor_calls, batch),
            lambda output: _record(postprocessor_calls, output),
        )

    mocker.patch(
        "dimos.robot_learning.policy_rollout.backends.lerobot.backend._load_vla_jepa_policy_class",
        return_value=FakePolicyClass,
    )
    mocker.patch(
        "dimos.robot_learning.policy_rollout.backends.lerobot.backend._make_pre_post_processors",
        side_effect=make_pre_post_processors,
    )
    mocker.patch(
        "dimos.robot_learning.policy_rollout.backends.lerobot.backend._torch_no_grad",
        return_value=FakeNoGrad(),
    )
    backend = LeRobotBackend(checkpoint_id="lerobot/VLA-JEPA-LIBERO", device="cpu")

    backend.initialize()
    output = backend.infer_batch(BackendBatch(payload={"observation.state": "state"}))
    backend.reset_episode()
    description = backend.describe()

    assert FakePolicyClass.checkpoint_ids == ["lerobot/VLA-JEPA-LIBERO"]
    assert factory_calls == [
        {
            "policy_cfg": policy.config,
            "pretrained_path": "lerobot/VLA-JEPA-LIBERO",
            "preprocessor_overrides": {"device_processor": {"device": "cpu"}},
        }
    ]
    assert policy.eval_called
    assert policy.to_devices == ["cpu"]
    assert policy.reset_called
    assert preprocessor_calls == [{"observation.state": "state"}]
    assert postprocessor_calls == [(0.0, 0.1, -0.1, 0.2, -0.2, 0.3, 1.0)]
    assert output.output == pytest.approx((0.0, 0.1, -0.1, 0.2, -0.2, 0.3, 1.0))
    assert output.metadata["inference_method"] == "select_action"
    assert output.metadata["output_shape"] == [7]
    assert description.policy_class is not None
    assert description.device == "cpu"
    assert description.metadata["processor_source"] == "checkpoint"


def test_lerobot_backend_can_route_to_action_chunk(mocker) -> None:
    policy = FakePolicy()
    FakePolicyClass.policy = policy
    FakePolicyClass.checkpoint_ids = []

    mocker.patch(
        "dimos.robot_learning.policy_rollout.backends.lerobot.backend._load_vla_jepa_policy_class",
        return_value=FakePolicyClass,
    )
    mocker.patch(
        "dimos.robot_learning.policy_rollout.backends.lerobot.backend._make_pre_post_processors",
        return_value=(lambda batch: batch, lambda output: output),
    )
    mocker.patch(
        "dimos.robot_learning.policy_rollout.backends.lerobot.backend._torch_no_grad",
        return_value=FakeNoGrad(),
    )
    backend = LeRobotBackend(use_action_chunk=True)

    output = backend.infer_batch(BackendBatch(payload={"observation.state": "state"}))

    assert policy.selected_batches == []
    assert policy.chunk_batches == [{"observation.state": "state"}]
    assert output.metadata["inference_method"] == "predict_action_chunk"
    assert output.metadata["output_shape"] == [1, 7]
    assert output.output == pytest.approx((0.0, 0.1, -0.1, 0.2, -0.2, 0.3, 1.0))


def test_lerobot_backend_tensorizes_numpy_inputs_before_lerobot_preprocessor() -> None:
    torch_module = cast("Any", pytest.importorskip("torch"))
    image = np.full((2, 3, 3), 255, dtype=np.uint8)
    state = np.zeros((8,), dtype=np.float32)

    prepared = _tensorized_preprocessor_input(
        {
            "observation.images.image": image,
            "observation.state": state,
            "task": "pick up the object",
        }
    )

    prepared_image = cast("Any", prepared["observation.images.image"])
    prepared_state = cast("Any", prepared["observation.state"])
    assert isinstance(prepared_image, torch_module.Tensor)
    assert prepared_image.shape == (3, 2, 3)
    assert prepared_image.dtype == torch_module.float32
    assert torch_module.max(prepared_image) == 1.0
    assert isinstance(prepared_state, torch_module.Tensor)
    assert prepared_state.shape == (8,)
    assert prepared["task"] == "pick up the object"


def test_lerobot_backend_missing_dependency_error(mocker) -> None:
    mocker.patch(
        "dimos.robot_learning.policy_rollout.backends.lerobot.backend._load_vla_jepa_policy_class",
        side_effect=RuntimeError("Install LeRobot"),
    )
    backend = LeRobotBackend()

    with pytest.raises(RuntimeError, match="Install LeRobot"):
        backend.initialize()


def _record(calls: list[object], value: object) -> object:
    calls.append(value)
    return value
