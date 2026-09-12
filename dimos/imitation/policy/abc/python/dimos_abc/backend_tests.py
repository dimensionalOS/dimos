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

"""Compare the adapter with the real upstream sampler using a small model."""

from collections.abc import Iterator

from abc_minimal.config import DiTConfig
from abc_minimal.preprocess import normalize, resize_pad_normalize, unnormalize
from dimos_abc import backend as abc
import numpy as np
import pytest
import pytest_mock
import torch

from dimos.imitation.policy.backend import joint_permutations
from dimos.imitation.policy.module import PolicyModuleConfig

HARDWARE = [f"joint{i}" for i in range(14)]
POLICY_INDICES = [0, 1, 2, 3, 4, 5, 12, 6, 7, 8, 9, 10, 11, 13]
HARDWARE_INDICES = [0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12, 6, 13]


@pytest.fixture
def config() -> PolicyModuleConfig:
    return PolicyModuleConfig(
        backend="abc",
        policy_path="test-checkpoint",
        task="bottles",
        device="cpu",
        fast_inference=False,
        joint_names=HARDWARE,
        policy_joint_names=[HARDWARE[i] for i in POLICY_INDICES],
        image_mapping={"top_image": "top", "left_image": "left", "right_image": "right"},
    )


@pytest.fixture
def backend(config: PolicyModuleConfig, mocker: pytest_mock.MockerFixture) -> Iterator[abc.Backend]:
    model_config = DiTConfig(
        hidden_size=32,
        depth=1,
        num_heads=4,
        vit_embed_dim=32,
        vit_depth=1,
        vit_num_heads=4,
        vision_pool_num_queries=2,
        vision_pool_num_heads=4,
    )
    mocker.patch.object(abc, "DiTConfig", return_value=model_config)
    stats = {
        kind: {"mean": (np.arange(14) / 10).tolist(), "std": [2.0] * 14}
        for kind in ("state", "actions")
    }
    mocker.patch.object(abc, "load_pretrained", return_value={"norm_stats": stats})
    embedder = mocker.patch.object(abc, "CLIPTextEmbedder")
    embedder.return_value.encode.return_value = torch.linspace(0, 1, 512)[None]
    with torch.random.fork_rng(devices=[]):
        torch.manual_seed(17)
        value = abc.Backend(config)
        try:
            yield value
        finally:
            value.close()


def test_adapter_matches_official_sampling_and_joint_order(backend: abc.Backend) -> None:
    images = {
        name: np.full((32, 48, 3), value, dtype=np.uint8)
        for name, value in (("left", 23), ("right", 91), ("top", 200))
    }
    state = (np.arange(14) / 20).astype(np.float32)
    # Construct the reference batch in ABC order independently of the adapter.
    batch = {
        "state": torch.from_numpy(
            normalize(state[POLICY_INDICES], backend.norm_stats["state"])[None]
        ),
        "actions": torch.zeros(1, 30, 14),
        "images": {
            name: resize_pad_normalize(image.transpose(2, 0, 1)).unsqueeze(0)
            for name, image in images.items()
        },
        "task_vec_clip": backend.task_vec,
    }
    with torch.inference_mode(), torch.random.fork_rng(devices=[]):
        torch.manual_seed(29)
        expected = backend.model.sample_actions(batch, num_steps=10)[0].numpy()
        expected = unnormalize(expected, backend.norm_stats["actions"])[:, HARDWARE_INDICES]
        torch.manual_seed(29)
        actual = backend.predict(images, state, task="bottles")
    np.testing.assert_allclose(actual, expected, rtol=1e-5, atol=1e-6)
    assert actual.shape == (30, 14)
    assert backend.n_action_steps == 15
    assert backend.fps == pytest.approx(1 / 0.034)


@pytest.mark.parametrize(
    "order", [HARDWARE[:-1], [*HARDWARE[:-1], HARDWARE[0]], [*HARDWARE[:-1], "unknown"]]
)
def test_joint_mapping_rejects_missing_duplicate_and_unknown_joints(order: list[str]) -> None:
    with pytest.raises(ValueError, match="unique and match"):
        joint_permutations(HARDWARE, order)


def test_missing_camera_rejected_before_loading_model(
    config: PolicyModuleConfig, mocker: pytest_mock.MockerFixture
) -> None:
    model = mocker.patch.object(abc, "DiTPolicy")
    config.image_mapping = {"left_image": "left", "right_image": "right"}
    with pytest.raises(ValueError, match="top, left, and right"):
        abc.Backend(config)
    model.assert_not_called()


def test_execution_horizon_cannot_exceed_prediction(
    config: PolicyModuleConfig, mocker: pytest_mock.MockerFixture
) -> None:
    model = mocker.patch.object(abc, "DiTPolicy")
    config.execution_steps = 31
    with pytest.raises(ValueError, match="30-action horizon"):
        abc.Backend(config)
    model.assert_not_called()
