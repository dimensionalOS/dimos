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

"""Adding demonstrations must preserve a warm-started ACT's physical predictions."""

import json

from dimos_lerobot.prepare_object_act import prepare
from lerobot.configs.types import FeatureType, NormalizationMode, PolicyFeature
from lerobot.policies.act.configuration_act import ACTConfig
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.factory import make_pre_post_processors
import pytest
import torch

from dimos.robot.galaxea.r1pro.object_packing import OBJECT_PACKING_IO


@pytest.fixture
def source_policy(tmp_path):
    config = ACTConfig(
        device="cpu",
        pretrained_backbone_weights=None,
        input_features={
            "observation.state": PolicyFeature(type=FeatureType.STATE, shape=(20,)),
            "observation.environment_state": PolicyFeature(type=FeatureType.ENV, shape=(52,)),
        },
        output_features={"action": PolicyFeature(type=FeatureType.ACTION, shape=(20,))},
        chunk_size=5,
        n_action_steps=5,
        dim_model=32,
        n_heads=4,
        dim_feedforward=64,
        n_encoder_layers=1,
        n_decoder_layers=1,
        n_vae_encoder_layers=1,
    )
    config.normalization_mapping["ENV"] = NormalizationMode.MEAN_STD
    stats = {
        key: {"mean": torch.full(feature.shape, 0.25), "std": torch.full(feature.shape, 0.5)}
        for key, feature in {**config.input_features, **config.output_features}.items()
    }
    source = tmp_path / "source"
    # Keep the test's seed local to this setup.
    with torch.random.fork_rng(devices=[]):
        torch.manual_seed(42)
        ACTPolicy(config).save_pretrained(source)
    pre, post = make_pre_post_processors(config, dataset_stats=stats)
    pre.save_pretrained(source)
    post.save_pretrained(source)
    (source / "deployment.json").write_text(json.dumps({"profile": OBJECT_PACKING_IO.name}))
    return source, config, stats


def test_new_demonstration_statistics_do_not_change_warm_start_predictions(tmp_path, source_policy):
    source, config, stats = source_policy
    dataset = tmp_path / "new_dataset"
    (dataset / "meta").mkdir(parents=True)
    new_stats = {
        key: {"mean": (value["mean"] + 0.4).tolist(), "std": (value["std"] * 3).tolist()}
        for key, value in stats.items()
    }
    stats_path = dataset / "meta/stats.json"
    stats_path.write_text(json.dumps(new_stats))
    output = tmp_path / "initialization"
    with torch.random.fork_rng(devices=[]):
        prepare(source, dataset, output)

    observation = {
        "observation.state": torch.linspace(-0.5, 0.5, 20).unsqueeze(0),
        "observation.environment_state": torch.linspace(-0.2, 0.7, 52).unsqueeze(0),
    }
    predictions = []
    for checkpoint in (source, output):
        policy = ACTPolicy.from_pretrained(checkpoint)
        policy.eval()
        pre, post = make_pre_post_processors(config, pretrained_path=checkpoint)
        with torch.inference_mode():
            predictions.append(post(policy.predict_action_chunk(pre(dict(observation)))))
    torch.testing.assert_close(predictions[0], predictions[1], rtol=0, atol=0)
    assert json.loads((dataset / "normalization-before-warm-start.json").read_text()) == new_stats
    assert json.loads(stats_path.read_text())["action"]["mean"] == [0.25] * 20
    assert json.loads((output / "initialization.json").read_text())["reinitialized"] == []


def test_same_width_with_different_feature_meaning_is_rejected(tmp_path, source_policy):
    source, _, _ = source_policy
    (source / "deployment.json").write_text(json.dumps({"profile": "incompatible-profile"}))
    with pytest.raises(ValueError, match="exact object profile"):
        prepare(source, tmp_path / "dataset", tmp_path / "output")
    assert not (tmp_path / "output").exists()
