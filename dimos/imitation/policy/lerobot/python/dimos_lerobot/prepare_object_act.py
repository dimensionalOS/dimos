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

"""Initialize the random-object observation projection from an existing ACT."""

import argparse
import json
from pathlib import Path

from lerobot.configs.policies import PreTrainedConfig
from lerobot.configs.types import FeatureType, NormalizationMode, PolicyFeature
from lerobot.policies.act.configuration_act import ACTConfig
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.factory import make_pre_post_processors
from safetensors.torch import load_file
import torch

from dimos.robot.galaxea.r1pro.object_packing import OBJECT_GOAL_FEATURES, OBJECT_PACKING_IO


def prepare(source: Path, dataset: Path, output: Path) -> None:
    if output.exists():
        raise FileExistsError(output)
    config = PreTrainedConfig.from_pretrained(source)
    if (
        not isinstance(config, ACTConfig)
        or "observation.environment_state" not in config.input_features
    ):
        raise ValueError("Initialize from an ACT with an existing environment token")
    torch.manual_seed(173)
    config.input_features["observation.environment_state"] = PolicyFeature(
        type=FeatureType.ENV, shape=(len(OBJECT_GOAL_FEATURES),)
    )
    config.normalization_mapping["ENV"] = NormalizationMode.MEAN_STD
    config.pretrained_path = None
    config.pretrained_backbone_weights = None
    config.device = "cpu"
    policy = ACTPolicy(config)
    weights = load_file(source / "model.safetensors")
    state = policy.state_dict()
    replaced = {
        "model.encoder_env_state_input_proj.weight",
        "model.encoder_env_state_input_proj.bias",
    }
    if state.keys() != weights.keys():
        raise ValueError("Source and target ACT architectures differ beyond goal features")
    for key, value in weights.items():
        if key in replaced:
            continue
        if value.shape != state[key].shape:
            raise ValueError(f"Unexpected checkpoint dimension: {key}")
        state[key].copy_(value)
    policy.load_state_dict(state)
    stats = {
        key: {name: torch.tensor(value) for name, value in values.items()}
        for key, values in json.loads((dataset / "meta/stats.json").read_text()).items()
    }
    pre, post = make_pre_post_processors(config, dataset_stats=stats)
    policy.save_pretrained(output)
    pre.save_pretrained(output)
    post.save_pretrained(output)
    (output / "initialization.json").write_text(
        json.dumps(
            dict(
                source=str(source.resolve()),
                profile=OBJECT_PACKING_IO.name,
                reinitialized=sorted(replaced),
                object_trained=False,
            ),
            indent=2,
        )
        + "\n"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    prepare(args.source, args.dataset, args.output)


if __name__ == "__main__":
    main()
