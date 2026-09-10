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

"""Initialize a distinct goal-conditioned ACT from the existing bottle policy.

The new environment token is untrained. This artifact must be fine-tuned and
physically evaluated before being called a working packing policy.
"""

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

from dimos.robot.galaxea.r1pro.learning import R1PRO_PACKING_GOAL_FEATURES, R1PRO_PACKING_IO


def prepare(source: Path, dataset: Path, output: Path) -> None:
    if output.exists():
        raise FileExistsError(output)
    config = PreTrainedConfig.from_pretrained(source)
    if not isinstance(config, ACTConfig):
        raise ValueError("Packing initialization requires an ACT checkpoint")
    torch.manual_seed(17)
    config.input_features["observation.environment_state"] = PolicyFeature(
        type=FeatureType.ENV,
        shape=(len(R1PRO_PACKING_GOAL_FEATURES),),
    )
    config.normalization_mapping["ENV"] = NormalizationMode.MEAN_STD
    config.pretrained_path = None
    config.pretrained_backbone_weights = None
    config.device = "cpu"
    policy = ACTPolicy(config)
    weights = load_file(source / "model.safetensors")
    state = policy.state_dict()
    positional = "model.encoder_1d_feature_pos_embed.weight"
    for key, value in weights.items():
        if key == positional:
            if value.shape[0] != 2 or state[key].shape[0] != 3:
                raise ValueError("Expected to add exactly one environment token")
            state[key][:2].copy_(value)
        elif key not in state or state[key].shape != value.shape:
            raise ValueError(f"Unexpected incompatible checkpoint weight: {key}")
        else:
            state[key].copy_(value)
    expected = {
        "model.encoder_env_state_input_proj.weight",
        "model.encoder_env_state_input_proj.bias",
    }
    if set(state) - set(weights) != expected:
        raise ValueError("Unexpected new weights beyond the goal input projection")
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
            {
                "source": str(source.resolve()),
                "profile": R1PRO_PACKING_IO.name,
                "new_parameters": sorted(expected),
                "packing_trained": False,
            },
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
