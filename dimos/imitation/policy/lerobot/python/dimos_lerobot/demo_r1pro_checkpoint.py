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

"""Create an untrained constant-output ACT artifact for deployment diagnostics.

The real ACT network executes, but its output head ignores encoded observations.
This verifies inference plumbing only and must never be used as grasp evidence.
"""

import argparse
from pathlib import Path

from lerobot.configs.types import FeatureType, PolicyFeature
from lerobot.policies.act.configuration_act import ACTConfig
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.factory import make_pre_post_processors
import torch

from dimos.robot.galaxea.r1pro.learning import R1PRO_SIM_ACT_JOINTS


def create_checkpoint(output: Path) -> None:
    """Save a small local ACT model whose wrists target +/- 0.05 radians."""
    output.mkdir(parents=True, exist_ok=False)
    width = len(R1PRO_SIM_ACT_JOINTS)
    torch.manual_seed(0)
    config = ACTConfig(
        device="cuda",
        pretrained_backbone_weights=None,
        input_features={
            "observation.images.overview": PolicyFeature(
                type=FeatureType.VISUAL, shape=(3, 240, 320)
            ),
            "observation.state": PolicyFeature(type=FeatureType.STATE, shape=(width,)),
        },
        output_features={"action": PolicyFeature(type=FeatureType.ACTION, shape=(width,))},
        chunk_size=8,
        n_action_steps=4,
        dim_model=64,
        dim_feedforward=128,
        n_heads=4,
        n_encoder_layers=1,
        n_decoder_layers=1,
        n_vae_encoder_layers=1,
    )
    policy = ACTPolicy(config)
    target = torch.zeros(width)
    target[R1PRO_SIM_ACT_JOINTS.index("r1pro/left_arm_joint7")] = 0.05
    target[R1PRO_SIM_ACT_JOINTS.index("r1pro/right_arm_joint7")] = -0.05
    with torch.no_grad():
        policy.model.action_head.weight.zero_()
        policy.model.action_head.bias.copy_(target)
    stats = {
        key: {
            "mean": torch.zeros(width),
            "std": torch.ones(width),
            "min": torch.full((width,), -0.1),
            "max": torch.full((width,), 0.1),
        }
        for key in ("observation.state", "action")
    }
    stats["observation.images.overview"] = {
        "mean": torch.zeros(3, 1, 1),
        "std": torch.ones(3, 1, 1),
    }
    pre, post = make_pre_post_processors(config, dataset_stats=stats)
    policy.save_pretrained(output)
    pre.save_pretrained(output)
    post.save_pretrained(output)
    (output / "DIAGNOSTIC_ONLY.txt").write_text(
        "UNTRAINED constant-output ACT. No grasping or navigation skill. "
        "Fixed base and grippers. Left/right wrist targets +0.05/-0.05 rad.\n"
    )
    print(f"Saved diagnostic ACT checkpoint to {output}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    create_checkpoint(parser.parse_args().output)
