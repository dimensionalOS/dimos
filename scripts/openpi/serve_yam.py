# Copyright 2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0.

from __future__ import annotations

import argparse
from pathlib import Path

import jax.numpy as jnp
from openpi import transforms
from openpi.models import model as model_lib
from openpi.models.pi0_config import Pi0Config
from openpi.models.tokenizer import PaligemmaTokenizer
from openpi.policies.aloha_policy import AlohaInputs, AlohaOutputs
from openpi.policies.policy import Policy
from openpi.serving.websocket_policy_server import WebsocketPolicyServer
from openpi.shared import normalize

parser = argparse.ArgumentParser(description="Serve the Robocurve YAM checkpoint to DimOS")
parser.add_argument("--checkpoint", required=True, type=Path)
parser.add_argument("--host", default="127.0.0.1")
parser.add_argument("--port", default=8000, type=int)
args = parser.parse_args()
ROOT = args.checkpoint

stats = normalize.load(ROOT / "assets/yam-bimanual-merged")
config = Pi0Config(pi05=True, action_horizon=16)
model = config.load(
    model_lib.restore_params(ROOT / "params", dtype=jnp.bfloat16), remove_extra_params=False
)
policy = Policy(
    model,
    transforms=[
        AlohaInputs(adapt_to_pi=False),
        transforms.Normalize(stats, use_quantiles=True),
        transforms.ResizeImages(224, 224),
        transforms.TokenizePrompt(
            PaligemmaTokenizer(config.max_token_len), discrete_state_input=True
        ),
        transforms.PadStatesAndActions(config.action_dim),
    ],
    output_transforms=[
        transforms.Unnormalize(stats, use_quantiles=True),
        AlohaOutputs(adapt_to_pi=False),
    ],
)
metadata = {
    "checkpoint": str(ROOT),
    "action_shape": [16, 14],
    "gripper_space": "normalized_opening_0_closed_1_open",
}
WebsocketPolicyServer(policy, host=args.host, port=args.port, metadata=metadata).serve_forever()
