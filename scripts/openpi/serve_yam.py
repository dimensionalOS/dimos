# Copyright 2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0.

from __future__ import annotations

import argparse
import logging
from pathlib import Path
import time
from typing import Any

import jax.numpy as jnp
from openpi import transforms
from openpi.models import model as model_lib
from openpi.models.pi0_config import Pi0Config
from openpi.models.tokenizer import PaligemmaTokenizer
from openpi.policies.aloha_policy import AlohaInputs, AlohaOutputs
from openpi.policies.policy import Policy
from openpi.serving.websocket_policy_server import WebsocketPolicyServer
from openpi.shared import normalize
from openpi_client.base_policy import BasePolicy


class LoggedPolicy(BasePolicy):
    """Log total policy inference time for each server request."""

    def __init__(self, policy: Policy) -> None:
        self._policy = policy

    def infer(self, obs: dict[str, Any]) -> dict[str, Any]:
        started = time.perf_counter()
        result = self._policy.infer(obs)
        total_ms = (time.perf_counter() - started) * 1000
        logging.info(
            "OpenPI policy inference complete total_ms=%.1f reported_model_call_ms=%s",
            total_ms,
            result.get("policy_timing", {}).get("infer_ms"),
        )
        return result

    def reset(self) -> None:
        self._policy.reset()


parser = argparse.ArgumentParser(description="Serve the Robocurve YAM checkpoint to DimOS")
parser.add_argument("--checkpoint", required=True, type=Path)
parser.add_argument("--host", default="127.0.0.1")
parser.add_argument("--port", default=8000, type=int)
args = parser.parse_args()
ROOT = args.checkpoint
logging.basicConfig(level=logging.INFO)

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
WebsocketPolicyServer(
    LoggedPolicy(policy), host=args.host, port=args.port, metadata=metadata
).serve_forever()
