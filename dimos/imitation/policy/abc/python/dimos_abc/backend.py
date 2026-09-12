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

"""ABC-DiT inference, aligned with the pinned upstream reference implementation."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from abc_minimal.config import ClipConfig, DiTConfig, SimEvalConfig
from abc_minimal.dit import CLIPTextEmbedder, DiTPolicy, load_pretrained
from abc_minimal.fast_inference import FastInferenceGraph
from abc_minimal.preprocess import (
    load_norm_stats,
    normalize,
    parse_norm_stats,
    resize_pad_normalize,
    unnormalize,
)
import numpy as np
from numpy.typing import NDArray
import torch

from dimos.imitation.policy.backend import Images, joint_permutations
from dimos.imitation.policy.module import PolicyModuleConfig


class Backend:
    """Predict 30 absolute targets and execute 15, as in ABC's default evaluator."""

    def __init__(self, config: PolicyModuleConfig) -> None:
        if set(config.image_mapping.values()) != {"top", "left", "right"}:
            raise ValueError("ABC requires camera bindings for top, left, and right")
        if config.policy_joint_names is None:
            raise ValueError("ABC requires policy_joint_names in checkpoint order")
        if len(config.joint_names) != 14:
            raise ValueError("ABC requires 14 hardware joints")
        self.to_policy, self.to_hardware = joint_permutations(
            config.joint_names, config.policy_joint_names
        )
        self.device = torch.device(config.device or "cuda")
        if self.device.type == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("ABC requested CUDA but CUDA is not available")
        if config.fast_inference and self.device.type != "cuda":
            raise ValueError("ABC fast inference requires CUDA; disable fast_inference for CPU")
        self.config = SimEvalConfig(
            checkpoint=config.policy_path, model=DiTConfig(), prompt=config.task
        )
        self.diffusion_steps = config.diffusion_steps
        self.chunk_size = self.config.model.chunk_length
        self.n_action_steps = config.execution_steps or 15
        if self.n_action_steps > self.chunk_size:
            raise ValueError("ABC execution_steps must not exceed its 30-action horizon")
        self.fps = config.fps or 1.0 / 0.034
        self.action_lower = None
        self.action_upper = None
        self.model = DiTPolicy(self.config.model).to(self.device)
        checkpoint = load_pretrained(self.model, Path(config.policy_path))
        self.model.eval()
        if config.norm_stats_path is not None:
            self.norm_stats = load_norm_stats(config.norm_stats_path)
        elif checkpoint.get("norm_stats") is not None:
            self.norm_stats = parse_norm_stats(checkpoint["norm_stats"])
        else:
            raise ValueError("ABC checkpoint has no norm_stats; configure norm_stats_path")
        for kind in ("state", "actions"):
            for field in ("mean", "std"):
                values = self.norm_stats[kind][field]
                if values.shape != (14,) or not np.all(np.isfinite(values)):
                    raise ValueError(f"ABC {kind}.{field} must contain 14 finite values")
            if np.any(self.norm_stats[kind]["std"] < 0):
                raise ValueError(f"ABC {kind}.std must not be negative")
        clip_config = (
            ClipConfig(cache_dir=config.clip_cache_dir) if config.clip_cache_dir else ClipConfig()
        )
        self.embedder = CLIPTextEmbedder(clip_config, device=self.device)
        self.task = config.task
        self.task_vec = self.embedder.encode([config.task]).to(self.device)
        self.fast_inference = config.fast_inference
        self._fast_graph: Any = None

    @torch.inference_mode()
    def predict(
        self, images: Images, state: NDArray[np.float32], *, task: str
    ) -> NDArray[np.float32]:
        if task != self.task:
            raise ValueError("ABC task changes require a new preflight")
        observation: dict[str, Any] = {
            "state": state[self.to_policy],
            "images": {name: image.transpose(2, 0, 1) for name, image in images.items()},
        }
        if self.fast_inference and self._fast_graph is None:
            # Same compile and capture sequence as SimPolicy.enable_fast_inference.
            torch.backends.cuda.matmul.allow_tf32 = True
            torch.backends.cudnn.allow_tf32 = True
            torch.set_float32_matmul_precision("high")
            self.model.to(torch.bfloat16)
            self.model.img_backbone.set_bfloat16(True)
            self.task_vec = self.task_vec.to(device=self.device, dtype=torch.bfloat16)
            self.model.predict_velocity = torch.compile(
                self.model.predict_velocity, dynamic=False, mode="max-autotune-no-cudagraphs"
            )
            graph = FastInferenceGraph(self)
            graph.capture(observation, np.zeros((30, 14), dtype=np.float32), 24)
            self._fast_graph = graph
        if self._fast_graph is not None:
            actions = self._fast_graph.infer(observation, None)
        else:
            normalized_state = normalize(observation["state"], self.norm_stats["state"])
            batch = {
                "state": torch.from_numpy(normalized_state[None]).float().to(self.device),
                "actions": torch.zeros(1, 30, 14, device=self.device),
                "images": {
                    name: resize_pad_normalize(image).unsqueeze(0).to(self.device)
                    for name, image in observation["images"].items()
                },
                "task_vec_clip": self.task_vec,
            }
            predicted = self.model.sample_actions(batch, num_steps=self.diffusion_steps)
            actions = unnormalize(predicted[0].float().cpu().numpy(), self.norm_stats["actions"])
        return np.asarray(actions[:, self.to_hardware], dtype=np.float32)

    def reset(self) -> None:
        """The default ABC sampler has no action queue or prefix state."""

    def close(self) -> None:
        self._fast_graph = None
