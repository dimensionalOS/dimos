"""CUDA graph helpers for fast ABC-DiT policy inference."""

from __future__ import annotations

from typing import Any, Protocol

import numpy as np
import torch

from abc_minimal.preprocess import normalize, resize_pad_normalize, unnormalize


class _PolicyForFastInference(Protocol):
    model: Any
    device: torch.device
    config: Any
    task_vec: torch.Tensor
    norm_stats: dict[str, Any]
    diffusion_steps: int

    def normalized_action_prefix(
        self,
        action_prefix: np.ndarray,
        prefix_length: int,
    ) -> np.ndarray: ...


class FastInferenceGraph:
    """Fixed-shape outer CUDA graph over `model.sample_actions`.

    Captures one bf16 batch on GPU; subsequent .infer() calls just memcpy
    fresh inputs into the static tensors and replay the graph.
    """

    def __init__(self, policy: _PolicyForFastInference):
        self.policy = policy
        self.model = policy.model
        self.device = policy.device
        self.dtype = torch.bfloat16
        self.graph = torch.cuda.CUDAGraph()
        self.output: torch.Tensor | None = None
        m = policy.config.model

        self.static_state = torch.empty(1, m.state_dim, device=self.device, dtype=self.dtype)
        self.static_noise = torch.empty(
            1, m.chunk_length, m.action_dim, device=self.device, dtype=self.dtype
        )
        self.static_images = {
            cam: torch.empty(1, 3, 224, 224, device=self.device, dtype=self.dtype)
            for cam in m.camera_keys
        }
        self.static_task_vec = policy.task_vec.to(device=self.device, dtype=self.dtype).clone()
        self.batch = {
            "state": self.static_state,
            "actions": torch.zeros(
                1, m.chunk_length, m.action_dim, device=self.device, dtype=self.dtype
            ),
            "images": self.static_images,
            "task_vec_clip": self.static_task_vec,
        }

    def _copy_inputs(self, obs: dict[str, Any], noise: np.ndarray | None) -> None:
        m = self.policy.config.model
        state = normalize(
            np.asarray(obs["state"], dtype=np.float32), self.policy.norm_stats["state"]
        )
        self.static_state.copy_(
            torch.from_numpy(state[None]).to(device=self.device, dtype=self.dtype)
        )
        if noise is None:
            self.static_noise.normal_()
        else:
            noise_arr = noise[None].astype(np.float32, copy=False)
            if noise_arr.shape != (1, m.chunk_length, m.action_dim):
                raise ValueError(
                    f"fast inference expects noise shape "
                    f"{(m.chunk_length, m.action_dim)}, got {noise.shape}"
                )
            self.static_noise.copy_(
                torch.from_numpy(noise_arr).to(device=self.device, dtype=self.dtype)
            )
        for cam in m.camera_keys:
            self.static_images[cam].copy_(
                resize_pad_normalize(obs["images"][cam])
                .unsqueeze(0)
                .to(device=self.device, dtype=self.dtype)
            )

    def capture(
        self,
        warmup_obs: dict[str, Any],
        warmup_noise: np.ndarray | None,
        replay_warmups: int,
    ) -> None:
        self._copy_inputs(warmup_obs, warmup_noise)

        stream = torch.cuda.Stream()
        stream.wait_stream(torch.cuda.current_stream())
        with torch.cuda.stream(stream):
            for _ in range(5):
                self.output = self.model.sample_actions(
                    self.batch,
                    num_steps=self.policy.diffusion_steps,
                    noise=self.static_noise,
                )
        torch.cuda.current_stream().wait_stream(stream)

        with torch.cuda.graph(self.graph):
            self.output = self.model.sample_actions(
                self.batch, num_steps=self.policy.diffusion_steps, noise=self.static_noise
            )

        for _ in range(replay_warmups):
            self._copy_inputs(warmup_obs, warmup_noise)
            self.graph.replay()
            assert self.output is not None
            _ = self.output[0].float().detach().cpu().numpy()
        torch.cuda.synchronize()

    def infer(self, obs: dict[str, Any], noise: np.ndarray | None) -> np.ndarray:
        self._copy_inputs(obs, noise)
        self.graph.replay()
        assert self.output is not None
        actions_np = self.output[0].float().detach().cpu().numpy()
        return unnormalize(actions_np, self.policy.norm_stats["actions"]).astype(np.float32)


class FastRTCInferenceGraph(FastInferenceGraph):
    def __init__(self, policy: _PolicyForFastInference, prefix_length: int):
        super().__init__(policy)
        self.prefix_length = prefix_length
        self.static_action_prefix = torch.empty_like(self.static_noise)

    def _copy_prefix(self, action_prefix: np.ndarray) -> None:
        prefix = self.policy.normalized_action_prefix(action_prefix, self.prefix_length)
        self.static_action_prefix.copy_(
            torch.from_numpy(prefix[None]).to(device=self.device, dtype=self.dtype)
        )

    def capture(
        self,
        warmup_obs: dict[str, Any],
        warmup_noise: np.ndarray | None,
        warmup_prefix: np.ndarray,
        replay_warmups: int,
    ) -> None:
        self._copy_inputs(warmup_obs, warmup_noise)
        self._copy_prefix(warmup_prefix)

        stream = torch.cuda.Stream()
        stream.wait_stream(torch.cuda.current_stream())
        with torch.cuda.stream(stream):
            for _ in range(5):
                self.output = self.model.sample_actions_rtc(
                    self.batch,
                    self.static_action_prefix,
                    prefix_length=self.prefix_length,
                    num_steps=self.policy.diffusion_steps,
                    noise=self.static_noise,
                )
        torch.cuda.current_stream().wait_stream(stream)

        with torch.cuda.graph(self.graph):
            self.output = self.model.sample_actions_rtc(
                self.batch,
                self.static_action_prefix,
                prefix_length=self.prefix_length,
                num_steps=self.policy.diffusion_steps,
                noise=self.static_noise,
            )

        for _ in range(replay_warmups):
            self._copy_inputs(warmup_obs, warmup_noise)
            self._copy_prefix(warmup_prefix)
            self.graph.replay()
            assert self.output is not None
            _ = self.output[0].float().detach().cpu().numpy()
        torch.cuda.synchronize()

    def infer(
        self,
        obs: dict[str, Any],
        noise: np.ndarray | None,
        action_prefix: np.ndarray,
    ) -> np.ndarray:
        self._copy_inputs(obs, noise)
        self._copy_prefix(action_prefix)
        self.graph.replay()
        assert self.output is not None
        actions_np = self.output[0].float().detach().cpu().numpy()
        return unnormalize(actions_np, self.policy.norm_stats["actions"]).astype(np.float32)
