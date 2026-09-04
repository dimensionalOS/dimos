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

"""Amazon ABC-DiT adapter for the shared DimOS rollout runtime."""

from __future__ import annotations

from collections.abc import Mapping
import json
from pathlib import Path
from types import SimpleNamespace
from typing import Any, cast

from abc_minimal.config import ClipConfig, DiTConfig
from abc_minimal.dit import CLIPTextEmbedder, DiTPolicy, load_pretrained
from abc_minimal.fast_inference import FastInferenceGraph
from abc_minimal.preprocess import normalize, parse_norm_stats, resize_pad_normalize, unnormalize
import numpy as np
from numpy.typing import NDArray
import torch

from dimos.imitation.policy.abc.module import AbcPolicyConfig, DualOpenYamAbcPolicy
from dimos.imitation.policy.backend import PolicyBackendInfo
from dimos.imitation.policy.runtime import declare_policy_runtime
from dimos.imitation.profile import ImageSource, JointPositionSource, PolicyIOProfile

torch.set_float32_matmul_precision("high")


class AbcBackend:
    """Run the released 14-D, three-camera ABC-DiT policy in process."""

    def __init__(self, config: AbcPolicyConfig) -> None:
        self._rollout_config = config
        self.config = SimpleNamespace(model=DiTConfig())
        self.device = torch.device("cpu")
        self.diffusion_steps = config.diffusion_steps
        self.model: DiTPolicy | None = None
        self.embedder: CLIPTextEmbedder | None = None
        self.task_vec = torch.empty(0)
        self.norm_stats: dict[str, Any] = {}
        self._task = ""
        self._fast_graph: FastInferenceGraph | None = None

    def load(self, profile: PolicyIOProfile) -> PolicyBackendInfo:
        _validate_profile(profile, self.config.model)
        checkpoint = Path(self._rollout_config.artifact).expanduser().resolve()
        if not checkpoint.is_file():
            raise FileNotFoundError(f"ABC checkpoint does not exist: {checkpoint}")
        device_name = self._rollout_config.device or (
            "cuda" if torch.cuda.is_available() else "cpu"
        )
        self.device = torch.device(device_name)
        if self.device.type == "cuda" and not torch.cuda.is_available():
            raise RuntimeError(f"ABC requested device {device_name!r}, but CUDA is unavailable")

        self.model = DiTPolicy(self.config.model).to(self.device)
        checkpoint_data = load_pretrained(self.model, checkpoint)
        self.model.eval()
        self.norm_stats = _resolve_norm_stats(
            checkpoint_data,
            self._rollout_config.norm_stats_path,
        )
        _validate_norm_stats(self.norm_stats, self.config.model)
        self.embedder = CLIPTextEmbedder(ClipConfig(), device=self.device)
        self._set_task(self._rollout_config.task)
        return PolicyBackendInfo(
            name="abc",
            chunk_length=self.config.model.chunk_length,
            preferred_execution_steps=15,
        )

    def reset(self) -> None:
        """ABC-DiT has no recurrent or queued model state."""

    @torch.no_grad()
    def predict(
        self,
        observations: Mapping[str, NDArray[Any]],
        task: str,
    ) -> NDArray[np.float32]:
        model = self._require_model()
        if task != self._task:
            if self._fast_graph is not None:
                raise ValueError("task text cannot change after ABC fast inference is captured")
            self._set_task(task)
        images: dict[str, NDArray[Any]] = {
            camera: np.asarray(observations[camera]).transpose(2, 0, 1)
            for camera in self.config.model.camera_keys
        }
        obs: dict[str, Any] = {
            "state": np.asarray(observations["state"], dtype=np.float32),
            "images": images,
        }
        if self._rollout_config.fast_inference and self.device.type == "cuda":
            if self._fast_graph is None:
                self._enable_fast_inference(obs)
            assert self._fast_graph is not None
            return self._fast_graph.infer(obs, noise=None)

        state = normalize(obs["state"], self.norm_stats["state"])
        batch = {
            "state": torch.from_numpy(state[None]).float().to(self.device),
            "actions": torch.zeros(
                1,
                self.config.model.chunk_length,
                self.config.model.action_dim,
                device=self.device,
            ),
            "images": {
                camera: resize_pad_normalize(obs["images"][camera]).unsqueeze(0).to(self.device)
                for camera in self.config.model.camera_keys
            },
            "task_vec_clip": self.task_vec,
        }
        actions = model.sample_actions(batch, num_steps=self.diffusion_steps)
        result = actions[0].float().detach().cpu().numpy()
        return np.asarray(
            unnormalize(result, self.norm_stats["actions"]),
            dtype=np.float32,
        )

    def _set_task(self, task: str) -> None:
        if self.embedder is None:
            raise RuntimeError("ABC text embedder is not loaded")
        self.task_vec = self.embedder.encode([task]).to(self.device)
        self._task = task

    def _enable_fast_inference(self, observation: dict[str, Any]) -> None:
        model = self._require_model()
        torch.backends.cuda.matmul.allow_tf32 = True
        torch.backends.cudnn.allow_tf32 = True
        model.to(torch.bfloat16)
        model.img_backbone.set_bfloat16(True)
        self.task_vec = self.task_vec.to(device=self.device, dtype=torch.bfloat16)
        model.predict_velocity = torch.compile(  # type: ignore[method-assign]
            model.predict_velocity,
            dynamic=False,
            mode="max-autotune-no-cudagraphs",
        )
        graph = FastInferenceGraph(self)
        graph.capture(observation, warmup_noise=None, replay_warmups=24)
        self._fast_graph = graph

    def _require_model(self) -> DiTPolicy:
        if self.model is None:
            raise RuntimeError("ABC backend is not loaded")
        return self.model


AbcPolicyRuntime = declare_policy_runtime(
    "AbcPolicyRuntime",
    __name__,
    DualOpenYamAbcPolicy,
    AbcBackend,
)


def _validate_profile(profile: PolicyIOProfile, model: DiTConfig) -> None:
    expected = {*model.camera_keys, "state"}
    if set(profile.observations) != expected:
        raise ValueError(f"released ABC checkpoint requires observation keys {sorted(expected)}")
    for key in model.camera_keys:
        if not isinstance(profile.observations[key], ImageSource):
            raise TypeError(f"ABC observation {key!r} must be an image")
    state = profile.observations["state"]
    if not isinstance(state, JointPositionSource) or len(state.joints) != model.state_dim:
        raise ValueError(f"ABC state must contain {model.state_dim} joints")
    if profile.action.key != "actions":
        raise ValueError("released ABC checkpoint requires the 'actions' output key")
    if len(profile.action.demonstration.joints) != model.action_dim:
        raise ValueError(f"ABC actions must contain {model.action_dim} joints")


def _resolve_norm_stats(
    checkpoint: dict[str, Any],
    override: str | None,
) -> dict[str, Any]:
    if override is not None:
        raw = json.loads(Path(override).expanduser().read_text())
    elif checkpoint.get("norm_stats") is not None:
        raw = checkpoint["norm_stats"]
    else:
        raise ValueError("ABC checkpoint has no norm_stats; set norm_stats_path")
    return cast("dict[str, Any]", parse_norm_stats(raw))


def _validate_norm_stats(stats: dict[str, Any], model: DiTConfig) -> None:
    for key, width in (("state", model.state_dim), ("actions", model.action_dim)):
        for statistic in ("mean", "std"):
            value = np.asarray(stats[key][statistic])
            if value.shape != (width,):
                raise ValueError(
                    f"ABC {key} {statistic} shape {value.shape} does not match {(width,)}"
                )
            if not np.all(np.isfinite(value)):
                raise ValueError(f"ABC {key} {statistic} contains non-finite values")
