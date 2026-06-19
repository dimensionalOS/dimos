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

"""MLP actor loader. Single-step `act(obs)` for reactive control.

Loads a PyTorch checkpoint (`.pt`) whose actor is a plain Linear/ELU
MLP with a Gaussian distribution head and an empirical observation
normalizer. Verified against on-policy actor-critic checkpoints
(PPO/A2C/TRPO families); compatible with any trainer that follows
this on-disk convention:

    {
      "actor_state_dict": {
        "obs_normalizer._mean": (1, obs_dim),
        "obs_normalizer._var":  (1, obs_dim),
        "obs_normalizer._std":  (1, obs_dim),
        "obs_normalizer.count": ...,
        "distribution.std_param": (action_dim,),   # we take the mean,
                                                   # ignore the std at deploy
        "mlp.{0,2,4,6}.weight": ...,               # Linear / ELU / Linear / ...
        "mlp.{0,2,4,6}.bias":   ...,
      },
      "critic_state_dict": ...,    # unused at deploy time
      "optimizer_state_dict": ...,
      "iter": int,
      "infos": {...},
    }

NOT compatible with: off-policy actors (SAC's tanh-squashing,
TD3's deterministic head), value-based methods (DQN), or
SB3 / CleanRL checkpoint formats. Those need their own loaders.

We reconstruct the MLP from the saved weight shapes (no training-library
dependency at deploy time), apply (obs - mean) / std normalization, and
return the mean action (no exploration noise at deploy time).
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass
class MLPPolicyConfig:
    """Architecture metadata recovered from the MLP-actor checkpoint."""

    obs_dim: int
    action_dim: int
    hidden_dims: list[int]
    activation: str = "elu"
    normalize_obs: bool = True


class MLPPolicy:
    """Deployed MLP actor with an empirical observation normalizer.

    Stateless. Same `obs` in, same `action` out. The caller (a ControlTask)
    owns any per-tick state (gait clock, last_action cache, etc.).
    """

    def __init__(
        self,
        actor,  # torch.nn.Module
        obs_mean: np.ndarray,  # (obs_dim,)
        obs_std: np.ndarray,  # (obs_dim,)
        cfg: MLPPolicyConfig,
        device: str,
    ) -> None:
        self._actor = actor
        self._obs_mean = obs_mean
        self._obs_std = obs_std
        self._cfg = cfg
        self._device = device
        # Cache a torch tensor handle to avoid re-importing torch on every act().
        import torch

        self._torch = torch
        self._mean_t = torch.from_numpy(obs_mean).to(device)
        self._std_t = torch.from_numpy(obs_std).to(device)

    @classmethod
    def load(cls, path: str | Path, device: str = "cpu") -> MLPPolicy:
        import torch
        from torch import nn

        path = Path(path)
        if path.is_dir():
            cands = sorted(path.glob("model_*.pt"), key=lambda p: int(p.stem.split("_")[1]))
            if not cands:
                raise FileNotFoundError(f"No model_*.pt in {path}")
            path = cands[-1]

        ckpt = torch.load(path, map_location=device, weights_only=False)
        sd = ckpt["actor_state_dict"]

        # Discover MLP layer widths from weight shapes (mlp.0, mlp.2, mlp.4, mlp.6).
        layer_keys = sorted(
            [k for k in sd if k.startswith("mlp.") and k.endswith(".weight")],
            key=lambda k: int(k.split(".")[1]),
        )
        widths = [sd[k].shape for k in layer_keys]  # [(out, in), ...]
        obs_dim = int(widths[0][1])
        action_dim = int(widths[-1][0])
        hidden_dims = [int(w[0]) for w in widths[:-1]]

        # Build the actor MLP: Linear -> ELU -> Linear -> ELU -> ... -> Linear.
        layers: list[nn.Module] = []
        prev = obs_dim
        for h in hidden_dims:
            layers.append(nn.Linear(prev, h))
            layers.append(nn.ELU())
            prev = h
        layers.append(nn.Linear(prev, action_dim))
        mlp = nn.Sequential(*layers)

        # Load weights into the MLP. Keys are mlp.0/2/4/6 - our Sequential
        # uses the same even-index convention (Linear at 0, 2, 4, ...).
        own_sd = {}
        for k, v in sd.items():
            if k.startswith("mlp."):
                own_sd[k[4:]] = v  # strip the "mlp." prefix
        mlp.load_state_dict(own_sd, strict=True)
        mlp.eval().to(device)

        # Obs normalizer: stored as (1, obs_dim) tensors of (mean, var, std).
        obs_mean = sd["obs_normalizer._mean"].cpu().numpy().reshape(-1).astype(np.float32)
        obs_std = sd["obs_normalizer._std"].cpu().numpy().reshape(-1).astype(np.float32)
        # Guard against zero std (e.g. unused dims).
        obs_std = np.where(obs_std < 1e-6, 1.0, obs_std)

        cfg = MLPPolicyConfig(
            obs_dim=obs_dim,
            action_dim=action_dim,
            hidden_dims=hidden_dims,
        )
        return cls(mlp, obs_mean, obs_std, cfg, device)

    @property
    def config(self) -> MLPPolicyConfig:
        return self._cfg

    def act(self, obs: np.ndarray) -> np.ndarray:
        """Return the RAW actor output. Caller applies scale+offset.

        mjlab's training pipeline stores `env.action_manager.action` (the raw
        output) as the `last_actions` obs term. Scaling lives in the action
        term (`JointPositionAction.scale * raw + offset`), not in the actor.
        Mirroring that here keeps `last_actions` in-distribution.
        """
        if obs.shape != (self._cfg.obs_dim,):
            raise ValueError(f"obs shape {obs.shape} != ({self._cfg.obs_dim},)")
        x = self._torch.from_numpy(np.asarray(obs, dtype=np.float32)).to(self._device)
        with self._torch.no_grad():
            if self._cfg.normalize_obs:
                x = (x - self._mean_t) / self._std_t
            action = self._actor(x.unsqueeze(0)).squeeze(0)
        return action.cpu().numpy()


__all__ = ["MLPPolicy", "MLPPolicyConfig"]
