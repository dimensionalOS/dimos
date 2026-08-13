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

"""The actor-critic: observation in, normalised twist out.

Small on purpose (~70k params, two hidden layers). The law it replaces is a
handful of arithmetic on the same inputs, the referee runs thousands of
episodes, and a shipped candidate is a rust/tract port -- none of that wants a
big net. Input scaling lives here (fixed constants from :mod:`obs`) so the
observation stays in real units.

Actions are normalised to ``[-1, 1]`` per axis and denormalised against the
:class:`ControllerConfig` limits by :func:`to_twist`, so the policy cannot
command outside the envelope the laws drive inside, whatever it learns.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np
import torch
from torch import Tensor, nn

from dimos.navigation.motion.control.research.ml import obs

ACT_DIM = 3  # vx, vy, wz -- normalised


@dataclass(frozen=True)
class PolicyCfg:
    hidden: int = 256
    layers: int = 2
    # std ~0.30 of the normalised range, so early exploration commands land
    # around the 0.28 m/s the gait actually initiates at (laws/hinted.py's
    # ENVELOPE documents the bifurcation). The PPO default of 1.0 flings the
    # body at its slew limit on tick one and every first episode is a fall,
    # which teaches the critic that moving is fatal.
    log_std_init: float = -1.2


def _trunk(hidden: int, layers: int) -> nn.Sequential:
    mods: list[nn.Module] = []
    last = obs.DIM
    for _ in range(layers):
        mods += [nn.Linear(last, hidden), nn.Tanh()]
        last = hidden
    return nn.Sequential(*mods)


class ActorCritic(nn.Module):
    """Diagonal-Gaussian actor and scalar critic over SEPARATE trunks.

    Separate, not shared, and that is the difference between fine-tuning a
    cloned policy and destroying it. Behaviour cloning fits the actor only, so
    PPO's first iteration starts with a random critic that has to regress
    returns two orders of magnitude larger than anything it outputs. Through a
    shared trunk those value-loss gradients tear up the exact features the
    clone just learned: measured, on a net that scored 115.08, one iteration
    took it to 8 disqualifications and the next to 18 of 24. With the trunks
    split the critic can be as wrong as it likes without touching the actor.
    """

    in_scale: Tensor  # registered buffer; annotated so it types as a Tensor

    def __init__(self, cfg: PolicyCfg = PolicyCfg()) -> None:
        super().__init__()
        self.cfg = cfg
        self.trunk = _trunk(cfg.hidden, cfg.layers)  # actor's
        self.critic_trunk = _trunk(cfg.hidden, cfg.layers)
        self.mu = nn.Linear(cfg.hidden, ACT_DIM)
        self.v = nn.Linear(cfg.hidden, 1)
        self.log_std = nn.Parameter(torch.full((ACT_DIM,), cfg.log_std_init))
        # Small final actor weights: the first rollouts should be near-zero
        # commands rather than a body flung at its slew limit.
        nn.init.orthogonal_(self.mu.weight, gain=0.01)
        nn.init.zeros_(self.mu.bias)
        scale = np.concatenate([np.tile(obs.MARK_SCALE, len(obs.ARC_MARKS)), obs.SCALAR_SCALE])
        self.register_buffer("in_scale", torch.from_numpy(scale.astype(np.float32)))

    def actor_parameters(self) -> list[nn.Parameter]:
        return [*self.trunk.parameters(), *self.mu.parameters(), self.log_std]

    def critic_parameters(self) -> list[nn.Parameter]:
        return [*self.critic_trunk.parameters(), *self.v.parameters()]

    def forward(self, x: Tensor) -> tuple[Tensor, Tensor, Tensor]:
        """-> (mean action, log std, value)."""
        xs = x * self.in_scale
        mu = self.mu(self.trunk(xs))
        return mu, self.log_std.expand_as(mu), self.v(self.critic_trunk(xs)).squeeze(-1)

    def act(self, x: Tensor, deterministic: bool = False) -> tuple[Tensor, Tensor, Tensor]:
        """Sample (or take the mean) -> (action, log prob, value)."""
        mu, log_std, value = self(x)
        if deterministic:
            return mu, torch.zeros(mu.shape[:-1]), value
        std = log_std.exp()
        action = mu + std * torch.randn_like(mu)
        return action, gaussian_logp(action, mu, log_std), value

    def evaluate(self, x: Tensor, action: Tensor) -> tuple[Tensor, Tensor, Tensor]:
        """-> (log prob of ``action``, entropy, value) for the PPO update."""
        mu, log_std, value = self(x)
        return gaussian_logp(action, mu, log_std), gaussian_entropy(log_std), value


def gaussian_logp(action: Tensor, mu: Tensor, log_std: Tensor) -> Tensor:
    var = torch.exp(2.0 * log_std)
    logp: Tensor = -0.5 * ((action - mu) ** 2 / var + 2.0 * log_std + float(np.log(2.0 * np.pi)))
    return logp.sum(-1)


def gaussian_entropy(log_std: Tensor) -> Tensor:
    return (log_std + 0.5 * float(np.log(2.0 * np.pi * np.e))).sum(-1)


def to_twist(
    action: np.ndarray, max_speed: float, max_yaw_rate: float
) -> tuple[float, float, float]:
    """Normalised action -> (vx, vy, wz), clipped into the config's envelope.

    The planar pair is clipped as a VECTOR, not per axis: a per-axis clip lets
    a diagonal command reach ``max_speed * sqrt(2)``, which the laws never do
    and the plant would not deliver.
    """
    a = np.clip(np.asarray(action, dtype=float), -1.0, 1.0)
    vx, vy = a[0] * max_speed, a[1] * max_speed
    speed = float(np.hypot(vx, vy))
    if speed > max_speed:
        vx, vy = vx / speed * max_speed, vy / speed * max_speed
    return float(vx), float(vy), float(a[2] * max_yaw_rate)


def param_count(m: nn.Module) -> int:
    return sum(p.numel() for p in m.parameters())


def save(path: str, net: ActorCritic) -> None:
    torch.save(
        {"cfg": asdict(net.cfg), "format": obs.FORMAT, "state": net.state_dict()},
        path,
    )


def load(path: str) -> ActorCritic:
    blob = torch.load(path, map_location="cpu", weights_only=False)
    if blob["format"] != obs.FORMAT:
        raise ValueError(
            f"checkpoint built for observation format {blob['format']}, have {obs.FORMAT}"
        )
    net = ActorCritic(PolicyCfg(**blob["cfg"]))
    net.load_state_dict(blob["state"])
    net.eval()
    return net
