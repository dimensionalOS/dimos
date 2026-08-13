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

"""The trained net behind the controller protocol.

    python -m dimos.navigation.motion.control --score --controller ml
    python -m dimos.navigation.motion.control --score --blind --controller ml

Same seam as every law: ``update(pose, path, t, clearance) -> Twist``, judged
by the identical battery, tracks and judge. Deterministic here -- the training
loop's exploration noise is not part of the candidate.

Python inference is a research vehicle. A shipped learned law is a rust port
under ``control/rust/`` held to ``test_rust_parity.py``'s 1e-9 discipline, as
the hand-written laws are; the net gets there via ONNX + tract, with
:mod:`obs` ported alongside.
"""

from __future__ import annotations

import os
from pathlib import Path as FsPath

import torch

from dimos.navigation.motion.control.controller import ControllerConfig
from dimos.navigation.motion.control.laws.hinted import ENVELOPE
from dimos.navigation.motion.control.research.ml import policy
from dimos.navigation.motion.control.research.ml.env import Agent

CKPT_ENV = "ML_TC_CKPT"
DEFAULT_CKPT = FsPath(__file__).resolve().parent / "checkpoints" / "base.pt"


def config() -> ControllerConfig:
    """The envelope the net commands inside.

    ``ENVELOPE`` is a PLANT measurement, not a preference of the hinted law
    that documents it: gait initiation is a bifurcation around 0.28 commanded,
    and 0.95 stops short of the 1.0 at which the walking policy hands over to
    a different expert. A net clipped at the referee's 0.5 default could not
    reach the hinted track's 0.75 m/s cruise target at all, so it drives the
    same envelope the researched law does. The governor fields go unread --
    the net has no governor, it has the room numbers themselves.
    """
    return ControllerConfig(min_speed=ENVELOPE["min_speed"], max_speed=ENVELOPE["max_speed"])


def _load_net() -> policy.ActorCritic:
    path = FsPath(os.environ.get(CKPT_ENV, str(DEFAULT_CKPT)))
    if not path.exists():
        raise FileNotFoundError(
            f"no checkpoint at {path}; train one first "
            f"(python -m dimos.navigation.motion.control.research.ml.train) "
            f"or point {CKPT_ENV} at one"
        )
    torch.set_num_threads(1)  # episodes parallelise by process; threads would fight
    net = policy.load(str(path))
    for p in net.parameters():
        p.requires_grad_(False)
    return net


_NET: policy.ActorCritic | None = None


def make(cfg: ControllerConfig | None = None) -> Agent:
    global _NET
    if _NET is None:
        _NET = _load_net()
    return Agent(_NET, config=cfg or config(), deterministic=True)
