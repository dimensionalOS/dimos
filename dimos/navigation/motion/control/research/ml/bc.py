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

"""Behaviour cloning: start the net where the shipped law already is.

    python -m dimos.navigation.motion.control.research.ml.bc --jobs 12

PPO from scratch cannot find this task, and the reason is the plant rather
than the algorithm. Gait initiation is a BIFURCATION near 0.28 commanded
(``laws/hinted.py``'s ENVELOPE documents the sweep), so a mean-zero policy with
enough exploration noise to be safe sits below the walk threshold and the body
never moves far enough to see progress. Meanwhile the judge pays ``10 *
precision`` for a world the robot never entered, so standing still is a stable
local optimum worth 10.1 points -- which is exactly what a from-scratch run
finds and never leaves (measured: 6 iterations, arrived 0.00, score 10.4 flat).

So the net starts from the law. ``TeacherAgent`` runs ``laws/hinted.py``
inside real episodes and tapes the observation the net WOULD have seen against
the action the law actually took; this fits the actor to that by regression.
The result is a net that tracks about as well as the law and, crucially, one
whose rollouts reach goals -- which is the state distribution PPO needs before
any of its gradients mean anything.

Cloning caps the net AT the teacher. Beating it is ``train.py``'s job, and the
honest comparison is always the same battery against ``--controller hinted``.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import os
from pathlib import Path
import time
from typing import Any
import zlib

import numpy as np
import torch
from torch import Tensor

from dimos.navigation.motion.control.research.ml import env, obs, policy
from dimos.navigation.motion.control.research.ml.train import LOGS, Pool
from dimos.navigation.motion.control.tracks import TRACKS

CKPT = Path(__file__).resolve().parent / "checkpoints"
DEFAULT_POLICY = "ml-trajectory-research/freewalk_mcf.bin"


def _teacher_job(job: tuple[Any, ...]) -> dict[str, Any]:
    """One teacher episode in a worker. Mirrors train._run_job's contract.

    ``train._WORKER`` is this process's own module global, populated by the
    pool initialiser -- the walking policy is loaded once per worker, not once
    per episode.
    """
    from dimos.navigation.motion.control.controller import load as load_controller
    from dimos.navigation.motion.control.referee.episode import run_episode
    from dimos.navigation.motion.control.referee.judge import score_episode
    from dimos.navigation.motion.control.research.ml.train import _WORKER

    sc, law_name, ep_cfg, seed = job
    tape = env.Tape()
    law = load_controller(law_name)()
    # The law runs on ITS config; the tape is normalised by the CANDIDATE's,
    # because that is the envelope `Agent` will denormalise through later. Using
    # the law's here would silently rescale every cloned action whenever the two
    # envelopes differ -- a `blind` teacher (0.5) cloned into the candidate
    # envelope (0.95) would come out commanding nearly twice the speed.
    agent = env.TeacherAgent(law, config=_WORKER["config"], tape=tape)
    rng = np.random.default_rng([seed, zlib.crc32(sc.name.encode())])
    result = run_episode(sc, agent, _WORKER["walk"], ep_cfg, rng=rng)
    row = score_episode(result)
    return {
        "feat": np.asarray(tape.feat, dtype=np.float32).reshape(-1, obs.DIM),
        "action": np.asarray(tape.action, dtype=np.float32).reshape(-1, policy.ACT_DIM),
        "row": row,
    }


def collect(pool: Pool, scenarios: list[Any], law: str, ep_cfg: Any, seed: int) -> dict[str, Any]:
    """Roll the teacher over every world; return the pooled (obs, action) set."""
    out = pool.map([(sc, law, ep_cfg, seed) for sc in scenarios], _teacher_job)
    feats = [o["feat"] for o in out if len(o["feat"])]
    acts = [o["action"] for o in out if len(o["feat"])]
    rows = [o["row"] for o in out]
    return {
        "feat": np.concatenate(feats) if feats else np.zeros((0, obs.DIM), dtype=np.float32),
        "action": np.concatenate(acts) if acts else np.zeros((0, policy.ACT_DIM), dtype=np.float32),
        "teacher_score": round(float(np.mean([r["total"] for r in rows])), 2),
        "teacher_dq": sum(1 for r in rows if r["dq"]),
        "teacher_arrived": round(float(np.mean([r["arrived"] for r in rows])), 4),
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--tag", default="bc", help="checkpoint name (checkpoints/<tag>.pt)")
    ap.add_argument("--track", default="hinted", choices=("hinted", "blind"))
    ap.add_argument("--teacher", default=None, help="law to clone (default: the track's)")
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 2))
    ap.add_argument("--worlds", type=int, default=120, help="teacher episodes to collect")
    ap.add_argument("--val-worlds", type=int, default=16)
    ap.add_argument("--policy", default=DEFAULT_POLICY)
    ap.add_argument("--epochs", type=int, default=60)
    ap.add_argument("--batch", type=int, default=512)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    law = args.teacher or TRACKS[args.track].controller
    ep_cfg = env.episode_config(args.track)

    net = policy.ActorCritic()
    pool = Pool(args.jobs, args.policy, net.cfg)
    t0 = time.perf_counter()
    try:
        print(f"collecting {args.worlds} teacher episodes from '{law}'...", flush=True)
        data = collect(pool, env.train_worlds(args.worlds), law, ep_cfg, args.seed)
        n = len(data["feat"])
        print(
            f"  {n:,} transitions  teacher judge {data['teacher_score']:.2f}"
            f"  arrived {data['teacher_arrived']:.2f}  dq {data['teacher_dq']}"
            f"  {time.perf_counter() - t0:.1f}s",
            flush=True,
        )
        if n == 0:
            raise SystemExit("teacher produced no transitions")

        feat = torch.from_numpy(data["feat"])
        act = torch.from_numpy(data["action"])
        # Held out from the fit so the reported loss is not the training loss.
        cut = int(n * 0.9)
        perm = torch.from_numpy(np.random.default_rng(args.seed).permutation(n))
        tr, va = perm[:cut], perm[cut:]

        opt = torch.optim.Adam(net.parameters(), lr=args.lr)
        rng = np.random.default_rng(args.seed)
        series = []
        for ep in range(args.epochs):
            order = tr[torch.from_numpy(rng.permutation(len(tr)))]
            losses = []
            for s in range(0, len(order), args.batch):
                idx = order[s : s + args.batch]
                mu, _, _ = net(feat[idx])
                loss: Tensor = torch.nn.functional.mse_loss(mu, act[idx])
                opt.zero_grad(set_to_none=True)
                # torch ships no annotation for Tensor.backward; the alternative
                # is leaving `loss` as Any, which is worse typing, not better
                loss.backward()  # type: ignore[no-untyped-call]
                opt.step()
                losses.append(float(loss.detach()))
            with torch.no_grad():
                mu, _, _ = net(feat[va])
                vl = float(torch.nn.functional.mse_loss(mu, act[va]))
            series.append(
                {"epoch": ep + 1, "train": round(float(np.mean(losses)), 6), "val": round(vl, 6)}
            )
            if (ep + 1) % 10 == 0 or ep == 0:
                print(
                    f"  epoch {ep + 1:3d}  train {series[-1]['train']:.5f}  val {vl:.5f}",
                    flush=True,
                )

        CKPT.mkdir(exist_ok=True)
        ckpt = CKPT / f"{args.tag}.pt"
        policy.save(str(ckpt), net)

        from dimos.navigation.motion.control.research.ml.train import evaluate

        ev = evaluate(pool, net, env.val_worlds(args.val_worlds), ep_cfg)
        print(
            f"cloned net on held-out worlds: judge {ev['score']:.2f}"
            f"  arrived {ev['arrived']:.2f}  precision {ev['precision']:.3f}"
            f"  dq {ev['dq']}  {ev['outcomes']}",
            flush=True,
        )
    finally:
        pool.close()

    LOGS.mkdir(exist_ok=True)
    log = {
        "tag": args.tag,
        "config": vars(args) | {"obs_format": obs.FORMAT, "law": law},
        "params": policy.param_count(net),
        "transitions": int(n),
        "teacher": {
            "score": data["teacher_score"],
            "arrived": data["teacher_arrived"],
            "dq": data["teacher_dq"],
        },
        "series": series,
        "cloned_eval": ev,
        "wall_s": round(time.perf_counter() - t0, 1),
        "policy_cfg": asdict(net.cfg),
    }
    (LOGS / f"bc_{args.tag}.json").write_text(json.dumps(log, indent=1) + "\n")
    print(
        json.dumps(
            {
                "tag": args.tag,
                "teacher": data["teacher_score"],
                "cloned": ev["score"],
                "checkpoint": str(ckpt),
            }
        )
    )


if __name__ == "__main__":
    main()
