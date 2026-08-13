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

"""PPO on the referee's own episodes, with an adaptive budget.

    python -m dimos.navigation.motion.control.research.ml.train --jobs 12

Each iteration rolls out ``--episodes`` training worlds in parallel worker
processes (one MuJoCo episode each, ~0.6 s), prices the tapes, and takes a few
PPO epochs over the pooled transitions. Every ``--eval-every`` iterations the
DETERMINISTIC policy is scored on held-out worlds by
:func:`judge.score_episode` -- the real objective, never the shaped reward --
and that number is what selects the checkpoint and what the stop rules read.

The budget is adaptive, the same three rules the planner-side lab uses:

  plateau   the best eval over the last ``--patience`` evals improved by less
            than ``--plateau-abs`` points versus the best before that window
  failfast  ``--failfast-min`` minutes in and still no better than the
            untrained baseline
  cap       ``--cap-min`` minutes of wall clock, the hard wall

The stop reason, the eval series, the config and the untrained baseline land
in ``logs/train_<tag>.json``.
"""

from __future__ import annotations

import argparse
from collections.abc import Callable
from concurrent import futures
from dataclasses import asdict
import json
import multiprocessing as mp
import os
from pathlib import Path
import time
from typing import Any
import zlib

import numpy as np
import torch

from dimos.navigation.motion.control.referee.episode import EpisodeConfig
from dimos.navigation.motion.control.research.ml import env, obs, policy
from dimos.navigation.motion.scenarios import Scenario

HERE = Path(__file__).resolve().parent
LOGS = HERE / "logs"
CKPT = HERE / "checkpoints"

DEFAULT_POLICY = "ml-trajectory-research/freewalk_mcf.bin"

_WORKER: dict[str, Any] = {}


def _init_worker(policy_path: str, cfg_blob: dict[str, Any]) -> None:
    for v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
        os.environ.setdefault(v, "1")
    torch.set_num_threads(1)
    from dimos.navigation.motion.control.research.ml.candidate import config as cand_config
    from dimos.navigation.motion.simulation.policy import FreePolicy
    from dimos.utils.data import get_data

    _WORKER["walk"] = FreePolicy.load(get_data(policy_path))
    _WORKER["net"] = policy.ActorCritic(policy.PolicyCfg(**cfg_blob))
    _WORKER["config"] = cand_config()


def _run_job(job: tuple[Scenario, dict[str, Any], EpisodeConfig, bool, int]) -> dict[str, Any]:
    """One episode in a worker. Returns arrays, not objects: this crosses a pipe."""
    sc, weights, ep_cfg, deterministic, seed = job
    net: policy.ActorCritic = _WORKER["net"]
    net.load_state_dict(weights)
    net.eval()
    rng = np.random.default_rng([seed, zlib.crc32(sc.name.encode())])
    ro = env.rollout(
        sc,
        net,
        _WORKER["walk"],
        ep_cfg,
        config=_WORKER["config"],
        deterministic=deterministic,
        rng=rng,
    )
    return {
        "feat": np.asarray(ro.tape.feat, dtype=np.float32).reshape(-1, len(ro.tape.feat[0]))
        if len(ro.tape)
        else np.zeros((0, 0), dtype=np.float32),
        "action": np.asarray(ro.tape.action, dtype=np.float32).reshape(-1, policy.ACT_DIM),
        "logp": np.asarray(ro.tape.logp, dtype=np.float32),
        "value": np.asarray(ro.tape.value, dtype=np.float32),
        "reward": np.asarray(ro.reward, dtype=np.float32),
        "row": ro.row,
    }


class Pool:
    """Worker pool, or serial execution when ``jobs <= 1``."""

    def __init__(self, jobs: int, policy_path: str, cfg: policy.PolicyCfg) -> None:
        self.jobs = jobs
        if jobs <= 1:
            _init_worker(policy_path, asdict(cfg))
            self._pool = None
            return
        self._pool = futures.ProcessPoolExecutor(
            max_workers=jobs,
            mp_context=mp.get_context("spawn"),
            initializer=_init_worker,
            initargs=(policy_path, asdict(cfg)),
        )

    def map(
        self, jobs: list[Any], fn: Callable[[Any], dict[str, Any]] = _run_job
    ) -> list[dict[str, Any]]:
        """Run ``fn`` over ``jobs``. ``fn`` must be importable in the workers."""
        if self._pool is None:
            return [fn(j) for j in jobs]
        return list(self._pool.map(fn, jobs))

    def close(self) -> None:
        if self._pool is not None:
            self._pool.shutdown()


def gae(
    reward: np.ndarray, value: np.ndarray, gamma: float, lam: float
) -> tuple[np.ndarray, np.ndarray]:
    """Advantages and returns for ONE finished episode.

    Every episode here terminates -- goal, collision, fall, refusal or the
    40 s cap -- so there is no bootstrap past the end. A timeout is a real
    failure in this referee, not a truncation to be discounted away.
    """
    n = len(reward)
    adv = np.zeros(n, dtype=np.float32)
    running = 0.0
    for i in range(n - 1, -1, -1):
        next_v = value[i + 1] if i + 1 < n else 0.0
        delta = reward[i] + gamma * next_v - value[i]
        running = delta + gamma * lam * running
        adv[i] = running
    return adv, adv + value


def evaluate(
    pool: Pool, net: policy.ActorCritic, scenarios: list[Scenario], cfg: EpisodeConfig
) -> dict[str, Any]:
    """Deterministic policy against the real judge on held-out worlds."""
    weights = {k: v.clone() for k, v in net.state_dict().items()}
    out = pool.map([(sc, weights, cfg, True, 0) for sc in scenarios])
    rows = [o["row"] for o in out]
    totals = [r["total"] for r in rows]
    outcomes: dict[str, int] = {}
    for r in rows:
        outcomes[r["outcome"]] = outcomes.get(r["outcome"], 0) + 1
    return {
        "score": round(float(np.mean(totals)), 3),
        "dq": sum(1 for r in rows if r["dq"]),
        "arrived": round(float(np.mean([r["arrived"] for r in rows])), 4),
        "precision": round(float(np.mean([r["precision"] for r in rows])), 4),
        "pace": round(float(np.mean([r["pace"] for r in rows])), 4),
        "outcomes": outcomes,
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--tag", default="base", help="run name: log + checkpoint filename")
    ap.add_argument("--track", default="hinted", choices=("hinted", "blind"))
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 2))
    ap.add_argument("--episodes", type=int, default=24, help="training episodes per iteration")
    ap.add_argument("--train-worlds", type=int, default=200, help="generated worlds to train on")
    ap.add_argument("--val-worlds", type=int, default=16, help="held-out worlds for the eval")
    ap.add_argument("--policy", default=DEFAULT_POLICY, help="FREE walking policy blob")
    ap.add_argument("--replan-hz", type=float, default=5.0)
    ap.add_argument(
        "--init",
        default=None,
        metavar="CKPT",
        help="warm start from a checkpoint (normally bc.py's; see its docstring)",
    )
    ap.add_argument(
        "--init-log-std",
        type=float,
        default=-3.0,
        help="exploration width on a warm start; scaled to the task's precision, not convention",
    )
    # PPO
    ap.add_argument("--lr", type=float, default=1e-4, help="actor lr (fine-tuning a clone)")
    ap.add_argument("--vf-lr", type=float, default=3e-4, help="critic lr")
    ap.add_argument(
        "--warmup-iters",
        type=int,
        default=5,
        help="iterations updating the CRITIC ONLY before the policy is allowed to move",
    )
    ap.add_argument("--gamma", type=float, default=0.99)
    ap.add_argument("--lam", type=float, default=0.95)
    ap.add_argument("--clip", type=float, default=0.2)
    ap.add_argument("--epochs", type=int, default=4, help="PPO epochs per iteration")
    ap.add_argument("--batch", type=int, default=1024, help="minibatch transitions")
    ap.add_argument("--vf-coef", type=float, default=0.5)
    # Zero by default, because log_std is a free parameter and an entropy bonus
    # pays the policy to widen it -- re-inflating exactly the noise that
    # --init-log-std exists to hold down. An entropy bonus buys exploration in
    # sparse-reward tasks; here the reward is dense and the starting policy is
    # already good, so it only buys collisions.
    ap.add_argument("--ent-coef", type=float, default=0.0)
    ap.add_argument("--max-grad-norm", type=float, default=0.5)
    ap.add_argument("--seed", type=int, default=0)
    # budget
    ap.add_argument("--eval-every", type=int, default=5, help="iterations between evals")
    ap.add_argument("--patience", type=int, default=6, help="evals in the plateau window")
    ap.add_argument("--plateau-abs", type=float, default=0.5, help="judge points that count")
    ap.add_argument("--cap-min", type=float, default=180.0)
    ap.add_argument("--failfast-min", type=float, default=20.0)
    args = ap.parse_args()

    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    rng = np.random.default_rng(args.seed)

    ep_cfg = env.episode_config(args.track, replan_hz=args.replan_hz)
    train_pool = env.train_worlds(args.train_worlds)
    val_pool_worlds = env.val_worlds(args.val_worlds)

    if args.init:
        net = policy.load(args.init)
        net.train()
        for p in net.parameters():
            p.requires_grad_(True)
        # Exploration has to be scaled to the PRECISION OF THE TASK, not to a
        # convention. The clone tracks the teacher to ~0.02 m/s; the PPO-ish
        # default of log_std -1.6 is sigma 0.2 normalised, which on this
        # envelope is 0.19 m/s of random command per tick -- ten times the
        # signal it is perturbing. Measured with the actor FROZEN, so nothing
        # but the noise was acting: 7 of 24 episodes disqualified and the
        # rollout score fell from 115.05 to 38.73. Sampling alone destroyed the
        # policy. -3.0 is sigma ~0.05 (0.047 m/s), a couple of times the
        # clone's own tracking error -- enough to explore, small enough that an
        # episode survives being explored.
        with torch.no_grad():
            net.log_std.fill_(args.init_log_std)
        print(f"warm start from {args.init}", flush=True)
    else:
        net = policy.ActorCritic()
    # Two optimisers, because the warmup has to move one half and not the
    # other: a cloned actor is good and a fresh critic is noise, and letting
    # the policy chase that noise is what wrecks the clone.
    opt_actor = torch.optim.Adam(net.actor_parameters(), lr=args.lr)
    opt_critic = torch.optim.Adam(net.critic_parameters(), lr=args.vf_lr)
    pool = Pool(args.jobs, args.policy, net.cfg)

    LOGS.mkdir(exist_ok=True)
    CKPT.mkdir(exist_ok=True)
    ckpt = CKPT / f"{args.tag}.pt"
    log: dict[str, Any] = {
        "tag": args.tag,
        "config": vars(args) | {"obs_format": obs.FORMAT},
        "params": policy.param_count(net),
        "evals": [],
        "iters": [],
    }

    t0 = time.perf_counter()
    try:
        base = evaluate(pool, net, val_pool_worlds, ep_cfg)
        log["baseline_eval"] = base
        log["evals"].append({"iter": 0, "elapsed_s": 0.0, **base})
        print(
            f"params {log['params']:,}  worlds {len(train_pool)} train / {len(val_pool_worlds)} val"
            f"  baseline judge score {base['score']:.2f}",
            flush=True,
        )

        best = base["score"]
        policy.save(str(ckpt), net)
        evals = [base["score"]]
        reason: str | None = None
        it = 0

        while reason is None:
            it += 1
            picks = rng.choice(
                len(train_pool), size=args.episodes, replace=args.episodes > len(train_pool)
            )
            weights = {k: v.clone() for k, v in net.state_dict().items()}
            jobs = [(train_pool[int(i)], weights, ep_cfg, False, args.seed + it) for i in picks]
            out = pool.map(jobs)

            feats, acts, logps, advs, rets = [], [], [], [], []
            for o in out:
                if len(o["reward"]) == 0:
                    continue
                a, r = gae(o["reward"], o["value"], args.gamma, args.lam)
                feats.append(o["feat"])
                acts.append(o["action"])
                logps.append(o["logp"])
                advs.append(a)
                rets.append(r)
            if not feats:
                print(f"iter {it}: every episode refused before a single command", flush=True)
                continue

            b_feat = torch.from_numpy(np.concatenate(feats))
            b_act = torch.from_numpy(np.concatenate(acts))
            b_logp = torch.from_numpy(np.concatenate(logps))
            b_adv = torch.from_numpy(np.concatenate(advs))
            b_ret = torch.from_numpy(np.concatenate(rets))
            b_adv = (b_adv - b_adv.mean()) / (b_adv.std() + 1e-8)

            warming = it <= args.warmup_iters
            n = len(b_feat)
            order = np.arange(n)
            losses: list[float] = []
            for _ in range(args.epochs):
                rng.shuffle(order)
                for s in range(0, n, args.batch):
                    idx = torch.from_numpy(order[s : s + args.batch].copy())
                    logp, ent, value = net.evaluate(b_feat[idx], b_act[idx])
                    vf = torch.nn.functional.mse_loss(value, b_ret[idx])
                    if warming:
                        loss = vf
                    else:
                        ratio = torch.exp(logp - b_logp[idx])
                        adv_mb = b_adv[idx]
                        pg = -torch.min(
                            ratio * adv_mb,
                            torch.clamp(ratio, 1 - args.clip, 1 + args.clip) * adv_mb,
                        ).mean()
                        loss = pg + args.vf_coef * vf - args.ent_coef * ent.mean()
                    opt_actor.zero_grad(set_to_none=True)
                    opt_critic.zero_grad(set_to_none=True)
                    loss.backward()  # type: ignore[no-untyped-call]  # torch stub gap
                    torch.nn.utils.clip_grad_norm_(net.parameters(), args.max_grad_norm)
                    opt_critic.step()
                    if not warming:
                        opt_actor.step()
                    losses.append(float(loss.detach()))

            rows = [o["row"] for o in out]
            rec = {
                "iter": it,
                "transitions": int(n),
                "phase": "warmup" if warming else "ppo",
                "loss": round(float(np.mean(losses)), 4),
                "train_score": round(float(np.mean([r["total"] for r in rows])), 2),
                "dq": sum(1 for r in rows if r["dq"]),
                "elapsed_s": round(time.perf_counter() - t0, 1),
            }
            log["iters"].append(rec)
            print(
                f"iter {it:4d}  {rec['phase']:<6s}  n {n:6d}  loss {rec['loss']:8.4f}"
                f"  train {rec['train_score']:6.2f}  dq {rec['dq']:2d}"
                f"  {rec['elapsed_s']:7.1f}s",
                flush=True,
            )

            # No eval during warmup: the policy has not moved, so the number
            # would be the baseline repeated, and it would start the plateau
            # window counting against iterations that cannot improve anything.
            if warming or it % args.eval_every:
                continue

            el = time.perf_counter() - t0
            ev = evaluate(pool, net, val_pool_worlds, ep_cfg)
            log["evals"].append({"iter": it, "elapsed_s": round(el, 1), **ev})
            evals.append(ev["score"])
            if ev["score"] > best:
                best = ev["score"]
                policy.save(str(ckpt), net)
            print(
                f"  eval  judge {ev['score']:6.2f}  best {best:6.2f}"
                f"  arrived {ev['arrived']:.2f}  precision {ev['precision']:.3f}"
                f"  dq {ev['dq']}  {ev['outcomes']}",
                flush=True,
            )

            if el > args.cap_min * 60:
                reason = "cap"
            elif el > args.failfast_min * 60 and max(evals[1:] or [-1e9]) <= evals[0]:
                reason = "failfast"
            elif len(evals) > args.patience + 1:
                ref = max(evals[: -args.patience])
                win = max(evals[-args.patience :])
                if win - ref < args.plateau_abs:
                    reason = "plateau"
    finally:
        pool.close()

    log.update(
        stop_reason=reason,
        iters_run=it,
        wall_s=round(time.perf_counter() - t0, 1),
        best_eval=round(best, 3),
        checkpoint=str(ckpt.relative_to(HERE)),
    )
    (LOGS / f"train_{args.tag}.json").write_text(json.dumps(log, indent=1) + "\n")
    print(
        json.dumps(
            {
                "tag": args.tag,
                "stop": reason,
                "iters": it,
                "wall_s": log["wall_s"],
                "baseline_eval": base["score"],
                "best_eval": round(best, 3),
                "checkpoint": str(ckpt),
            }
        )
    )


if __name__ == "__main__":
    main()
