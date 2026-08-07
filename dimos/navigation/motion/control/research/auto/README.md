# control/research/auto — the trajectory-controller autoresearch lab

The evo lab that wrote the laws in [`control/laws/`](../../laws/). It is not
checked in here: the loop needs write access to its candidate and *no* write
access to the referee scoring it, so the mutating copy lives outside this repo
with the referee vendored and hash-pinned.

| | |
|---|---|
| lab repo | `motion-tc-autoresearch` (`ssh://…/home/git/repos/motion-tc-autoresearch.git`) |
| local checkout | `~/coding/motion-tc-autoresearch` |
| run hosts | frankfurt-research01 (blind), frankfurt-research02 (hinted) |
| referee | this repo: `control/referee/`, `simulation/`, `planner/referee/` |
| candidate seam | `candidate.controller:make` → `--controller candidate.controller:make` |

## What it produced

Both tracks started from the seed law (holonomic pursuit + clearance governor)
and finished within 0.3 of their ceiling, over 228 episodes each — curated 16
+ 40 generated + 20 held-out OOD worlds, × 3 domain-randomization draws, at
5 Hz replanning:

| track | seed | landed | ceiling | as |
|---|---|---|---|---|
| hinted | 73.61 | **115.23** (exp_0045, `hint_research01`) | 115.5 | [`laws/hinted.py`](../../laws/hinted.py) |
| blind | 68.24 | **110.77** (exp_0013, `blind_research01`) | 111 | [`laws/blind.py`](../../laws/blind.py) |

hinted: 225 goal / 3 refused, zero collisions, zero timeouts, zero falls.

**The benchmark is saturated.** Neither track has room left to research on this
referee — a new candidate, learned or evolved, can at best tie. Making the
referee harder (cloud noise, occlusion, pose drift, moving obstacles) is what
would reopen it; see [`../ml/README.md`](../ml/README.md).

## How a result lands

The candidate source comes home as a law module under `control/laws/`, with a
header naming the lab branch, the evo experiment id and the score it was
measured at — see the top of `laws/hinted.py`. `tracks.py` then points the
track at it, which is the one line that makes a generation live. The evo run
tree (`.evo/`, worktrees, cargo targets) stays on the research host and is
never committed.

Because scores are pinned to the referee's bytes, **any change under
`control/referee/` or `planner/referee/` obsoletes the lab's `referee.lock`**:
re-export, re-baseline, and re-record the numbers in the lab's `PROVENANCE.md`
before trusting a comparison across the change.
