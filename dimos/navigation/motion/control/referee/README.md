# referee: what scores a controller

Ground-truth worlds, the closed-loop episode (referee world → planner →
controller → matched Go2 MuJoCo sim), and the judge that scores what the robot
*did*, not what was planned.

| | |
|---|---|
| `world.py` | scenario → MuJoCo model, wall contacts, planner cloud, plan → `Path` |
| `episode.py` | one episode: 29 Hz pose ZOH, transport delay, hardware slew, actuator lag, 5 Hz replan |
| `judge.py` | `gate * (100*arrived + 10*precision + pace_weight*pace + 0.5*composure)` |
| `battery.py` | the curated 16, generated worlds, held-out OOD rules, parallel episode workers |
| `probe_walk_slip.py` | what the gait actually delivers — provenance for `walk_gain`/`walk_slip` |

Scoring, tracks and the metrics are documented in [../tools.md](../tools.md).

## The rule

Shared by every candidate — the shipped laws in [`../laws/`](../laws/), an
autoresearch lab's, a learned one — so their numbers are comparable by
construction. The dependency runs one way:

```
research/*  →  referee/  →  simulation/, planner/referee/
```

Nothing here may import a candidate. The moment a judge knows which candidate
it is scoring, the comparison it exists to support is gone.
[`../../test_layering.py`](../../test_layering.py) holds it.

Scores are pinned to this directory's bytes: changing anything here obsoletes
the labs' `referee.lock`, so re-export and re-baseline before comparing numbers
across the change.
