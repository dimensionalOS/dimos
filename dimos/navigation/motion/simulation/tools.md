# Tools

Every entry point in this package, copy-paste ready. Run from the repo root.
Two recording/net pairs ship in the `ml-trajectory-research` LFS archive:

| recording | net | |
|---|---|---|
| `unitree_himloco01.mcap` | `freewalk_mcf.bin` | plain walking, 45-obs |
| `unitree_v11_gait_height01.mcap` | `v11_final.bin` | 46-obs, commandable body height (crouches to 0.10 m around t=32) |

**Watch the fitted sim next to reality** — MuJoCo window, recorded commands
drive the policy, green ghost box is the recorded pose:

```bash
python -m dimos.navigation.motion.simulation data/ml-trajectory-research/unitree_himloco01.mcap --policy data/ml-trajectory-research/freewalk_mcf.bin --view --ghost --fitted --start 6
```

`--start 6` skips the stand-up (the sim always begins standing). `--speed 0.5`
slow motion, `--seconds 20` cut short. Without `--fitted` you watch stock
menagerie physics — visibly wobblier than the real robot.

**Score a configuration** — per-statistic sim vs real table, noise-floor
units; SNR under ~1 means matched to within what chaos already does:

```bash
python -m dimos.navigation.motion.simulation data/ml-trajectory-research/unitree_himloco01.mcap --policy data/ml-trajectory-research/freewalk_mcf.bin --eval --fitted
```

**Try your own physics** — override any key on top of (or instead of) the
preset:

```bash
python -m dimos.navigation.motion.simulation data/ml-trajectory-research/unitree_himloco01.mcap --policy data/ml-trajectory-research/freewalk_mcf.bin --eval --fitted --physics damping=0.5,leg_mass_scale=1.2 --command-delay 0.02 --actuator-tau 0.01
```

**Refit** — CMA-ES over the 11-parameter space against one recording:

```bash
python -m dimos.navigation.motion.simulation.search data/ml-trajectory-research/unitree_himloco01.mcap data/ml-trajectory-research/freewalk_mcf.bin --trials 100
```

`--multi` switches to NSGA-II and prints the Pareto front over the
gait/translation/rotation/legs groups. `--storage sqlite:///search.db`
resumes, `--json out.json` saves.

**Joint refit** — score every trial on several recordings at once (this is
how `FITTED_*` was produced; single-recording fits absorb that run's style):

```bash
python -m dimos.navigation.motion.simulation.search data/ml-trajectory-research/unitree_himloco01.mcap data/ml-trajectory-research/freewalk_mcf.bin --also data/ml-trajectory-research/unitree_v11_gait_height01.mcap data/ml-trajectory-research/v11_final.bin --seed-fitted --trials 300
```

`--seed-fitted` enqueues the current preset as trial 0, so the search has to
beat it, not rediscover it. `--seed-preset <name|path.json>` seeds from any
other preset instead.

**Presets** — named physics, `fitted` (the validated two-recording tune) by
default everywhere, `stock` for bare menagerie. A search writes its winner to a
NEW name and never over a built-in:

```bash
python -m dimos.navigation.motion.simulation.search ... --seed-preset fitted --save-preset field_20260806
python -m dimos.navigation.motion.simulation --preset field_20260806.json --eval ...
```

`Preset.save()` refuses a built-in name outright. Field data is messy and a
refit that wins on one pair may lose everywhere else, so a new tune stands
*next to* the old one and has to earn the swap.

**Replay the recorded lowcmd** — drop `--policy`. The robot collapses in
seconds; that is what open-loop replay of a closed loop's commands does, and
it is why everything above compares command-to-command instead:

```bash
python -m dimos.navigation.motion.simulation data/ml-trajectory-research/unitree_himloco01.mcap --view
```

**Score a field navigation run** — a `*.zenoh.mcap` + sidecar pair instead of a
VR session: commands from `cmd_vel`, ground truth from Point-LIO resolved to
`base_link`. Reports twist-tracking error binned by how aggressive the command
was, which is what says whether the fitted physics covers a regime:

```bash
python -m dimos.navigation.motion.simulation.field data/ml-trajectory-research/20260806-063428.zenoh.mcap --policy data/ml-trajectory-research/freewalk_mcf.bin --fitted
```

`--from 6.9 --to 9.0` bounds what gets reported (seconds in, or a UTC
`HH:MM:SS`) while the rollout still starts where the robot was standing —
`--sidecar` names the pair's other half when it is not the matching `*.mcap`,
`--view --ghost` watches it against the LIO track, `--lag 0` disables the
measured LIO clock correction (leave it on: it is 170 ms on this pair, and
skipping it manufactures a quarter-metre of path error during a fast spin). On
a recording whose odometry speaks sensor time the lag comes off the stamps
themselves and the cross-correlation only cross-checks it.

**Drive it by hand** — constant command instead of a recording:

```python
import numpy as np
from dimos.navigation.motion.simulation.policy import FreePolicy
from dimos.navigation.motion.simulation.walk import walk

policy = FreePolicy.load("data/ml-trajectory-research/freewalk_mcf.bin")
walk(policy, command=np.array([0.5, 0.0, 0.0]), seconds=4, view=True)
```

**Freeze a real run into a static world** — stability-filtered voxel union of
the raycaster's `local_map`, plus the floor, the recorded start pose, and the
recorded global path/goal. Small npz, no recording needed afterwards:

```bash
python -m dimos.navigation.motion.simulation.recorded_world ml-trajectory-research/20260805-033007.zenoh.mcap --out athens.npz
```

`--stability 0.5` is the filter: a voxel survives only if it was seen in that
fraction of the frames spanning its own first-to-last sighting (the recorded
map flickers ~10% frame to frame). `--voxel 0.08` grid, `--carve 0.10` the
clearance granted to whatever the robot's body swept — it maps its own legs,
so without a carve the spawn pose starts inside a wall.

**Eyeball it** — the fitted Go2 standing at the recorded start pose inside the
recorded room, walls as greedy-merged static boxes, floor plane at the
estimated floor:

```bash
python -m dimos.navigation.motion.simulation.recorded_world athens.npz --view
```

`--max-z 1.5` is how much of the room becomes collision geometry (the ceiling
is not a wall); `--radius 3` crops to a corridor around the recorded track
when the geom count gets silly.

**Run it as a world** — the same npz is a referee scenario and a closed-loop
episode, alongside the curated and generated batteries:

```bash
python -m dimos.navigation.motion.planner --recorded athens.npz -s athens
python -m dimos.navigation.motion.control --recorded athens.npz -s athens --view
```

**Tests and types**:

```bash
python -m pytest dimos/navigation/motion/simulation -q
python -m mypy dimos/navigation/motion/simulation
```

Tracker knobs, rarely needed: `--mount-yaw 94` (robot forward within the
tracker xy plane, fitted) and `--tracker-z 0.207` (tracker offset from
`base`; a guess — tune until the ghost sits on the body).
